/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/pwm.h>
#include <linux/err.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/ctype.h>
#endif

#include <asm/system_info.h>

#include "mdss_dsi.h"

#define DT_CMD_HDR 6
#define GAMMA_COMPAT 11

static bool mdss_panel_flip_ud = false;
static int mdss_panel_id = PANEL_QCOM;

DEFINE_LED_TRIGGER(bl_led_trigger);

#if defined(CONFIG_BACKLIGHT_LM3630)
extern void lm3630_lcd_backlight_set_level(int level);
#endif

static struct mdss_dsi_phy_ctrl phy_params;
static struct mdss_panel_common_pdata *local_pdata;
static struct work_struct send_cmds_work;
struct mdss_panel_data *cmds_panel_data;
static struct platform_driver this_driver;
static struct kobject *module_kobj;

void mdss_dsi_panel_pwm_cfg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret;

	if (!gpio_is_valid(ctrl->pwm_pmic_gpio)) {
		pr_err("%s: pwm_pmic_gpio=%d Invalid\n", __func__,
				ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
		return;
	}

	ret = gpio_request(ctrl->pwm_pmic_gpio, "disp_pwm");
	if (ret) {
		pr_err("%s: pwm_pmic_gpio=%d request failed\n", __func__,
				ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
		return;
	}

	ctrl->pwm_bl = pwm_request(ctrl->pwm_lpg_chan, "lcd-bklt");
	if (ctrl->pwm_bl == NULL || IS_ERR(ctrl->pwm_bl)) {
		pr_err("%s: lpg_chan=%d pwm request failed", __func__,
				ctrl->pwm_lpg_chan);
		gpio_free(ctrl->pwm_pmic_gpio);
		ctrl->pwm_pmic_gpio = -1;
	}
}

static void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	int ret;
	u32 duty;

	if (ctrl->pwm_bl == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

	duty = level * ctrl->pwm_period;
	duty /= ctrl->bklt_max;

	pr_debug("%s: bklt_ctrl=%d pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__, ctrl->bklt_ctrl, ctrl->pwm_period,
				ctrl->pwm_pmic_gpio, ctrl->pwm_lpg_chan);

	pr_debug("%s: ndx=%d level=%d duty=%d\n", __func__,
					ctrl->ndx, level, duty);

	ret = pwm_config(ctrl->pwm_bl, duty, ctrl->pwm_period);
	if (ret) {
		pr_err("%s: pwm_config() failed err=%d.\n", __func__, ret);
		return;
	}

	ret = pwm_enable(ctrl->pwm_bl);
	if (ret)
		pr_err("%s: pwm_enable() failed err=%d\n", __func__, ret);
}

static char dcs_cmd[2] = {0x54, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dcs_cmd)},
	dcs_cmd
};

u32 mdss_dsi_dcs_read(struct mdss_dsi_ctrl_pdata *ctrl,
			char cmd0, char cmd1)
{
	struct dcs_cmd_req cmdreq;

	dcs_cmd[0] = cmd0;
	dcs_cmd[1] = cmd1;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 1;
	cmdreq.cb = NULL; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	/*
	 * blocked here, until call back called
	 */

	return 0;
}

static void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static char led_pwm1[2] = {0x51, 0x0};	/* DTYPE_DCS_WRITE1 */
static struct dsi_cmd_desc backlight_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(led_pwm1)},
	led_pwm1
};

static void mdss_dsi_panel_bklt_dcs(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	struct dcs_cmd_req cmdreq;

	pr_debug("%s: level=%d\n", __func__, level);

	led_pwm1[1] = (unsigned char)level;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

void mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);

	if (enable) {
		if (mdss_panel_id == PANEL_LGE_JDI_ORISE_VIDEO ||
			mdss_panel_id == PANEL_LGE_JDI_ORISE_CMD ||
			mdss_panel_id == PANEL_LGE_JDI_NOVATEK_VIDEO ||
			mdss_panel_id == PANEL_LGE_JDI_NOVATEK_CMD) {
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
				gpio_set_value((ctrl_pdata->disp_en_gpio), 1);
			usleep(20 * 1000);
			gpio_set_value((ctrl_pdata->rst_gpio), 1);
			usleep(15 * 1000);
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			udelay(20);
			gpio_set_value((ctrl_pdata->rst_gpio), 1);
			usleep(10 * 1000);
		} else {
			gpio_set_value((ctrl_pdata->rst_gpio), 1);
			msleep(20);
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			udelay(200);
			gpio_set_value((ctrl_pdata->rst_gpio), 1);
			msleep(20);
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
				gpio_set_value((ctrl_pdata->disp_en_gpio), 1);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (mdss_panel_id == PANEL_LGE_JDI_ORISE_VIDEO ||
			mdss_panel_id == PANEL_LGE_JDI_ORISE_CMD ||
			mdss_panel_id == PANEL_LGE_JDI_NOVATEK_VIDEO ||
			mdss_panel_id == PANEL_LGE_JDI_NOVATEK_CMD) {
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
				gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
			usleep(20 * 1000);
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
		} else {
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
				gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
		}
	}
}

static char caset[] = {0x2a, 0x00, 0x00, 0x03, 0x00};	/* DTYPE_DCS_LWRITE */
static char paset[] = {0x2b, 0x00, 0x00, 0x05, 0x00};	/* DTYPE_DCS_LWRITE */

static struct dsi_cmd_desc partial_update_enable_cmd[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(caset)}, caset},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(paset)}, paset},
};

static int mdss_dsi_panel_partial_update(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct dcs_cmd_req cmdreq;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mipi  = &pdata->panel_info.mipi;

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	caset[1] = (((pdata->panel_info.roi_x) & 0xFF00) >> 8);
	caset[2] = (((pdata->panel_info.roi_x) & 0xFF));
	caset[3] = (((pdata->panel_info.roi_x - 1 + pdata->panel_info.roi_w)
								& 0xFF00) >> 8);
	caset[4] = (((pdata->panel_info.roi_x - 1 + pdata->panel_info.roi_w)
								& 0xFF));
	partial_update_enable_cmd[0].payload = caset;

	paset[1] = (((pdata->panel_info.roi_y) & 0xFF00) >> 8);
	paset[2] = (((pdata->panel_info.roi_y) & 0xFF));
	paset[3] = (((pdata->panel_info.roi_y - 1 + pdata->panel_info.roi_h)
								& 0xFF00) >> 8);
	paset[4] = (((pdata->panel_info.roi_y - 1 + pdata->panel_info.roi_h)
								& 0xFF));
	partial_update_enable_cmd[1].payload = paset;

	pr_debug("%s: enabling partial update\n", __func__);
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = partial_update_enable_cmd;
	cmdreq.cmds_cnt = 2;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	return rc;
}

static void mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata,
							u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
#if defined(CONFIG_BACKLIGHT_LM3630)
		lm3630_lcd_backlight_set_level(bl_level);
#else
		led_trigger_event(bl_led_trigger, bl_level);
#endif
		break;
	case BL_PWM:
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
		break;
	case BL_DCS_CMD:
		mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
			__func__);
		break;
	}
}

static int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mipi  = &pdata->panel_info.mipi;

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (local_pdata->on_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &local_pdata->on_cmds);

	pr_info("%s\n", __func__);
	return 0;
}

static int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	mipi  = &pdata->panel_info.mipi;

	if (!gpio_get_value(ctrl->disp_en_gpio))
		return 0;

	if (ctrl->off_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds);

	pr_info("%s:\n", __func__);
	return 0;
}

static int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len > sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			return -ENOMEM;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		kfree(buf);
		return -ENOMEM;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		return -ENOMEM;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	pcmds->link_state = DSI_LP_MODE; /* default */

	data = of_get_property(np, link_key, NULL);
	if (!strncmp(data, "DSI_HS_MODE", 11))
		pcmds->link_state = DSI_HS_MODE;

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;
}

int mdss_dsi_panel_id(void)
{
	return mdss_panel_id;
}

bool mdss_dsi_panel_flip_ud(void)
{
	return mdss_panel_flip_ud;
}

static int mdss_panel_parse_dt(struct platform_device *pdev,
			       struct mdss_panel_common_pdata *panel_data)
{
	struct device_node *np = pdev->dev.of_node;
	u32 res[6], tmp;
	u32 fbc_res[7];
	int rc, i, len;
	const char *data;
	static const char *bl_ctrl_type, *pdest;
	bool fbc_enabled = false;

	rc = of_property_read_u32_array(np, "qcom,mdss-pan-res", res, 2);
	if (rc) {
		pr_err("%s:%d, panel resolution not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	panel_data->panel_info.xres = (!rc ? res[0] : 640);
	panel_data->panel_info.yres = (!rc ? res[1] : 480);

	rc = of_property_read_u32(np, "qcom,mdss-pan-id", &tmp);
	if (!rc)
		mdss_panel_id = tmp;
	pr_info("%s: Panel ID = %d\n", __func__, mdss_panel_id);

	mdss_panel_flip_ud = of_property_read_bool(np, "qcom,mdss-pan-flip-ud");
	if (mdss_panel_flip_ud)
		pr_info("%s: Panel FLIP UD\n", __func__);

	rc = of_property_read_u32_array(np, "qcom,mdss-pan-active-res", res, 2);
	if (rc == 0) {
		panel_data->panel_info.lcdc.xres_pad =
			panel_data->panel_info.xres - res[0];
		panel_data->panel_info.lcdc.yres_pad =
			panel_data->panel_info.yres - res[1];
	}

	rc = of_property_read_u32(np, "qcom,mdss-pan-bpp", &tmp);
	if (rc) {
		pr_err("%s:%d, panel bpp not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	panel_data->panel_info.bpp = (!rc ? tmp : 24);

	rc = of_property_read_u32(np, "qcom,mdss-pan-width", &tmp);
	if (rc)
		pr_warn("%s:%d, panel width not specified\n",
						__func__, __LINE__);
	panel_data->panel_info.width = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-pan-height", &tmp);
	if (rc)
		pr_warn("%s:%d, panel height not specified\n",
						__func__, __LINE__);
	panel_data->panel_info.height = (!rc ? tmp : 0);

	pdest = of_get_property(pdev->dev.of_node,
				"qcom,mdss-pan-dest", NULL);
	if (strlen(pdest) != 9) {
		pr_err("%s: Unknown pdest specified\n", __func__);
		return -EINVAL;
	}
	if (!strncmp(pdest, "display_1", 9))
		panel_data->panel_info.pdest = DISPLAY_1;
	else if (!strncmp(pdest, "display_2", 9))
		panel_data->panel_info.pdest = DISPLAY_2;
	else {
		pr_debug("%s: pdest not specified. Set Default\n",
							__func__);
		panel_data->panel_info.pdest = DISPLAY_1;
	}

	rc = of_property_read_u32_array(np,
		"qcom,mdss-pan-porch-values", res, 6);
	panel_data->panel_info.lcdc.h_back_porch = (!rc ? res[0] : 6);
	panel_data->panel_info.lcdc.h_pulse_width = (!rc ? res[1] : 2);
	panel_data->panel_info.lcdc.h_front_porch = (!rc ? res[2] : 6);
	panel_data->panel_info.lcdc.v_back_porch = (!rc ? res[3] : 6);
	panel_data->panel_info.lcdc.v_pulse_width = (!rc ? res[4] : 2);
	panel_data->panel_info.lcdc.v_front_porch = (!rc ? res[5] : 6);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-underflow-clr", &tmp);
	panel_data->panel_info.lcdc.underflow_clr = (!rc ? tmp : 0xff);

	bl_ctrl_type = of_get_property(pdev->dev.of_node,
				  "qcom,mdss-pan-bl-ctrl", NULL);
	if ((bl_ctrl_type) && (!strncmp(bl_ctrl_type, "bl_ctrl_wled", 12))) {
		led_trigger_register_simple("bkl-trigger", &bl_led_trigger);
		pr_debug("%s: SUCCESS-> WLED TRIGGER register\n", __func__);

		panel_data->panel_info.bklt_ctrl = BL_WLED;
	} else if (!strncmp(bl_ctrl_type, "bl_ctrl_pwm", 11)) {
		panel_data->panel_info.bklt_ctrl = BL_PWM;

		rc = of_property_read_u32(np, "qcom,pwm-period", &tmp);
		if (rc) {
			pr_err("%s:%d, Error, panel pwm_period\n",
						__func__, __LINE__);
			return -EINVAL;
		}
		panel_data->panel_info.pwm_period = tmp;

		rc = of_property_read_u32(np, "qcom,pwm-lpg-channel", &tmp);
		if (rc) {
			pr_err("%s:%d, Error, dsi lpg channel\n",
						__func__, __LINE__);
			return -EINVAL;
		}
		panel_data->panel_info.pwm_lpg_chan = tmp;

		tmp = of_get_named_gpio(np, "qcom,pwm-pmic-gpio", 0);
		panel_data->panel_info.pwm_pmic_gpio =  tmp;
	} else if (!strncmp(bl_ctrl_type, "bl_ctrl_dcs", 11)) {
		panel_data->panel_info.bklt_ctrl = BL_DCS_CMD;
	} else {
		pr_debug("%s: Unknown backlight control\n", __func__);
		panel_data->panel_info.bklt_ctrl = UNKNOWN_CTRL;
	}

	rc = of_property_read_u32_array(np,
		"qcom,mdss-pan-bl-levels", res, 2);
	panel_data->panel_info.bl_min = (!rc ? res[0] : 0);
	panel_data->panel_info.bl_max = (!rc ? res[1] : 255);

	rc = of_property_read_u32(np, "qcom,mdss-pan-dsi-mode", &tmp);
	panel_data->panel_info.mipi.mode = (!rc ? tmp : DSI_VIDEO_MODE);

	rc = of_property_read_u32(np, "qcom,mdss-vsync-enable", &tmp);
	panel_data->panel_info.mipi.vsync_enable = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-hw-vsync-mode", &tmp);
	panel_data->panel_info.mipi.hw_vsync_mode = (!rc ? tmp : 0);


	rc = of_property_read_u32(np,
		"qcom,mdss-pan-dsi-h-pulse-mode", &tmp);
	panel_data->panel_info.mipi.pulse_mode_hsa_he = (!rc ? tmp : false);

	rc = of_property_read_u32_array(np,
		"qcom,mdss-pan-dsi-h-power-stop", res, 3);
	panel_data->panel_info.mipi.hbp_power_stop = (!rc ? res[0] : false);
	panel_data->panel_info.mipi.hsa_power_stop = (!rc ? res[1] : false);
	panel_data->panel_info.mipi.hfp_power_stop = (!rc ? res[2] : false);

	rc = of_property_read_u32_array(np,
		"qcom,mdss-pan-dsi-bllp-power-stop", res, 2);
	panel_data->panel_info.mipi.bllp_power_stop =
					(!rc ? res[0] : false);
	panel_data->panel_info.mipi.eof_bllp_power_stop =
					(!rc ? res[1] : false);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-dsi-traffic-mode", &tmp);
	panel_data->panel_info.mipi.traffic_mode =
			(!rc ? tmp : DSI_NON_BURST_SYNCH_PULSE);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-insert-dcs-cmd", &tmp);
	panel_data->panel_info.mipi.insert_dcs_cmd =
			(!rc ? tmp : 1);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-wr-mem-continue", &tmp);
	panel_data->panel_info.mipi.wr_mem_continue =
			(!rc ? tmp : 0x3c);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-wr-mem-start", &tmp);
	panel_data->panel_info.mipi.wr_mem_start =
			(!rc ? tmp : 0x2c);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-te-sel", &tmp);
	panel_data->panel_info.mipi.te_sel =
			(!rc ? tmp : 1);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-dsi-dst-format", &tmp);
	panel_data->panel_info.mipi.dst_format =
			(!rc ? tmp : DSI_VIDEO_DST_FORMAT_RGB888);

	rc = of_property_read_u32(np, "qcom,mdss-pan-dsi-vc", &tmp);
	panel_data->panel_info.mipi.vc = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-pan-dsi-rgb-swap", &tmp);
	panel_data->panel_info.mipi.rgb_swap = (!rc ? tmp : DSI_RGB_SWAP_RGB);

	rc = of_property_read_u32_array(np,
		"qcom,mdss-pan-dsi-data-lanes", res, 4);
	panel_data->panel_info.mipi.data_lane0 = (!rc ? res[0] : true);
	panel_data->panel_info.mipi.data_lane1 = (!rc ? res[1] : false);
	panel_data->panel_info.mipi.data_lane2 = (!rc ? res[2] : false);
	panel_data->panel_info.mipi.data_lane3 = (!rc ? res[3] : false);

	rc = of_property_read_u32(np, "qcom,mdss-pan-dsi-dlane-swap", &tmp);
	panel_data->panel_info.mipi.dlane_swap = (!rc ? tmp : 0);

	rc = of_property_read_u32_array(np, "qcom,mdss-pan-dsi-t-clk", res, 2);
	panel_data->panel_info.mipi.t_clk_pre = (!rc ? res[0] : 0x24);
	panel_data->panel_info.mipi.t_clk_post = (!rc ? res[1] : 0x03);

	rc = of_property_read_u32(np, "qcom,mdss-pan-dsi-stream", &tmp);
	panel_data->panel_info.mipi.stream = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-pan-dsi-mdp-tr", &tmp);
	panel_data->panel_info.mipi.mdp_trigger =
			(!rc ? tmp : DSI_CMD_TRIGGER_SW);
	if (panel_data->panel_info.mipi.mdp_trigger > 6) {
		pr_err("%s:%d, Invalid mdp trigger. Forcing to sw trigger",
						 __func__, __LINE__);
		panel_data->panel_info.mipi.mdp_trigger =
					DSI_CMD_TRIGGER_SW;
	}

	rc = of_property_read_u32(np, "qcom,mdss-pan-dsi-dma-tr", &tmp);
	panel_data->panel_info.mipi.dma_trigger =
			(!rc ? tmp : DSI_CMD_TRIGGER_SW);
	if (panel_data->panel_info.mipi.dma_trigger > 6) {
		pr_err("%s:%d, Invalid dma trigger. Forcing to sw trigger",
						 __func__, __LINE__);
		panel_data->panel_info.mipi.dma_trigger =
					DSI_CMD_TRIGGER_SW;
	}

	rc = of_property_read_u32(np, "qcom,mdss-pan-dsi-frame-rate", &tmp);
	panel_data->panel_info.mipi.frame_rate = (!rc ? tmp : 60);

	rc = of_property_read_u32(np, "qcom,mdss-pan-clk-rate", &tmp);
	panel_data->panel_info.clk_rate = (!rc ? tmp : 0);

	data = of_get_property(np, "qcom,panel-phy-regulatorSettings", &len);
	if ((!data) || (len != 7)) {
		pr_err("%s:%d, Unable to read Phy regulator settings",
		       __func__, __LINE__);
		goto error;
	}
	for (i = 0; i < len; i++)
		phy_params.regulator[i] = data[i];

	data = of_get_property(np, "qcom,panel-phy-timingSettings", &len);
	if ((!data) || (len != 12)) {
		pr_err("%s:%d, Unable to read Phy timing settings",
		       __func__, __LINE__);
		goto error;
	}
	for (i = 0; i < len; i++)
		phy_params.timing[i] = data[i];

	data = of_get_property(np, "qcom,panel-phy-strengthCtrl", &len);
	if ((!data) || (len != 2)) {
		pr_err("%s:%d, Unable to read Phy Strength ctrl settings",
		       __func__, __LINE__);
		goto error;
	}
	phy_params.strength[0] = data[0];
	phy_params.strength[1] = data[1];

	data = of_get_property(np, "qcom,panel-phy-bistCtrl", &len);
	if ((!data) || (len != 6)) {
		pr_err("%s:%d, Unable to read Phy Bist Ctrl settings",
		       __func__, __LINE__);
		goto error;
	}
	for (i = 0; i < len; i++)
		phy_params.bistCtrl[i] = data[i];

	data = of_get_property(np, "qcom,panel-phy-laneConfig", &len);
	if ((!data) || (len != 45)) {
		pr_err("%s:%d, Unable to read Phy lane configure settings",
		       __func__, __LINE__);
		goto error;
	}
	for (i = 0; i < len; i++)
		phy_params.laneCfg[i] = data[i];

	panel_data->panel_info.mipi.dsi_phy_db = &phy_params;

	fbc_enabled = of_property_read_bool(np,
			"qcom,fbc-enabled");
	if (fbc_enabled) {
		pr_debug("%s:%d FBC panel enabled.\n", __func__, __LINE__);
		panel_data->panel_info.fbc.enabled = 1;

		rc = of_property_read_u32_array(np,
				"qcom,fbc-mode", fbc_res, 7);
		panel_data->panel_info.fbc.target_bpp =
			(!rc ?	fbc_res[0] : panel_data->panel_info.bpp);
		panel_data->panel_info.fbc.comp_mode = (!rc ? fbc_res[1] : 0);
		panel_data->panel_info.fbc.qerr_enable =
			(!rc ? fbc_res[2] : 0);
		panel_data->panel_info.fbc.cd_bias = (!rc ? fbc_res[3] : 0);
		panel_data->panel_info.fbc.pat_enable = (!rc ? fbc_res[4] : 0);
		panel_data->panel_info.fbc.vlc_enable = (!rc ? fbc_res[5] : 0);
		panel_data->panel_info.fbc.bflc_enable =
			(!rc ? fbc_res[6] : 0);

		rc = of_property_read_u32_array(np,
				"qcom,fbc-budget-ctl", fbc_res, 3);
		panel_data->panel_info.fbc.line_x_budget =
			(!rc ? fbc_res[0] : 0);
		panel_data->panel_info.fbc.block_x_budget =
			(!rc ? fbc_res[1] : 0);
		panel_data->panel_info.fbc.block_budget =
			(!rc ? fbc_res[2] : 0);

		rc = of_property_read_u32_array(np,
				"qcom,fbc-lossy-mode", fbc_res, 4);
		panel_data->panel_info.fbc.lossless_mode_thd =
			(!rc ? fbc_res[0] : 0);
		panel_data->panel_info.fbc.lossy_mode_thd =
			(!rc ? fbc_res[1] : 0);
		panel_data->panel_info.fbc.lossy_rgb_thd =
			(!rc ? fbc_res[2] : 0);
		panel_data->panel_info.fbc.lossy_mode_idx =
			(!rc ? fbc_res[3] : 0);

	} else {
		pr_debug("%s:%d Panel does not support FBC.\n",
				__func__, __LINE__);
		panel_data->panel_info.fbc.enabled = 0;
		panel_data->panel_info.fbc.target_bpp =
			panel_data->panel_info.bpp;
	}

	mdss_dsi_parse_dcs_cmds(np, &panel_data->on_cmds,
		"qcom,panel-on-cmds", "qcom,on-cmds-dsi-state");

	mdss_dsi_parse_dcs_cmds(np, &panel_data->off_cmds,
		"qcom,panel-off-cmds", "qcom,off-cmds-dsi-state");

	return 0;

error:
	return -EINVAL;
}

#ifdef CONFIG_DEBUG_FS
#define MAX_ON_CMD_SIZE (PAGE_SIZE * 8)

ssize_t on_cmd_read(struct file *filp, char __user *user_buf, size_t count,
	loff_t *ppos)
{
	int ret;
	int i, j;
	char *buf;
	int buf_size;
	struct dsi_panel_cmds *pcmds;

	struct mdss_panel_common_pdata *panel_data = filp->private_data;
	if (!panel_data)
		return -ENODEV;

	pcmds = &panel_data->on_cmds;
	if (!pcmds)
		return -EINVAL;

	buf = kzalloc(MAX_ON_CMD_SIZE, GFP_KERNEL);

	strncpy(buf, "qcom,panel-on-cmds = [", 22);
	buf_size = 22;

	for (i = 0; i < pcmds->cmd_cnt; ++i) {
		if (buf_size + 20 +
			(pcmds->cmds[i].dchdr.dlen * 3) >
			MAX_ON_CMD_SIZE - 4) {
			pr_warn("Too many on_commands!(< 32KB)\n");
			ret = -EINVAL;
			goto read_error;
		}

		strncpy(&buf[buf_size], "\r\n", 2);
		buf_size += 2;

		ret = snprintf(&buf[buf_size], 21,
			"%02x %02x %02x %02x %02x %02x %02x",
			pcmds->cmds[i].dchdr.dtype,
			pcmds->cmds[i].dchdr.last,
			pcmds->cmds[i].dchdr.vc,
			pcmds->cmds[i].dchdr.ack,
			pcmds->cmds[i].dchdr.wait,
			(char)(pcmds->cmds[i].dchdr.dlen & 0xff00) >> 8,
			(char)(pcmds->cmds[i].dchdr.dlen & 0x00ff));

		if (ret < 0)
			goto read_error;

		buf_size += ret;

		if (0 < pcmds->cmds[i].dchdr.dlen && pcmds->cmds[i].dchdr.dlen < 3) {
			ret = snprintf(&buf[buf_size], 4, " %02x",
				pcmds->cmds[i].payload[0]);
			if (ret < 0)
				goto read_error;

			buf_size += ret;

			if (pcmds->cmds[i].dchdr.dlen == 2) {
				ret = snprintf(&buf[buf_size], 4, " %02x",
					pcmds->cmds[i].payload[1]);
				if (ret < 0)
					goto read_error;

				buf_size += ret;
			}
		} else if (pcmds->cmds[i].dchdr.dlen > 2) {
			for (j = 0; j < pcmds->cmds[i].dchdr.dlen; ++j) {
				if ((j % 6) == 0) {
					ret = snprintf(&buf[buf_size], 5, "\r\n%02x",
						pcmds->cmds[i].payload[j]);
					if (ret < 0)
						goto read_error;

					buf_size += ret;
				} else {
					ret = snprintf(&buf[buf_size], 4, " %02x",
						pcmds->cmds[i].payload[j]);
					if (ret < 0)
						goto read_error;

					buf_size += ret;
				}
			}
		} else {
			pr_err("Invalid data length!\n");
			ret = -EINVAL;
			goto read_error;
		}
	}

	strncpy(&buf[buf_size], "]", 2);
	buf_size += 2;

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, buf_size);
read_error:
	kfree(buf);

	return ret;
}

static int parse_on_cmds(char **on_cmds, const char *buf, size_t count)
{
	int i, j;
	int on_cmds_len;
	char *endptr;

	i = 0;
	do {
		if (!strncmp(&buf[i], "qcom,panel-on-cmds", 18)) {
			i += 18;
			while (i < count && buf[i] != '=')
				++i;
			while (i < count && buf[i] != '[')
				++i;
			++i;
			break;
		}
	} while (++i < count);

	if (i >= count) {
		pr_err("Invalid on_cmds start format!\n");
		return -EINVAL;
	}

	on_cmds_len = 0;

	for (j = i; j < count; ++j) {
		if (isxdigit(buf[j]) && isxdigit(buf[j + 1])) {
			++on_cmds_len;
			++j;
		} else if (buf[j] == ']')
			break;
	}

	if (j == count)
		return 0;

	if (buf[j] != ']') {
		pr_err("Invalid on_cmds end format!\n");
		return -EINVAL;
	}

	*on_cmds = kzalloc(on_cmds_len, GFP_KERNEL);

	if (!(*on_cmds))
		return -ENOMEM;

	j = 0;
	for (i = 0; i < count; ++i) {
		if (isxdigit(buf[i]) && isxdigit(buf[i + 1])) {
			endptr = (char *)&buf[i + 1];
			(*on_cmds)[j] = (char)simple_strtoul(&buf[i], &endptr, 16);
			++i;
			++j;
		}
	}

	return on_cmds_len;
}

static char *user_buf_total = NULL;
static int user_buf_total_pos = 0;

ssize_t on_cmd_write(struct file *filp, const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	static char *buf = NULL;
	char *prev_on_cmds = NULL;
	int blen = 0, len;
	char *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;
	int ret;
	struct dsi_panel_cmds *pcmds;

	struct mdss_panel_common_pdata *panel_data = filp->private_data;
	if (!panel_data)
		return -ENODEV;

	pcmds = &panel_data->on_cmds;
	if (!pcmds)
		return -EINVAL;

	prev_on_cmds = buf;

	if (user_buf_total) {
		if (user_buf_total_pos + count < MAX_ON_CMD_SIZE) {
			memcpy(&user_buf_total[user_buf_total_pos], user_buf, count);
			user_buf_total_pos += count;
			blen = parse_on_cmds(&buf, user_buf_total,
				user_buf_total_pos);
		} else {
			pr_warn("Too large file size(< 32KB)!\n");
			ret = -EINVAL;
			goto write_error;
		}
	} else {
		blen = parse_on_cmds(&buf, user_buf, count);
	}

	if (blen == 0) {
		if (!user_buf_total) {
			user_buf_total = kzalloc(MAX_ON_CMD_SIZE, GFP_KERNEL);
			if (!user_buf_total) {
				ret = -ENOMEM;
				goto write_error;
			}
			memcpy(user_buf_total, user_buf, count);
			user_buf_total_pos += count;
		}
		return count;
	} else if (blen < 0) {
		pr_err("Invalid on_cmd format or length!\n");
		ret = -EINVAL;
		goto write_error;
	}

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len > sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			return -ENOMEM;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		kfree(buf);
		ret = -ENOMEM;
		goto write_error;
	}

	kfree(pcmds->cmds);

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds) {
		if (buf) {
			kfree(buf);
			buf = prev_on_cmds;
		}
		ret = -ENOMEM;
		goto write_error;
	}

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	if (prev_on_cmds)
		kfree(prev_on_cmds);

	ret = count;
write_error:
	if (user_buf_total) {
		kfree(user_buf_total);
		user_buf_total = NULL;
		user_buf_total_pos = 0;
	}

	return ret;
}

static const struct file_operations on_cmd_fops = {
	.open = simple_open,
	.read = on_cmd_read,
	.write = on_cmd_write,
};

static int debug_fs_init(struct mdss_panel_common_pdata *panel_data)
{
	struct dentry *dsi_base;

	dsi_base = debugfs_create_dir("mdss_dsi", NULL);
	if (!dsi_base)
		return -ENOMEM;

	debugfs_create_file("on_cmd", S_IRUSR | S_IRGRP | S_IWUSR, dsi_base,
		panel_data, &on_cmd_fops);

	return 0;
}
#endif

static int read_local_on_cmds(char *buf, size_t cmd)
{
	int i, len = 0;
	int dlen;

	if (system_rev != GAMMA_COMPAT) {
		pr_err("Incompatible hardware revision: %d\n", system_rev);
		return -EINVAL;
	}

	/* Skip last bit */
	dlen = local_pdata->on_cmds.cmds[cmd].dchdr.dlen - 1;
	if (!dlen)
		return -ENOMEM;

	/* Skip first bit */
	for (i = 1; i < dlen; i++)
		len += sprintf(buf + len, "%d ",
			       local_pdata->on_cmds.cmds[cmd].payload[i]);

	len += sprintf(buf + len, "\n");

	return len;
}

static unsigned int cnt;

static int write_local_on_cmds(struct device *dev, const char *buf,
			       size_t cmd)
{
	int i, rc = 0;
	int dlen;
	unsigned int val;
	char tmp[3];
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_common_pdata *prev_local_data;

	if (cnt) {
		cnt = 0;
		return -EINVAL;
	}

	if (system_rev != GAMMA_COMPAT) {
		pr_err("Incompatible hardware revision: %d\n", system_rev);
		return -EINVAL;
	}

	if (cmds_panel_data == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(cmds_panel_data, struct mdss_dsi_ctrl_pdata,
			    panel_data);

	/*
	 * Last bit is not written because it's either fixed at 0x00 for
	 * RGB or a duplicate of the previous bit for the white point.
	 */
	dlen = local_pdata->on_cmds.cmds[cmd].dchdr.dlen - 1;
	if (!dlen)
		return -EINVAL;

	/* Backup previous panel data */
	prev_local_data = local_pdata;

	/* Skip first bit again */
	for (i = 1; i < dlen; i++) {
		rc = sscanf(buf, "%u", &val);
		if (rc != 1)
			return -EINVAL;

		if (val > 255) {
			pr_err("%s: Invalid input data %u (0-255)\n", __func__, val);
			local_pdata = prev_local_data;
			return -EINVAL;
		}

		local_pdata->on_cmds.cmds[cmd].payload[i] = val;
		/*
		 * Duplicate positive/negative polarities for both,
		 * white point and RGB values.
		 */
		if (cmd == 5)
			local_pdata->on_cmds.cmds[cmd].payload[i + 1] = val;
		else
			local_pdata->on_cmds.cmds[cmd + 2].payload[i] = val;

		sscanf(buf, "%s", tmp);
		buf += strlen(tmp) + 1;
		cnt = strlen(tmp);
	}

	pr_info("%s\n", __func__);

	return rc;
}

static void send_local_on_cmds(struct work_struct *work)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (cmds_panel_data == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl = container_of(cmds_panel_data, struct mdss_dsi_ctrl_pdata,
			    panel_data);

	if (local_pdata->on_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &local_pdata->on_cmds);

	pr_info("%s\n", __func__);
}

/************************** sysfs interface ************************/

static ssize_t write_kgamma_send(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	if (!cmds_panel_data->panel_info.panel_power_on) {
		pr_err("%s: Panel off, failed to send commands\n", __func__);
		return -EPERM;
	}

	schedule_work(&send_cmds_work);

	return count;
}

static DEVICE_ATTR(kgamma_send, 0644, NULL, write_kgamma_send);

#define read_one(file_name, cmd)				\
static ssize_t read_##file_name					\
(struct device *dev, struct device_attribute *attr, char *buf)  \
{								\
	return read_local_on_cmds(buf, cmd);			\
}

read_one(kgamma_w,  5);
read_one(kgamma_r,  7);
read_one(kgamma_g, 11);
read_one(kgamma_b, 15);

#define write_one(file_name, cmd)				\
static ssize_t write_##file_name				\
(struct device *dev, struct device_attribute *attr, 		\
		const char *buf, size_t count)  		\
{								\
	return write_local_on_cmds(dev, buf, cmd);		\
}

write_one(kgamma_w,  5);
write_one(kgamma_r,  7);
write_one(kgamma_g, 11);
write_one(kgamma_b, 15);

#define define_one_rw(_name)					\
static DEVICE_ATTR(_name, 0644, read_##_name, write_##_name);

define_one_rw(kgamma_w);
define_one_rw(kgamma_r);
define_one_rw(kgamma_g);
define_one_rw(kgamma_b);

static struct attribute *dsi_panel_attributes[] = {
	&dev_attr_kgamma_w.attr,
	&dev_attr_kgamma_r.attr,
	&dev_attr_kgamma_g.attr,
	&dev_attr_kgamma_b.attr,
	&dev_attr_kgamma_send.attr,
	NULL
};

static struct attribute_group dsi_panel_attribute_group = {
	.attrs = dsi_panel_attributes,
};

/**************************** sysfs end **************************/

static int __devinit mdss_dsi_panel_probe(struct platform_device *pdev)
{
	int rc = 0;
	static struct mdss_panel_common_pdata vendor_pdata;
	static const char *panel_name;
	const char *driver_name = this_driver.driver.name;
	bool partial_update_enabled;

	pr_debug("%s:%d, debug info id=%d", __func__, __LINE__, pdev->id);
	if (!pdev->dev.of_node)
		return -ENODEV;

	panel_name = of_get_property(pdev->dev.of_node, "label", NULL);
	if (!panel_name)
		pr_info("%s:%d, panel name not specified\n",
						__func__, __LINE__);
	else
		pr_info("%s: Panel Name = %s\n", __func__, panel_name);

	rc = mdss_panel_parse_dt(pdev, &vendor_pdata);
	if (rc)
		return rc;

	vendor_pdata.on = mdss_dsi_panel_on;
	vendor_pdata.off = mdss_dsi_panel_off;
	vendor_pdata.bl_fnc = mdss_dsi_panel_bl_ctrl;

	partial_update_enabled = of_property_read_bool(pdev->dev.of_node,
						"qcom,partial-update-enabled");
	if (partial_update_enabled) {
		pr_info("%s:%d Partial update enabled.\n", __func__, __LINE__);
		vendor_pdata.panel_info.partial_update_enabled = 1;
		vendor_pdata.partial_update_fnc = mdss_dsi_panel_partial_update;
	} else {
		pr_info("%s:%d Partial update disabled.\n", __func__, __LINE__);
		vendor_pdata.panel_info.partial_update_enabled = 0;
		vendor_pdata.partial_update_fnc = NULL;
	}

	rc = dsi_panel_device_register(pdev, &vendor_pdata);
	if (rc)
		return rc;

	INIT_WORK(&send_cmds_work, send_local_on_cmds);

	local_pdata = &vendor_pdata;
	if (!local_pdata)
		return -EINVAL;

#ifdef CONFIG_DEBUG_FS
	debug_fs_init(&vendor_pdata);
#endif

	module_kobj = kobject_create_and_add(driver_name, &module_kset->kobj);
	if (!module_kobj) {
		pr_err("%s: kobject create failed\n", driver_name);
		return -ENOMEM;
	}

	rc = sysfs_create_group(module_kobj, &dsi_panel_attribute_group);
	if (rc)
		pr_err("%s: sysfs create failed: %d\n", panel_name, rc);

	return rc;
}

static const struct of_device_id mdss_dsi_panel_match[] = {
	{.compatible = "qcom,mdss-dsi-panel"},
	{}
};

static struct platform_driver this_driver = {
	.probe  = mdss_dsi_panel_probe,
	.driver = {
		.name   = "dsi_panel",
		.of_match_table = mdss_dsi_panel_match,
	},
};

static int __init mdss_dsi_panel_init(void)
{
	return platform_driver_register(&this_driver);
}
module_init(mdss_dsi_panel_init);
