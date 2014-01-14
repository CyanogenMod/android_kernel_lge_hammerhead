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
 *
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/iopoll.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include "mdss_mdp.h"
#include "mdss_panel.h"

/* wait for at least 2 vsyncs for lowest refresh rate (24hz) */
#define VSYNC_TIMEOUT_US 100000

#define MDP_INTR_MASK_INTF_VSYNC(intf_num) \
	(1 << (2 * (intf_num - MDSS_MDP_INTF0) + MDSS_MDP_IRQ_INTF_VSYNC))

/* intf timing settings */
struct intf_timing_params {
	u32 width;
	u32 height;
	u32 xres;
	u32 yres;

	u32 h_back_porch;
	u32 h_front_porch;
	u32 v_back_porch;
	u32 v_front_porch;
	u32 hsync_pulse_width;
	u32 vsync_pulse_width;

	u32 border_clr;
	u32 underflow_clr;
	u32 hsync_skew;
};

struct mdss_mdp_video_ctx {
	u32 intf_num;
	char __iomem *base;
	u32 intf_type;
	u8 ref_cnt;

	u8 timegen_en;
	bool polling_en;
	u32 poll_cnt;
	struct completion vsync_comp;
	int wait_pending;

	atomic_t vsync_ref;
	spinlock_t vsync_lock;
	struct list_head vsync_handlers;
};

static inline void mdp_video_write(struct mdss_mdp_video_ctx *ctx,
				   u32 reg, u32 val)
{
	writel_relaxed(val, ctx->base + reg);
}

static inline u32 mdp_video_read(struct mdss_mdp_video_ctx *ctx,
				   u32 reg)
{
	return readl_relaxed(ctx->base + reg);
}

static inline u32 mdss_mdp_video_line_count(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_video_ctx *ctx;
	u32 line_cnt = 0;
	if (!ctl || !ctl->priv_data)
		goto line_count_exit;
	ctx = ctl->priv_data;
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);
	line_cnt = mdp_video_read(ctx, MDSS_MDP_REG_INTF_LINE_COUNT);
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);
line_count_exit:
	return line_cnt;
}

int mdss_mdp_video_addr_setup(struct mdss_data_type *mdata,
				u32 *offsets,  u32 count)
{
	struct mdss_mdp_video_ctx *head;
	u32 i;

	head = devm_kzalloc(&mdata->pdev->dev,
			sizeof(struct mdss_mdp_video_ctx) * count, GFP_KERNEL);
	if (!head)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		head[i].base = mdata->mdp_base + offsets[i];
		pr_debug("adding Video Intf #%d offset=0x%x virt=%p\n", i,
				offsets[i], head[i].base);
		head[i].ref_cnt = 0;
		head[i].intf_num = i + MDSS_MDP_INTF0;
		INIT_LIST_HEAD(&head[i].vsync_handlers);
	}

	mdata->video_intf = head;
	mdata->nintf = count;
	return 0;
}

static int mdss_mdp_video_timegen_setup(struct mdss_mdp_video_ctx *ctx,
					struct intf_timing_params *p)
{
	u32 hsync_period, vsync_period;
	u32 hsync_start_x, hsync_end_x, display_v_start, display_v_end;
	u32 active_h_start, active_h_end, active_v_start, active_v_end;
	u32 den_polarity, hsync_polarity, vsync_polarity;
	u32 display_hctl, active_hctl, hsync_ctl, polarity_ctl;

	hsync_period = p->hsync_pulse_width + p->h_back_porch +
			p->width + p->h_front_porch;
	vsync_period = p->vsync_pulse_width + p->v_back_porch +
			p->height + p->v_front_porch;

	display_v_start = ((p->vsync_pulse_width + p->v_back_porch) *
			hsync_period) + p->hsync_skew;
	display_v_end = ((vsync_period - p->v_front_porch) * hsync_period) +
			p->hsync_skew - 1;

	if (ctx->intf_type == MDSS_INTF_EDP) {
		display_v_start += p->hsync_pulse_width + p->h_back_porch;
		display_v_end -= p->h_front_porch;
	}

	hsync_start_x = p->h_back_porch + p->hsync_pulse_width;
	hsync_end_x = hsync_period - p->h_front_porch - 1;

	if (p->width != p->xres) {
		active_h_start = hsync_start_x;
		active_h_end = active_h_start + p->xres - 1;
	} else {
		active_h_start = 0;
		active_h_end = 0;
	}

	if (p->height != p->yres) {
		active_v_start = display_v_start;
		active_v_end = active_v_start + (p->yres * hsync_period) - 1;
	} else {
		active_v_start = 0;
		active_v_end = 0;
	}


	if (active_h_end) {
		active_hctl = (active_h_end << 16) | active_h_start;
		active_hctl |= BIT(31);	/* ACTIVE_H_ENABLE */
	} else {
		active_hctl = 0;
	}

	if (active_v_end)
		active_v_start |= BIT(31); /* ACTIVE_V_ENABLE */

	hsync_ctl = (hsync_period << 16) | p->hsync_pulse_width;
	display_hctl = (hsync_end_x << 16) | hsync_start_x;

	den_polarity = 0;
	if (MDSS_INTF_HDMI == ctx->intf_type) {
		hsync_polarity = p->yres >= 720 ? 0 : 1;
		vsync_polarity = p->yres >= 720 ? 0 : 1;
	} else {
		hsync_polarity = 0;
		vsync_polarity = 0;
	}
	polarity_ctl = (den_polarity << 2)   | /*  DEN Polarity  */
		       (vsync_polarity << 1) | /* VSYNC Polarity */
		       (hsync_polarity << 0);  /* HSYNC Polarity */

	mdp_video_write(ctx, MDSS_MDP_REG_INTF_HSYNC_CTL, hsync_ctl);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_VSYNC_PERIOD_F0,
			   vsync_period * hsync_period);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_VSYNC_PULSE_WIDTH_F0,
			   p->vsync_pulse_width * hsync_period);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_DISPLAY_HCTL, display_hctl);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_DISPLAY_V_START_F0,
			   display_v_start);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_DISPLAY_V_END_F0, display_v_end);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_ACTIVE_HCTL, active_hctl);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_ACTIVE_V_START_F0,
			   active_v_start);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_ACTIVE_V_END_F0, active_v_end);

	mdp_video_write(ctx, MDSS_MDP_REG_INTF_BORDER_COLOR, p->border_clr);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_UNDERFLOW_COLOR,
			   p->underflow_clr);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_HSYNC_SKEW, p->hsync_skew);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_POLARITY_CTL, polarity_ctl);
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_FRAME_LINE_COUNT_EN, 0x3);

	return 0;
}


static inline void video_vsync_irq_enable(struct mdss_mdp_ctl *ctl, bool clear)
{
	struct mdss_mdp_video_ctx *ctx = ctl->priv_data;

	if (atomic_inc_return(&ctx->vsync_ref) == 1)
		mdss_mdp_irq_enable(MDSS_MDP_IRQ_INTF_VSYNC, ctl->intf_num);
	else if (clear)
		mdss_mdp_irq_clear(ctl->mdata, MDSS_MDP_IRQ_INTF_VSYNC,
				ctl->intf_num);
}

static inline void video_vsync_irq_disable(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_video_ctx *ctx = ctl->priv_data;

	if (atomic_dec_return(&ctx->vsync_ref) == 0)
		mdss_mdp_irq_disable(MDSS_MDP_IRQ_INTF_VSYNC, ctl->intf_num);
}

static int mdss_mdp_video_add_vsync_handler(struct mdss_mdp_ctl *ctl,
		struct mdss_mdp_vsync_handler *handle)
{
	struct mdss_mdp_video_ctx *ctx;
	unsigned long flags;
	int ret = 0;
	bool irq_en = false;

	if (!handle || !(handle->vsync_handler)) {
		ret = -EINVAL;
		goto exit;
	}

	ctx = (struct mdss_mdp_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx for ctl=%d\n", ctl->num);
		ret = -ENODEV;
		goto exit;
	}

	spin_lock_irqsave(&ctx->vsync_lock, flags);
	if (!handle->enabled) {
		handle->enabled = true;
		list_add(&handle->list, &ctx->vsync_handlers);
		irq_en = true;
	}
	spin_unlock_irqrestore(&ctx->vsync_lock, flags);
	if (irq_en)
		video_vsync_irq_enable(ctl, false);
exit:
	return ret;
}

static int mdss_mdp_video_remove_vsync_handler(struct mdss_mdp_ctl *ctl,
		struct mdss_mdp_vsync_handler *handle)
{
	struct mdss_mdp_video_ctx *ctx;
	unsigned long flags;
	bool irq_dis = false;

	ctx = (struct mdss_mdp_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx for ctl=%d\n", ctl->num);
		return -ENODEV;
	}

	spin_lock_irqsave(&ctx->vsync_lock, flags);
	if (handle->enabled) {
		handle->enabled = false;
		list_del_init(&handle->list);
		irq_dis = true;
	}
	spin_unlock_irqrestore(&ctx->vsync_lock, flags);
	if (irq_dis)
		video_vsync_irq_disable(ctl);
	return 0;
}

static int mdss_mdp_video_stop(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_video_ctx *ctx;
	struct mdss_mdp_vsync_handler *tmp, *handle;
	int rc;

	pr_debug("stop ctl=%d\n", ctl->num);

	ctx = (struct mdss_mdp_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx for ctl=%d\n", ctl->num);
		return -ENODEV;
	}

	if (ctx->timegen_en) {
		rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_BLANK, NULL);
		if (rc == -EBUSY) {
			pr_debug("intf #%d busy don't turn off\n",
				 ctl->intf_num);
			return rc;
		}
		WARN(rc, "intf %d blank error (%d)\n", ctl->intf_num, rc);

		mdp_video_write(ctx, MDSS_MDP_REG_INTF_TIMING_ENGINE_EN, 0);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);
		ctx->timegen_en = false;

		rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_PANEL_OFF, NULL);
		WARN(rc, "intf %d timegen off error (%d)\n", ctl->intf_num, rc);

		mdss_mdp_irq_disable(MDSS_MDP_IRQ_INTF_UNDER_RUN,
			ctl->intf_num);
	}

	list_for_each_entry_safe(handle, tmp, &ctx->vsync_handlers, list)
		mdss_mdp_video_remove_vsync_handler(ctl, handle);

	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_INTF_VSYNC, ctl->intf_num,
				   NULL, NULL);
	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_INTF_UNDER_RUN, ctl->intf_num,
				   NULL, NULL);

	ctx->ref_cnt--;
	ctl->priv_data = NULL;

	return 0;
}

static void mdss_mdp_video_vsync_intr_done(void *arg)
{
	struct mdss_mdp_ctl *ctl = arg;
	struct mdss_mdp_video_ctx *ctx = ctl->priv_data;
	struct mdss_mdp_vsync_handler *tmp;
	ktime_t vsync_time;

	if (!ctx) {
		pr_err("invalid ctx\n");
		return;
	}

	vsync_time = ktime_get();
	ctl->vsync_cnt++;

	pr_debug("intr ctl=%d vsync cnt=%u vsync_time=%d\n",
		 ctl->num, ctl->vsync_cnt, (int)ktime_to_ms(vsync_time));

	ctx->polling_en = false;
	complete_all(&ctx->vsync_comp);
	spin_lock(&ctx->vsync_lock);
	list_for_each_entry(tmp, &ctx->vsync_handlers, list) {
		tmp->vsync_handler(ctl, vsync_time);
	}
	spin_unlock(&ctx->vsync_lock);
}

static int mdss_mdp_video_pollwait(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_video_ctx *ctx = ctl->priv_data;
	u32 mask, status;
	int rc;

	mask = MDP_INTR_MASK_INTF_VSYNC(ctl->intf_num);

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);
	rc = readl_poll_timeout(ctl->mdata->mdp_base + MDSS_MDP_REG_INTR_STATUS,
		status,
		(status & mask) || try_wait_for_completion(&ctx->vsync_comp),
		1000,
		VSYNC_TIMEOUT_US);
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);

	if (rc == 0) {
		pr_debug("vsync poll successful! rc=%d status=0x%x\n",
				rc, status);
		ctx->poll_cnt++;
		if (status) {
			struct mdss_mdp_vsync_handler *tmp;
			unsigned long flags;
			ktime_t vsync_time = ktime_get();

			spin_lock_irqsave(&ctx->vsync_lock, flags);
			list_for_each_entry(tmp, &ctx->vsync_handlers, list)
				tmp->vsync_handler(ctl, vsync_time);
			spin_unlock_irqrestore(&ctx->vsync_lock, flags);
		}
	} else {
		pr_warn("vsync poll timed out! rc=%d status=0x%x mask=0x%x\n",
				rc, status, mask);
	}

	return rc;
}

static int mdss_mdp_video_wait4comp(struct mdss_mdp_ctl *ctl, void *arg)
{
	struct mdss_mdp_video_ctx *ctx;
	int rc;

	ctx = (struct mdss_mdp_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	WARN(!ctx->wait_pending, "waiting without commit! ctl=%d", ctl->num);

	if (ctx->polling_en) {
		rc = mdss_mdp_video_pollwait(ctl);
	} else {
		rc = wait_for_completion_timeout(&ctx->vsync_comp,
				usecs_to_jiffies(VSYNC_TIMEOUT_US));
		if (rc == 0) {
			pr_warn("vsync wait timeout %d, fallback to poll mode\n",
					ctl->num);
			ctx->polling_en++;
			rc = mdss_mdp_video_pollwait(ctl);
		} else {
			rc = 0;
		}

		mdss_mdp_ctl_notify(ctl,
			rc ? MDP_NOTIFY_FRAME_TIMEOUT : MDP_NOTIFY_FRAME_DONE);
	}

	if (ctx->wait_pending) {
		ctx->wait_pending = 0;
		video_vsync_irq_disable(ctl);
	}

	return rc;
}

static void mdss_mdp_video_underrun_intr_done(void *arg)
{
	struct mdss_mdp_ctl *ctl = arg;
	if (unlikely(!ctl))
		return;

	ctl->underrun_cnt++;
	pr_debug("display underrun detected for ctl=%d count=%d\n", ctl->num,
			ctl->underrun_cnt);
}

static int mdss_mdp_video_display(struct mdss_mdp_ctl *ctl, void *arg)
{
	struct mdss_mdp_video_ctx *ctx;
	int rc;

	pr_debug("kickoff ctl=%d\n", ctl->num);

	ctx = (struct mdss_mdp_video_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	if (!ctx->wait_pending) {
		ctx->wait_pending++;
		video_vsync_irq_enable(ctl, true);
		INIT_COMPLETION(ctx->vsync_comp);
	} else {
		WARN(1, "commit without wait! ctl=%d", ctl->num);
	}

	if (!ctx->timegen_en) {
		rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_UNBLANK, NULL);
		if (rc) {
			pr_warn("intf #%d unblank error (%d)\n",
					ctl->intf_num, rc);
			video_vsync_irq_disable(ctl);
			ctx->wait_pending = 0;
			return rc;
		}

		pr_debug("enabling timing gen for intf=%d\n", ctl->intf_num);

		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);

		mdss_mdp_irq_enable(MDSS_MDP_IRQ_INTF_UNDER_RUN, ctl->intf_num);
		mdp_video_write(ctx, MDSS_MDP_REG_INTF_TIMING_ENGINE_EN, 1);
		wmb();

		rc = wait_for_completion_timeout(&ctx->vsync_comp,
				usecs_to_jiffies(VSYNC_TIMEOUT_US));
		WARN(rc == 0, "timeout (%d) enabling timegen on ctl=%d\n",
				rc, ctl->num);

		ctx->timegen_en = true;
		rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_PANEL_ON, NULL);
		WARN(rc, "intf %d panel on error (%d)\n", ctl->intf_num, rc);
	}

	return 0;
}

int mdss_mdp_video_copy_splash_screen(struct mdss_panel_data *pdata)
{
	void *virt = NULL;
	unsigned long bl_fb_addr = 0;
	unsigned long *bl_fb_addr_va;
	unsigned long  pipe_addr, pipe_src_size;
	u32 height, width, rgb_size, bpp;
	size_t size;
	static struct ion_handle *ihdl;
	struct ion_client *iclient = mdss_get_ionclient();
	static ion_phys_addr_t phys;

	pipe_addr = MDSS_MDP_REG_SSPP_OFFSET(3) +
		MDSS_MDP_REG_SSPP_SRC0_ADDR;
	pipe_src_size =
		MDSS_MDP_REG_SSPP_OFFSET(3) + MDSS_MDP_REG_SSPP_SRC_SIZE;

	bpp        = 3;
	rgb_size   = MDSS_MDP_REG_READ(pipe_src_size);
	bl_fb_addr = MDSS_MDP_REG_READ(pipe_addr);

	height = (rgb_size >> 16) & 0xffff;
	width  = rgb_size & 0xffff;
	size = PAGE_ALIGN(height * width * bpp);
	pr_debug("%s:%d splash_height=%d splash_width=%d Buffer size=%d\n",
			__func__, __LINE__, height, width, size);

	ihdl = ion_alloc(iclient, size, SZ_1M,
			ION_HEAP(ION_QSECOM_HEAP_ID), 0);
	if (IS_ERR_OR_NULL(ihdl)) {
		pr_err("unable to alloc fbmem from ion (%p)\n", ihdl);
		return -ENOMEM;
	}

	pdata->panel_info.splash_ihdl = ihdl;

	virt = ion_map_kernel(iclient, ihdl);
	ion_phys(iclient, ihdl, &phys, &size);

	pr_debug("%s %d Allocating %u bytes at 0x%lx (%pa phys)\n",
			__func__, __LINE__, size,
			(unsigned long int)virt, &phys);

	bl_fb_addr_va = (unsigned long *)ioremap(bl_fb_addr, size);

	memcpy(virt, bl_fb_addr_va, size);

	MDSS_MDP_REG_WRITE(pipe_addr, phys);
	MDSS_MDP_REG_WRITE(MDSS_MDP_REG_CTL_FLUSH + MDSS_MDP_REG_CTL_OFFSET(0),
			0x48);

	return 0;
}

int mdss_mdp_video_reconfigure_splash_done(struct mdss_mdp_ctl *ctl)
{
	struct ion_client *iclient = mdss_get_ionclient();
	struct mdss_panel_data *pdata;
	int ret = 0, off;
	int mdss_mdp_rev = MDSS_MDP_REG_READ(MDSS_MDP_REG_HW_VERSION);
	int mdss_v2_intf_off = 0;

	off = 0;

	pdata = ctl->panel_data;

	pdata->panel_info.cont_splash_enabled = 0;

	ret = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_CONT_SPLASH_BEGIN,
				      NULL);
	if (ret) {
		pr_err("%s: Failed to handle 'CONT_SPLASH_BEGIN' event\n",
					__func__);
		return ret;
	}

	mdss_mdp_ctl_write(ctl, 0, MDSS_MDP_LM_BORDER_COLOR);
	off = MDSS_MDP_REG_INTF_OFFSET(ctl->intf_num);

	if (mdss_mdp_rev >= MDSS_MDP_HW_REV_102)
		mdss_v2_intf_off =  0xEC00;

	MDSS_MDP_REG_WRITE(off + MDSS_MDP_REG_INTF_TIMING_ENGINE_EN -
			mdss_v2_intf_off, 0);
	/* wait for 1 VSYNC for the pipe to be unstaged */
	msleep(20);
	ion_free(iclient, pdata->panel_info.splash_ihdl);
	ret = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_CONT_SPLASH_FINISH,
			NULL);
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);
	return ret;
}

int mdss_mdp_video_start(struct mdss_mdp_ctl *ctl)
{
	struct mdss_data_type *mdata;
	struct mdss_panel_info *pinfo;
	struct mdss_mdp_video_ctx *ctx;
	struct mdss_mdp_mixer *mixer;
	struct intf_timing_params itp = {0};
	u32 dst_bpp;
	int i;

	mdata = ctl->mdata;
	pinfo = &ctl->panel_data->panel_info;
	mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_LEFT);

	if (!mixer) {
		pr_err("mixer not setup correctly\n");
		return -ENODEV;
	}

	i = ctl->intf_num - MDSS_MDP_INTF0;
	if (i < mdata->nintf) {
		ctx = ((struct mdss_mdp_video_ctx *) mdata->video_intf) + i;
		if (ctx->ref_cnt) {
			pr_err("Intf %d already in use\n", ctl->intf_num);
			return -EBUSY;
		}
		pr_debug("video Intf #%d base=%p", ctx->intf_num, ctx->base);
		ctx->ref_cnt++;
	} else {
		pr_err("Invalid intf number: %d\n", ctl->intf_num);
		return -EINVAL;
	}

	pr_debug("start ctl=%u\n", ctl->num);

	ctl->priv_data = ctx;
	ctx->intf_type = ctl->intf_type;
	init_completion(&ctx->vsync_comp);
	spin_lock_init(&ctx->vsync_lock);
	atomic_set(&ctx->vsync_ref, 0);

	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_INTF_VSYNC, ctl->intf_num,
				   mdss_mdp_video_vsync_intr_done, ctl);
	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_INTF_UNDER_RUN, ctl->intf_num,
				   mdss_mdp_video_underrun_intr_done, ctl);

	dst_bpp = pinfo->fbc.enabled ? (pinfo->fbc.target_bpp) : (pinfo->bpp);

	itp.width = mult_frac((pinfo->xres + pinfo->lcdc.xres_pad),
				dst_bpp, pinfo->bpp);
	itp.height = pinfo->yres + pinfo->lcdc.yres_pad;
	itp.border_clr = pinfo->lcdc.border_clr;
	itp.underflow_clr = pinfo->lcdc.underflow_clr;
	itp.hsync_skew = pinfo->lcdc.hsync_skew;

	itp.xres =  mult_frac(pinfo->xres, dst_bpp, pinfo->bpp);
	itp.yres = pinfo->yres;
	itp.h_back_porch =  mult_frac(pinfo->lcdc.h_back_porch, dst_bpp,
			pinfo->bpp);
	itp.h_front_porch = mult_frac(pinfo->lcdc.h_front_porch, dst_bpp,
			pinfo->bpp);
	itp.v_back_porch =  mult_frac(pinfo->lcdc.v_back_porch, dst_bpp,
			pinfo->bpp);
	itp.v_front_porch = mult_frac(pinfo->lcdc.v_front_porch, dst_bpp,
			pinfo->bpp);
	itp.hsync_pulse_width = mult_frac(pinfo->lcdc.h_pulse_width, dst_bpp,
			pinfo->bpp);
	itp.vsync_pulse_width = pinfo->lcdc.v_pulse_width;

	if (mdss_mdp_video_timegen_setup(ctx, &itp)) {
		pr_err("unable to get timing parameters\n");
		return -EINVAL;
	}
	mdp_video_write(ctx, MDSS_MDP_REG_INTF_PANEL_FORMAT, ctl->dst_format);

	ctl->stop_fnc = mdss_mdp_video_stop;
	ctl->display_fnc = mdss_mdp_video_display;
	ctl->wait_fnc = mdss_mdp_video_wait4comp;
	ctl->read_line_cnt_fnc = mdss_mdp_video_line_count;
	ctl->add_vsync_handler = mdss_mdp_video_add_vsync_handler;
	ctl->remove_vsync_handler = mdss_mdp_video_remove_vsync_handler;

	return 0;
}
