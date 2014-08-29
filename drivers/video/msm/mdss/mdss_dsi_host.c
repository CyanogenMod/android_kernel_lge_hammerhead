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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/iopoll.h>
#include <linux/ratelimit.h>

#include <mach/iommu_domains.h>

#include "mdss.h"
#include "mdss_dsi.h"

#define VSYNC_PERIOD 17

static struct mdss_dsi_ctrl_pdata *left_ctrl_pdata;

static struct mdss_dsi_ctrl_pdata *ctrl_list[DSI_CTRL_MAX];


struct mdss_hw mdss_dsi0_hw = {
	.hw_ndx = MDSS_HW_DSI0,
	.ptr = NULL,
	.irq_handler = mdss_dsi_isr,
};

struct mdss_hw mdss_dsi1_hw = {
	.hw_ndx = MDSS_HW_DSI1,
	.ptr = NULL,
	.irq_handler = mdss_dsi_isr,
};

void mdss_dsi_ctrl_init(struct mdss_dsi_ctrl_pdata *ctrl)
{
	if (ctrl->shared_pdata.broadcast_enable)
		if (ctrl->panel_data.panel_info.pdest
					== DISPLAY_1) {
			pr_debug("%s: Broadcast mode enabled.\n",
				 __func__);
			left_ctrl_pdata = ctrl;
		}

	if (ctrl->panel_data.panel_info.pdest == DISPLAY_1) {
		mdss_dsi0_hw.ptr = (void *)(ctrl);
		ctrl->dsi_hw = &mdss_dsi0_hw;
		ctrl->ndx = DSI_CTRL_0;
	} else {
		mdss_dsi1_hw.ptr = (void *)(ctrl);
		ctrl->dsi_hw = &mdss_dsi1_hw;
		ctrl->ndx = DSI_CTRL_1;
	}

	ctrl_list[ctrl->ndx] = ctrl;	/* keep it */

	if (mdss_register_irq(ctrl->dsi_hw))
		pr_err("%s: mdss_register_irq failed.\n", __func__);

	pr_debug("%s: ndx=%d base=%p\n", __func__, ctrl->ndx, ctrl->ctrl_base);

	init_completion(&ctrl->dma_comp);
	init_completion(&ctrl->mdp_comp);
	init_completion(&ctrl->video_comp);
	spin_lock_init(&ctrl->irq_lock);
	spin_lock_init(&ctrl->mdp_lock);
	mutex_init(&ctrl->mutex);
	mutex_init(&ctrl->cmd_mutex);
	mdss_dsi_buf_alloc(&ctrl->tx_buf, SZ_4K);
	mdss_dsi_buf_alloc(&ctrl->rx_buf, SZ_4K);
}

void mdss_dsi_clk_req(struct mdss_dsi_ctrl_pdata *ctrl, int enable)
{
	if (enable == 0) {
		/* need wait before disable */
		mutex_lock(&ctrl->cmd_mutex);
		mdss_dsi_cmd_mdp_busy(ctrl);
		mutex_unlock(&ctrl->cmd_mutex);
	}

	mdss_dsi_clk_ctrl(ctrl, enable);
}

void mdss_dsi_enable_irq(struct mdss_dsi_ctrl_pdata *ctrl, u32 term)
{
	unsigned long flags;

	spin_lock_irqsave(&ctrl->irq_lock, flags);
	if (ctrl->dsi_irq_mask & term) {
		spin_unlock_irqrestore(&ctrl->irq_lock, flags);
		return;
	}
	if (ctrl->dsi_irq_mask == 0) {
		mdss_enable_irq(ctrl->dsi_hw);
		pr_debug("%s: IRQ Enable, ndx=%d mask=%x term=%x\n", __func__,
			ctrl->ndx, (int)ctrl->dsi_irq_mask, (int)term);
	}
	ctrl->dsi_irq_mask |= term;
	spin_unlock_irqrestore(&ctrl->irq_lock, flags);
}

void mdss_dsi_disable_irq(struct mdss_dsi_ctrl_pdata *ctrl, u32 term)
{
	unsigned long flags;

	spin_lock_irqsave(&ctrl->irq_lock, flags);
	if (!(ctrl->dsi_irq_mask & term)) {
		spin_unlock_irqrestore(&ctrl->irq_lock, flags);
		return;
	}
	ctrl->dsi_irq_mask &= ~term;
	if (ctrl->dsi_irq_mask == 0) {
		mdss_disable_irq(ctrl->dsi_hw);
		pr_debug("%s: IRQ Disable, ndx=%d mask=%x term=%x\n", __func__,
			ctrl->ndx, (int)ctrl->dsi_irq_mask, (int)term);
	}
	spin_unlock_irqrestore(&ctrl->irq_lock, flags);
}

/*
 * mdss_dsi_disale_irq_nosync() should be called
 * from interrupt context
 */
void mdss_dsi_disable_irq_nosync(struct mdss_dsi_ctrl_pdata *ctrl, u32 term)
{
	spin_lock(&ctrl->irq_lock);
	if (!(ctrl->dsi_irq_mask & term)) {
		spin_unlock(&ctrl->irq_lock);
		return;
	}
	ctrl->dsi_irq_mask &= ~term;
	if (ctrl->dsi_irq_mask == 0) {
		mdss_disable_irq_nosync(ctrl->dsi_hw);
		pr_debug("%s: IRQ Disable, ndx=%d mask=%x term=%x\n", __func__,
			ctrl->ndx, (int)ctrl->dsi_irq_mask, (int)term);
	}
	spin_unlock(&ctrl->irq_lock);
}

/*
 * mipi dsi buf mechanism
 */
char *mdss_dsi_buf_reserve(struct dsi_buf *dp, int len)
{
	dp->data += len;
	return dp->data;
}

char *mdss_dsi_buf_unreserve(struct dsi_buf *dp, int len)
{
	dp->data -= len;
	return dp->data;
}

char *mdss_dsi_buf_push(struct dsi_buf *dp, int len)
{
	dp->data -= len;
	dp->len += len;
	return dp->data;
}

char *mdss_dsi_buf_reserve_hdr(struct dsi_buf *dp, int hlen)
{
	dp->hdr = (u32 *)dp->data;
	return mdss_dsi_buf_reserve(dp, hlen);
}

char *mdss_dsi_buf_init(struct dsi_buf *dp)
{
	int off;

	dp->data = dp->start;
	off = (int)dp->data;
	/* 8 byte align */
	off &= 0x07;
	if (off)
		off = 8 - off;
	dp->data += off;
	dp->len = 0;
	return dp->data;
}

int mdss_dsi_buf_alloc(struct dsi_buf *dp, int size)
{

	dp->start = dma_alloc_writecombine(NULL, size, &dp->dmap, GFP_KERNEL);
	if (dp->start == NULL) {
		pr_err("%s:%u\n", __func__, __LINE__);
		return -ENOMEM;
	}

	dp->end = dp->start + size;
	dp->size = size;

	if ((int)dp->start & 0x07)
		pr_err("%s: buf NOT 8 bytes aligned\n", __func__);

	dp->data = dp->start;
	dp->len = 0;
	return size;
}

/*
 * mipi dsi generic long write
 */
static int mdss_dsi_generic_lwrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	char *bp;
	u32 *hp;
	int i, len = 0;

	dchdr = &cm->dchdr;
	bp = mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);

	/* fill up payload */
	if (cm->payload) {
		len = dchdr->dlen;
		len += 3;
		len &= ~0x03;	/* multipled by 4 */
		for (i = 0; i < dchdr->dlen; i++)
			*bp++ = cm->payload[i];

		/* append 0xff to the end */
		for (; i < len; i++)
			*bp++ = 0xff;

		dp->len += len;
	}

	/* fill up header */
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(dchdr->dlen);
	*hp |= DSI_HDR_VC(dchdr->vc);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_DTYPE(DTYPE_GEN_LWRITE);
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	len += DSI_HOST_HDR_SIZE;

	return len;
}

/*
 * mipi dsi generic short write with 0, 1 2 parameters
 */
static int mdss_dsi_generic_swrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	u32 *hp;
	int len;

	dchdr = &cm->dchdr;
	if (dchdr->dlen && cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return 0;
	}

	mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(dchdr->vc);
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;


	len = (dchdr->dlen > 2) ? 2 : dchdr->dlen;

	if (len == 1) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_WRITE1);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(0);
	} else if (len == 2) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_WRITE2);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(cm->payload[1]);
	} else {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_WRITE);
		*hp |= DSI_HDR_DATA1(0);
		*hp |= DSI_HDR_DATA2(0);
	}

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return DSI_HOST_HDR_SIZE; /* 4 bytes */
}

/*
 * mipi dsi gerneric read with 0, 1 2 parameters
 */
static int mdss_dsi_generic_read(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	u32 *hp;
	int len;

	dchdr = &cm->dchdr;
	if (dchdr->dlen && cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return 0;
	}

	mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(dchdr->vc);
	*hp |= DSI_HDR_BTA;
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;

	len = (dchdr->dlen > 2) ? 2 : dchdr->dlen;

	if (len == 1) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_READ1);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(0);
	} else if (len == 2) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_READ2);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(cm->payload[1]);
	} else {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_READ);
		*hp |= DSI_HDR_DATA1(0);
		*hp |= DSI_HDR_DATA2(0);
	}

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return DSI_HOST_HDR_SIZE; /* 4 bytes */
}

/*
 * mipi dsi dcs long write
 */
static int mdss_dsi_dcs_lwrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	char *bp;
	u32 *hp;
	int i, len = 0;

	dchdr = &cm->dchdr;
	bp = mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);

	/*
	 * fill up payload
	 * dcs command byte (first byte) followed by payload
	 */
	if (cm->payload) {
		len = dchdr->dlen;
		len += 3;
		len &= ~0x03;	/* multipled by 4 */
		for (i = 0; i < dchdr->dlen; i++)
			*bp++ = cm->payload[i];

		/* append 0xff to the end */
		for (; i < len; i++)
			*bp++ = 0xff;

		dp->len += len;
	}

	/* fill up header */
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(dchdr->dlen);
	*hp |= DSI_HDR_VC(dchdr->vc);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_LWRITE);
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	len += DSI_HOST_HDR_SIZE;
	return len;
}

/*
 * mipi dsi dcs short write with 0 parameters
 */
static int mdss_dsi_dcs_swrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	u32 *hp;
	int len;

	dchdr = &cm->dchdr;
	if (cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return -EINVAL;
	}

	mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(dchdr->vc);
	if (dchdr->ack)		/* ask ACK trigger msg from peripeheral */
		*hp |= DSI_HDR_BTA;
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;

	len = (dchdr->dlen > 1) ? 1 : dchdr->dlen;

	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_WRITE);
	*hp |= DSI_HDR_DATA1(cm->payload[0]);	/* dcs command byte */
	*hp |= DSI_HDR_DATA2(0);

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return DSI_HOST_HDR_SIZE; /* 4 bytes */
}

/*
 * mipi dsi dcs short write with 1 parameters
 */
static int mdss_dsi_dcs_swrite1(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	u32 *hp;

	dchdr = &cm->dchdr;
	if (dchdr->dlen < 2 || cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return -EINVAL;
	}

	mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(dchdr->vc);
	if (dchdr->ack)		/* ask ACK trigger msg from peripeheral */
		*hp |= DSI_HDR_BTA;
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;

	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_WRITE1);
	*hp |= DSI_HDR_DATA1(cm->payload[0]);	/* dcs comamnd byte */
	*hp |= DSI_HDR_DATA2(cm->payload[1]);	/* parameter */

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return DSI_HOST_HDR_SIZE; /* 4 bytes */
}
/*
 * mipi dsi dcs read with 0 parameters
 */

static int mdss_dsi_dcs_read(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	u32 *hp;

	dchdr = &cm->dchdr;
	if (cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return -EINVAL;
	}

	mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(dchdr->vc);
	*hp |= DSI_HDR_BTA;
	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_READ);
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;

	*hp |= DSI_HDR_DATA1(cm->payload[0]);	/* dcs command byte */
	*hp |= DSI_HDR_DATA2(0);

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return DSI_HOST_HDR_SIZE; /* 4 bytes */
}

static int mdss_dsi_cm_on(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	u32 *hp;

	dchdr = &cm->dchdr;
	mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(dchdr->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_CM_ON);
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return DSI_HOST_HDR_SIZE; /* 4 bytes */
}

static int mdss_dsi_cm_off(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	u32 *hp;

	dchdr = &cm->dchdr;
	mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(dchdr->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_CM_OFF);
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return DSI_HOST_HDR_SIZE; /* 4 bytes */
}

static int mdss_dsi_peripheral_on(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	u32 *hp;

	dchdr = &cm->dchdr;
	mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(dchdr->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_PERIPHERAL_ON);
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return DSI_HOST_HDR_SIZE; /* 4 bytes */
}

static int mdss_dsi_peripheral_off(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	u32 *hp;

	dchdr = &cm->dchdr;
	mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(dchdr->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_PERIPHERAL_OFF);
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return DSI_HOST_HDR_SIZE; /* 4 bytes */
}

static int mdss_dsi_set_max_pktsize(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	u32 *hp;

	dchdr = &cm->dchdr;
	if (cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return 0;
	}

	mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(dchdr->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_MAX_PKTSIZE);
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;

	*hp |= DSI_HDR_DATA1(cm->payload[0]);
	*hp |= DSI_HDR_DATA2(cm->payload[1]);

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return DSI_HOST_HDR_SIZE; /* 4 bytes */
}

static int mdss_dsi_null_pkt(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	u32 *hp;

	dchdr = &cm->dchdr;
	mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(dchdr->dlen);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_VC(dchdr->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_NULL_PKT);
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return DSI_HOST_HDR_SIZE; /* 4 bytes */
}

static int mdss_dsi_blank_pkt(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	u32 *hp;

	dchdr = &cm->dchdr;
	mdss_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(dchdr->dlen);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_VC(dchdr->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_BLANK_PKT);
	if (dchdr->last)
		*hp |= DSI_HDR_LAST;

	mdss_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return DSI_HOST_HDR_SIZE; /* 4 bytes */
}

/*
 * prepare cmd buffer to be txed
 */
int mdss_dsi_cmd_dma_add(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	struct dsi_ctrl_hdr *dchdr;
	int len = 0;

	dchdr = &cm->dchdr;

	switch (dchdr->dtype) {
	case DTYPE_GEN_WRITE:
	case DTYPE_GEN_WRITE1:
	case DTYPE_GEN_WRITE2:
		len = mdss_dsi_generic_swrite(dp, cm);
		break;
	case DTYPE_GEN_LWRITE:
		len = mdss_dsi_generic_lwrite(dp, cm);
		break;
	case DTYPE_GEN_READ:
	case DTYPE_GEN_READ1:
	case DTYPE_GEN_READ2:
		len = mdss_dsi_generic_read(dp, cm);
		break;
	case DTYPE_DCS_LWRITE:
		len = mdss_dsi_dcs_lwrite(dp, cm);
		break;
	case DTYPE_DCS_WRITE:
		len = mdss_dsi_dcs_swrite(dp, cm);
		break;
	case DTYPE_DCS_WRITE1:
		len = mdss_dsi_dcs_swrite1(dp, cm);
		break;
	case DTYPE_DCS_READ:
		len = mdss_dsi_dcs_read(dp, cm);
		break;
	case DTYPE_MAX_PKTSIZE:
		len = mdss_dsi_set_max_pktsize(dp, cm);
		break;
	case DTYPE_NULL_PKT:
		len = mdss_dsi_null_pkt(dp, cm);
		break;
	case DTYPE_BLANK_PKT:
		len = mdss_dsi_blank_pkt(dp, cm);
		break;
	case DTYPE_CM_ON:
		len = mdss_dsi_cm_on(dp, cm);
		break;
	case DTYPE_CM_OFF:
		len = mdss_dsi_cm_off(dp, cm);
		break;
	case DTYPE_PERIPHERAL_ON:
		len = mdss_dsi_peripheral_on(dp, cm);
		break;
	case DTYPE_PERIPHERAL_OFF:
		len = mdss_dsi_peripheral_off(dp, cm);
		break;
	default:
		pr_debug("%s: dtype=%x NOT supported\n",
					__func__, dchdr->dtype);
		break;

	}

	return len;
}

/*
 * mdss_dsi_short_read1_resp: 1 parameter
 */
static int mdss_dsi_short_read1_resp(struct dsi_buf *rp)
{
	/* strip out dcs type */
	rp->data++;
	rp->len = 1;
	return rp->len;
}

/*
 * mdss_dsi_short_read2_resp: 2 parameter
 */
static int mdss_dsi_short_read2_resp(struct dsi_buf *rp)
{
	/* strip out dcs type */
	rp->data++;
	rp->len = 2;
	return rp->len;
}

static int mdss_dsi_long_read_resp(struct dsi_buf *rp)
{
	short len;

	len = rp->data[2];
	len <<= 8;
	len |= rp->data[1];
	/* strip out dcs header */
	rp->data += 4;
	rp->len -= 4;
	/* strip out 2 bytes of checksum */
	rp->len -= 2;
	return len;
}

void mdss_dsi_cmd_test_pattern(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	int i;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x015c, 0x201);
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x016c, 0xff0000); /* red */
	i = 0;
	while (i++ < 50) {
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0184, 0x1);
		/* Add sleep to get ~50 fps frame rate*/
		msleep(20);
	}
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x015c, 0x0);
}

void mdss_dsi_host_init(struct mipi_panel_info *pinfo,
				struct mdss_panel_data *pdata)
{
	u32 dsi_ctrl, intr_ctrl;
	u32 data;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo->rgb_swap = DSI_RGB_SWAP_RGB;

	ctrl_pdata->panel_mode = pinfo->mode;

	if (pinfo->mode == DSI_VIDEO_MODE) {
		data = 0;
		if (pinfo->pulse_mode_hsa_he)
			data |= BIT(28);
		if (pinfo->hfp_power_stop)
			data |= BIT(24);
		if (pinfo->hbp_power_stop)
			data |= BIT(20);
		if (pinfo->hsa_power_stop)
			data |= BIT(16);
		if (pinfo->eof_bllp_power_stop)
			data |= BIT(15);
		if (pinfo->bllp_power_stop)
			data |= BIT(12);
		data |= ((pinfo->traffic_mode & 0x03) << 8);
		data |= ((pinfo->dst_format & 0x03) << 4); /* 2 bits */
		data |= (pinfo->vc & 0x03);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0010, data);

		data = 0;
		data |= ((pinfo->rgb_swap & 0x07) << 12);
		if (pinfo->b_sel)
			data |= BIT(8);
		if (pinfo->g_sel)
			data |= BIT(4);
		if (pinfo->r_sel)
			data |= BIT(0);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0020, data);
	} else if (pinfo->mode == DSI_CMD_MODE) {
		data = 0;
		data |= ((pinfo->interleave_max & 0x0f) << 20);
		data |= ((pinfo->rgb_swap & 0x07) << 16);
		if (pinfo->b_sel)
			data |= BIT(12);
		if (pinfo->g_sel)
			data |= BIT(8);
		if (pinfo->r_sel)
			data |= BIT(4);
		data |= (pinfo->dst_format & 0x0f);	/* 4 bits */
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0040, data);

		/* DSI_COMMAND_MODE_MDP_DCS_CMD_CTRL */
		data = pinfo->wr_mem_continue & 0x0ff;
		data <<= 8;
		data |= (pinfo->wr_mem_start & 0x0ff);
		if (pinfo->insert_dcs_cmd)
			data |= BIT(16);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0044, data);
	} else
		pr_err("%s: Unknown DSI mode=%d\n", __func__, pinfo->mode);

	dsi_ctrl = BIT(8) | BIT(2);	/* clock enable & cmd mode */
	intr_ctrl = 0;
	intr_ctrl = (DSI_INTR_CMD_DMA_DONE_MASK | DSI_INTR_CMD_MDP_DONE_MASK);

	if (pinfo->crc_check)
		dsi_ctrl |= BIT(24);
	if (pinfo->ecc_check)
		dsi_ctrl |= BIT(20);
	if (pinfo->data_lane3)
		dsi_ctrl |= BIT(7);
	if (pinfo->data_lane2)
		dsi_ctrl |= BIT(6);
	if (pinfo->data_lane1)
		dsi_ctrl |= BIT(5);
	if (pinfo->data_lane0)
		dsi_ctrl |= BIT(4);

	/* from frame buffer, low power mode */
	/* DSI_COMMAND_MODE_DMA_CTRL */
	if (ctrl_pdata->shared_pdata.broadcast_enable)
		MIPI_OUTP(ctrl_pdata->ctrl_base + 0x3C, 0x94000000);
	else
		MIPI_OUTP(ctrl_pdata->ctrl_base + 0x3C, 0x14000000);

	data = 0;
	if (pinfo->te_sel)
		data |= BIT(31);
	data |= pinfo->mdp_trigger << 4;/* cmd mdp trigger */
	data |= pinfo->dma_trigger;	/* cmd dma trigger */
	data |= (pinfo->stream & 0x01) << 8;
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0084,
				data); /* DSI_TRIG_CTRL */

	/* DSI_LAN_SWAP_CTRL */
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x00b0, pinfo->dlane_swap);

	/* clock out ctrl */
	data = pinfo->t_clk_post & 0x3f;	/* 6 bits */
	data <<= 8;
	data |= pinfo->t_clk_pre & 0x3f;	/*  6 bits */
	/* DSI_CLKOUT_TIMING_CTRL */
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0xc4, data);

	data = 0;
	if (pinfo->rx_eot_ignore)
		data |= BIT(4);
	if (pinfo->tx_eot_append)
		data |= BIT(0);
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x00cc,
				data); /* DSI_EOT_PACKET_CTRL */


	/* allow only ack-err-status  to generate interrupt */
	/* DSI_ERR_INT_MASK0 */
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x010c, 0x13ff3fe0);

	intr_ctrl |= DSI_INTR_ERROR_MASK;
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0110,
				intr_ctrl); /* DSI_INTL_CTRL */

	/* turn esc, byte, dsi, pclk, sclk, hclk on */
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x11c,
					0x23f); /* DSI_CLK_CTRL */

	dsi_ctrl |= BIT(0);	/* enable dsi */
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0004, dsi_ctrl);

	wmb();
}

void mdss_set_tx_power_mode(int mode, struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	u32 data;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	data = MIPI_INP((ctrl_pdata->ctrl_base) + 0x3c);

	if (mode == 0)
		data &= ~BIT(26);
	else
		data |= BIT(26);

	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x3c, data);
}

void mdss_dsi_sw_reset(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	u32 dsi_ctrl;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	dsi_ctrl = MIPI_INP((ctrl_pdata->ctrl_base) + 0x0004);
	dsi_ctrl &= ~0x01;
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0004, dsi_ctrl);
	wmb();

	/* turn esc, byte, dsi, pclk, sclk, hclk on */
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x11c,
					0x23f); /* DSI_CLK_CTRL */
	wmb();

	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x118, 0x01);
	wmb();
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x118, 0x00);
	wmb();
}

void mdss_dsi_controller_cfg(int enable,
			     struct mdss_panel_data *pdata)
{

	u32 dsi_ctrl;
	u32 status;
	u32 sleep_us = 1000;
	u32 timeout_us = 16000;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	/* Check for CMD_MODE_DMA_BUSY */
	if (readl_poll_timeout(((ctrl_pdata->ctrl_base) + 0x0008),
			   status,
			   ((status & 0x02) == 0),
			       sleep_us, timeout_us))
		pr_info("%s: DSI status=%x failed\n", __func__, status);

	/* Check for x_HS_FIFO_EMPTY */
	if (readl_poll_timeout(((ctrl_pdata->ctrl_base) + 0x000c),
			   status,
			   ((status & 0x11111000) == 0x11111000),
			       sleep_us, timeout_us))
		pr_info("%s: FIFO status=%x failed\n", __func__, status);

	/* Check for VIDEO_MODE_ENGINE_BUSY */
	if (readl_poll_timeout(((ctrl_pdata->ctrl_base) + 0x0008),
			   status,
			   ((status & 0x08) == 0),
			       sleep_us, timeout_us)) {
		pr_debug("%s: DSI status=%x\n", __func__, status);
		pr_debug("%s: Doing sw reset\n", __func__);
		mdss_dsi_sw_reset(pdata);
	}

	dsi_ctrl = MIPI_INP((ctrl_pdata->ctrl_base) + 0x0004);
	if (enable)
		dsi_ctrl |= 0x01;
	else
		dsi_ctrl &= ~0x01;

	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0004, dsi_ctrl);
	wmb();
}

void mdss_dsi_op_mode_config(int mode,
			     struct mdss_panel_data *pdata)
{
	u32 dsi_ctrl, intr_ctrl;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (ctrl_pdata->shared_pdata.broadcast_enable)
		if (pdata->panel_info.pdest == DISPLAY_1) {
			pr_debug("%s: Broadcast mode. 1st ctrl\n",
				 __func__);
			return;
		}

	dsi_ctrl = MIPI_INP((ctrl_pdata->ctrl_base) + 0x0004);
	/*If Video enabled, Keep Video and Cmd mode ON */
	if (dsi_ctrl & 0x02)
		dsi_ctrl &= ~0x05;
	else
		dsi_ctrl &= ~0x07;

	if (mode == DSI_VIDEO_MODE) {
		dsi_ctrl |= 0x03;
		intr_ctrl = DSI_INTR_CMD_DMA_DONE_MASK;
	} else {		/* command mode */
		dsi_ctrl |= 0x05;
		if (pdata->panel_info.type == MIPI_VIDEO_PANEL)
			dsi_ctrl |= 0x02;

		intr_ctrl = DSI_INTR_CMD_DMA_DONE_MASK | DSI_INTR_ERROR_MASK |
				DSI_INTR_CMD_MDP_DONE_MASK;
	}

	if (ctrl_pdata->shared_pdata.broadcast_enable)
		if ((pdata->panel_info.pdest == DISPLAY_2)
		  && (left_ctrl_pdata != NULL)) {
			MIPI_OUTP(left_ctrl_pdata->ctrl_base + 0x0110,
				  intr_ctrl); /* DSI_INTL_CTRL */
			MIPI_OUTP(left_ctrl_pdata->ctrl_base + 0x0004,
					dsi_ctrl);
		}

	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0110,
				intr_ctrl); /* DSI_INTL_CTRL */
	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x0004, dsi_ctrl);
	wmb();
}

void mdss_dsi_cmd_bta_sw_trigger(struct mdss_panel_data *pdata)
{
	u32 status;
	int timeout_us = 10000;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x098, 0x01);	/* trigger */
	wmb();

	/* Check for CMD_MODE_DMA_BUSY */
	if (readl_poll_timeout(((ctrl_pdata->ctrl_base) + 0x0008),
				status, ((status & 0x0010) == 0),
				0, timeout_us))
		pr_info("%s: DSI status=%x failed\n", __func__, status);

	mdss_dsi_ack_err_status((ctrl_pdata->ctrl_base));

	pr_debug("%s: BTA done, status = %d\n", __func__, status);
}

static char set_tear_on[2] = {0x35, 0x00};
static struct dsi_cmd_desc dsi_tear_on_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(set_tear_on)}, set_tear_on};

static char set_tear_off[2] = {0x34, 0x00};
static struct dsi_cmd_desc dsi_tear_off_cmd = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(set_tear_off)}, set_tear_off};

void mdss_dsi_set_tear_on(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dcs_cmd_req cmdreq;

	cmdreq.cmds = &dsi_tear_on_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

void mdss_dsi_set_tear_off(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dcs_cmd_req cmdreq;

	cmdreq.cmds = &dsi_tear_off_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

int mdss_dsi_cmd_reg_tx(u32 data,
			unsigned char *ctrl_base)
{
	int i;
	char *bp;

	bp = (char *)&data;
	pr_debug("%s: ", __func__);
	for (i = 0; i < 4; i++)
		pr_debug("%x ", *bp++);

	pr_debug("\n");

	MIPI_OUTP(ctrl_base + 0x0084, 0x04);/* sw trigger */
	MIPI_OUTP(ctrl_base + 0x0004, 0x135);

	wmb();

	MIPI_OUTP(ctrl_base + 0x03c, data);
	wmb();
	MIPI_OUTP(ctrl_base + 0x090, 0x01);	/* trigger */
	wmb();

	udelay(300);

	return 4;
}

static int mdss_dsi_wait4video_eng_busy(struct mdss_dsi_ctrl_pdata *ctrl);

static int mdss_dsi_cmd_dma_tx(struct mdss_dsi_ctrl_pdata *ctrl,
					struct dsi_buf *tp);

static int mdss_dsi_cmd_dma_rx(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_buf *rp, int rlen);

static int mdss_dsi_cmds2buf_tx(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_cmd_desc *cmds, int cnt)
{
	struct dsi_buf *tp;
	struct dsi_cmd_desc *cm;
	struct dsi_ctrl_hdr *dchdr;
	int len, wait, tot = 0;

	tp = &ctrl->tx_buf;
	mdss_dsi_buf_init(tp);
	cm = cmds;
	len = 0;
	while (cnt--) {
		dchdr = &cm->dchdr;
		mdss_dsi_buf_reserve(tp, len);
		len = mdss_dsi_cmd_dma_add(tp, cm);
		if (!len) {
			pr_err("%s: failed to call cmd_dma_add\n", __func__);
			return -EINVAL;
		}
		tot += len;
		if (dchdr->last) {
			tp->data = tp->start; /* begin of buf */

			wait = mdss_dsi_wait4video_eng_busy(ctrl);

			mdss_dsi_enable_irq(ctrl, DSI_CMD_TERM);
			len = mdss_dsi_cmd_dma_tx(ctrl, tp);
			if (IS_ERR_VALUE(len)) {
				mdss_dsi_disable_irq(ctrl, DSI_CMD_TERM);
				pr_err("%s: failed to call cmd_dma_tx for cmd = 0x%x\n",
					__func__,  cmds->payload[0]);
				return -EINVAL;
			}

			if (!wait || dchdr->wait > VSYNC_PERIOD)
				usleep(dchdr->wait * 1000);

			mdss_dsi_buf_init(tp);
			len = 0;
		}
		cm++;
	}
	return tot;
}

/*
 * mdss_dsi_cmds_tx:
 * thread context only
 */
int mdss_dsi_cmds_tx(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_cmd_desc *cmds, int cnt)
{
	u32 dsi_ctrl, data;
	int video_mode, ret = 0;
	u32 left_dsi_ctrl = 0;
	bool left_ctrl_restore = false;

	if (ctrl->shared_pdata.broadcast_enable) {
		if (ctrl->ndx == DSI_CTRL_0) {
			pr_debug("%s: Broadcast mode. 1st ctrl\n",
				 __func__);
			return 0;
		}
	}

	if (ctrl->shared_pdata.broadcast_enable) {
		if ((ctrl->ndx == DSI_CTRL_1)
		  && (left_ctrl_pdata != NULL)) {
			left_dsi_ctrl = MIPI_INP(left_ctrl_pdata->ctrl_base
								+ 0x0004);
			video_mode =
				left_dsi_ctrl & 0x02; /* VIDEO_MODE_EN */
			if (video_mode) {
				data = left_dsi_ctrl | 0x04; /* CMD_MODE_EN */
				MIPI_OUTP(left_ctrl_pdata->ctrl_base + 0x0004,
						data);
				left_ctrl_restore = true;
			}
		}
	}

	/* turn on cmd mode
	* for video mode, do not send cmds more than
	* one pixel line, since it only transmit it
	* during BLLP.
	*/
	dsi_ctrl = MIPI_INP((ctrl->ctrl_base) + 0x0004);
	video_mode = dsi_ctrl & 0x02; /* VIDEO_MODE_EN */
	if (video_mode) {
		data = dsi_ctrl | 0x04; /* CMD_MODE_EN */
		MIPI_OUTP((ctrl->ctrl_base) + 0x0004, data);
	}

	ret = mdss_dsi_cmds2buf_tx(ctrl, cmds, cnt);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s: failed to call\n",
			__func__);
		cnt = -EINVAL;
	}

	if (left_ctrl_restore)
		MIPI_OUTP(left_ctrl_pdata->ctrl_base + 0x0004,
					left_dsi_ctrl); /*restore */

	if (video_mode)
		MIPI_OUTP((ctrl->ctrl_base) + 0x0004,
					dsi_ctrl); /* restore */
	return cnt;
}

/* MIPI_DSI_MRPS, Maximum Return Packet Size */
static char max_pktsize[2] = {0x00, 0x00}; /* LSB tx first, 10 bytes */

static struct dsi_cmd_desc pkt_size_cmd = {
	{DTYPE_MAX_PKTSIZE, 1, 0, 0, 0, sizeof(max_pktsize)},
	max_pktsize,
};

/*
 * DSI panel reply with  MAX_RETURN_PACKET_SIZE bytes of data
 * plus DCS header, ECC and CRC for DCS long read response
 * mdss_dsi_controller only have 4x32 bits register ( 16 bytes) to
 * hold data per transaction.
 * MIPI_DSI_LEN equal to 8
 * len should be either 4 or 8
 * any return data more than MIPI_DSI_LEN need to be break down
 * to multiple transactions.
 *
 * ov_mutex need to be acquired before call this function.
 */

int mdss_dsi_cmds_rx(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_cmd_desc *cmds, int rlen, u32 rx_flags)
{
	int cnt, len, diff, pkt_size, ret = 0;
	struct dsi_buf *tp, *rp;
	int no_max_pkt_size;
	char cmd;
	u32 dsi_ctrl, data;
	int video_mode;
	u32 left_dsi_ctrl = 0;
	bool left_ctrl_restore = false;

	if (ctrl->shared_pdata.broadcast_enable) {
		if (ctrl->ndx == DSI_CTRL_0) {
			pr_debug("%s: Broadcast mode. 1st ctrl\n",
				 __func__);
			return 0;
		}
	}

	if (ctrl->shared_pdata.broadcast_enable) {
		if ((ctrl->ndx == DSI_CTRL_1)
		  && (left_ctrl_pdata != NULL)) {
			left_dsi_ctrl = MIPI_INP(left_ctrl_pdata->ctrl_base
								+ 0x0004);
			video_mode = left_dsi_ctrl & 0x02; /* VIDEO_MODE_EN */
			if (video_mode) {
				data = left_dsi_ctrl | 0x04; /* CMD_MODE_EN */
				MIPI_OUTP(left_ctrl_pdata->ctrl_base + 0x0004,
						data);
				left_ctrl_restore = true;
			}
		}
	}

	/* turn on cmd mode
	* for video mode, do not send cmds more than
	* one pixel line, since it only transmit it
	* during BLLP.
	*/
	dsi_ctrl = MIPI_INP((ctrl->ctrl_base) + 0x0004);
	video_mode = dsi_ctrl & 0x02; /* VIDEO_MODE_EN */
	if (video_mode) {
		data = dsi_ctrl | 0x04; /* CMD_MODE_EN */
		MIPI_OUTP((ctrl->ctrl_base) + 0x0004, data);
	}

	no_max_pkt_size = rx_flags & CMD_REQ_NO_MAX_PKT_SIZE;
	if (no_max_pkt_size)
		rlen = ALIGN(rlen, 4); /* Only support rlen = 4*n */

	len = rlen;
	diff = 0;

	if (len <= 2)
		cnt = 4;	/* short read */
	else {
		if (len > MDSS_DSI_LEN)
			len = MDSS_DSI_LEN;	/* 8 bytes at most */

		len = ALIGN(len, 4); /* len 4 bytes align */
		diff = len - rlen;
		/*
		 * add extra 2 bytes to len to have overall
		 * packet size is multipe by 4. This also make
		 * sure 4 bytes dcs headerlocates within a
		 * 32 bits register after shift in.
		 * after all, len should be either 6 or 10.
		 */
		len += 2;
		cnt = len + 6; /* 4 bytes header + 2 bytes crc */
	}

	tp = &ctrl->tx_buf;
	rp = &ctrl->rx_buf;

	if (!no_max_pkt_size) {
		/* packet size need to be set at every read */
		pkt_size = len;
		max_pktsize[0] = pkt_size;
		mdss_dsi_buf_init(tp);
		ret = mdss_dsi_cmd_dma_add(tp, &pkt_size_cmd);
		if (!ret) {
			pr_err("%s: failed to call\n",
				__func__);
			rp->len = 0;
			goto end;
		}

		mdss_dsi_wait4video_eng_busy(ctrl);

		mdss_dsi_enable_irq(ctrl, DSI_CMD_TERM);
		ret = mdss_dsi_cmd_dma_tx(ctrl, tp);
		if (IS_ERR_VALUE(ret)) {
			mdss_dsi_disable_irq(ctrl, DSI_CMD_TERM);
			pr_err("%s: failed to call\n",
				__func__);
			rp->len = 0;
			goto end;
		}
		pr_debug("%s: Max packet size sent\n", __func__);
	}
	mdss_dsi_buf_init(tp);
	ret = mdss_dsi_cmd_dma_add(tp, cmds);
	if (!ret) {
		pr_err("%s: failed to call cmd_dma_add for cmd = 0x%x\n",
			__func__,  cmds->payload[0]);
		rp->len = 0;
		goto end;
	}

	mdss_dsi_wait4video_eng_busy(ctrl);

	mdss_dsi_enable_irq(ctrl, DSI_CMD_TERM);
	/* transmit read comamnd to client */
	ret = mdss_dsi_cmd_dma_tx(ctrl, tp);
	if (IS_ERR_VALUE(ret)) {
		mdss_dsi_disable_irq(ctrl, DSI_CMD_TERM);
		pr_err("%s: failed to call\n",
			__func__);
		rp->len = 0;
		goto end;
	}
	/*
	 * once cmd_dma_done interrupt received,
	 * return data from client is ready and stored
	 * at RDBK_DATA register already
	 */
	mdss_dsi_buf_init(rp);
	if (no_max_pkt_size) {
		/*
		 * expect rlen = n * 4
		 * short alignement for start addr
		 */
		rp->data += 2;
	}

	mdss_dsi_cmd_dma_rx(ctrl, rp, cnt);

	if (no_max_pkt_size) {
		/*
		 * remove extra 2 bytes from previous
		 * rx transaction at shift register
		 * which was inserted during copy
		 * shift registers to rx buffer
		 * rx payload start from long alignment addr
		 */
		rp->data += 2;
	}

	cmd = rp->data[0];
	switch (cmd) {
	case DTYPE_ACK_ERR_RESP:
		pr_debug("%s: rx ACK_ERR_PACLAGE\n", __func__);
		rp->len = 0;
	case DTYPE_GEN_READ1_RESP:
	case DTYPE_DCS_READ1_RESP:
		mdss_dsi_short_read1_resp(rp);
		break;
	case DTYPE_GEN_READ2_RESP:
	case DTYPE_DCS_READ2_RESP:
		mdss_dsi_short_read2_resp(rp);
		break;
	case DTYPE_GEN_LREAD_RESP:
	case DTYPE_DCS_LREAD_RESP:
		mdss_dsi_long_read_resp(rp);
		rp->len -= 2; /* extra 2 bytes added */
		rp->len -= diff; /* align bytes */
		break;
	default:
		pr_warning("%s:Invalid response cmd\n", __func__);
		rp->len = 0;
	}
end:
	if (left_ctrl_restore)
		MIPI_OUTP(left_ctrl_pdata->ctrl_base + 0x0004,
					left_dsi_ctrl); /*restore */
	if (video_mode)
		MIPI_OUTP((ctrl->ctrl_base) + 0x0004,
					dsi_ctrl); /* restore */

	return rp->len;
}

#define DMA_TX_TIMEOUT 200

static int mdss_dsi_cmd_dma_tx(struct mdss_dsi_ctrl_pdata *ctrl,
					struct dsi_buf *tp)
{
	int len, ret = 0;
	int domain = MDSS_IOMMU_DOMAIN_UNSECURE;
	char *bp;
	unsigned long size, addr;

	bp = tp->data;

	len = ALIGN(tp->len, 4);
	size = ALIGN(tp->len, SZ_4K);


	if (is_mdss_iommu_attached()) {
		int ret = msm_iommu_map_contig_buffer(tp->dmap,
					mdss_get_iommu_domain(domain), 0,
					size, SZ_4K, 0, &(addr));
		if (IS_ERR_VALUE(ret)) {
			pr_err("unable to map dma memory to iommu(%d)\n", ret);
			return -ENOMEM;
		}
	} else {
		addr = tp->dmap;
	}

	INIT_COMPLETION(ctrl->dma_comp);

	if (ctrl->shared_pdata.broadcast_enable)
		if ((ctrl->ndx == DSI_CTRL_1)
		  && (left_ctrl_pdata != NULL)) {
			MIPI_OUTP(left_ctrl_pdata->ctrl_base + 0x048, addr);
			MIPI_OUTP(left_ctrl_pdata->ctrl_base + 0x04c, len);
		}

	MIPI_OUTP((ctrl->ctrl_base) + 0x048, addr);
	MIPI_OUTP((ctrl->ctrl_base) + 0x04c, len);
	wmb();

	if (ctrl->shared_pdata.broadcast_enable)
		if ((ctrl->ndx == DSI_CTRL_1)
		  && (left_ctrl_pdata != NULL)) {
			MIPI_OUTP(left_ctrl_pdata->ctrl_base + 0x090, 0x01);
		}

	MIPI_OUTP((ctrl->ctrl_base) + 0x090, 0x01);	/* trigger */
	wmb();

	ret = wait_for_completion_timeout(&ctrl->dma_comp,
				msecs_to_jiffies(DMA_TX_TIMEOUT));
	if (ret == 0)
		ret = -ETIMEDOUT;
	else
		ret = tp->len;

	if (is_mdss_iommu_attached())
		msm_iommu_unmap_contig_buffer(addr,
			mdss_get_iommu_domain(domain), 0, size);

	return ret;
}

static int mdss_dsi_cmd_dma_rx(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_buf *rp, int rlen)

{
	u32 *lp, data;
	int i, off, cnt;

	lp = (u32 *)rp->data;
	cnt = rlen;
	cnt += 3;
	cnt >>= 2;

	if (cnt > 4)
		cnt = 4; /* 4 x 32 bits registers only */

	off = 0x06c;	/* DSI_RDBK_DATA0 */
	off += ((cnt - 1) * 4);

	for (i = 0; i < cnt; i++) {
		data = (u32)MIPI_INP((ctrl->ctrl_base) + off);
		*lp++ = ntohl(data);	/* to network byte order */
		pr_debug("%s: data = 0x%x and ntohl(data) = 0x%x\n",
					 __func__, data, ntohl(data));
		off -= 4;
		rp->len += sizeof(*lp);
	}
	return rlen;
}


void mdss_dsi_wait4video_done(struct mdss_dsi_ctrl_pdata *ctrl)
{
	unsigned long flag;
	u32 data;

	/* DSI_INTL_CTRL */
	data = MIPI_INP((ctrl->ctrl_base) + 0x0110);
	data |= DSI_INTR_VIDEO_DONE_MASK;

	MIPI_OUTP((ctrl->ctrl_base) + 0x0110, data);

	spin_lock_irqsave(&ctrl->mdp_lock, flag);
	INIT_COMPLETION(ctrl->video_comp);
	mdss_dsi_enable_irq(ctrl, DSI_VIDEO_TERM);
	spin_unlock_irqrestore(&ctrl->mdp_lock, flag);

	wait_for_completion_timeout(&ctrl->video_comp,
			msecs_to_jiffies(VSYNC_PERIOD * 4));

	data = MIPI_INP((ctrl->ctrl_base) + 0x0110);
	data &= ~DSI_INTR_VIDEO_DONE_MASK;
	MIPI_OUTP((ctrl->ctrl_base) + 0x0110, data);
}

static int mdss_dsi_wait4video_eng_busy(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int ret = 0;

	if (ctrl->panel_mode == DSI_CMD_MODE)
		return ret;

	if (ctrl->ctrl_state & CTRL_STATE_MDP_ACTIVE) {
		mdss_dsi_wait4video_done(ctrl);
		/* delay 4 ms to skip BLLP */
		usleep(4000);
		ret = 1;
	}

	return ret;
}

void mdss_dsi_cmd_mdp_start(struct mdss_dsi_ctrl_pdata *ctrl)
{
	unsigned long flag;

	spin_lock_irqsave(&ctrl->mdp_lock, flag);
	mdss_dsi_enable_irq(ctrl, DSI_MDP_TERM);
	ctrl->mdp_busy = true;
	INIT_COMPLETION(ctrl->mdp_comp);
	spin_unlock_irqrestore(&ctrl->mdp_lock, flag);
}

void mdss_dsi_cmd_mdp_busy(struct mdss_dsi_ctrl_pdata *ctrl)
{
	unsigned long flags;
	int need_wait = 0;
	uint32_t timeout = 1;

	pr_debug("%s: start pid=%d\n",
				__func__, current->pid);
	spin_lock_irqsave(&ctrl->mdp_lock, flags);
	if (ctrl->mdp_busy == true)
		need_wait++;
	spin_unlock_irqrestore(&ctrl->mdp_lock, flags);

	if (need_wait) {
		/* wait until DMA finishes the current job */
		pr_debug("%s: pending pid=%d\n",
				__func__, current->pid);
		timeout = wait_for_completion_timeout(&ctrl->mdp_comp,
			msecs_to_jiffies(2000));
		if (!timeout)
			pr_warn("%s: timedout\n", __func__);
	}
	pr_debug("%s: done pid=%d\n",
				__func__, current->pid);
}

void mdss_dsi_cmdlist_tx(struct mdss_dsi_ctrl_pdata *ctrl,
				struct dcs_cmd_req *req)
{
	int ret;

	ret = mdss_dsi_cmds_tx(ctrl, req->cmds, req->cmds_cnt);

	if (req->cb)
		req->cb(ret);

}

void mdss_dsi_cmdlist_rx(struct mdss_dsi_ctrl_pdata *ctrl,
				struct dcs_cmd_req *req)
{
	int len;
	u32 data, *dp;
	struct dsi_buf *rp;

	len = mdss_dsi_cmds_rx(ctrl, req->cmds, req->rlen, req->flags);
	rp = &ctrl->rx_buf;
	dp = (u32 *)rp->data;
	data = *dp;

	if (req->cb)
		req->cb(data);
}

void mdss_dsi_cmdlist_commit(struct mdss_dsi_ctrl_pdata *ctrl, int from_mdp)
{
	struct dcs_cmd_req *req;

	mutex_lock(&ctrl->cmd_mutex);
	req = mdss_dsi_cmdlist_get(ctrl);

	/* make sure dsi_cmd_mdp is idle */
	mdss_dsi_cmd_mdp_busy(ctrl);

	pr_debug("%s:  from_mdp=%d pid=%d\n", __func__, from_mdp, current->pid);

	if (req == NULL)
		goto need_lock;

	/*
	 * mdss interrupt is generated in mdp core clock domain
	 * mdp clock need to be enabled to receive dsi interrupt
	 * also, axi bus bandwidth need since dsi controller will
	 * fetch dcs commands from axi bus
	 */
	mdss_bus_bandwidth_ctrl(1);

	pr_debug("%s:  from_mdp=%d pid=%d\n", __func__, from_mdp, current->pid);
	mdss_dsi_clk_ctrl(ctrl, 1);

	if (req->flags & CMD_REQ_RX)
		mdss_dsi_cmdlist_rx(ctrl, req);
	else
		mdss_dsi_cmdlist_tx(ctrl, req);

	mdss_dsi_clk_ctrl(ctrl, 0);
	mdss_bus_bandwidth_ctrl(0);

need_lock:

	if (from_mdp) /* from pipe_commit */
		mdss_dsi_cmd_mdp_start(ctrl);

	mutex_unlock(&ctrl->cmd_mutex);
}

/*
 * mdss_dsi_cmd_get: ctrl->cmd_mutex acquired by caller
 */
struct dcs_cmd_req *mdss_dsi_cmdlist_get(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dcs_cmd_list *clist;
	struct dcs_cmd_req *req = NULL;

	clist = &ctrl->cmdlist;
	if (clist->get != clist->put) {
		req = &clist->list[clist->get];
		clist->get++;
		clist->get %= CMD_REQ_MAX;
		clist->tot--;
		pr_debug("%s: tot=%d put=%d get=%d\n", __func__,
		clist->tot, clist->put, clist->get);
	}
	return req;
}

int mdss_dsi_cmdlist_put(struct mdss_dsi_ctrl_pdata *ctrl,
				struct dcs_cmd_req *cmdreq)
{
	struct dcs_cmd_req *req;
	struct dcs_cmd_list *clist;
	int ret = 0;

	mutex_lock(&ctrl->cmd_mutex);
	clist = &ctrl->cmdlist;
	req = &clist->list[clist->put];
	*req = *cmdreq;
	clist->put++;
	clist->put %= CMD_REQ_MAX;
	clist->tot++;
	if (clist->put == clist->get) {
		/* drop the oldest one */
		pr_debug("%s: DROP, tot=%d put=%d get=%d\n", __func__,
			clist->tot, clist->put, clist->get);
		clist->get++;
		clist->get %= CMD_REQ_MAX;
		clist->tot--;
	}
	mutex_unlock(&ctrl->cmd_mutex);

	ret++;
	pr_debug("%s: tot=%d put=%d get=%d\n", __func__,
		clist->tot, clist->put, clist->get);

	if (req->flags & CMD_REQ_COMMIT)
		mdss_dsi_cmdlist_commit(ctrl, 0);

	return ret;
}

void mdss_dsi_ack_err_status(unsigned char *base)
{
	u32 status;

	status = MIPI_INP(base + 0x0068);/* DSI_ACK_ERR_STATUS */

	if (status) {
		MIPI_OUTP(base + 0x0068, status);
		pr_err("%s: status=%x\n", __func__, status);
	}
}

void mdss_dsi_timeout_status(unsigned char *base)
{
	u32 status;

	status = MIPI_INP(base + 0x00c0);/* DSI_TIMEOUT_STATUS */
	if (status & 0x0111) {
		MIPI_OUTP(base + 0x00c0, status);
		pr_err("%s: status=%x\n", __func__, status);
	}
}

void mdss_dsi_dln0_phy_err(unsigned char *base)
{
	u32 status;

	status = MIPI_INP(base + 0x00b4);/* DSI_DLN0_PHY_ERR */

	if (status & 0x011111) {
		MIPI_OUTP(base + 0x00b4, status);
		pr_err("%s: status=%x\n", __func__, status);
	}
}

void mdss_dsi_fifo_status(unsigned char *base)
{
	u32 status;

	status = MIPI_INP(base + 0x000c);/* DSI_FIFO_STATUS */

	if (status & 0x44444489) {
		MIPI_OUTP(base + 0x000c, status);
		pr_err("%s: status=%x\n", __func__, status);
	}
}

void mdss_dsi_status(unsigned char *base)
{
	u32 status;

	status = MIPI_INP(base + 0x0008);/* DSI_STATUS */

	if (status & 0x80000000) {
		MIPI_OUTP(base + 0x0008, status);
		pr_err("%s: status=%x\n", __func__, status);
	}
}

void mdss_dsi_error(struct mdss_dsi_ctrl_pdata *ctrl)
{
	unsigned char *base;

	base = ctrl->ctrl_base;

	/* DSI_ERR_INT_MASK0 */
	mdss_dsi_ack_err_status(base);	/* mask0, 0x01f */
	mdss_dsi_timeout_status(base);	/* mask0, 0x0e0 */
	mdss_dsi_fifo_status(base);		/* mask0, 0x133d00 */
	mdss_dsi_status(base);		/* mask0, 0xc0100 */
	mdss_dsi_dln0_phy_err(base);	/* mask0, 0x3e00000 */
}


irqreturn_t mdss_dsi_isr(int irq, void *ptr)
{
	u32 isr;
	struct mdss_dsi_ctrl_pdata *ctrl =
			(struct mdss_dsi_ctrl_pdata *)ptr;

	if (!ctrl->ctrl_base)
		pr_err("%s:%d DSI base adr no Initialized",
				       __func__, __LINE__);

	isr = MIPI_INP(ctrl->ctrl_base + 0x0110);/* DSI_INTR_CTRL */
	MIPI_OUTP(ctrl->ctrl_base + 0x0110, isr);

	if (ctrl->shared_pdata.broadcast_enable)
		if ((ctrl->panel_data.panel_info.pdest == DISPLAY_2)
		    && (left_ctrl_pdata != NULL)) {
			u32 isr0;
			isr0 = MIPI_INP(left_ctrl_pdata->ctrl_base
						+ 0x0110);/* DSI_INTR_CTRL */
			if (isr0 & DSI_INTR_CMD_DMA_DONE)
				MIPI_OUTP(left_ctrl_pdata->ctrl_base + 0x0110,
					DSI_INTR_CMD_DMA_DONE);
		}

	pr_debug("%s: isr=%x", __func__, isr);

	if (isr & DSI_INTR_ERROR) {
		pr_err_ratelimited("%s: isr=%x %x", __func__, isr, (int)DSI_INTR_ERROR);
		spin_lock(&ctrl->mdp_lock);
		ctrl->mdp_busy = false;
		mdss_dsi_disable_irq_nosync(ctrl, DSI_MDP_TERM);
		complete(&ctrl->mdp_comp);
		mdss_dsi_error(ctrl);
		spin_unlock(&ctrl->mdp_lock);
	}

	if (isr & DSI_INTR_VIDEO_DONE) {
		spin_lock(&ctrl->mdp_lock);
		mdss_dsi_disable_irq_nosync(ctrl, DSI_VIDEO_TERM);
		complete(&ctrl->video_comp);
		spin_unlock(&ctrl->mdp_lock);
	}

	if (isr & DSI_INTR_CMD_DMA_DONE) {
		spin_lock(&ctrl->mdp_lock);
		mdss_dsi_disable_irq_nosync(ctrl, DSI_CMD_TERM);
		complete(&ctrl->dma_comp);
		spin_unlock(&ctrl->mdp_lock);
	}

	if (isr & DSI_INTR_CMD_MDP_DONE) {
		spin_lock(&ctrl->mdp_lock);
		ctrl->mdp_busy = false;
		mdss_dsi_disable_irq_nosync(ctrl, DSI_MDP_TERM);
		complete(&ctrl->mdp_comp);
		spin_unlock(&ctrl->mdp_lock);
	}

	return IRQ_HANDLED;
}
