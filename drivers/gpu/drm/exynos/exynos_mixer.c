/*
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Authors:
 * Seung-Woo Kim <sw0312.kim@samsung.com>
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * Based on drivers/media/video/s5p-tv/mixer_reg.c
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <drm/drmP.h>

#include "regs-mixer.h"
#include "regs-vp.h"

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/component.h>

#include <drm/exynos_drm.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_crtc.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_plane.h"
#include "exynos_drm_iommu.h"

#define MIXER_WIN_NR		3
#define VP_DEFAULT_WIN		2

/* The pixelformats that are natively supported by the mixer. */
#define MXR_FORMAT_RGB565	4
#define MXR_FORMAT_ARGB1555	5
#define MXR_FORMAT_ARGB4444	6
#define MXR_FORMAT_ARGB8888	7

struct mixer_resources {
	int			irq;
	void __iomem		*mixer_regs;
	void __iomem		*vp_regs;
	spinlock_t		reg_slock;
	struct clk		*mixer;
	struct clk		*vp;
	struct clk		*hdmi;
	struct clk		*sclk_mixer;
	struct clk		*sclk_hdmi;
	struct clk		*mout_mixer;
};

enum mixer_version_id {
	MXR_VER_0_0_0_16,
	MXR_VER_16_0_33_0,
	MXR_VER_128_0_0_184,
};

enum mixer_flag_bits {
	MXR_BIT_POWERED,
	MXR_BIT_VSYNC,
	MXR_BIT_INTERLACE,
	MXR_BIT_VP_ENABLED,
	MXR_BIT_HAS_SCLK,
};

enum mixer_range_mode {
	MXR_RANGE_AUTOMATIC,
	MXR_RANGE_FULL,
	MXR_RANGE_NARROW,
};

static const uint32_t mixer_formats[] = {
	DRM_FORMAT_XRGB4444,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static const uint32_t vp_formats[] = {
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
};

/*
 * Mixer context structure.
 *
 * @crtc: The HDMI CRTC attached to the mixer.
 * @planes: Array of plane objects for each of the mixer windows.
 * @active_windows: Cache of the mixer's hardware state.
 *       Tracks which mixer windows are active/inactive.
 * @pipe: The CRTC index.
 * @flags: Bitfield build from the mixer_flag_bits enumerator.
 * @mixer_resources: A struct containing registers, clocks, etc.
 * @mxr_ver: The hardware revision/version of the mixer.
 */
struct mixer_context {
	struct platform_device *pdev;
	struct device		*dev;
	struct drm_device	*drm_dev;
	struct exynos_drm_crtc	*crtc;
	struct exynos_drm_plane	planes[MIXER_WIN_NR];
	unsigned long		active_windows;
	int			pipe;
	unsigned long		flags;
	enum mixer_range_mode	range_mode;
	struct drm_property	*mixer_range_property;

	struct mixer_resources	mixer_res;
	enum mixer_version_id	mxr_ver;
};

struct mixer_drv_data {
	enum mixer_version_id	version;
	bool					is_vp_enabled;
	bool					has_sclk;
};

static const struct exynos_drm_plane_config plane_configs[MIXER_WIN_NR] = {
	{
		.zpos = 0,
		.type = DRM_PLANE_TYPE_PRIMARY,
		.pixel_formats = mixer_formats,
		.num_pixel_formats = ARRAY_SIZE(mixer_formats),
		.capabilities = EXYNOS_DRM_PLANE_CAP_DOUBLE |
				EXYNOS_DRM_PLANE_CAP_ZPOS,
	}, {
		.zpos = 1,
		.type = DRM_PLANE_TYPE_CURSOR,
		.pixel_formats = mixer_formats,
		.num_pixel_formats = ARRAY_SIZE(mixer_formats),
		.capabilities = EXYNOS_DRM_PLANE_CAP_DOUBLE |
				EXYNOS_DRM_PLANE_CAP_ZPOS,
	}, {
		.zpos = 2,
		.type = DRM_PLANE_TYPE_OVERLAY,
		.pixel_formats = vp_formats,
		.num_pixel_formats = ARRAY_SIZE(vp_formats),
		.capabilities = EXYNOS_DRM_PLANE_CAP_SCALE |
				EXYNOS_DRM_PLANE_CAP_ZPOS |
				EXYNOS_DRM_PLANE_CAP_TILE,
	},
};

static const u8 filter_y_horiz_tap8[] = {
	0,	-1,	-1,	-1,	-1,	-1,	-1,	-1,
	-1,	-1,	-1,	-1,	-1,	0,	0,	0,
	0,	2,	4,	5,	6,	6,	6,	6,
	6,	5,	5,	4,	3,	2,	1,	1,
	0,	-6,	-12,	-16,	-18,	-20,	-21,	-20,
	-20,	-18,	-16,	-13,	-10,	-8,	-5,	-2,
	127,	126,	125,	121,	114,	107,	99,	89,
	79,	68,	57,	46,	35,	25,	16,	8,
};

static const u8 filter_y_vert_tap4[] = {
	0,	-3,	-6,	-8,	-8,	-8,	-8,	-7,
	-6,	-5,	-4,	-3,	-2,	-1,	-1,	0,
	127,	126,	124,	118,	111,	102,	92,	81,
	70,	59,	48,	37,	27,	19,	11,	5,
	0,	5,	11,	19,	27,	37,	48,	59,
	70,	81,	92,	102,	111,	118,	124,	126,
	0,	0,	-1,	-1,	-2,	-3,	-4,	-5,
	-6,	-7,	-8,	-8,	-8,	-8,	-6,	-3,
};

static const u8 filter_cr_horiz_tap4[] = {
	0,	-3,	-6,	-8,	-8,	-8,	-8,	-7,
	-6,	-5,	-4,	-3,	-2,	-1,	-1,	0,
	127,	126,	124,	118,	111,	102,	92,	81,
	70,	59,	48,	37,	27,	19,	11,	5,
};

static inline bool is_alpha_format(unsigned int pixel_format)
{
	switch (pixel_format) {
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_ARGB4444:
		return true;
	default:
		return false;
	}
}

static inline u32 vp_reg_read(struct mixer_resources *res, u32 reg_id)
{
	return readl(res->vp_regs + reg_id);
}

static inline void vp_reg_write(struct mixer_resources *res, u32 reg_id,
				 u32 val)
{
	writel(val, res->vp_regs + reg_id);
}

static inline void vp_reg_writemask(struct mixer_resources *res, u32 reg_id,
				 u32 val, u32 mask)
{
	u32 old = vp_reg_read(res, reg_id);

	val = (val & mask) | (old & ~mask);
	writel(val, res->vp_regs + reg_id);
}

static inline u32 mixer_reg_read(struct mixer_resources *res, u32 reg_id)
{
	return readl(res->mixer_regs + reg_id);
}

static inline void mixer_reg_write(struct mixer_resources *res, u32 reg_id,
				 u32 val)
{
	writel(val, res->mixer_regs + reg_id);
}

static inline void mixer_reg_writemask(struct mixer_resources *res,
				 u32 reg_id, u32 val, u32 mask)
{
	u32 old = mixer_reg_read(res, reg_id);

	val = (val & mask) | (old & ~mask);
	writel(val, res->mixer_regs + reg_id);
}

static void mixer_regs_dump(struct mixer_context *ctx)
{
#define DUMPREG(reg_id) \
do { \
	DRM_DEBUG_KMS(#reg_id " = %08x\n", \
		(u32)readl(ctx->mixer_res.mixer_regs + reg_id)); \
} while (0)

	DUMPREG(MXR_STATUS);
	DUMPREG(MXR_CFG);
	DUMPREG(MXR_INT_EN);
	DUMPREG(MXR_INT_STATUS);

	DUMPREG(MXR_LAYER_CFG);
	DUMPREG(MXR_VIDEO_CFG);

	DUMPREG(MXR_GRAPHIC0_CFG);
	DUMPREG(MXR_GRAPHIC0_BASE);
	DUMPREG(MXR_GRAPHIC0_SPAN);
	DUMPREG(MXR_GRAPHIC0_WH);
	DUMPREG(MXR_GRAPHIC0_SXY);
	DUMPREG(MXR_GRAPHIC0_DXY);

	DUMPREG(MXR_GRAPHIC1_CFG);
	DUMPREG(MXR_GRAPHIC1_BASE);
	DUMPREG(MXR_GRAPHIC1_SPAN);
	DUMPREG(MXR_GRAPHIC1_WH);
	DUMPREG(MXR_GRAPHIC1_SXY);
	DUMPREG(MXR_GRAPHIC1_DXY);
#undef DUMPREG
}

static void vp_regs_dump(struct mixer_context *ctx)
{
#define DUMPREG(reg_id) \
do { \
	DRM_DEBUG_KMS(#reg_id " = %08x\n", \
		(u32) readl(ctx->mixer_res.vp_regs + reg_id)); \
} while (0)

	DUMPREG(VP_ENABLE);
	DUMPREG(VP_SRESET);
	DUMPREG(VP_SHADOW_UPDATE);
	DUMPREG(VP_FIELD_ID);
	DUMPREG(VP_MODE);
	DUMPREG(VP_IMG_SIZE_Y);
	DUMPREG(VP_IMG_SIZE_C);
	DUMPREG(VP_PER_RATE_CTRL);
	DUMPREG(VP_TOP_Y_PTR);
	DUMPREG(VP_BOT_Y_PTR);
	DUMPREG(VP_TOP_C_PTR);
	DUMPREG(VP_BOT_C_PTR);
	DUMPREG(VP_ENDIAN_MODE);
	DUMPREG(VP_SRC_H_POSITION);
	DUMPREG(VP_SRC_V_POSITION);
	DUMPREG(VP_SRC_WIDTH);
	DUMPREG(VP_SRC_HEIGHT);
	DUMPREG(VP_DST_H_POSITION);
	DUMPREG(VP_DST_V_POSITION);
	DUMPREG(VP_DST_WIDTH);
	DUMPREG(VP_DST_HEIGHT);
	DUMPREG(VP_H_RATIO);
	DUMPREG(VP_V_RATIO);

#undef DUMPREG
}

static inline void vp_filter_set(struct mixer_resources *res,
		int reg_id, const u8 *data, unsigned int size)
{
	/* assure 4-byte align */
	BUG_ON(size & 3);
	for (; size; size -= 4, reg_id += 4, data += 4) {
		u32 val = (data[0] << 24) |  (data[1] << 16) |
			(data[2] << 8) | data[3];
		vp_reg_write(res, reg_id, val);
	}
}

static void vp_default_filter(struct mixer_resources *res)
{
	vp_filter_set(res, VP_POLY8_Y0_LL,
		filter_y_horiz_tap8, sizeof(filter_y_horiz_tap8));
	vp_filter_set(res, VP_POLY4_Y0_LL,
		filter_y_vert_tap4, sizeof(filter_y_vert_tap4));
	vp_filter_set(res, VP_POLY4_C0_LL,
		filter_cr_horiz_tap4, sizeof(filter_cr_horiz_tap4));
}

static void mixer_cfg_gfx_blend(struct mixer_context *ctx, unsigned int win,
				bool alpha)
{
	struct mixer_resources *res = &ctx->mixer_res;
	u32 val;

	val  = MXR_GRP_CFG_COLOR_KEY_DISABLE; /* no blank key */
	if (alpha) {
		/* blending based on pixel alpha */
		val |= MXR_GRP_CFG_BLEND_PRE_MUL;
		val |= MXR_GRP_CFG_PIXEL_BLEND_EN;
	}
	mixer_reg_writemask(res, MXR_GRAPHIC_CFG(win),
			    val, MXR_GRP_CFG_MISC_MASK);
}

static void mixer_cfg_vp_blend(struct mixer_context *ctx)
{
	struct mixer_resources *res = &ctx->mixer_res;
	u32 val;

	/*
	 * No blending at the moment since the NV12/NV21 pixelformats don't
	 * have an alpha channel. However the mixer supports a global alpha
	 * value for a layer. Once this functionality is exposed, we can
	 * support blending of the video layer through this.
	 */
	val = 0;
	mixer_reg_write(res, MXR_VIDEO_CFG, val);
}

static void mixer_vsync_set_update(struct mixer_context *ctx, bool enable)
{
	struct mixer_resources *res = &ctx->mixer_res;

	/* block update on vsync */
	mixer_reg_writemask(res, MXR_STATUS, enable ?
			MXR_STATUS_SYNC_ENABLE : 0, MXR_STATUS_SYNC_ENABLE);

	if (test_bit(MXR_BIT_VP_ENABLED, &ctx->flags))
		vp_reg_write(res, VP_SHADOW_UPDATE, enable ?
			VP_SHADOW_UPDATE_ENABLE : 0);
}

static void mixer_cfg_scan(struct mixer_context *ctx, unsigned int height)
{
	struct mixer_resources *res = &ctx->mixer_res;
	u32 val;

	/* choosing between interlace and progressive mode */
	val = test_bit(MXR_BIT_INTERLACE, &ctx->flags) ?
		MXR_CFG_SCAN_INTERLACE : MXR_CFG_SCAN_PROGRESSIVE;

	if (ctx->mxr_ver != MXR_VER_128_0_0_184) {
		/* choosing between proper HD and SD mode */
		if (height <= 480)
			val |= MXR_CFG_SCAN_NTSC | MXR_CFG_SCAN_SD;
		else if (height <= 576)
			val |= MXR_CFG_SCAN_PAL | MXR_CFG_SCAN_SD;
		else if (height <= 720)
			val |= MXR_CFG_SCAN_HD_720 | MXR_CFG_SCAN_HD;
		else if (height <= 1080)
			val |= MXR_CFG_SCAN_HD_1080 | MXR_CFG_SCAN_HD;
		else
			val |= MXR_CFG_SCAN_HD_720 | MXR_CFG_SCAN_HD;
	}

	mixer_reg_writemask(res, MXR_CFG, val, MXR_CFG_SCAN_MASK);
}

static void mixer_cfg_rgb_fmt(struct mixer_context *ctx, unsigned int height)
{
	struct mixer_resources *res = &ctx->mixer_res;
	enum mixer_range_mode rm = ctx->range_mode;
	u32 val;

	switch (height) {
	case 480:
	case 576:
	case 600:
		if (rm == MXR_RANGE_AUTOMATIC || rm == MXR_RANGE_FULL)
			val = MXR_CFG_RGB601_0_255;
		else
			val = MXR_CFG_RGB601_16_235;
		break;
	case 720:
	case 1080:
	default:
		if (rm == MXR_RANGE_AUTOMATIC || rm == MXR_RANGE_NARROW)
			val = MXR_CFG_RGB709_16_235;
		else
			val = MXR_CFG_RGB709_0_255;
		mixer_reg_write(res, MXR_CM_COEFF_Y,
				(1 << 30) | (94 << 20) | (314 << 10) |
				(32 << 0));
		mixer_reg_write(res, MXR_CM_COEFF_CB,
				(972 << 20) | (851 << 10) | (225 << 0));
		mixer_reg_write(res, MXR_CM_COEFF_CR,
				(225 << 20) | (820 << 10) | (1004 << 0));
		break;
	}

	mixer_reg_writemask(res, MXR_CFG, val, MXR_CFG_RGB_FMT_MASK);
}

/**
 * mixer_cfg_layer - apply layer configuration to hardware
 * @ctx: mixer context
 *
 * This configures the MXR_CFG and MXR_LAYER_CFG hardware registers
 * using the 'active_windows' field of the the mixer content, and
 * the pixel format of the framebuffers associated with the enabled
 * windows.
 *
 * Has to be called under mixer lock.
 */
static void mixer_cfg_layer(struct mixer_context *ctx, struct exynos_drm_crtc *crtc)
{
	struct mixer_resources *res = &ctx->mixer_res;
	unsigned int win;

	struct exynos_drm_plane_state *state;
	struct drm_framebuffer *fb;
	unsigned int priority;
	u32 mxr_cfg = 0, mxr_layer_cfg = 0, vp_enable = 0;
	bool enable;

	for (win = 0; win < MIXER_WIN_NR; ++win) {
		state = to_exynos_plane_state(ctx->planes[win].base.state);
		fb = state->fb;

		priority = state->base.normalized_zpos + 1;
		enable = test_bit(win, &ctx->active_windows);

		if (!enable)
			continue;

		BUG_ON(!fb);

		/*
		 * TODO: Don't enable alpha blending for the bottom window.
		 */
		switch (win) {
		case 0:
			mxr_cfg |=  MXR_CFG_GRP0_ENABLE;
			mxr_layer_cfg |= MXR_LAYER_CFG_GRP0_VAL(priority);
			mixer_cfg_gfx_blend(ctx, win, is_alpha_format(fb->pixel_format));
			break;

		case 1:
			mxr_cfg |=  MXR_CFG_GRP1_ENABLE;
			mxr_layer_cfg |= MXR_LAYER_CFG_GRP1_VAL(priority);
			mixer_cfg_gfx_blend(ctx, win, is_alpha_format(fb->pixel_format));
			break;

		case VP_DEFAULT_WIN:
			vp_enable = VP_ENABLE_ON;
			mxr_cfg |=  MXR_CFG_VP_ENABLE;
			mxr_layer_cfg |= MXR_LAYER_CFG_VP_VAL(priority);
			mixer_cfg_vp_blend(ctx);
			break;
		}
	}

	if (test_bit(MXR_BIT_VP_ENABLED, &ctx->flags))
		vp_reg_writemask(res, VP_ENABLE, vp_enable, VP_ENABLE_ON);

	mixer_reg_writemask(res, MXR_CFG, mxr_cfg, MXR_CFG_ENABLE_MASK);
	mixer_reg_writemask(res, MXR_LAYER_CFG, mxr_layer_cfg, MXR_LAYER_CFG_MASK);

	if (crtc && ctx->active_windows) {
		struct drm_display_mode *mode = &crtc->base.state->adjusted_mode;

		mixer_cfg_scan(ctx, mode->vdisplay);
		mixer_cfg_rgb_fmt(ctx, mode->vdisplay);
	}
}

static void mixer_run(struct mixer_context *ctx)
{
	struct mixer_resources *res = &ctx->mixer_res;

	mixer_reg_writemask(res, MXR_STATUS, ~0, MXR_STATUS_REG_RUN);
}

static void mixer_stop(struct mixer_context *ctx)
{
	struct mixer_resources *res = &ctx->mixer_res;
	int timeout = 20;

	mixer_reg_writemask(res, MXR_STATUS, 0, MXR_STATUS_REG_RUN);

	while (!(mixer_reg_read(res, MXR_STATUS) & MXR_STATUS_REG_IDLE) &&
			--timeout)
		usleep_range(10000, 12000);
}

static void vp_video_buffer(struct mixer_context *ctx,
			    struct exynos_drm_plane *plane)
{
	struct exynos_drm_plane_state *state =
				to_exynos_plane_state(plane->base.state);
	struct drm_display_mode *mode = &state->base.crtc->state->adjusted_mode;
	struct mixer_resources *res = &ctx->mixer_res;
	struct drm_framebuffer *fb = state->fb;
	unsigned long flags;
	dma_addr_t luma_addr[2], chroma_addr[2];
	bool tiled_mode = false;
	bool crcb_mode = false;
	u32 val;

	switch (fb->pixel_format) {
	case DRM_FORMAT_NV12:
		crcb_mode = false;
		break;
	case DRM_FORMAT_NV21:
		crcb_mode = true;
		break;
	default:
		DRM_ERROR("pixel format for vp is wrong [%d].\n",
				fb->pixel_format);
		return;
	}

	if (fb->modifier[0] == DRM_FORMAT_MOD_SAMSUNG_64_32_TILE)
		tiled_mode = true;

	luma_addr[0] = exynos_drm_fb_dma_addr(fb, 0);
	chroma_addr[0] = exynos_drm_fb_dma_addr(fb, 1);

	if (mode->flags & DRM_MODE_FLAG_INTERLACE) {
		__set_bit(MXR_BIT_INTERLACE, &ctx->flags);
		if (tiled_mode) {
			luma_addr[1] = luma_addr[0] + 0x40;
			chroma_addr[1] = chroma_addr[0] + 0x40;
		} else {
			luma_addr[1] = luma_addr[0] + fb->pitches[0];
			chroma_addr[1] = chroma_addr[0] + fb->pitches[0];
		}
	} else {
		__clear_bit(MXR_BIT_INTERLACE, &ctx->flags);
		luma_addr[1] = 0;
		chroma_addr[1] = 0;
	}

	spin_lock_irqsave(&res->reg_slock, flags);

	/* interlace or progressive scan mode */
	val = (test_bit(MXR_BIT_INTERLACE, &ctx->flags) ? ~0 : 0);
	vp_reg_writemask(res, VP_MODE, val, VP_MODE_LINE_SKIP);

	/* setup format */
	val = (crcb_mode ? VP_MODE_NV21 : VP_MODE_NV12);
	val |= (tiled_mode ? VP_MODE_MEM_TILED : VP_MODE_MEM_LINEAR);
	vp_reg_writemask(res, VP_MODE, val, VP_MODE_FMT_MASK);

	/* setting size of input image */
	vp_reg_write(res, VP_IMG_SIZE_Y, VP_IMG_HSIZE(fb->pitches[0]) |
		VP_IMG_VSIZE(fb->height));
	/* chroma height has to reduced by 2 to avoid chroma distorions */
	vp_reg_write(res, VP_IMG_SIZE_C, VP_IMG_HSIZE(fb->pitches[0]) |
		VP_IMG_VSIZE(fb->height / 2));

	vp_reg_write(res, VP_SRC_WIDTH, state->src.w);
	vp_reg_write(res, VP_SRC_HEIGHT, state->src.h);
	vp_reg_write(res, VP_SRC_H_POSITION,
			VP_SRC_H_POSITION_VAL(state->src.x));
	vp_reg_write(res, VP_SRC_V_POSITION, state->src.y);

	vp_reg_write(res, VP_DST_WIDTH, state->crtc.w);
	vp_reg_write(res, VP_DST_H_POSITION, state->crtc.x);
	if (test_bit(MXR_BIT_INTERLACE, &ctx->flags)) {
		vp_reg_write(res, VP_DST_HEIGHT, state->crtc.h / 2);
		vp_reg_write(res, VP_DST_V_POSITION, state->crtc.y / 2);
	} else {
		vp_reg_write(res, VP_DST_HEIGHT, state->crtc.h);
		vp_reg_write(res, VP_DST_V_POSITION, state->crtc.y);
	}

	vp_reg_write(res, VP_H_RATIO, state->h_ratio);
	vp_reg_write(res, VP_V_RATIO, state->v_ratio);

	vp_reg_write(res, VP_ENDIAN_MODE, VP_ENDIAN_MODE_LITTLE);

	/* set buffer address to vp */
	vp_reg_write(res, VP_TOP_Y_PTR, luma_addr[0]);
	vp_reg_write(res, VP_BOT_Y_PTR, luma_addr[1]);
	vp_reg_write(res, VP_TOP_C_PTR, chroma_addr[0]);
	vp_reg_write(res, VP_BOT_C_PTR, chroma_addr[1]);

	mixer_run(ctx);

	spin_unlock_irqrestore(&res->reg_slock, flags);

	mixer_regs_dump(ctx);
	vp_regs_dump(ctx);
}

static void mixer_layer_update(struct mixer_context *ctx)
{
	struct mixer_resources *res = &ctx->mixer_res;

	mixer_reg_writemask(res, MXR_CFG, ~0, MXR_CFG_LAYER_UPDATE);
}

static void mixer_graph_buffer(struct mixer_context *ctx,
			       struct exynos_drm_plane *plane)
{
	struct exynos_drm_plane_state *state =
				to_exynos_plane_state(plane->base.state);
	struct drm_display_mode *mode = &state->base.crtc->state->adjusted_mode;
	struct mixer_resources *res = &ctx->mixer_res;
	struct drm_framebuffer *fb = state->fb;
	unsigned long flags;
	unsigned int win = plane->index;
	unsigned int x_ratio = 0, y_ratio = 0;
	unsigned int src_x_offset, src_y_offset, dst_x_offset, dst_y_offset;
	dma_addr_t dma_addr;
	unsigned int fmt;
	u32 val;

	switch (fb->pixel_format) {
	case DRM_FORMAT_XRGB4444:
	case DRM_FORMAT_ARGB4444:
		fmt = MXR_FORMAT_ARGB4444;
		break;

	case DRM_FORMAT_XRGB1555:
	case DRM_FORMAT_ARGB1555:
		fmt = MXR_FORMAT_ARGB1555;
		break;

	case DRM_FORMAT_RGB565:
		fmt = MXR_FORMAT_RGB565;
		break;

	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		fmt = MXR_FORMAT_ARGB8888;
		break;

	default:
		DRM_DEBUG_KMS("pixelformat unsupported by mixer\n");
		return;
	}

	/* ratio is already checked by common plane code */
	x_ratio = state->h_ratio == (1 << 15);
	y_ratio = state->v_ratio == (1 << 15);

	dst_x_offset = state->crtc.x;
	dst_y_offset = state->crtc.y;

	/* converting dma address base and source offset */
	dma_addr = exynos_drm_fb_dma_addr(fb, 0)
		+ (state->src.x * fb->bits_per_pixel >> 3)
		+ (state->src.y * fb->pitches[0]);
	src_x_offset = 0;
	src_y_offset = 0;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		__set_bit(MXR_BIT_INTERLACE, &ctx->flags);
	else
		__clear_bit(MXR_BIT_INTERLACE, &ctx->flags);

	spin_lock_irqsave(&res->reg_slock, flags);

	/* setup format */
	mixer_reg_writemask(res, MXR_GRAPHIC_CFG(win),
		MXR_GRP_CFG_FORMAT_VAL(fmt), MXR_GRP_CFG_FORMAT_MASK);

	/* setup geometry */
	mixer_reg_write(res, MXR_GRAPHIC_SPAN(win),
			fb->pitches[0] / (fb->bits_per_pixel >> 3));

	/* setup display size */
	if (ctx->mxr_ver == MXR_VER_128_0_0_184 &&
		win == DEFAULT_WIN) {
		val  = MXR_MXR_RES_HEIGHT(mode->vdisplay);
		val |= MXR_MXR_RES_WIDTH(mode->hdisplay);
		mixer_reg_write(res, MXR_RESOLUTION, val);
	}

	val  = MXR_GRP_WH_WIDTH(state->src.w);
	val |= MXR_GRP_WH_HEIGHT(state->src.h);
	val |= MXR_GRP_WH_H_SCALE(x_ratio);
	val |= MXR_GRP_WH_V_SCALE(y_ratio);
	mixer_reg_write(res, MXR_GRAPHIC_WH(win), val);

	/* setup offsets in source image */
	val  = MXR_GRP_SXY_SX(src_x_offset);
	val |= MXR_GRP_SXY_SY(src_y_offset);
	mixer_reg_write(res, MXR_GRAPHIC_SXY(win), val);

	/* setup offsets in display image */
	val  = MXR_GRP_DXY_DX(dst_x_offset);
	val |= MXR_GRP_DXY_DY(dst_y_offset);
	mixer_reg_write(res, MXR_GRAPHIC_DXY(win), val);

	/* set buffer address to mixer */
	mixer_reg_write(res, MXR_GRAPHIC_BASE(win), dma_addr);

	/* layer update mandatory for mixer 16.0.33.0 */
	if (ctx->mxr_ver == MXR_VER_16_0_33_0 ||
		ctx->mxr_ver == MXR_VER_128_0_0_184)
		mixer_layer_update(ctx);

	mixer_run(ctx);

	spin_unlock_irqrestore(&res->reg_slock, flags);

	mixer_regs_dump(ctx);
}

static void vp_win_reset(struct mixer_context *ctx)
{
	struct mixer_resources *res = &ctx->mixer_res;
	unsigned int tries = 100;

	vp_reg_write(res, VP_SRESET, VP_SRESET_PROCESSING);
	while (tries--) {
		/* waiting until VP_SRESET_PROCESSING is 0 */
		if (~vp_reg_read(res, VP_SRESET) & VP_SRESET_PROCESSING)
			break;
		mdelay(10);
	}
	WARN(tries == 0, "failed to reset Video Processor\n");
}

static void mixer_win_reset(struct mixer_context *ctx)
{
	struct mixer_resources *res = &ctx->mixer_res;
	unsigned long flags;

	spin_lock_irqsave(&res->reg_slock, flags);

	mixer_reg_writemask(res, MXR_CFG, MXR_CFG_DST_HDMI, MXR_CFG_DST_MASK);

	/* set output in RGB888 mode */
	mixer_reg_writemask(res, MXR_CFG, MXR_CFG_OUT_RGB888, MXR_CFG_OUT_MASK);

	/* 16 beat burst in DMA */
	mixer_reg_writemask(res, MXR_STATUS, MXR_STATUS_16_BURST,
		MXR_STATUS_BURST_MASK);

	/* setting background color */
	mixer_reg_write(res, MXR_BG_COLOR0, 0x008080);
	mixer_reg_write(res, MXR_BG_COLOR1, 0x008080);
	mixer_reg_write(res, MXR_BG_COLOR2, 0x008080);

	if (test_bit(MXR_BIT_VP_ENABLED, &ctx->flags)) {
		/* configuration of Video Processor Registers */
		vp_win_reset(ctx);
		vp_default_filter(res);
	}

	/* disable all layers and reset layer priority to default */
	ctx->active_windows = 0;
	mixer_cfg_layer(ctx, NULL);

	spin_unlock_irqrestore(&res->reg_slock, flags);
}

static irqreturn_t mixer_irq_handler(int irq, void *arg)
{
	struct mixer_context *ctx = arg;
	struct mixer_resources *res = &ctx->mixer_res;
	u32 val, base, shadow;

	spin_lock(&res->reg_slock);

	/* read interrupt status for handling and clearing flags for VSYNC */
	val = mixer_reg_read(res, MXR_INT_STATUS);

	/* handling VSYNC */
	if (val & MXR_INT_STATUS_VSYNC) {
		/* vsync interrupt use different bit for read and clear */
		val |= MXR_INT_CLEAR_VSYNC;
		val &= ~MXR_INT_STATUS_VSYNC;

		/* interlace scan need to check shadow register */
		if (test_bit(MXR_BIT_INTERLACE, &ctx->flags)) {
			base = mixer_reg_read(res, MXR_GRAPHIC_BASE(0));
			shadow = mixer_reg_read(res, MXR_GRAPHIC_BASE_S(0));
			if (base != shadow)
				goto out;

			base = mixer_reg_read(res, MXR_GRAPHIC_BASE(1));
			shadow = mixer_reg_read(res, MXR_GRAPHIC_BASE_S(1));
			if (base != shadow)
				goto out;
		}

		drm_crtc_handle_vblank(&ctx->crtc->base);
	}

out:
	/* clear interrupts */
	mixer_reg_write(res, MXR_INT_STATUS, val);

	spin_unlock(&res->reg_slock);

	return IRQ_HANDLED;
}

static int mixer_resources_init(struct mixer_context *mixer_ctx)
{
	struct device *dev = &mixer_ctx->pdev->dev;
	struct mixer_resources *mixer_res = &mixer_ctx->mixer_res;
	struct resource *res;
	int ret;

	spin_lock_init(&mixer_res->reg_slock);

	mixer_res->mixer = devm_clk_get(dev, "mixer");
	if (IS_ERR(mixer_res->mixer)) {
		dev_err(dev, "failed to get clock 'mixer'\n");
		return -ENODEV;
	}

	mixer_res->hdmi = devm_clk_get(dev, "hdmi");
	if (IS_ERR(mixer_res->hdmi)) {
		dev_err(dev, "failed to get clock 'hdmi'\n");
		return PTR_ERR(mixer_res->hdmi);
	}

	mixer_res->sclk_hdmi = devm_clk_get(dev, "sclk_hdmi");
	if (IS_ERR(mixer_res->sclk_hdmi)) {
		dev_err(dev, "failed to get clock 'sclk_hdmi'\n");
		return -ENODEV;
	}
	res = platform_get_resource(mixer_ctx->pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "get memory resource failed.\n");
		return -ENXIO;
	}

	mixer_res->mixer_regs = devm_ioremap(dev, res->start,
							resource_size(res));
	if (mixer_res->mixer_regs == NULL) {
		dev_err(dev, "register mapping failed.\n");
		return -ENXIO;
	}

	res = platform_get_resource(mixer_ctx->pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(dev, "get interrupt resource failed.\n");
		return -ENXIO;
	}

	ret = devm_request_irq(dev, res->start, mixer_irq_handler,
						0, "drm_mixer", mixer_ctx);
	if (ret) {
		dev_err(dev, "request interrupt failed.\n");
		return ret;
	}
	mixer_res->irq = res->start;

	return 0;
}

static int vp_resources_init(struct mixer_context *mixer_ctx)
{
	struct device *dev = &mixer_ctx->pdev->dev;
	struct mixer_resources *mixer_res = &mixer_ctx->mixer_res;
	struct resource *res;

	mixer_res->vp = devm_clk_get(dev, "vp");
	if (IS_ERR(mixer_res->vp)) {
		dev_err(dev, "failed to get clock 'vp'\n");
		return -ENODEV;
	}

	if (test_bit(MXR_BIT_HAS_SCLK, &mixer_ctx->flags)) {
		mixer_res->sclk_mixer = devm_clk_get(dev, "sclk_mixer");
		if (IS_ERR(mixer_res->sclk_mixer)) {
			dev_err(dev, "failed to get clock 'sclk_mixer'\n");
			return -ENODEV;
		}
		mixer_res->mout_mixer = devm_clk_get(dev, "mout_mixer");
		if (IS_ERR(mixer_res->mout_mixer)) {
			dev_err(dev, "failed to get clock 'mout_mixer'\n");
			return -ENODEV;
		}

		if (mixer_res->sclk_hdmi && mixer_res->mout_mixer)
			clk_set_parent(mixer_res->mout_mixer,
				       mixer_res->sclk_hdmi);
	}

	res = platform_get_resource(mixer_ctx->pdev, IORESOURCE_MEM, 1);
	if (res == NULL) {
		dev_err(dev, "get memory resource failed.\n");
		return -ENXIO;
	}

	mixer_res->vp_regs = devm_ioremap(dev, res->start,
							resource_size(res));
	if (mixer_res->vp_regs == NULL) {
		dev_err(dev, "register mapping failed.\n");
		return -ENXIO;
	}

	return 0;
}

static int mixer_initialize(struct mixer_context *mixer_ctx,
			struct drm_device *drm_dev)
{
	int ret;
	struct exynos_drm_private *priv;
	priv = drm_dev->dev_private;

	mixer_ctx->drm_dev = drm_dev;
	mixer_ctx->pipe = priv->pipe++;

	/* acquire resources: regs, irqs, clocks */
	ret = mixer_resources_init(mixer_ctx);
	if (ret) {
		DRM_ERROR("mixer_resources_init failed ret=%d\n", ret);
		return ret;
	}

	if (test_bit(MXR_BIT_VP_ENABLED, &mixer_ctx->flags)) {
		/* acquire vp resources: regs, irqs, clocks */
		ret = vp_resources_init(mixer_ctx);
		if (ret) {
			DRM_ERROR("vp_resources_init failed ret=%d\n", ret);
			return ret;
		}
	}

	ret = drm_iommu_attach_device(drm_dev, mixer_ctx->dev);
	if (ret)
		priv->pipe--;

	return ret;
}

static void mixer_ctx_remove(struct mixer_context *mixer_ctx)
{
	drm_iommu_detach_device(mixer_ctx->drm_dev, mixer_ctx->dev);
}

static int mixer_enable_vblank(struct exynos_drm_crtc *crtc)
{
	struct mixer_context *mixer_ctx = crtc->ctx;
	struct mixer_resources *res = &mixer_ctx->mixer_res;

	__set_bit(MXR_BIT_VSYNC, &mixer_ctx->flags);
	if (!test_bit(MXR_BIT_POWERED, &mixer_ctx->flags))
		return 0;

	/* enable vsync interrupt */
	mixer_reg_writemask(res, MXR_INT_STATUS, ~0, MXR_INT_CLEAR_VSYNC);
	mixer_reg_writemask(res, MXR_INT_EN, ~0, MXR_INT_EN_VSYNC);

	return 0;
}

static void mixer_disable_vblank(struct exynos_drm_crtc *crtc)
{
	struct mixer_context *mixer_ctx = crtc->ctx;
	struct mixer_resources *res = &mixer_ctx->mixer_res;

	__clear_bit(MXR_BIT_VSYNC, &mixer_ctx->flags);

	if (!test_bit(MXR_BIT_POWERED, &mixer_ctx->flags))
		return;

	/* disable vsync interrupt */
	mixer_reg_writemask(res, MXR_INT_STATUS, ~0, MXR_INT_CLEAR_VSYNC);
	mixer_reg_writemask(res, MXR_INT_EN, 0, MXR_INT_EN_VSYNC);
}

static void mixer_atomic_begin(struct exynos_drm_crtc *crtc)
{
	struct mixer_context *mixer_ctx = crtc->ctx;

	if (!test_bit(MXR_BIT_POWERED, &mixer_ctx->flags))
		return;

	mixer_vsync_set_update(mixer_ctx, false);
}

static void mixer_update_plane(struct exynos_drm_crtc *crtc,
			       struct exynos_drm_plane *plane)
{
	struct mixer_context *mixer_ctx = crtc->ctx;

	DRM_DEBUG_KMS("win: %d\n", plane->index);

	if (!test_bit(MXR_BIT_POWERED, &mixer_ctx->flags))
		return;

	if (plane->index == VP_DEFAULT_WIN)
		vp_video_buffer(mixer_ctx, plane);
	else
		mixer_graph_buffer(mixer_ctx, plane);

	__set_bit(plane->index, &mixer_ctx->active_windows);
}

static void mixer_disable_plane(struct exynos_drm_crtc *crtc,
				struct exynos_drm_plane *plane)
{
	struct mixer_context *mixer_ctx = crtc->ctx;

	DRM_DEBUG_KMS("win: %d\n", plane->index);

	if (!test_bit(MXR_BIT_POWERED, &mixer_ctx->flags))
		return;

	__clear_bit(plane->index, &mixer_ctx->active_windows);
}

static void mixer_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct mixer_context *mixer_ctx = crtc->ctx;
	struct mixer_resources *res = &mixer_ctx->mixer_res;
	unsigned long flags;

	if (!test_bit(MXR_BIT_POWERED, &mixer_ctx->flags))
		return;

	spin_lock_irqsave(&res->reg_slock, flags);
	mixer_cfg_layer(mixer_ctx, crtc);
	spin_unlock_irqrestore(&res->reg_slock, flags);

	mixer_vsync_set_update(mixer_ctx, true);
}

static void mixer_enable(struct exynos_drm_crtc *crtc)
{
	struct mixer_context *ctx = crtc->ctx;
	struct mixer_resources *res = &ctx->mixer_res;

	if (test_bit(MXR_BIT_POWERED, &ctx->flags))
		return;

	pm_runtime_get_sync(ctx->dev);

	exynos_drm_pipe_clk_enable(crtc, true);

	mixer_vsync_set_update(ctx, false);

	mixer_reg_writemask(res, MXR_STATUS, ~0, MXR_STATUS_SOFT_RESET);

	if (test_bit(MXR_BIT_VSYNC, &ctx->flags)) {
		mixer_reg_writemask(res, MXR_INT_STATUS, ~0, MXR_INT_CLEAR_VSYNC);
		mixer_reg_writemask(res, MXR_INT_EN, ~0, MXR_INT_EN_VSYNC);
	}
	mixer_win_reset(ctx);

	mixer_vsync_set_update(ctx, true);

	set_bit(MXR_BIT_POWERED, &ctx->flags);
}

static void mixer_disable(struct exynos_drm_crtc *crtc)
{
	struct mixer_context *ctx = crtc->ctx;
	struct mixer_resources *res = &ctx->mixer_res;
	unsigned long flags;
	int i;

	if (!test_bit(MXR_BIT_POWERED, &ctx->flags))
		return;

	mixer_stop(ctx);
	mixer_regs_dump(ctx);

	for (i = 0; i < MIXER_WIN_NR; i++)
		mixer_disable_plane(crtc, &ctx->planes[i]);

	spin_lock_irqsave(&res->reg_slock, flags);
	mixer_cfg_layer(ctx, NULL);
	spin_unlock_irqrestore(&res->reg_slock, flags);

	exynos_drm_pipe_clk_enable(crtc, false);

	pm_runtime_put(ctx->dev);

	clear_bit(MXR_BIT_POWERED, &ctx->flags);
}

/* Only valid for Mixer version 16.0.33.0 */
static int mixer_atomic_check(struct exynos_drm_crtc *crtc,
		       struct drm_crtc_state *state)
{
	struct drm_display_mode *mode = &state->adjusted_mode;
	u32 w, h;

	w = mode->hdisplay;
	h = mode->vdisplay;

	DRM_DEBUG_KMS("xres=%d, yres=%d, refresh=%d, intl=%d\n",
		mode->hdisplay, mode->vdisplay, mode->vrefresh,
		(mode->flags & DRM_MODE_FLAG_INTERLACE) ? 1 : 0);

	/* Check against resolution ranges. */
	if ((w >= 464 && w <= 720 && h >= 261 && h <= 576) ||
		(w >= 1024 && w <= 1280 && h >= 576 && h <= 720) ||
		(w >= 1664 && w <= 1920 && h >= 936 && h <= 1080))
		return 0;

	/* Check against some specific resolutions. */
	if ((w == 1024 && (h == 768 || h == 600)) ||
		(w == 1366 && h == 768) ||
		(w == 1280 && h == 1024))
		return 0;

	return -EINVAL;
}

static const struct drm_prop_enum_list mixer_range_names[] = {
        { MXR_RANGE_AUTOMATIC, "Automatic" },
        { MXR_RANGE_FULL, "Full" },
        { MXR_RANGE_NARROW, "Narrow (16:235)" },
};

static void mixer_attach_range_property(struct drm_device *dev,
					struct exynos_drm_crtc *crtc)
{
	struct mixer_context *ctx = crtc->ctx;
        struct drm_property *prop;

        prop = ctx->mixer_range_property;
        if (!prop) {
                prop = drm_property_create_enum(dev, DRM_MODE_PROP_ENUM,
						"RGB Range",
						mixer_range_names,
						ARRAY_SIZE(mixer_range_names));
                if (!prop)
                        return;

                ctx->mixer_range_property = prop;
        }

        drm_object_attach_property(&crtc->base.base, prop, 0);
}

static int mixer_atomic_set_property(struct exynos_drm_crtc *crtc,
				     struct drm_crtc_state *state,
				     struct drm_property *property,
				     uint64_t val)
{
	struct mixer_context *ctx = crtc->ctx;

	if (property == ctx->mixer_range_property) {
		switch (val) {
		case MXR_RANGE_AUTOMATIC:
		case MXR_RANGE_FULL:
		case MXR_RANGE_NARROW:
			ctx->range_mode = val;
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

static int mixer_atomic_get_property(struct exynos_drm_crtc *crtc,
				     const struct drm_crtc_state *state,
				     struct drm_property *property,
				     uint64_t *val)
{
	struct mixer_context *ctx = crtc->ctx;

	if (property == ctx->mixer_range_property) {
		*val = ctx->range_mode;
		return 0;
        }

        return -EINVAL;
}

static const struct exynos_drm_crtc_ops mixer_crtc_ops = {
	.enable			= mixer_enable,
	.disable		= mixer_disable,
	.enable_vblank		= mixer_enable_vblank,
	.disable_vblank		= mixer_disable_vblank,
	.atomic_begin		= mixer_atomic_begin,
	.update_plane		= mixer_update_plane,
	.disable_plane		= mixer_disable_plane,
	.atomic_flush		= mixer_atomic_flush,
	.atomic_check		= mixer_atomic_check,
	.atomic_set_property	= mixer_atomic_set_property,
	.atomic_get_property	= mixer_atomic_get_property,
};

static struct mixer_drv_data exynos5420_mxr_drv_data = {
	.version = MXR_VER_128_0_0_184,
	.is_vp_enabled = 0,
};

static struct mixer_drv_data exynos5250_mxr_drv_data = {
	.version = MXR_VER_16_0_33_0,
	.is_vp_enabled = 0,
};

static struct mixer_drv_data exynos4212_mxr_drv_data = {
	.version = MXR_VER_0_0_0_16,
	.is_vp_enabled = 1,
};

static struct mixer_drv_data exynos4210_mxr_drv_data = {
	.version = MXR_VER_0_0_0_16,
	.is_vp_enabled = 1,
	.has_sclk = 1,
};

static struct of_device_id mixer_match_types[] = {
	{
		.compatible = "samsung,exynos4210-mixer",
		.data	= &exynos4210_mxr_drv_data,
	}, {
		.compatible = "samsung,exynos4212-mixer",
		.data	= &exynos4212_mxr_drv_data,
	}, {
		.compatible = "samsung,exynos5-mixer",
		.data	= &exynos5250_mxr_drv_data,
	}, {
		.compatible = "samsung,exynos5250-mixer",
		.data	= &exynos5250_mxr_drv_data,
	}, {
		.compatible = "samsung,exynos5420-mixer",
		.data	= &exynos5420_mxr_drv_data,
	}, {
		/* end node */
	}
};
MODULE_DEVICE_TABLE(of, mixer_match_types);

static int mixer_bind(struct device *dev, struct device *manager, void *data)
{
	struct mixer_context *ctx = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct exynos_drm_plane *exynos_plane;
	unsigned int i;
	int ret;

	ret = mixer_initialize(ctx, drm_dev);
	if (ret)
		return ret;

	for (i = 0; i < MIXER_WIN_NR; i++) {
		if (i == VP_DEFAULT_WIN && !test_bit(MXR_BIT_VP_ENABLED, &ctx->flags))
			continue;

		ret = exynos_plane_init(drm_dev, &ctx->planes[i], i,
					1 << ctx->pipe, &plane_configs[i]);
		if (ret)
			return ret;
	}

	exynos_plane = &ctx->planes[DEFAULT_WIN];
	ctx->crtc = exynos_drm_crtc_create(drm_dev, &exynos_plane->base,
					   ctx->pipe, EXYNOS_DISPLAY_TYPE_HDMI,
					   &mixer_crtc_ops, ctx);
	if (IS_ERR(ctx->crtc)) {
		mixer_ctx_remove(ctx);
		ret = PTR_ERR(ctx->crtc);
		goto free_ctx;
	}

	mixer_attach_range_property(drm_dev, ctx->crtc);

	return 0;

free_ctx:
	devm_kfree(dev, ctx);
	return ret;
}

static void mixer_unbind(struct device *dev, struct device *master, void *data)
{
	struct mixer_context *ctx = dev_get_drvdata(dev);

	mixer_ctx_remove(ctx);
}

static const struct component_ops mixer_component_ops = {
	.bind	= mixer_bind,
	.unbind	= mixer_unbind,
};

static int mixer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct mixer_drv_data *drv;
	struct mixer_context *ctx;
	int ret;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		DRM_ERROR("failed to alloc mixer context.\n");
		return -ENOMEM;
	}

	drv = of_device_get_match_data(dev);

	ctx->pdev = pdev;
	ctx->dev = dev;
	ctx->mxr_ver = drv->version;

	if (drv->is_vp_enabled)
		__set_bit(MXR_BIT_VP_ENABLED, &ctx->flags);
	if (drv->has_sclk)
		__set_bit(MXR_BIT_HAS_SCLK, &ctx->flags);

	platform_set_drvdata(pdev, ctx);

	ret = component_add(&pdev->dev, &mixer_component_ops);
	if (!ret)
		pm_runtime_enable(dev);

	return ret;
}

static int mixer_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	component_del(&pdev->dev, &mixer_component_ops);

	return 0;
}

static int __maybe_unused exynos_mixer_suspend(struct device *dev)
{
	struct mixer_context *ctx = dev_get_drvdata(dev);
	struct mixer_resources *res = &ctx->mixer_res;

	clk_disable_unprepare(res->hdmi);
	clk_disable_unprepare(res->mixer);
	if (test_bit(MXR_BIT_VP_ENABLED, &ctx->flags)) {
		clk_disable_unprepare(res->vp);
		if (test_bit(MXR_BIT_HAS_SCLK, &ctx->flags))
			clk_disable_unprepare(res->sclk_mixer);
	}

	return 0;
}

static int __maybe_unused exynos_mixer_resume(struct device *dev)
{
	struct mixer_context *ctx = dev_get_drvdata(dev);
	struct mixer_resources *res = &ctx->mixer_res;
	int ret;

	ret = clk_prepare_enable(res->mixer);
	if (ret < 0) {
		DRM_ERROR("Failed to prepare_enable the mixer clk [%d]\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(res->hdmi);
	if (ret < 0) {
		DRM_ERROR("Failed to prepare_enable the hdmi clk [%d]\n", ret);
		return ret;
	}
	if (test_bit(MXR_BIT_VP_ENABLED, &ctx->flags)) {
		ret = clk_prepare_enable(res->vp);
		if (ret < 0) {
			DRM_ERROR("Failed to prepare_enable the vp clk [%d]\n",
				  ret);
			return ret;
		}
		if (test_bit(MXR_BIT_HAS_SCLK, &ctx->flags)) {
			ret = clk_prepare_enable(res->sclk_mixer);
			if (ret < 0) {
				DRM_ERROR("Failed to prepare_enable the " \
					   "sclk_mixer clk [%d]\n",
					  ret);
				return ret;
			}
		}
	}

	return 0;
}

static const struct dev_pm_ops exynos_mixer_pm_ops = {
	SET_RUNTIME_PM_OPS(exynos_mixer_suspend, exynos_mixer_resume, NULL)
};

struct platform_driver mixer_driver = {
	.driver = {
		.name = "exynos-mixer",
		.owner = THIS_MODULE,
		.pm = &exynos_mixer_pm_ops,
		.of_match_table = mixer_match_types,
	},
	.probe = mixer_probe,
	.remove = mixer_remove,
};
