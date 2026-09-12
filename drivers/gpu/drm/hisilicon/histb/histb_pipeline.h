/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __HISTB_PIPELINE_H__
#define __HISTB_PIPELINE_H__

#include <linux/types.h>

#include <drm/drm_color_mgmt.h>

struct device_node;
struct histb_vdp;
struct drm_display_mode;

enum histb_vdp_gfx_ifmt {
	HISTB_VDP_GFX_IFMT_ARGB8888 = 0x68,
	HISTB_VDP_GFX_IFMT_ABGR8888 = 0xef,
	HISTB_VDP_GFX_IFMT_ARGB4444 = 0x48,
	HISTB_VDP_GFX_IFMT_ARGB1555 = 0x49,
	HISTB_VDP_GFX_IFMT_RGB565 = 0x42,
};

struct histb_vdp *histb_vdp_get_from_node(struct device_node *np);
int histb_vdp_pipeline_prepare(struct histb_vdp *vdp);
int histb_vdp_pipeline_mode_valid(const struct drm_display_mode *mode);
int histb_vdp_pipeline_setup_output(struct histb_vdp *vdp);
int histb_vdp_pipeline_set_mode(struct histb_vdp *vdp,
				const struct drm_display_mode *mode);
int histb_vdp_pipeline_enable(struct histb_vdp *vdp);
void histb_vdp_pipeline_disable(struct histb_vdp *vdp);
int histb_vdp_pipeline_gfx_setup(struct histb_vdp *vdp,
				 u32 width, u32 height, u32 stride,
				 enum histb_vdp_gfx_ifmt ifmt);
int histb_vdp_pipeline_gfx_set_addr(struct histb_vdp *vdp, u32 addr);
int histb_vdp_pipeline_gfx_enable(struct histb_vdp *vdp, bool enable);
int histb_vdp_pipeline_gfx_set_blend(struct histb_vdp *vdp,
				     u16 alpha, u16 pixel_blend_mode);
int histb_vdp_pipeline_video_setup(struct histb_vdp *vdp,
				   u32 src_x, u32 src_y,
				   u32 src_width, u32 src_height,
				   u32 dst_x, u32 dst_y,
				   u32 dst_width, u32 dst_height,
				   u32 luma_stride, u32 chroma_stride);
int histb_vdp_pipeline_video_set_addr(struct histb_vdp *vdp,
				      u32 luma_addr, u32 chroma_addr);
int histb_vdp_pipeline_video_enable(struct histb_vdp *vdp, bool enable);
void histb_vdp_pipeline_video_dump(struct histb_vdp *vdp, const char *tag);

void histb_vdp_set_vblank_handler(struct histb_vdp *vdp,
				  void (*cb)(void *data), void *data);
void histb_vdp_enable_vblank(struct histb_vdp *vdp);
void histb_vdp_disable_vblank(struct histb_vdp *vdp);
int histb_vdp_pipeline_video_set_alpha(struct histb_vdp *vdp, u16 alpha);
int histb_vdp_pipeline_video_set_colorspace(struct histb_vdp *vdp,
					    enum drm_color_encoding encoding,
					    enum drm_color_range range);
int histb_vdp_pipeline_set_zorder(struct histb_vdp *vdp, bool video_on_top);

#endif /* __HISTB_PIPELINE_H__ */
