// SPDX-License-Identifier: GPL-2.0-only
/*
 * Atomic DRM/KMS front end for the HiSilicon STB display pipeline.
 */

#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/string.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_module.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_prime.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_writeback.h>

#include "histb_pipeline.h"

struct histb_drm {
	struct drm_device drm;
	struct histb_vdp *vdp;
	struct drm_bridge *hdmi_bridge;
	struct drm_connector *connector;
	struct drm_writeback_connector wb_connector;
	struct drm_plane primary_plane;
	struct drm_plane video_plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_encoder wb_encoder;
};

#define to_histb_drm(x) container_of(x, struct histb_drm, drm)

#define HISTB_DRM_VIDEO_MIN_SCALE (DRM_PLANE_NO_SCALING / 4)
#define HISTB_DRM_VIDEO_MAX_SCALE (DRM_PLANE_NO_SCALING * 4)

#define HISTB_DRM_PORT_VDP		0
#define HISTB_DRM_PORT_HDMI		1

static struct device_node *histb_drm_port_parent(struct drm_device *drm,
						 unsigned int index)
{
	struct device_node *ep;
	struct device_node *np;

	ep = of_parse_phandle(drm->dev->of_node, "ports", index);
	if (!ep)
		return NULL;

	np = of_graph_get_port_parent(ep);
	of_node_put(ep);

	return np;
}

static struct histb_drm *histb_drm_from_wb_connector(
	struct drm_writeback_connector *wb_connector)
{
	return container_of(wb_connector, struct histb_drm, wb_connector);
}

static int histb_drm_atomic_check(struct drm_device *drm,
				  struct drm_atomic_state *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	struct histb_drm *priv;
	struct drm_plane *plane;
	const struct drm_plane_state *plane_state;
	int ret;
	int i;

	ret = drm_atomic_helper_check(drm, state);
	if (ret)
		return ret;

	ret = drm_atomic_normalize_zpos(drm, state);
	if (ret)
		return ret;

	for_each_new_crtc_in_state(state, crtc, crtc_state, i) {
		unsigned int primary_visible = 0;
		unsigned int video_visible = 0;

		if (!crtc_state->active)
			continue;

		priv = to_histb_drm(crtc->dev);

		drm_atomic_crtc_state_for_each_plane_state(plane, plane_state,
							   crtc_state) {
			if (!plane_state->visible)
				continue;

			if (plane == crtc->primary) {
				primary_visible++;
				continue;
			}

			if (plane == &priv->video_plane) {
				video_visible++;
				continue;
			}

			/*
			 * Keep composition bounded to known VDP mixers:
			 * - primary GFX plane (G0/GP0)
			 * - video overlay plane (V0/VP0)
			 */
			return -EINVAL;
		}

		if (!primary_visible && !video_visible)
			return -EINVAL;

		if (primary_visible > 1 || video_visible > 1)
			return -EINVAL;
	}

	return 0;
}

static const struct drm_mode_config_funcs histb_drm_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = histb_drm_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static unsigned int histb_drm_fixed16_to_frac4(u32 value)
{
	return DIV_ROUND_CLOSEST((value & 0xffff) * 10000, 65536);
}

static void histb_drm_debugfs_dump_plane_state(struct seq_file *m,
						const char *name,
						const struct drm_plane_state *state)
{
	if (!state) {
		seq_printf(m, "%s: <none>\n", name);
		return;
	}

	seq_printf(m,
		   "%s: visible=%d zpos=%u norm_zpos=%u alpha=%u blend=%u fence=%d enc=%u range=%u crtc_xywh=%d,%d %ux%u src_xywh=%u.%04u,%u.%04u %u.%04ux%u.%04u\n",
		   name, state->visible,
		   state->zpos, state->normalized_zpos,
		   state->alpha, state->pixel_blend_mode,
		   !!state->fence,
		   state->color_encoding, state->color_range,
		   state->crtc_x, state->crtc_y,
		   state->crtc_w, state->crtc_h,
		   state->src_x >> 16,
		   histb_drm_fixed16_to_frac4(state->src_x),
		   state->src_y >> 16,
		   histb_drm_fixed16_to_frac4(state->src_y),
		   state->src_w >> 16,
		   histb_drm_fixed16_to_frac4(state->src_w),
		   state->src_h >> 16,
		   histb_drm_fixed16_to_frac4(state->src_h));
}

static void histb_drm_debugfs_dump_fb_state(struct seq_file *m,
					    const char *name,
					    const struct drm_framebuffer *fb)
{
	if (!fb) {
		seq_printf(m, "%s: <none>\n", name);
		return;
	}

	seq_printf(m,
		   "%s: id=%u %ux%u format=%p4cc modifier=%#llx pitch0=%u pitch1=%u\n",
		   name, fb->base.id, fb->width, fb->height,
		   &fb->format->format, (unsigned long long)fb->modifier,
		   fb->pitches[0], fb->pitches[1]);
}

static int histb_drm_debugfs_state_show(struct seq_file *m, void *data)
{
	struct drm_debugfs_entry *entry = m->private;
	struct drm_device *drm = entry->dev;
	struct histb_drm *priv = to_histb_drm(drm);
	const struct drm_crtc_state *crtc_state;
	const struct drm_plane_state *primary_state;
	const struct drm_plane_state *video_state;
	const struct drm_framebuffer *primary_fb;
	const struct drm_framebuffer *video_fb;
	struct drm_bridge *bridge;
	int bridge_idx = 0;

	(void)data;

	drm_modeset_lock_all(drm);

	crtc_state = priv->crtc.state;
	primary_state = priv->primary_plane.state;
	video_state = priv->video_plane.state;
	primary_fb = primary_state ? primary_state->fb : NULL;
	video_fb = video_state ? video_state->fb : NULL;

	seq_puts(m, "histb drm state\n");
	seq_printf(m, "crtc=%u encoder=%u connector=%u\n",
		   priv->crtc.base.id, priv->encoder.base.id,
		   priv->connector ? priv->connector->base.id : 0);
	seq_printf(m, "connector_status=%s\n",
		   priv->connector ?
		   drm_get_connector_status_name(priv->connector->status) :
		   "unbound");
	seq_printf(m, "vblank_count=%llu\n",
		   drm_crtc_vblank_count(&priv->crtc));

	if (crtc_state) {
		const struct drm_display_mode *mode = &crtc_state->mode;

		seq_printf(m, "crtc_state: active=%d enable=%d event_pending=%d plane_mask=%#lx\n",
			   crtc_state->active, crtc_state->enable,
			   !!crtc_state->event,
			   (unsigned long)crtc_state->plane_mask);
		if (mode->clock) {
			seq_printf(m,
				   "mode: %s %ux%u@%d clock=%d flags=%#x\n",
				   mode->name, mode->hdisplay, mode->vdisplay,
				   drm_mode_vrefresh(mode), mode->clock,
				   mode->flags);
		} else {
			seq_puts(m, "mode: <unset>\n");
		}
	} else {
		seq_puts(m, "crtc_state: <none>\n");
	}

	seq_printf(m, "path: vdp=%p\n", priv->vdp);
	drm_for_each_bridge_in_chain(&priv->encoder, bridge) {
		seq_printf(m,
			   "bridge[%d]: type=%d ops=%#x of_node=%pOF\n",
			   bridge_idx++, bridge->type, bridge->ops,
			   bridge->of_node);
	}
	if (!bridge_idx)
		seq_puts(m, "bridge: <none>\n");
	seq_printf(m, "connector_of_node=%pOF\n",
		   (priv->connector && priv->connector->dev &&
		    priv->connector->dev->dev) ?
		   priv->connector->dev->dev->of_node : NULL);

	histb_drm_debugfs_dump_plane_state(m, "primary_plane", primary_state);
	histb_drm_debugfs_dump_fb_state(m, "primary_fb", primary_fb);
	histb_drm_debugfs_dump_plane_state(m, "video_plane", video_state);
	histb_drm_debugfs_dump_fb_state(m, "video_fb", video_fb);

	drm_modeset_unlock_all(drm);
	return 0;
}

static void histb_drm_debugfs_init(struct drm_minor *minor)
{
	drm_debugfs_add_file(minor->dev, "histb_state",
			     histb_drm_debugfs_state_show, NULL);
}

static int histb_drm_dumb_create(struct drm_file *file_priv,
				 struct drm_device *drm,
				 struct drm_mode_create_dumb *args)
{
	unsigned int min_pitch = DIV_ROUND_UP(args->width * args->bpp, 8);

	args->pitch = ALIGN(max(args->pitch, min_pitch), 16);
	args->size = args->pitch * args->height;

	return drm_gem_dma_dumb_create_internal(file_priv, drm, args);
}

static void histb_drm_vblank_handler(void *data)
{
	struct histb_drm *priv = data;

	drm_crtc_handle_vblank(&priv->crtc);
}

static int histb_drm_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct histb_drm *priv = container_of(crtc, struct histb_drm, crtc);

	histb_vdp_enable_vblank(priv->vdp);

	return 0;
}

static void histb_drm_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct histb_drm *priv = container_of(crtc, struct histb_drm, crtc);

	histb_vdp_disable_vblank(priv->vdp);
}

static void histb_drm_complete_event(struct drm_crtc *crtc, bool on_vblank)
{
	struct drm_pending_vblank_event *event;

	spin_lock_irq(&crtc->dev->event_lock);
	event = crtc->state->event;
	if (event) {
		if (on_vblank && drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);

		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static struct histb_drm *histb_drm_from_plane(struct drm_plane *plane)
{
	return to_histb_drm(plane->dev);
}

static bool histb_drm_is_video_plane(struct histb_drm *priv,
				     struct drm_plane *plane)
{
	return plane == &priv->video_plane;
}

static int histb_drm_fourcc_to_ifmt(u32 fourcc, enum histb_vdp_gfx_ifmt *ifmt)
{
	switch (fourcc) {
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
		*ifmt = HISTB_VDP_GFX_IFMT_ARGB8888;
		return 0;
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
		*ifmt = HISTB_VDP_GFX_IFMT_ABGR8888;
		return 0;
	case DRM_FORMAT_ARGB4444:
	case DRM_FORMAT_XRGB4444:
		*ifmt = HISTB_VDP_GFX_IFMT_ARGB4444;
		return 0;
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
		*ifmt = HISTB_VDP_GFX_IFMT_ARGB1555;
		return 0;
	case DRM_FORMAT_RGB565:
		*ifmt = HISTB_VDP_GFX_IFMT_RGB565;
		return 0;
	default:
		return -EINVAL;
	}
}

static bool histb_drm_primary_fourcc_supported(u32 fourcc)
{
	enum histb_vdp_gfx_ifmt ifmt;

	return !histb_drm_fourcc_to_ifmt(fourcc, &ifmt);
}

static int histb_drm_program_primary_plane(struct histb_drm *priv,
					   struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	dma_addr_t addr;
	enum histb_vdp_gfx_ifmt ifmt;
	u16 blend_mode = state->pixel_blend_mode;
	int ret;

	if (!fb)
		return -EINVAL;

	ret = histb_drm_fourcc_to_ifmt(fb->format->format, &ifmt);
	if (ret)
		return ret;

	ret = histb_vdp_pipeline_gfx_setup(priv->vdp, fb->width, fb->height,
					   fb->pitches[0],
					   ifmt);
	if (ret)
		return ret;

	/*
	 * DRM plane state defaults pixel_blend_mode to PREMULTI. For XRGB
	 * scanout there is no alpha channel, so enabling pixel alpha can make
	 * fbcon fully transparent/black on this pipeline.
	 */
	if (!fb->format->has_alpha)
		blend_mode = DRM_MODE_BLEND_PIXEL_NONE;

	ret = histb_vdp_pipeline_gfx_set_blend(priv->vdp, state->alpha,
					       blend_mode);
	if (ret)
		return ret;

	addr = drm_fb_dma_get_gem_addr(fb, state, 0);
	if (addr > U32_MAX)
		return -ERANGE;

	ret = histb_vdp_pipeline_gfx_set_addr(priv->vdp, (u32)addr);
	if (ret)
		return ret;

	return histb_vdp_pipeline_gfx_enable(priv->vdp, true);
}

static bool histb_drm_video_fourcc_supported(u32 fourcc)
{
	return fourcc == DRM_FORMAT_NV21;
}

static int histb_drm_video_get_rects(const struct drm_plane_state *state,
				     u32 *src_x, u32 *src_y,
				     u32 *src_w, u32 *src_h,
				     u32 *dst_x, u32 *dst_y,
				     u32 *dst_w, u32 *dst_h)
{
	if (!state || !state->fb || !src_x || !src_y || !src_w || !src_h ||
	    !dst_x || !dst_y || !dst_w || !dst_h)
		return -EINVAL;

	if ((state->src_x & 0xffff) || (state->src_y & 0xffff) ||
	    (state->src_w & 0xffff) || (state->src_h & 0xffff))
		return -EINVAL;

	if (state->crtc_x < 0 || state->crtc_y < 0)
		return -EINVAL;

	*src_x = state->src_x >> 16;
	*src_y = state->src_y >> 16;
	*src_w = state->src_w >> 16;
	*src_h = state->src_h >> 16;
	*dst_x = state->crtc_x;
	*dst_y = state->crtc_y;
	*dst_w = state->crtc_w;
	*dst_h = state->crtc_h;

	if (!*src_w || !*src_h || !*dst_w || !*dst_h)
		return -EINVAL;

	return 0;
}

static int histb_drm_check_dma_addrs(struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	unsigned int i;

	if (!fb)
		return -EINVAL;

	for (i = 0; i < fb->format->num_planes; i++) {
		dma_addr_t addr;

		if (!drm_fb_dma_get_gem_obj(fb, i))
			return -EINVAL;

		addr = drm_fb_dma_get_gem_addr(fb, state, i);
		if ((u64)addr > U32_MAX)
			return -ERANGE;
	}

	return 0;
}

static bool histb_drm_primary_blend_mode_supported(u16 blend_mode)
{
	return blend_mode == DRM_MODE_BLEND_PIXEL_NONE ||
	       blend_mode == DRM_MODE_BLEND_COVERAGE ||
	       blend_mode == DRM_MODE_BLEND_PREMULTI;
}

static bool histb_drm_video_blend_mode_supported(u16 blend_mode)
{
	return blend_mode == DRM_MODE_BLEND_PIXEL_NONE ||
	       blend_mode == DRM_MODE_BLEND_PREMULTI;
}

static bool histb_drm_video_color_encoding_supported(enum drm_color_encoding encoding)
{
	return encoding == DRM_COLOR_YCBCR_BT601;
}

static bool histb_drm_video_color_range_supported(enum drm_color_range range)
{
	return range == DRM_COLOR_YCBCR_LIMITED_RANGE;
}

static struct drm_plane_state *
histb_drm_get_new_or_current_plane_state(struct drm_atomic_state *state,
					 struct drm_plane *plane)
{
	struct drm_plane_state *new_state;

	new_state = drm_atomic_get_new_plane_state(state, plane);
	if (new_state)
		return new_state;

	return plane->state;
}

static int histb_drm_writeback_blit_primary(
	const struct drm_plane_state *primary_state,
	struct drm_framebuffer *dst_fb)
{
	struct drm_framebuffer *src_fb = primary_state->fb;
	struct drm_gem_dma_object *src_obj;
	struct drm_gem_dma_object *dst_obj;
	u8 *src;
	u8 *dst;
	unsigned int cpp;
	unsigned int row;
	unsigned int line_bytes;

	if (!src_fb || !dst_fb)
		return -EINVAL;

	if (!histb_drm_primary_fourcc_supported(src_fb->format->format))
		return -EINVAL;

	if (src_fb->format->format != dst_fb->format->format)
		return -EINVAL;

	if (src_fb->width != dst_fb->width || src_fb->height != dst_fb->height)
		return -EINVAL;

	if (src_fb->modifier != DRM_FORMAT_MOD_LINEAR ||
	    dst_fb->modifier != DRM_FORMAT_MOD_LINEAR)
		return -EINVAL;

	cpp = src_fb->format->cpp[0];
	if (!cpp)
		return -EINVAL;

	line_bytes = src_fb->width * cpp;
	if (line_bytes > src_fb->pitches[0] || line_bytes > dst_fb->pitches[0])
		return -EINVAL;

	src_obj = drm_fb_dma_get_gem_obj(src_fb, 0);
	dst_obj = drm_fb_dma_get_gem_obj(dst_fb, 0);
	if (!src_obj || !dst_obj || !src_obj->vaddr || !dst_obj->vaddr)
		return -EOPNOTSUPP;

	src = (u8 *)src_obj->vaddr + src_fb->offsets[0];
	dst = (u8 *)dst_obj->vaddr + dst_fb->offsets[0];

	for (row = 0; row < src_fb->height; row++) {
		memcpy(dst + row * dst_fb->pitches[0],
		       src + row * src_fb->pitches[0],
		       line_bytes);
	}

	return 0;
}

static int histb_drm_writeback_atomic_check(struct drm_connector *connector,
					    struct drm_atomic_state *state)
{
	struct drm_writeback_connector *wb_connector =
		drm_connector_to_writeback(connector);
	struct histb_drm *priv = histb_drm_from_wb_connector(wb_connector);
	struct drm_connector_state *conn_state;
	struct drm_crtc_state *crtc_state;
	struct drm_plane_state *primary_state;
	struct drm_plane_state *video_state;
	struct drm_framebuffer *primary_fb;
	struct drm_framebuffer *wb_fb;
	const struct drm_display_mode *mode;
	int ret;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (!conn_state || !conn_state->writeback_job ||
	    !conn_state->writeback_job->fb)
		return 0;

	if (!conn_state->crtc)
		return -EINVAL;

	ret = drm_atomic_helper_check_wb_connector_state(connector, state);
	if (ret)
		return ret;

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (!crtc_state)
		return -EINVAL;

	if (!crtc_state->active)
		return -EINVAL;

	mode = &crtc_state->mode;
	wb_fb = conn_state->writeback_job->fb;
	if (wb_fb->width != mode->hdisplay || wb_fb->height != mode->vdisplay)
		return -EINVAL;

	primary_state = histb_drm_get_new_or_current_plane_state(state,
								  &priv->primary_plane);
	video_state = histb_drm_get_new_or_current_plane_state(state,
								&priv->video_plane);
	if (!primary_state || !primary_state->visible || !primary_state->fb)
		return -EINVAL;

	/* Temporary limitation: write back the primary scanout only. */
	if (video_state && video_state->visible && video_state->fb)
		return -EOPNOTSUPP;

	primary_fb = primary_state->fb;
	if (!histb_drm_primary_fourcc_supported(primary_fb->format->format))
		return -EINVAL;

	if (primary_fb->format->format != wb_fb->format->format)
		return -EINVAL;

	if (primary_fb->width != wb_fb->width ||
	    primary_fb->height != wb_fb->height)
		return -EINVAL;

	if (primary_fb->modifier != DRM_FORMAT_MOD_LINEAR ||
	    wb_fb->modifier != DRM_FORMAT_MOD_LINEAR)
		return -EINVAL;

	if (!drm_fb_dma_get_gem_obj(primary_fb, 0) ||
	    !drm_fb_dma_get_gem_obj(wb_fb, 0))
		return -EINVAL;

	if (!drm_fb_dma_get_gem_obj(primary_fb, 0)->vaddr ||
	    !drm_fb_dma_get_gem_obj(wb_fb, 0)->vaddr)
		return -EOPNOTSUPP;

	return 0;
}

static void histb_drm_writeback_atomic_commit(struct drm_connector *connector,
					      struct drm_atomic_state *state)
{
	struct drm_writeback_connector *wb_connector =
		drm_connector_to_writeback(connector);
	struct histb_drm *priv = histb_drm_from_wb_connector(wb_connector);
	struct drm_connector_state *conn_state;
	struct drm_plane_state *primary_state;
	struct drm_writeback_job *job;
	int ret;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (!conn_state || !conn_state->writeback_job ||
	    !conn_state->writeback_job->fb)
		return;

	primary_state = histb_drm_get_new_or_current_plane_state(state,
								  &priv->primary_plane);
	job = conn_state->writeback_job;

	drm_writeback_queue_job(wb_connector, conn_state);

	if (!primary_state || !primary_state->fb)
		ret = -EINVAL;
	else
		ret = histb_drm_writeback_blit_primary(primary_state, job->fb);

	drm_writeback_signal_completion(wb_connector, ret);
}

static int histb_drm_writeback_connector_get_modes(
	struct drm_connector *connector)
{
	struct drm_device *drm = connector->dev;

	return drm_add_modes_noedid(connector, drm->mode_config.max_width,
				    drm->mode_config.max_height);
}

static bool histb_drm_video_on_top(const struct histb_drm *priv)
{
	const struct drm_plane_state *video_state = priv->video_plane.state;
	const struct drm_plane_state *primary_state = priv->primary_plane.state;

	if (video_state && video_state->visible) {
		if (!primary_state || !primary_state->visible)
			return true;

		return video_state->normalized_zpos >
		       primary_state->normalized_zpos;
	}

	return false;
}

static void histb_drm_program_plane_zorder(struct histb_drm *priv)
{
	const struct drm_plane_state *vs = priv->video_plane.state;
	const struct drm_plane_state *ps = priv->primary_plane.state;
	bool video_on_top = histb_drm_video_on_top(priv);
	int ret;

	drm_dbg_atomic(&priv->drm,
		       "zorder: video_on_top=%d video[vis=%d nz=%u] primary[vis=%d nz=%u]\n",
		       video_on_top,
		       vs ? vs->visible : -1, vs ? vs->normalized_zpos : 0,
		       ps ? ps->visible : -1, ps ? ps->normalized_zpos : 0);

	ret = histb_vdp_pipeline_set_zorder(priv->vdp, video_on_top);
	if (ret == -EPIPE)
		return;

	if (ret)
		drm_warn(&priv->drm, "VDP zorder setup failed: %d\n", ret);
}

static int histb_drm_program_video_plane(struct histb_drm *priv,
					 struct drm_plane_state *state)
{
	struct drm_framebuffer *fb = state->fb;
	dma_addr_t luma_addr;
	dma_addr_t chroma_addr;
	u32 src_x;
	u32 src_y;
	u32 src_w;
	u32 src_h;
	u32 dst_x;
	u32 dst_y;
	u32 dst_w;
	u32 dst_h;
	int ret;

	if (!fb || !histb_drm_video_fourcc_supported(fb->format->format))
		return -EINVAL;

	ret = histb_drm_video_get_rects(state,
					&src_x, &src_y, &src_w, &src_h,
					&dst_x, &dst_y, &dst_w, &dst_h);
	if (ret)
		return ret;

	ret = histb_vdp_pipeline_video_setup(priv->vdp,
					     src_x, src_y,
					     src_w, src_h,
					     dst_x, dst_y, dst_w, dst_h,
					     fb->pitches[0], fb->pitches[1]);
	if (ret)
		return ret;

	ret = histb_vdp_pipeline_video_set_colorspace(priv->vdp,
						      state->color_encoding,
						      state->color_range);
	if (ret)
		return ret;

	ret = histb_vdp_pipeline_video_set_alpha(priv->vdp, state->alpha);
	if (ret)
		return ret;

	/* Crop offsets are reflected by the DMA helper via src_x/src_y. */
	luma_addr = drm_fb_dma_get_gem_addr(fb, state, 0);
	chroma_addr = drm_fb_dma_get_gem_addr(fb, state, 1);
	if (luma_addr > U32_MAX || chroma_addr > U32_MAX)
		return -ERANGE;

	/* A dumb buffer and an imported dmabuf differ only in the DMA address. */
	drm_dbg_atomic(&priv->drm,
		       "video: luma %pad chroma %pad pitches %u/%u src %ux%u+%u+%u dst %ux%u+%u+%u\n",
		       &luma_addr, &chroma_addr,
		       fb->pitches[0], fb->pitches[1],
		       src_w, src_h, src_x, src_y,
		       dst_w, dst_h, dst_x, dst_y);

	ret = histb_vdp_pipeline_video_set_addr(priv->vdp, (u32)luma_addr,
						(u32)chroma_addr);
	if (ret)
		return ret;

	ret = histb_vdp_pipeline_video_enable(priv->vdp, true);
	if (ret)
		return ret;

	return 0;
}

static int histb_drm_check_fullscreen_scanout(const struct drm_plane_state *state,
					      const struct drm_crtc_state *crtc_state)
{
	if (!state || !crtc_state)
		return -EINVAL;

	if (!state->fb)
		return -EINVAL;

	if (state->crtc_x || state->crtc_y)
		return -EINVAL;

	if (state->src_x || state->src_y)
		return -EINVAL;

	if (state->src_w != (state->fb->width << 16) ||
	    state->src_h != (state->fb->height << 16))
		return -EINVAL;

	if (state->crtc_w != crtc_state->mode.hdisplay ||
	    state->crtc_h != crtc_state->mode.vdisplay)
		return -EINVAL;

	if (state->fb->width != crtc_state->mode.hdisplay ||
	    state->fb->height != crtc_state->mode.vdisplay)
		return -EINVAL;

	return 0;
}

static int histb_drm_primary_plane_atomic_check(struct drm_plane_state *state,
						const struct drm_crtc_state *crtc_state)
{
	uint64_t min_pitch;
	enum histb_vdp_gfx_ifmt ifmt;
	int ret;

	ret = histb_drm_check_fullscreen_scanout(state, crtc_state);
	if (ret)
		return ret;

	if (!histb_drm_primary_blend_mode_supported(state->pixel_blend_mode))
		return -EINVAL;

	if (state->fb->modifier != DRM_FORMAT_MOD_LINEAR)
		return -EINVAL;

	ret = histb_drm_fourcc_to_ifmt(state->fb->format->format, &ifmt);
	if (ret)
		return ret;

	min_pitch = drm_format_info_min_pitch(state->fb->format, 0,
					      state->fb->width);
	if (!min_pitch || state->fb->pitches[0] < min_pitch)
		return -EINVAL;

	if (state->fb->pitches[0] & 0xf)
		return -EINVAL;

	return histb_drm_check_dma_addrs(state);
}

static int histb_drm_video_plane_atomic_check(struct drm_plane_state *state,
					      const struct drm_crtc_state *crtc_state)
{
	struct drm_device *dev = state->plane->dev;
	uint64_t min_pitch_luma;
	uint64_t min_pitch_chroma;
	u32 src_x;
	u32 src_y;
	u32 src_w;
	u32 src_h;
	u32 dst_x;
	u32 dst_y;
	u32 dst_w;
	u32 dst_h;
	int ret;

	/*
	 * Every rejection below says why. A silent -EINVAL from atomic_check
	 * tells userspace only that the commit failed
	 */
	ret = histb_drm_video_get_rects(state,
					&src_x, &src_y, &src_w, &src_h,
					&dst_x, &dst_y, &dst_w, &dst_h);
	if (ret) {
		drm_dbg_atomic(dev, "video: bad rectangles\n");
		return ret;
	}

	if (!crtc_state) {
		drm_dbg_atomic(dev, "video: no CRTC state\n");
		return -EINVAL;
	}

	if (!histb_drm_video_fourcc_supported(state->fb->format->format)) {
		drm_dbg_atomic(dev, "video: format %p4cc not supported\n",
			       &state->fb->format->format);
		return -EINVAL;
	}

	if (!histb_drm_video_blend_mode_supported(state->pixel_blend_mode)) {
		drm_dbg_atomic(dev, "video: blend mode %u not supported\n",
			       state->pixel_blend_mode);
		return -EINVAL;
	}

	if (!histb_drm_video_color_encoding_supported(state->color_encoding)) {
		drm_dbg_atomic(dev, "video: colour encoding %u not supported\n",
			       state->color_encoding);
		return -EINVAL;
	}

	if (!histb_drm_video_color_range_supported(state->color_range)) {
		drm_dbg_atomic(dev, "video: colour range %u not supported\n",
			       state->color_range);
		return -EINVAL;
	}

	if (state->fb->format->num_planes != 2) {
		drm_dbg_atomic(dev, "video: %u planes, expected 2\n",
			       state->fb->format->num_planes);
		return -EINVAL;
	}

	if (state->fb->modifier != DRM_FORMAT_MOD_LINEAR) {
		drm_dbg_atomic(dev, "video: modifier %#llx is not linear\n",
			       state->fb->modifier);
		return -EINVAL;
	}

	if ((state->fb->width & 1) || (state->fb->height & 1)) {
		drm_dbg_atomic(dev, "video: odd framebuffer %ux%u\n",
			       state->fb->width, state->fb->height);
		return -EINVAL;
	}

	if ((src_x & 1) || (src_y & 1) || (src_w & 1) || (src_h & 1)) {
		drm_dbg_atomic(dev, "video: odd source rect %ux%u+%u+%u\n",
			       src_w, src_h, src_x, src_y);
		return -EINVAL;
	}

	if (src_w > state->fb->width || src_h > state->fb->height ||
	    src_x > state->fb->width - src_w ||
	    src_y > state->fb->height - src_h) {
		drm_dbg_atomic(dev,
			       "video: source rect %ux%u+%u+%u outside %ux%u framebuffer\n",
			       src_w, src_h, src_x, src_y,
			       state->fb->width, state->fb->height);
		return -EINVAL;
	}

	if (dst_w > crtc_state->mode.hdisplay ||
	    dst_h > crtc_state->mode.vdisplay ||
	    dst_x > crtc_state->mode.hdisplay - dst_w ||
	    dst_y > crtc_state->mode.vdisplay - dst_h) {
		drm_dbg_atomic(dev,
			       "video: destination rect %ux%u+%u+%u outside %ux%u mode\n",
			       dst_w, dst_h, dst_x, dst_y,
			       crtc_state->mode.hdisplay,
			       crtc_state->mode.vdisplay);
		return -EINVAL;
	}

	/*
	 * The helper multiplies the width it is given by the plane's bytes per
	 * block without applying chroma subsampling, so feed it the width of
	 * that plane rather than of the framebuffer, as drm_framebuffer.c does.
	 */
	min_pitch_luma = drm_format_info_min_pitch(state->fb->format, 0,
			drm_format_info_plane_width(state->fb->format,
						    state->fb->width, 0));
	min_pitch_chroma = drm_format_info_min_pitch(state->fb->format, 1,
			drm_format_info_plane_width(state->fb->format,
						    state->fb->width, 1));
	if (!min_pitch_luma || !min_pitch_chroma) {
		drm_dbg_atomic(dev, "video: cannot compute minimum pitches\n");
		return -EINVAL;
	}

	if (state->fb->pitches[0] < min_pitch_luma ||
	    state->fb->pitches[1] < min_pitch_chroma) {
		drm_dbg_atomic(dev,
			       "video: pitches %u/%u below minimum %llu/%llu\n",
			       state->fb->pitches[0], state->fb->pitches[1],
			       min_pitch_luma, min_pitch_chroma);
		return -EINVAL;
	}

	if ((state->fb->pitches[0] & 0xf) || (state->fb->pitches[1] & 0xf)) {
		drm_dbg_atomic(dev,
			       "video: pitches %u/%u are not 16-byte aligned\n",
			       state->fb->pitches[0], state->fb->pitches[1]);
		return -EINVAL;
	}

	ret = histb_drm_check_dma_addrs(state);
	if (ret)
		drm_dbg_atomic(dev, "video: unusable DMA addresses: %d\n", ret);

	return ret;
}

static int histb_drm_plane_atomic_check(struct drm_plane *plane,
					struct drm_atomic_state *state)
{
	struct histb_drm *priv = histb_drm_from_plane(plane);
	struct drm_plane_state *new_state;
	struct drm_crtc_state *crtc_state = NULL;
	int ret;

	new_state = drm_atomic_get_new_plane_state(state, plane);
	if (!new_state) {
		drm_dbg_atomic(plane->dev, "[PLANE:%d] no new state\n",
			       plane->base.id);
		return -EINVAL;
	}

	if (new_state->crtc)
		crtc_state = drm_atomic_get_new_crtc_state(state,
							   new_state->crtc);

	if (histb_drm_is_video_plane(priv, plane))
		ret = drm_atomic_helper_check_plane_state(new_state, crtc_state,
							  HISTB_DRM_VIDEO_MIN_SCALE,
							  HISTB_DRM_VIDEO_MAX_SCALE,
							  true, false);
	else
		ret = drm_atomic_helper_check_plane_state(new_state, crtc_state,
							  DRM_PLANE_NO_SCALING,
							  DRM_PLANE_NO_SCALING,
							  false, false);

	if (ret) {
		drm_dbg_atomic(plane->dev, "[PLANE:%d] helper check failed: %d\n",
			       plane->base.id, ret);
		return ret;
	}

	if (!new_state->visible) {
		drm_dbg_atomic(plane->dev, "[PLANE:%d] not visible, skipping\n",
			       plane->base.id);
		return 0;
	}

	if (histb_drm_is_video_plane(priv, plane))
		return histb_drm_video_plane_atomic_check(new_state, crtc_state);

	return histb_drm_primary_plane_atomic_check(new_state, crtc_state);
}

static void histb_drm_plane_atomic_update(struct drm_plane *plane,
					  struct drm_atomic_state *state)
{
	struct histb_drm *priv = histb_drm_from_plane(plane);
	struct drm_plane_state *new_state;
	int ret;

	new_state = drm_atomic_get_new_plane_state(state, plane);
	if (!new_state || !new_state->fb || !new_state->visible)
		return;

	if (histb_drm_is_video_plane(priv, plane))
		ret = histb_drm_program_video_plane(priv, new_state);
	else
		ret = histb_drm_program_primary_plane(priv, new_state);

	if (ret == -EPIPE)
		return;

	if (ret)
		drm_warn(&priv->drm, "%s plane update failed: %d\n",
			 histb_drm_is_video_plane(priv, plane) ?
			 "video" : "primary", ret);
}

static void histb_drm_plane_atomic_disable(struct drm_plane *plane,
					   struct drm_atomic_state *state)
{
	struct histb_drm *priv = histb_drm_from_plane(plane);
	int ret;

	(void)state;

	if (histb_drm_is_video_plane(priv, plane))
		ret = histb_vdp_pipeline_video_enable(priv->vdp, false);
	else
		ret = histb_vdp_pipeline_gfx_enable(priv->vdp, false);

	/*
	 * -EPIPE means the VDP is already powered down, which happens whenever
	 * planes are torn down after the CRTC. A disable request that finds the
	 * block already off has got what it asked for, so it is not a failure.
	 */
	if (ret && ret != -EPIPE)
		drm_warn(&priv->drm, "VDP %s disable failed: %d\n",
			 histb_drm_is_video_plane(priv, plane) ?
			 "video" : "gfx", ret);
}

static int histb_drm_plane_prepare_fb(struct drm_plane *plane,
				      struct drm_plane_state *state)
{
	/*
	 * Standard DRM sync path: merge implicit dma_resv fences (or explicit
	 * IN_FENCE_FD when provided) into state->fence for commit-time waits.
	 */
	return drm_gem_plane_helper_prepare_fb(plane, state);
}

static const struct drm_plane_helper_funcs histb_drm_plane_helper_funcs = {
	.prepare_fb = histb_drm_plane_prepare_fb,
	.atomic_check = histb_drm_plane_atomic_check,
	.atomic_update = histb_drm_plane_atomic_update,
	.atomic_disable = histb_drm_plane_atomic_disable,
};

static bool histb_drm_plane_format_mod_supported(
	struct drm_plane *plane, u32 format, u64 modifier)
{
	(void)plane;
	(void)format;

	return modifier == DRM_FORMAT_MOD_LINEAR;
}

static const struct drm_plane_funcs histb_drm_plane_funcs = {
	.format_mod_supported = histb_drm_plane_format_mod_supported,
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static int histb_drm_crtc_atomic_check(struct drm_crtc *crtc,
				       struct drm_atomic_state *state)
{
	struct histb_drm *priv = to_histb_drm(crtc->dev);
	struct drm_crtc_state *crtc_state;
	int ret;

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	if (!crtc_state) {
		drm_dbg_atomic(crtc->dev, "crtc: no new state\n");
		return -EINVAL;
	}

	if (crtc_state->active) {
		/*
		 * Deliberately no drm_atomic_helper_check_crtc_primary_plane():
		 * the video layer is independent of the graphics one, so a CRTC
		 * showing only video is valid, and demanding a primary plane
		 * makes removing the graphics framebuffer fail with -EINVAL.
		 */
		ret = histb_vdp_pipeline_mode_valid(&crtc_state->mode);
		if (ret) {
			drm_dbg_atomic(crtc->dev,
				       "crtc: mode " DRM_MODE_FMT " rejected: %d\n",
				       DRM_MODE_ARG(&crtc_state->mode), ret);
			return ret;
		}
	}

	return drm_atomic_add_affected_planes(state, crtc);
}

static void histb_drm_crtc_atomic_enable(struct drm_crtc *crtc,
					 struct drm_atomic_state *state)
{
	struct histb_drm *priv = container_of(crtc, struct histb_drm, crtc);
	struct drm_plane_state *primary_state;
	struct drm_plane_state *video_state;
	int ret;

	primary_state = drm_atomic_get_new_plane_state(state, crtc->primary);
	if (!primary_state)
		primary_state = crtc->primary->state;

	video_state = drm_atomic_get_new_plane_state(state, &priv->video_plane);
	if (!video_state)
		video_state = priv->video_plane.state;

	ret = histb_vdp_pipeline_mode_valid(&crtc->state->mode);
	if (ret) {
		drm_warn(&priv->drm, "unsupported mode in atomic_enable\n");
		return;
	}

	ret = histb_vdp_pipeline_setup_output(priv->vdp);
	if (ret) {
		drm_warn(&priv->drm, "VDP output path setup failed: %d\n", ret);
		return;
	}

	ret = histb_vdp_pipeline_set_mode(priv->vdp, &crtc->state->mode);
	if (ret)
		drm_warn(&priv->drm, "VDP mode setup failed: %d\n", ret);

	ret = histb_vdp_pipeline_enable(priv->vdp);
	if (ret) {
		drm_warn(&priv->drm, "VDP output enable failed: %d\n", ret);
		return;
	}

	drm_crtc_vblank_on(crtc);
	histb_drm_program_plane_zorder(priv);

	if (video_state && video_state->fb && video_state->visible) {
		ret = histb_drm_program_video_plane(priv, video_state);
		if (ret)
			drm_warn(&priv->drm,
				 "initial video plane program failed: %d\n",
				 ret);
	} else {
		ret = histb_vdp_pipeline_video_enable(priv->vdp, false);
		if (ret)
			drm_warn(&priv->drm, "VDP video disable failed: %d\n",
				 ret);
	}

	if (primary_state && primary_state->fb) {
		ret = histb_drm_program_primary_plane(priv, primary_state);
		if (ret)
			drm_warn(&priv->drm,
				 "initial primary plane program failed: %d\n",
				 ret);
	} else {
		ret = histb_vdp_pipeline_gfx_enable(priv->vdp, false);
		if (ret)
			drm_warn(&priv->drm, "VDP GFX disable failed: %d\n", ret);
	}

	histb_vdp_pipeline_video_dump(priv->vdp, "modeset");
}

static void histb_drm_crtc_atomic_disable(struct drm_crtc *crtc,
					  struct drm_atomic_state *state)
{
	struct histb_drm *priv = container_of(crtc, struct histb_drm, crtc);
	int ret;

	(void)state;

	ret = histb_vdp_pipeline_video_enable(priv->vdp, false);
	if (ret)
		drm_warn(&priv->drm, "VDP video disable failed: %d\n", ret);

	ret = histb_vdp_pipeline_gfx_enable(priv->vdp, false);
	if (ret)
		drm_warn(&priv->drm, "VDP GFX disable failed: %d\n", ret);

	/*
	 * Stop vblank before the pipeline: disable_vblank touches VDP
	 * registers, and the block is only guaranteed powered until
	 * histb_vdp_pipeline_disable() drops its runtime PM reference.
	 */
	drm_crtc_vblank_off(crtc);
	histb_vdp_pipeline_disable(priv->vdp);
	histb_drm_complete_event(crtc, false);
}

static void histb_drm_crtc_atomic_flush(struct drm_crtc *crtc,
					struct drm_atomic_state *state)
{
	struct histb_drm *priv = container_of(crtc, struct histb_drm, crtc);

	(void)state;

	if (crtc->state->active)
		histb_drm_program_plane_zorder(priv);

	histb_drm_complete_event(crtc, crtc->state->active);
}

static const struct drm_crtc_helper_funcs histb_drm_crtc_helper_funcs = {
	.atomic_check = histb_drm_crtc_atomic_check,
	.atomic_enable = histb_drm_crtc_atomic_enable,
	.atomic_disable = histb_drm_crtc_atomic_disable,
	.atomic_flush = histb_drm_crtc_atomic_flush,
};

static const struct drm_crtc_funcs histb_drm_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.destroy = drm_crtc_cleanup,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = histb_drm_crtc_enable_vblank,
	.disable_vblank = histb_drm_crtc_disable_vblank,
};

static const struct drm_encoder_funcs histb_drm_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const u32 histb_drm_primary_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XRGB4444,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_RGB565,
};

static const u32 histb_drm_video_formats[] = {
	DRM_FORMAT_NV21,
};

static const u64 histb_drm_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static int histb_drm_setup_mode_config(struct drm_device *drm)
{
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = 64;
	drm->mode_config.min_height = 64;
	drm->mode_config.max_width = 1920;
	drm->mode_config.max_height = 1080;
	drm->mode_config.funcs = &histb_drm_mode_config_funcs;

	return 0;
}

static int histb_drm_create_primary_plane(struct drm_device *drm)
{
	struct histb_drm *priv = to_histb_drm(drm);
	int ret;

	ret = drm_universal_plane_init(drm, &priv->primary_plane, 0,
				       &histb_drm_plane_funcs,
				       histb_drm_primary_formats,
				       ARRAY_SIZE(histb_drm_primary_formats),
				       histb_drm_modifiers,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;

	drm_plane_helper_add(&priv->primary_plane, &histb_drm_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(&priv->primary_plane);

	ret = drm_plane_create_zpos_property(&priv->primary_plane, 0, 0, 1);
	if (ret)
		return ret;

	ret = drm_plane_create_alpha_property(&priv->primary_plane);
	if (ret)
		return ret;

	ret = drm_plane_create_blend_mode_property(&priv->primary_plane,
						   BIT(DRM_MODE_BLEND_PIXEL_NONE) |
						   BIT(DRM_MODE_BLEND_COVERAGE) |
						   BIT(DRM_MODE_BLEND_PREMULTI));
	if (ret)
		return ret;

	return 0;
}

static int histb_drm_create_video_plane(struct drm_device *drm)
{
	struct histb_drm *priv = to_histb_drm(drm);
	int ret;

	ret = drm_universal_plane_init(drm, &priv->video_plane,
				       drm_crtc_mask(&priv->crtc),
				       &histb_drm_plane_funcs,
				       histb_drm_video_formats,
				       ARRAY_SIZE(histb_drm_video_formats),
				       histb_drm_modifiers,
				       DRM_PLANE_TYPE_OVERLAY, NULL);
	if (ret)
		return ret;

	drm_plane_helper_add(&priv->video_plane, &histb_drm_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(&priv->video_plane);

	ret = drm_plane_create_zpos_property(&priv->video_plane, 1, 0, 1);
	if (ret)
		return ret;

	ret = drm_plane_create_alpha_property(&priv->video_plane);
	if (ret)
		return ret;

	ret = drm_plane_create_blend_mode_property(&priv->video_plane,
						   BIT(DRM_MODE_BLEND_PIXEL_NONE) |
						   BIT(DRM_MODE_BLEND_PREMULTI));
	if (ret)
		return ret;

	ret = drm_plane_create_color_properties(&priv->video_plane,
						BIT(DRM_COLOR_YCBCR_BT601),
						BIT(DRM_COLOR_YCBCR_LIMITED_RANGE),
						DRM_COLOR_YCBCR_BT601,
						DRM_COLOR_YCBCR_LIMITED_RANGE);
	if (ret)
		return ret;

	return 0;
}

static int histb_drm_create_crtc(struct drm_device *drm)
{
	struct histb_drm *priv = to_histb_drm(drm);
	int ret;

	ret = drm_crtc_init_with_planes(drm, &priv->crtc, &priv->primary_plane,
					NULL, &histb_drm_crtc_funcs, NULL);
	if (ret)
		return ret;

	drm_crtc_helper_add(&priv->crtc, &histb_drm_crtc_helper_funcs);
	histb_vdp_set_vblank_handler(priv->vdp, histb_drm_vblank_handler, priv);

	return 0;
}

static int histb_drm_create_encoder(struct drm_device *drm)
{
	struct histb_drm *priv = to_histb_drm(drm);
	int ret;

	ret = drm_encoder_init(drm, &priv->encoder, &histb_drm_encoder_funcs,
			       DRM_MODE_ENCODER_TMDS, NULL);
	if (ret)
		return ret;

	priv->encoder.possible_crtcs = drm_crtc_mask(&priv->crtc);

	return 0;
}

static const struct drm_connector_funcs histb_drm_wb_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs histb_drm_wb_connector_helper_funcs = {
	.get_modes = histb_drm_writeback_connector_get_modes,
	.atomic_check = histb_drm_writeback_atomic_check,
	.atomic_commit = histb_drm_writeback_atomic_commit,
};

static int histb_drm_create_hdmi_connector(struct drm_device *drm)
{
#if !IS_ENABLED(CONFIG_DRM_BRIDGE_CONNECTOR)
	(void)drm;
	return -ENODEV;
#else
	struct histb_drm *priv = to_histb_drm(drm);
	struct drm_connector *connector;
	int ret;

	ret = drm_bridge_attach(&priv->encoder, priv->hdmi_bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return ret;

	connector = drm_bridge_connector_init(drm, &priv->encoder);
	if (IS_ERR(connector))
		return PTR_ERR(connector);

	connector->doublescan_allowed = false;
	priv->connector = connector;

	/*
	 * No colorspace property: it expects a non-default colorimetry mask,
	 * and this pipeline supports only DRM_MODE_COLORIMETRY_DEFAULT.
	 */

	ret = drm_connector_attach_broadcast_rgb_property(connector);
	if (ret)
		return ret;

	return drm_connector_attach_encoder(connector, &priv->encoder);
#endif
}

static int histb_drm_create_connector(struct drm_device *drm)
{
	return histb_drm_create_hdmi_connector(drm);
}

static int histb_drm_create_writeback_connector(struct drm_device *drm)
{
	struct histb_drm *priv = to_histb_drm(drm);
	struct drm_writeback_connector *wb = &priv->wb_connector;
	int ret;

	ret = drmm_encoder_init(drm, &priv->wb_encoder, NULL,
				DRM_MODE_ENCODER_VIRTUAL, NULL);
	if (ret)
		return ret;

	priv->wb_encoder.possible_crtcs = drm_crtc_mask(&priv->crtc);
	priv->wb_encoder.possible_clones = drm_encoder_mask(&priv->wb_encoder);

	ret = drmm_writeback_connector_init(drm, wb,
					    &histb_drm_wb_connector_funcs,
					    &priv->wb_encoder,
					    histb_drm_primary_formats,
					    ARRAY_SIZE(histb_drm_primary_formats));
	if (ret)
		return ret;

	drm_connector_helper_add(&wb->base, &histb_drm_wb_connector_helper_funcs);
	wb->base.doublescan_allowed = false;
	wb->base.interlace_allowed = false;

	return 0;
}

static int histb_drm_create_kms(struct drm_device *drm)
{
	int ret;

	ret = histb_drm_create_primary_plane(drm);
	if (ret)
		return ret;

	ret = histb_drm_create_crtc(drm);
	if (ret)
		return ret;

	ret = histb_drm_create_video_plane(drm);
	if (ret)
		return ret;

	ret = histb_drm_create_encoder(drm);
	if (ret)
		return ret;

	ret = histb_drm_create_connector(drm);
	if (ret)
		return ret;

	return histb_drm_create_writeback_connector(drm);
}

static int histb_drm_attach_pipeline(struct drm_device *drm)
{
	struct histb_drm *priv = to_histb_drm(drm);
	struct device_node *np;
	int ret;

	np = histb_drm_port_parent(drm, HISTB_DRM_PORT_VDP);
	if (!np)
		return dev_err_probe(drm->dev, -EINVAL,
				     "no VDP entry in the ports property\n");

	priv->vdp = histb_vdp_get_from_node(np);
	if (IS_ERR(priv->vdp)) {
		of_node_put(np);
		return dev_err_probe(drm->dev, PTR_ERR(priv->vdp),
				     "VDP provider is not ready\n");
	}

	of_node_put(np);

	np = histb_drm_port_parent(drm, HISTB_DRM_PORT_HDMI);
	if (!np)
		return dev_err_probe(drm->dev, -EINVAL,
				     "no HDMI entry in the ports property\n");

	priv->hdmi_bridge = of_drm_find_bridge(np);
	of_node_put(np);
	if (!priv->hdmi_bridge)
		return dev_err_probe(drm->dev, -EPROBE_DEFER,
				     "HDMI bridge is not ready\n");

	ret = histb_vdp_pipeline_setup_output(priv->vdp);
	if (ret)
		return dev_err_probe(drm->dev, ret,
				     "failed to set output type\n");

	ret = histb_vdp_pipeline_prepare(priv->vdp);
	if (ret)
		return dev_err_probe(drm->dev, ret, "VDP is not ready yet\n");

	return 0;
}

static int histb_drm_load(struct drm_device *drm)
{
	int ret;

	ret = histb_drm_attach_pipeline(drm);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(drm->dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(drm->dev, ret, "failed to set DMA mask\n");

	ret = histb_drm_setup_mode_config(drm);
	if (ret)
		return ret;

	ret = histb_drm_create_kms(drm);
	if (ret)
		return ret;

	ret = drm_vblank_init(drm, drm->mode_config.num_crtc);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);
	return 0;
}

DEFINE_DRM_GEM_DMA_FOPS(histb_drm_fops);

static const struct drm_driver histb_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.dumb_create = histb_drm_dumb_create,
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_import_sg_table = drm_gem_dma_prime_import_sg_table,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.debugfs_init = histb_drm_debugfs_init,
	.fops = &histb_drm_fops,
	.name = "histb-drm",
	.desc = "HiSilicon STB DRM",
	.major = 1,
	.minor = 0,
};

static int histb_drm_probe(struct platform_device *pdev)
{
	struct histb_drm *priv;
	int ret;

	priv = devm_drm_dev_alloc(&pdev->dev, &histb_drm_driver,
				  struct histb_drm, drm);
	if (IS_ERR(priv))
		return PTR_ERR(priv);


	ret = histb_drm_load(&priv->drm);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, &priv->drm);

	ret = drm_dev_register(&priv->drm, 0);
	if (ret)
		return ret;

	drm_client_setup(&priv->drm, NULL);
	dev_info(&pdev->dev, "registered atomic DRM/KMS on HDMI path\n");
	return 0;
}

static void histb_drm_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	struct histb_drm *priv = to_histb_drm(drm);

	drm_atomic_helper_shutdown(drm);
	histb_vdp_set_vblank_handler(priv->vdp, NULL, NULL);
	drm_dev_unregister(drm);
}

static int __maybe_unused histb_drm_pm_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(drm);
}

static int __maybe_unused histb_drm_pm_resume(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(drm);
}

static const struct of_device_id histb_drm_of_match[] = {
	{ .compatible = "hisilicon,hi3798mv100-vdp-drm" },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_drm_of_match);

static const struct dev_pm_ops histb_drm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(histb_drm_pm_suspend,
				histb_drm_pm_resume)
};

static struct platform_driver histb_drm_platform_driver = {
	.probe = histb_drm_probe,
	.remove = histb_drm_remove,
	.driver = {
		.name = "histb-drm",
		.of_match_table = histb_drm_of_match,
		.pm = pm_ptr(&histb_drm_pm_ops),
	},
};
drm_module_platform_driver(histb_drm_platform_driver);

MODULE_AUTHOR("Hisilicon community");
MODULE_DESCRIPTION("HiSilicon STB atomic DRM/KMS skeleton driver");
MODULE_LICENSE("GPL");
