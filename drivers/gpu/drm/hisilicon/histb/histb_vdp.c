// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon STB VDP driver for hi3798mv100.
 *
 * Scanout for GFX0 (graphics) and VID0/VP0 (video).
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <drm/drm_blend.h>
#include <drm/drm_modes.h>

#include "histb_pipeline.h"

#define HISTB_VDP_VOCTRL			0x0000
#define HISTB_VDP_VOINTSTA			0x0004
#define HISTB_VDP_VOMSKINTSTA			0x0008
#define HISTB_VDP_VOINTMSK			0x000c
#define HISTB_VDP_INT_DHD0_VTTHD1		BIT(0)
#define HISTB_VDP_V0_CTRL			0x0800
#define HISTB_VDP_V0_UPD			0x0804
#define HISTB_VDP_V0_IRESO			0x0828
#define HISTB_VDP_V0_ORESO			0x082c
#define HISTB_VDP_V0_CBMPARA			0x0838
#define HISTB_VDP_V0_DFPOS			0x0860
#define HISTB_VDP_V0_DLPOS			0x0864
#define HISTB_VDP_V0_VFPOS			0x0868
#define HISTB_VDP_V0_VLPOS			0x086c
#define HISTB_VDP_V0_ALPHA			0x0874
#define HISTB_VDP_V0_P0RESO			0x0a00
#define HISTB_VDP_V0_P0LADDR			0x0a04
#define HISTB_VDP_V0_P0CADDR			0x0a08
#define HISTB_VDP_V0_CSC_IDC			0x0880
#define HISTB_VDP_V0_CSC_ODC			0x0884
#define HISTB_VDP_V0_CSC_IODC			0x0888
#define HISTB_VDP_V0_CSC_P0			0x088c
#define HISTB_VDP_V0_CSC_P1			0x0890
#define HISTB_VDP_V0_CSC_P2			0x0894
#define HISTB_VDP_V0_CSC_P3			0x0898
#define HISTB_VDP_V0_CSC_P4			0x089c
#define HISTB_VDP_V0_P0STRIDE			0x0a0c
#define HISTB_VDP_V0_P0VFPOS			0x0a10
#define HISTB_VDP_V0_P0VLPOS			0x0a14
#define HISTB_VDP_V0_NADDR			0x0e00
#define HISTB_VDP_V0_NCADDR			0x0e04
#define HISTB_VDP_V0_MULTI_MODE			0x0e30
#define HISTB_VDP_V0_16REGIONENL		0x0f00
#define HISTB_VDP_VP0_CTRL			0x4000
#define HISTB_VDP_VP0_UPD			0x4004
#define HISTB_VDP_VP0_IRESO			0x4020
#define HISTB_VDP_VP0_DFPOS			0x4200
#define HISTB_VDP_VP0_DLPOS			0x4204
#define HISTB_VDP_VP0_VFPOS			0x4208
#define HISTB_VDP_VP0_VLPOS			0x420c
#define HISTB_VDP_DHD0_CTRL			0xc000
#define HISTB_VDP_DHD0_VSYNC			0xc004
#define HISTB_VDP_DHD0_HSYNC1			0xc008
#define HISTB_VDP_DHD0_HSYNC2			0xc00c
#define HISTB_VDP_DHD0_VPLUS			0xc010
#define HISTB_VDP_DHD0_PWR			0xc014
#define HISTB_VDP_DHD0_VTTHD			0xc01c
#define HISTB_VDP_VTTHD_VTMGTHD1		GENMASK(12, 0)
#define HISTB_VDP_VTTHD_THD1_MODE		BIT(15)
#define HISTB_VDP_DHD0_SYNC_INV			0xc020
#define HISTB_VDP_G0_CTRL			0x6000
#define HISTB_VDP_G0_UPD			0x6004
#define HISTB_VDP_G0_ADDR			0x6010
#define HISTB_VDP_G0_NADDR			0x6018
#define HISTB_VDP_G0_STRIDE			0x601c
#define HISTB_VDP_G0_IRESO			0x6020
#define HISTB_VDP_G0_CBMPARA			0x6030
#define HISTB_VDP_G0_DFPOS			0x6080
#define HISTB_VDP_G0_DLPOS			0x6084
#define HISTB_VDP_G0_VFPOS			0x6088
#define HISTB_VDP_G0_VLPOS			0x608c
#define HISTB_VDP_GP0_CTRL			0x9000
#define HISTB_VDP_GP0_UPD			0x9004
#define HISTB_VDP_GP0_ORESO			0x9008
#define HISTB_VDP_GP0_IRESO			0x900c
#define HISTB_VDP_GP0_GALPHA			0x9020
#define HISTB_VDP_GP0_DFPOS			0x9100
#define HISTB_VDP_GP0_DLPOS			0x9104
#define HISTB_VDP_GP0_VFPOS			0x9108
#define HISTB_VDP_GP0_VLPOS			0x910c
#define HISTB_VDP_MIXG0_MIX			0xb208
#define HISTB_VDP_CBM_MIX1			0xb408
#define HISTB_VDP_CBM_ATTR			0xb440
#define HISTB_VDP_VO_MUX				0x0100
#define HISTB_VDP_VO_MUX_DAC			0x0104

#define HISTB_VDP_VOCTRL_CK_GT_EN		BIT(31)

#define HISTB_VDP_DHD_CTRL_REGUP		BIT(0)
#define HISTB_VDP_DHD_CTRL_DISP_MODE		GENMASK(3, 1)
#define HISTB_VDP_DHD_CTRL_IOP			BIT(4)
#define HISTB_VDP_DHD_CTRL_GMM_EN		BIT(13)
#define HISTB_VDP_DHD_CTRL_HDMI_MODE		BIT(14)
#define HISTB_VDP_DHD_CTRL_FPGA_LMT_EN		BIT(27)
#define HISTB_VDP_DHD_CTRL_P2I_EN		BIT(28)
#define HISTB_VDP_DHD_CTRL_CBAR_SEL		BIT(29)
#define HISTB_VDP_DHD_CTRL_CBAR_EN		BIT(30)
#define HISTB_VDP_DHD_CTRL_INTF_EN		BIT(31)

#define HISTB_VDP_HSYNC1_HACT			GENMASK(15, 0)
#define HISTB_VDP_HSYNC1_HBB			GENMASK(31, 16)
#define HISTB_VDP_HSYNC2_HFB			GENMASK(15, 0)
#define HISTB_VDP_HSYNC2_HMID			GENMASK(31, 16)
#define HISTB_VDP_VSYNC_VACT			GENMASK(11, 0)
#define HISTB_VDP_VSYNC_VBB			GENMASK(19, 12)
#define HISTB_VDP_VSYNC_VFB			GENMASK(27, 20)
#define HISTB_VDP_VPLUS_BVACT			GENMASK(11, 0)
#define HISTB_VDP_VPLUS_BVBB			GENMASK(19, 12)
#define HISTB_VDP_VPLUS_BVFB			GENMASK(27, 20)
#define HISTB_VDP_PWR_HPW			GENMASK(15, 0)
#define HISTB_VDP_PWR_VPW			GENMASK(23, 16)
#define HISTB_VDP_V0_CTRL_IFMT			GENMASK(3, 0)
#define HISTB_VDP_V0_CTRL_NOSEC_FLAG		BIT(10)
/*
 * Chroma byte order within the interleaved plane: set for NV21 (Cr first),
 * clear for NV12. Only NV21 is advertised, so it is always set.
 */
#define HISTB_VDP_V0_CTRL_UV_ORDER		BIT(11)
#define HISTB_VDP_V0_CTRL_MUTE_EN		BIT(27)
/*
 * Fetch enables, one per plane. Both are needed for a semi-planar format -
 * enabling only the luma surface leaves the layer with no data to convert and
 * it emits a flat colour, which looks exactly like a broken colour matrix.
 */
#define HISTB_VDP_V0_CTRL_SURFACE_C_EN		BIT(30)
#define HISTB_VDP_V0_CTRL_SURFACE_EN		BIT(31)
#define HISTB_VDP_V0_UPD_REGUP			BIT(0)
#define HISTB_VDP_V0_IRESO_IW			GENMASK(11, 0)
#define HISTB_VDP_V0_IRESO_IH			GENMASK(23, 12)
#define HISTB_VDP_V0_ORESO_OW			GENMASK(11, 0)
#define HISTB_VDP_V0_ORESO_OH			GENMASK(23, 12)
#define HISTB_VDP_V0_CBMPARA_GALPHA		GENMASK(7, 0)
#define HISTB_VDP_V0_ALPHA_VBK_ALPHA		GENMASK(7, 0)
#define HISTB_VDP_V0_DFPOS_X			GENMASK(11, 0)
#define HISTB_VDP_V0_DFPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_V0_DLPOS_X			GENMASK(11, 0)
#define HISTB_VDP_V0_DLPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_V0_VFPOS_X			GENMASK(11, 0)
#define HISTB_VDP_V0_VFPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_V0_VLPOS_X			GENMASK(11, 0)
#define HISTB_VDP_V0_VLPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_V0_P0RESO_W			GENMASK(11, 0)
#define HISTB_VDP_V0_CSC_IDC_DC0		GENMASK(10, 0)
#define HISTB_VDP_V0_CSC_IDC_DC1		GENMASK(21, 11)
#define HISTB_VDP_V0_CSC_IDC_EN			BIT(22)
#define HISTB_VDP_V0_CSC_ODC_DC0		GENMASK(10, 0)
#define HISTB_VDP_V0_CSC_ODC_DC1		GENMASK(21, 11)
#define HISTB_VDP_V0_CSC_IODC_IDC2		GENMASK(10, 0)
#define HISTB_VDP_V0_CSC_IODC_ODC2		GENMASK(21, 11)
#define HISTB_VDP_V0_CSC_LO			GENMASK(14, 0)
#define HISTB_VDP_V0_CSC_HI			GENMASK(30, 16)
#define HISTB_VDP_V0_P0STRIDE_LUMA		GENMASK(15, 0)
#define HISTB_VDP_V0_P0STRIDE_CHROMA		GENMASK(31, 16)
#define HISTB_VDP_V0_P0VFPOS_X			GENMASK(11, 0)
#define HISTB_VDP_V0_P0VFPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_V0_P0VLPOS_X			GENMASK(11, 0)
#define HISTB_VDP_V0_P0VLPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_V0_MULTI_MODE_MRG_MODE	BIT(0)
#define HISTB_VDP_V0_16REGIONENL_P0_EN		BIT(0)
#define HISTB_VDP_VP0_CTRL_GALPHA		GENMASK(7, 0)
#define HISTB_VDP_VP0_CTRL_MUTE_EN		BIT(8)
#define HISTB_VDP_VP0_UPD_REGUP			BIT(0)
#define HISTB_VDP_VP0_IRESO_IW			GENMASK(11, 0)
#define HISTB_VDP_VP0_IRESO_IH			GENMASK(23, 12)
#define HISTB_VDP_VP0_DFPOS_X			GENMASK(11, 0)
#define HISTB_VDP_VP0_DFPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_VP0_DLPOS_X			GENMASK(11, 0)
#define HISTB_VDP_VP0_DLPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_VP0_VFPOS_X			GENMASK(11, 0)
#define HISTB_VDP_VP0_VFPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_VP0_VLPOS_X			GENMASK(11, 0)
#define HISTB_VDP_VP0_VLPOS_Y			GENMASK(23, 12)

#define HISTB_VDP_VID_IFMT_SP_420		0x3
#define HISTB_VDP_G0_CTRL_IFMT			GENMASK(7, 0)
#define HISTB_VDP_G0_CTRL_READ_MODE		BIT(26)
#define HISTB_VDP_G0_CTRL_MUTE_EN		BIT(28)
#define HISTB_VDP_G0_CTRL_NOSEC_FLAG		BIT(30)
#define HISTB_VDP_G0_CTRL_SURFACE_EN		BIT(31)
#define HISTB_VDP_G0_UPD_REGUP			BIT(0)
#define HISTB_VDP_G0_STRIDE_SURFACE		GENMASK(15, 0)
#define HISTB_VDP_G0_IRESO_IW			GENMASK(11, 0)
#define HISTB_VDP_G0_IRESO_IH			GENMASK(23, 12)
#define HISTB_VDP_G0_CBMPARA_GALPHA		GENMASK(7, 0)
#define HISTB_VDP_G0_CBMPARA_PALPHA_RANGE	BIT(8)
#define HISTB_VDP_G0_CBMPARA_PALPHA_EN		BIT(12)
#define HISTB_VDP_G0_CBMPARA_PREMULT_EN	BIT(13)
#define HISTB_VDP_G0_CBMPARA_KEY_EN		BIT(14)
#define HISTB_VDP_G0_CBMPARA_KEY_MODE		BIT(15)
#define HISTB_VDP_G0_DFPOS_X			GENMASK(11, 0)
#define HISTB_VDP_G0_DFPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_G0_DLPOS_X			GENMASK(11, 0)
#define HISTB_VDP_G0_DLPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_G0_VFPOS_X			GENMASK(11, 0)
#define HISTB_VDP_G0_VFPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_G0_VLPOS_X			GENMASK(11, 0)
#define HISTB_VDP_G0_VLPOS_Y			GENMASK(23, 12)

#define HISTB_VDP_GP0_CTRL_READ_MODE		BIT(31)
#define HISTB_VDP_GP0_CTRL_MUTE_EN		BIT(30)
#define HISTB_VDP_GP0_UPD_REGUP			BIT(0)
#define HISTB_VDP_GP0_ORESO_OW			GENMASK(11, 0)
#define HISTB_VDP_GP0_ORESO_OH			GENMASK(23, 12)
#define HISTB_VDP_GP0_IRESO_IW			GENMASK(11, 0)
#define HISTB_VDP_GP0_IRESO_IH			GENMASK(23, 12)
#define HISTB_VDP_GP0_GALPHA_VAL		GENMASK(7, 0)
#define HISTB_VDP_GP0_DFPOS_X			GENMASK(11, 0)
#define HISTB_VDP_GP0_DFPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_GP0_DLPOS_X			GENMASK(11, 0)
#define HISTB_VDP_GP0_DLPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_GP0_VFPOS_X			GENMASK(11, 0)
#define HISTB_VDP_GP0_VFPOS_Y			GENMASK(23, 12)
#define HISTB_VDP_GP0_VLPOS_X			GENMASK(11, 0)
#define HISTB_VDP_GP0_VLPOS_Y			GENMASK(23, 12)

#define HISTB_VDP_MIXG0_MIX_PRIO0		GENMASK(3, 0)
#define HISTB_VDP_MIXG0_MIX_PRIO1		GENMASK(7, 4)
#define HISTB_VDP_MIXG0_MIX_PRIO2		GENMASK(11, 8)
#define HISTB_VDP_MIXG0_MIX_PRIO3		GENMASK(15, 12)

#define HISTB_VDP_CBM_MIX1_PRIO0		GENMASK(3, 0)
#define HISTB_VDP_CBM_MIX1_PRIO1		GENMASK(7, 4)

#define HISTB_VDP_CBM_ATTR_SUR_ATTR0		BIT(0)
#define HISTB_VDP_CBM_ATTR_SUR_ATTR1		BIT(1)
#define HISTB_VDP_CBM_ATTR_SUR_ATTR2		BIT(2)
#define HISTB_VDP_CBM_ATTR_SUR_ATTR3		BIT(3)
#define HISTB_VDP_CBM_ATTR_SUR_ATTR4		BIT(4)

#define HISTB_VDP_VO_MUX_HDMI_SEL		BIT(3)

#define HISTB_VDP_CRG54				0x00d8
#define HISTB_VDP_CRG_PLL10			0x0028
#define HISTB_VDP_CRG_PLL11			0x002c
#define HISTB_VDP_CRG54_VO_SD_CLK_SEL		GENMASK(13, 12)
#define HISTB_VDP_CRG54_VO_SD_CLK_DIV		GENMASK(15, 14)
#define HISTB_VDP_CRG54_VO_HD_CLK_SEL		GENMASK(17, 16)
#define HISTB_VDP_CRG54_VO_HD_CLK_DIV		GENMASK(19, 18)
#define HISTB_VDP_CRG54_HDMI_CLK_SEL		BIT(26)
#define HISTB_VDP_CRG54_VO_SD_HDMI_CLK_SEL	BIT(27)
#define HISTB_VDP_CRG54_VDP_CLK_SEL		BIT(28)
#define HISTB_VDP_CRG54_VO_HD_HDMI_CLK_SEL	BIT(29)

enum histb_vdp_clk_id {
	HISTB_VDP_CLK_BUS,
	HISTB_VDP_CLK_VO,
	HISTB_VDP_CLK_SD,
	HISTB_VDP_CLK_SDATE,
	HISTB_VDP_CLK_HD,
	HISTB_VDP_CLK_HDATE,
	HISTB_VDP_NUM_CLKS,
};

struct histb_vdp_timing {
	u32 vact;
	u32 vbb;
	u32 vfb;
	u32 hact;
	u32 hbb;
	u32 hfb;
	u32 bvact;
	u32 bvbb;
	u32 bvfb;
	u32 hpw;
	u32 vpw;
	u32 hmid;
	u32 pll_ctrl0;
	u32 pll_ctrl1;
	bool progressive;
};

struct histb_vdp_video_cfg {
	u32 src_x;
	u32 src_y;
	u32 src_width;
	u32 src_height;
	u32 dst_x;
	u32 dst_y;
	u32 dst_width;
	u32 dst_height;
	u32 luma_stride;
	u32 chroma_stride;
};

struct histb_vdp {
	struct device *dev;
	void __iomem *regs;
	int irq;
	/*
	 * Frame interrupt hand-off to the DRM side. Written once while the
	 * IRQ is disabled, read from hard IRQ context; data is published
	 * before the callback and torn down in the opposite order.
	 */
	void (*vblank_cb)(void *data);
	void *vblank_data;
	struct regmap *crg;
	struct reset_control *rst;
	struct clk_bulk_data clks[HISTB_VDP_NUM_CLKS];
	struct mutex lock;
	bool pm_active;
	bool irq_enabled;
	bool cbar_enabled;
	/*
	 * Last geometry applied. Players reprogram the plane every frame with
	 * only the buffer address changed, and rewriting the rest at frame
	 * rate visibly disturbs the output, so reapply only on a real change.
	 */
	struct histb_vdp_video_cfg video_cfg;
	bool video_cfg_valid;

	enum drm_color_encoding video_color_encoding;
	enum drm_color_range video_color_range;
};

struct histb_vdp_gfx_cfg {
	u32 width;
	u32 height;
	u32 stride;
	enum histb_vdp_gfx_ifmt ifmt;
};

static const struct histb_vdp_timing histb_vdp_720p60 = {
	.vact = 720,
	.vbb = 25,
	.vfb = 5,
	.hact = 1280,
	.hbb = 260,
	.hfb = 110,
	.bvact = 1,
	.bvbb = 1,
	.bvfb = 1,
	.hpw = 40,
	.vpw = 5,
	.hmid = 1,
	.pll_ctrl0 = 0x14000000,
	.pll_ctrl1 = 0x02002063,
	.progressive = true,
};

#define HISTB_VDP_PLL_CTRL0_74M25			0x14000000
#define HISTB_VDP_PLL_CTRL0_148M5			0x12000000
#define HISTB_VDP_PLL_CTRL1_STD				0x02002063

static const char * const histb_vdp_clk_names[HISTB_VDP_NUM_CLKS] = {
	[HISTB_VDP_CLK_BUS] = "bus",
	[HISTB_VDP_CLK_VO] = "vo",
	[HISTB_VDP_CLK_SD] = "sd",
	[HISTB_VDP_CLK_SDATE] = "sdate",
	[HISTB_VDP_CLK_HD] = "hd",
	[HISTB_VDP_CLK_HDATE] = "hdate",
};

static inline u32 histb_vdp_read(struct histb_vdp *vdp, u32 reg)
{
	return readl_relaxed(vdp->regs + reg);
}

/* Diagnostic: log every register write. Off by default. */
static bool histb_vdp_trace;
module_param_named(trace, histb_vdp_trace, bool, 0644);
MODULE_PARM_DESC(trace, "Log every VDP register write (very noisy)");

static inline void histb_vdp_write(struct histb_vdp *vdp, u32 reg, u32 val)
{
	if (unlikely(histb_vdp_trace))
		pr_info("vdpw %04x %08x\n", reg, val);

	writel_relaxed(val, vdp->regs + reg);
}

static u32 histb_vdp_minus_one(u32 value)
{
	return value ? value - 1 : 0;
}

static u8 histb_vdp_alpha_to_hw(u16 alpha)
{
	return DIV_ROUND_CLOSEST(alpha * 0xff, DRM_BLEND_ALPHA_OPAQUE);
}

static bool histb_vdp_rect_fits(u32 x, u32 y, u32 width, u32 height,
				u32 xf_mask, u32 yf_mask,
				u32 xl_mask, u32 yl_mask)
{
	u32 xl;
	u32 yl;

	if (!width || !height)
		return false;

	if (x > U32_MAX - (width - 1) || y > U32_MAX - (height - 1))
		return false;

	xl = x + width - 1;
	yl = y + height - 1;

	return FIELD_FIT(xf_mask, x) &&
	       FIELD_FIT(yf_mask, y) &&
	       FIELD_FIT(xl_mask, xl) &&
	       FIELD_FIT(yl_mask, yl);
}

static int histb_vdp_configure_crg(struct histb_vdp *vdp)
{
	unsigned int mask;
	unsigned int val;

	mask = HISTB_VDP_CRG54_VO_SD_CLK_SEL |
	       HISTB_VDP_CRG54_VO_SD_CLK_DIV |
	       HISTB_VDP_CRG54_VO_HD_CLK_SEL |
	       HISTB_VDP_CRG54_VO_HD_CLK_DIV |
	       HISTB_VDP_CRG54_HDMI_CLK_SEL |
	       HISTB_VDP_CRG54_VO_SD_HDMI_CLK_SEL |
	       HISTB_VDP_CRG54_VDP_CLK_SEL |
	       HISTB_VDP_CRG54_VO_HD_HDMI_CLK_SEL;

	val = FIELD_PREP(HISTB_VDP_CRG54_VO_SD_CLK_SEL, 0) |
	      FIELD_PREP(HISTB_VDP_CRG54_VO_SD_CLK_DIV, 2) |
	      FIELD_PREP(HISTB_VDP_CRG54_VO_HD_CLK_SEL, 1) |
	      FIELD_PREP(HISTB_VDP_CRG54_VO_HD_CLK_DIV, 0) |
	      HISTB_VDP_CRG54_HDMI_CLK_SEL;

	return regmap_update_bits(vdp->crg, HISTB_VDP_CRG54, mask, val);
}

static int histb_vdp_pulse_reset(struct histb_vdp *vdp)
{
	int ret;

	ret = reset_control_assert(vdp->rst);
	if (ret)
		return ret;

	usleep_range(5000, 6000);

	ret = reset_control_deassert(vdp->rst);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	return 0;
}

static int histb_vdp_program_pll(struct histb_vdp *vdp,
				 const struct histb_vdp_timing *timing)
{
	int ret;

	/*
	 * Mode-dependent HPLL programming from hi3798mv100 SDK tables:
	 * CRG_PLL10.hpll_ctrl0 (bits 30:0) + CRG_PLL11.hpll_ctrl1 (bits 26:0).
	 */
	ret = regmap_update_bits(vdp->crg, HISTB_VDP_CRG_PLL10,
				 GENMASK(30, 0), timing->pll_ctrl0);
	if (ret)
		return ret;

	ret = regmap_update_bits(vdp->crg, HISTB_VDP_CRG_PLL11,
				 GENMASK(26, 0), timing->pll_ctrl1);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	return 0;
}

static void histb_vdp_hdmi_path_setup_locked(struct histb_vdp *vdp)
{
	u32 val;

	val = histb_vdp_read(vdp, HISTB_VDP_VO_MUX);
	val &= ~HISTB_VDP_VO_MUX_HDMI_SEL;
	histb_vdp_write(vdp, HISTB_VDP_VO_MUX, val);
}

static int histb_vdp_runtime_ensure_active(struct histb_vdp *vdp)
{
	int ret;

	if (READ_ONCE(vdp->pm_active))
		return 0;

	ret = pm_runtime_resume_and_get(vdp->dev);
	if (ret < 0)
		return ret;

	WRITE_ONCE(vdp->pm_active, true);
	return 0;
}

static void histb_vdp_runtime_release(struct histb_vdp *vdp)
{
	if (!READ_ONCE(vdp->pm_active))
		return;

	WRITE_ONCE(vdp->pm_active, false);
	pm_runtime_mark_last_busy(vdp->dev);
	pm_runtime_put_autosuspend(vdp->dev);
}

static void histb_vdp_set_output(struct histb_vdp *vdp, bool enable)
{
	u32 val;

	val = histb_vdp_read(vdp, HISTB_VDP_DHD0_CTRL);
	if (enable) {
		val |= HISTB_VDP_DHD_CTRL_INTF_EN;
		if (vdp->cbar_enabled) {
			val |= HISTB_VDP_DHD_CTRL_CBAR_EN;
			val &= ~HISTB_VDP_DHD_CTRL_CBAR_SEL;
		} else {
			val &= ~HISTB_VDP_DHD_CTRL_CBAR_EN;
		}
	} else {
		val &= ~(HISTB_VDP_DHD_CTRL_CBAR_EN | HISTB_VDP_DHD_CTRL_INTF_EN);
	}
	val |= HISTB_VDP_DHD_CTRL_REGUP;
	histb_vdp_write(vdp, HISTB_VDP_DHD0_CTRL, val);
}

static void histb_vdp_gfx_regup(struct histb_vdp *vdp)
{
	histb_vdp_write(vdp, HISTB_VDP_G0_UPD, HISTB_VDP_G0_UPD_REGUP);
}

static void histb_vdp_gp0_regup(struct histb_vdp *vdp)
{
	histb_vdp_write(vdp, HISTB_VDP_GP0_UPD, HISTB_VDP_GP0_UPD_REGUP);
}

static void histb_vdp_vid_regup(struct histb_vdp *vdp)
{
	histb_vdp_write(vdp, HISTB_VDP_V0_UPD, HISTB_VDP_V0_UPD_REGUP);
}

static void histb_vdp_vp0_regup(struct histb_vdp *vdp)
{
	histb_vdp_write(vdp, HISTB_VDP_VP0_UPD, HISTB_VDP_VP0_UPD_REGUP);
}

static void histb_vdp_gfx_set_rect(struct histb_vdp *vdp, u32 width, u32 height)
{
	u32 w = histb_vdp_minus_one(width);
	u32 h = histb_vdp_minus_one(height);
	u32 val;

	val = FIELD_PREP(HISTB_VDP_G0_IRESO_IW, w) |
	      FIELD_PREP(HISTB_VDP_G0_IRESO_IH, h);
	histb_vdp_write(vdp, HISTB_VDP_G0_IRESO, val);

	val = FIELD_PREP(HISTB_VDP_G0_DFPOS_X, 0) |
	      FIELD_PREP(HISTB_VDP_G0_DFPOS_Y, 0);
	histb_vdp_write(vdp, HISTB_VDP_G0_DFPOS, val);

	val = FIELD_PREP(HISTB_VDP_G0_DLPOS_X, w) |
	      FIELD_PREP(HISTB_VDP_G0_DLPOS_Y, h);
	histb_vdp_write(vdp, HISTB_VDP_G0_DLPOS, val);

	val = FIELD_PREP(HISTB_VDP_G0_VFPOS_X, 0) |
	      FIELD_PREP(HISTB_VDP_G0_VFPOS_Y, 0);
	histb_vdp_write(vdp, HISTB_VDP_G0_VFPOS, val);

	val = FIELD_PREP(HISTB_VDP_G0_VLPOS_X, w) |
	      FIELD_PREP(HISTB_VDP_G0_VLPOS_Y, h);
	histb_vdp_write(vdp, HISTB_VDP_G0_VLPOS, val);
}

static void histb_vdp_gp0_set_rect(struct histb_vdp *vdp, u32 width, u32 height)
{
	u32 w = histb_vdp_minus_one(width);
	u32 h = histb_vdp_minus_one(height);
	u32 val;

	val = FIELD_PREP(HISTB_VDP_GP0_IRESO_IW, w) |
	      FIELD_PREP(HISTB_VDP_GP0_IRESO_IH, h);
	histb_vdp_write(vdp, HISTB_VDP_GP0_IRESO, val);

	val = FIELD_PREP(HISTB_VDP_GP0_ORESO_OW, w) |
	      FIELD_PREP(HISTB_VDP_GP0_ORESO_OH, h);
	histb_vdp_write(vdp, HISTB_VDP_GP0_ORESO, val);

	val = FIELD_PREP(HISTB_VDP_GP0_DFPOS_X, 0) |
	      FIELD_PREP(HISTB_VDP_GP0_DFPOS_Y, 0);
	histb_vdp_write(vdp, HISTB_VDP_GP0_DFPOS, val);

	val = FIELD_PREP(HISTB_VDP_GP0_DLPOS_X, w) |
	      FIELD_PREP(HISTB_VDP_GP0_DLPOS_Y, h);
	histb_vdp_write(vdp, HISTB_VDP_GP0_DLPOS, val);

	val = FIELD_PREP(HISTB_VDP_GP0_VFPOS_X, 0) |
	      FIELD_PREP(HISTB_VDP_GP0_VFPOS_Y, 0);
	histb_vdp_write(vdp, HISTB_VDP_GP0_VFPOS, val);

	val = FIELD_PREP(HISTB_VDP_GP0_VLPOS_X, w) |
	      FIELD_PREP(HISTB_VDP_GP0_VLPOS_Y, h);
	histb_vdp_write(vdp, HISTB_VDP_GP0_VLPOS, val);
}

static void histb_vdp_v0_set_rect(struct histb_vdp *vdp,
				  u32 src_width, u32 src_height,
				  u32 dst_x, u32 dst_y,
				  u32 dst_width, u32 dst_height)
{
	u32 src_w = histb_vdp_minus_one(src_width);
	u32 src_h = histb_vdp_minus_one(src_height);
	u32 dst_w = histb_vdp_minus_one(dst_width);
	u32 dst_h = histb_vdp_minus_one(dst_height);
	u32 dst_xl = dst_x + dst_w;
	u32 dst_yl = dst_y + dst_h;
	u32 val;

	val = FIELD_PREP(HISTB_VDP_V0_IRESO_IW, src_w) |
	      FIELD_PREP(HISTB_VDP_V0_IRESO_IH, src_h);
	histb_vdp_write(vdp, HISTB_VDP_V0_IRESO, val);

	val = FIELD_PREP(HISTB_VDP_V0_ORESO_OW, dst_w) |
	      FIELD_PREP(HISTB_VDP_V0_ORESO_OH, dst_h);
	histb_vdp_write(vdp, HISTB_VDP_V0_ORESO, val);

	val = FIELD_PREP(HISTB_VDP_V0_DFPOS_X, dst_x) |
	      FIELD_PREP(HISTB_VDP_V0_DFPOS_Y, dst_y);
	histb_vdp_write(vdp, HISTB_VDP_V0_DFPOS, val);

	val = FIELD_PREP(HISTB_VDP_V0_DLPOS_X, dst_xl) |
	      FIELD_PREP(HISTB_VDP_V0_DLPOS_Y, dst_yl);
	histb_vdp_write(vdp, HISTB_VDP_V0_DLPOS, val);

	val = FIELD_PREP(HISTB_VDP_V0_VFPOS_X, dst_x) |
	      FIELD_PREP(HISTB_VDP_V0_VFPOS_Y, dst_y);
	histb_vdp_write(vdp, HISTB_VDP_V0_VFPOS, val);

	val = FIELD_PREP(HISTB_VDP_V0_VLPOS_X, dst_xl) |
	      FIELD_PREP(HISTB_VDP_V0_VLPOS_Y, dst_yl);
	histb_vdp_write(vdp, HISTB_VDP_V0_VLPOS, val);

	val = FIELD_PREP(HISTB_VDP_V0_P0RESO_W, dst_w);
	histb_vdp_write(vdp, HISTB_VDP_V0_P0RESO, val);

	val = FIELD_PREP(HISTB_VDP_V0_P0VFPOS_X, dst_x) |
	      FIELD_PREP(HISTB_VDP_V0_P0VFPOS_Y, dst_y);
	histb_vdp_write(vdp, HISTB_VDP_V0_P0VFPOS, val);

	val = FIELD_PREP(HISTB_VDP_V0_P0VLPOS_X, dst_xl) |
	      FIELD_PREP(HISTB_VDP_V0_P0VLPOS_Y, dst_yl);
	histb_vdp_write(vdp, HISTB_VDP_V0_P0VLPOS, val);
}

static void histb_vdp_vp0_set_rect(struct histb_vdp *vdp,
				   u32 src_width, u32 src_height,
				   u32 dst_x, u32 dst_y,
				   u32 dst_width, u32 dst_height)
{
	u32 src_w = histb_vdp_minus_one(src_width);
	u32 src_h = histb_vdp_minus_one(src_height);
	u32 dst_xl = dst_x + histb_vdp_minus_one(dst_width);
	u32 dst_yl = dst_y + histb_vdp_minus_one(dst_height);
	u32 val;

	val = FIELD_PREP(HISTB_VDP_VP0_IRESO_IW, src_w) |
	      FIELD_PREP(HISTB_VDP_VP0_IRESO_IH, src_h);
	histb_vdp_write(vdp, HISTB_VDP_VP0_IRESO, val);

	val = FIELD_PREP(HISTB_VDP_VP0_DFPOS_X, dst_x) |
	      FIELD_PREP(HISTB_VDP_VP0_DFPOS_Y, dst_y);
	histb_vdp_write(vdp, HISTB_VDP_VP0_DFPOS, val);

	val = FIELD_PREP(HISTB_VDP_VP0_DLPOS_X, dst_xl) |
	      FIELD_PREP(HISTB_VDP_VP0_DLPOS_Y, dst_yl);
	histb_vdp_write(vdp, HISTB_VDP_VP0_DLPOS, val);

	val = FIELD_PREP(HISTB_VDP_VP0_VFPOS_X, dst_x) |
	      FIELD_PREP(HISTB_VDP_VP0_VFPOS_Y, dst_y);
	histb_vdp_write(vdp, HISTB_VDP_VP0_VFPOS, val);

	val = FIELD_PREP(HISTB_VDP_VP0_VLPOS_X, dst_xl) |
	      FIELD_PREP(HISTB_VDP_VP0_VLPOS_Y, dst_yl);
	histb_vdp_write(vdp, HISTB_VDP_VP0_VLPOS, val);
}

static void histb_vdp_gp0_setup(struct histb_vdp *vdp, u32 width, u32 height)
{
	u32 val;

	histb_vdp_gp0_set_rect(vdp, width, height);

	val = histb_vdp_read(vdp, HISTB_VDP_GP0_GALPHA);
	val &= ~HISTB_VDP_GP0_GALPHA_VAL;
	val |= FIELD_PREP(HISTB_VDP_GP0_GALPHA_VAL, 0xff);
	histb_vdp_write(vdp, HISTB_VDP_GP0_GALPHA, val);

	val = histb_vdp_read(vdp, HISTB_VDP_GP0_CTRL);
	val |= HISTB_VDP_GP0_CTRL_READ_MODE;
	val &= ~HISTB_VDP_GP0_CTRL_MUTE_EN;
	histb_vdp_write(vdp, HISTB_VDP_GP0_CTRL, val);

	histb_vdp_gp0_regup(vdp);
}

static void histb_vdp_gfx_set_blend_defaults(struct histb_vdp *vdp)
{
	u32 val;

	/* GFX0 fully opaque: global alpha 0xff, no pixel alpha, no colorkey. */
	val = histb_vdp_read(vdp, HISTB_VDP_G0_CBMPARA);
	val &= ~(HISTB_VDP_G0_CBMPARA_GALPHA |
		 HISTB_VDP_G0_CBMPARA_PALPHA_RANGE |
		 HISTB_VDP_G0_CBMPARA_PALPHA_EN |
		 HISTB_VDP_G0_CBMPARA_PREMULT_EN |
		 HISTB_VDP_G0_CBMPARA_KEY_EN |
		 HISTB_VDP_G0_CBMPARA_KEY_MODE);
	val |= FIELD_PREP(HISTB_VDP_G0_CBMPARA_GALPHA, 0xff);
	histb_vdp_write(vdp, HISTB_VDP_G0_CBMPARA, val);
}

static void histb_vdp_set_mix1_zorder_locked(struct histb_vdp *vdp,
					      bool video_on_top)
{
	u32 val;

	/*
	 * Each priority field names a layer as id + 1: 0 is an unused slot,
	 * 1 is VP0 (video), 2 is GP0 (graphics). prio0 is the topmost slot,
	 * not the background. Encoding from the vendor HAL, VDP_CBM_SetMixerPrio
	 * in vdp_v2_0/hal/3798m.
	 */
	val = histb_vdp_read(vdp, HISTB_VDP_CBM_MIX1);
	val &= ~(HISTB_VDP_CBM_MIX1_PRIO0 | HISTB_VDP_CBM_MIX1_PRIO1);
	if (video_on_top) {
		val |= FIELD_PREP(HISTB_VDP_CBM_MIX1_PRIO0, 1); /* VP0 */
		val |= FIELD_PREP(HISTB_VDP_CBM_MIX1_PRIO1, 2); /* GP0 */
	} else {
		val |= FIELD_PREP(HISTB_VDP_CBM_MIX1_PRIO0, 2); /* GP0 */
		val |= FIELD_PREP(HISTB_VDP_CBM_MIX1_PRIO1, 1); /* VP0 */
	}
	histb_vdp_write(vdp, HISTB_VDP_CBM_MIX1, val);

	dev_dbg(vdp->dev, "cbm_mix1 <- %08x (video_on_top=%d)\n", val,
		video_on_top);
}

static void histb_vdp_configure_cbm_path(struct histb_vdp *vdp)
{
	u32 val;

	/*
	 * Route G0 -> GP0 -> DHD0 path exactly like SDK layering:
	 * - MIXG0 priorities: prio0=G0
	 * - CBM_ATTR.sur_attr0/1 -> mixer1 (DHD0), sur_attr2/3 -> mixer2
	 * - default z-order keeps GP0 (DRM/fbcon) on top until video is enabled
	 */
	val = histb_vdp_read(vdp, HISTB_VDP_MIXG0_MIX);
	val &= ~(HISTB_VDP_MIXG0_MIX_PRIO0 |
		 HISTB_VDP_MIXG0_MIX_PRIO1 |
		 HISTB_VDP_MIXG0_MIX_PRIO2 |
		 HISTB_VDP_MIXG0_MIX_PRIO3);
	val |= FIELD_PREP(HISTB_VDP_MIXG0_MIX_PRIO0, 1); /* G0 */
	histb_vdp_write(vdp, HISTB_VDP_MIXG0_MIX, val);

	val = histb_vdp_read(vdp, HISTB_VDP_CBM_ATTR);
	val &= ~(HISTB_VDP_CBM_ATTR_SUR_ATTR0 |
		 HISTB_VDP_CBM_ATTR_SUR_ATTR1 |
		 HISTB_VDP_CBM_ATTR_SUR_ATTR2 |
		 HISTB_VDP_CBM_ATTR_SUR_ATTR3 |
		 HISTB_VDP_CBM_ATTR_SUR_ATTR4);
	val |= HISTB_VDP_CBM_ATTR_SUR_ATTR2 | HISTB_VDP_CBM_ATTR_SUR_ATTR3;
	histb_vdp_write(vdp, HISTB_VDP_CBM_ATTR, val);

	histb_vdp_set_mix1_zorder_locked(vdp, false);
}

static int histb_vdp_mode_to_timing_hdmi(const struct drm_display_mode *mode,
					 struct histb_vdp_timing *timing)
{
	u32 hfront;
	u32 hsync;
	u32 hback;
	u32 vfront;
	u32 vsync;
	u32 vback;

	if (!mode || !timing)
		return -EINVAL;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return -EINVAL;

	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return -EINVAL;

	if (mode->hdisplay > 1920 || mode->vdisplay > 1080)
		return -EINVAL;

	if (mode->hsync_start <= mode->hdisplay ||
	    mode->hsync_end <= mode->hsync_start ||
	    mode->htotal <= mode->hsync_end)
		return -EINVAL;

	if (mode->vsync_start <= mode->vdisplay ||
	    mode->vsync_end <= mode->vsync_start ||
	    mode->vtotal <= mode->vsync_end)
		return -EINVAL;

	hfront = mode->hsync_start - mode->hdisplay;
	hsync = mode->hsync_end - mode->hsync_start;
	hback = mode->htotal - mode->hsync_end;
	vfront = mode->vsync_start - mode->vdisplay;
	vsync = mode->vsync_end - mode->vsync_start;
	vback = mode->vtotal - mode->vsync_end;

	if (!hfront || !hsync || !hback || !vfront || !vsync || !vback)
		return -EINVAL;

	switch (mode->clock) {
	case 74250:
		timing->pll_ctrl0 = HISTB_VDP_PLL_CTRL0_74M25;
		break;
	case 148500:
		timing->pll_ctrl0 = HISTB_VDP_PLL_CTRL0_148M5;
		break;
	default:
		return -EINVAL;
	}

	timing->vact = mode->vdisplay;
	timing->vbb = vsync + vback;
	timing->vfb = vfront;
	timing->hact = mode->hdisplay;
	timing->hbb = hsync + hback;
	timing->hfb = hfront;
	timing->bvact = 1;
	timing->bvbb = 1;
	timing->bvfb = 1;
	timing->hpw = hsync;
	timing->vpw = vsync;
	timing->hmid = 1;
	timing->pll_ctrl1 = HISTB_VDP_PLL_CTRL1_STD;
	timing->progressive = true;

	return 0;
}

/* Timings block */
static int histb_vdp_timings_apply(struct histb_vdp *vdp,
				   const struct histb_vdp_timing *timing)
{
	u32 val;
	int ret;

	ret = histb_vdp_program_pll(vdp, timing);
	if (ret)
		return ret;

	val = histb_vdp_read(vdp, HISTB_VDP_VOCTRL);
	val |= HISTB_VDP_VOCTRL_CK_GT_EN;
	histb_vdp_write(vdp, HISTB_VDP_VOCTRL, val);

	histb_vdp_write(vdp, HISTB_VDP_DHD0_SYNC_INV, 0x2000);

	val = histb_vdp_read(vdp, HISTB_VDP_DHD0_CTRL);
	val &= ~(HISTB_VDP_DHD_CTRL_DISP_MODE |
		 HISTB_VDP_DHD_CTRL_GMM_EN |
		 HISTB_VDP_DHD_CTRL_FPGA_LMT_EN |
		 HISTB_VDP_DHD_CTRL_P2I_EN |
		 HISTB_VDP_DHD_CTRL_CBAR_SEL |
		 HISTB_VDP_DHD_CTRL_CBAR_EN |
		 HISTB_VDP_DHD_CTRL_INTF_EN);
	if (timing->progressive)
		val |= HISTB_VDP_DHD_CTRL_IOP;
	else
		val &= ~HISTB_VDP_DHD_CTRL_IOP;
	val |= HISTB_VDP_DHD_CTRL_HDMI_MODE;
	val |= HISTB_VDP_DHD_CTRL_REGUP;
	histb_vdp_write(vdp, HISTB_VDP_DHD0_CTRL, val);

	val = FIELD_PREP(HISTB_VDP_HSYNC1_HACT,
			 histb_vdp_minus_one(timing->hact)) |
	      FIELD_PREP(HISTB_VDP_HSYNC1_HBB,
			 histb_vdp_minus_one(timing->hbb));
	histb_vdp_write(vdp, HISTB_VDP_DHD0_HSYNC1, val);

	val = FIELD_PREP(HISTB_VDP_HSYNC2_HFB,
			 histb_vdp_minus_one(timing->hfb)) |
	      FIELD_PREP(HISTB_VDP_HSYNC2_HMID,
			 histb_vdp_minus_one(timing->hmid));
	histb_vdp_write(vdp, HISTB_VDP_DHD0_HSYNC2, val);

	val = FIELD_PREP(HISTB_VDP_VSYNC_VACT,
			 histb_vdp_minus_one(timing->vact)) |
	      FIELD_PREP(HISTB_VDP_VSYNC_VBB,
			 histb_vdp_minus_one(timing->vbb)) |
	      FIELD_PREP(HISTB_VDP_VSYNC_VFB,
			 histb_vdp_minus_one(timing->vfb));
	histb_vdp_write(vdp, HISTB_VDP_DHD0_VSYNC, val);

	val = FIELD_PREP(HISTB_VDP_VPLUS_BVACT,
			 histb_vdp_minus_one(timing->bvact)) |
	      FIELD_PREP(HISTB_VDP_VPLUS_BVBB,
			 histb_vdp_minus_one(timing->bvbb)) |
	      FIELD_PREP(HISTB_VDP_VPLUS_BVFB,
			 histb_vdp_minus_one(timing->bvfb));
	histb_vdp_write(vdp, HISTB_VDP_DHD0_VPLUS, val);

	val = FIELD_PREP(HISTB_VDP_PWR_HPW,
			 histb_vdp_minus_one(timing->hpw)) |
	      FIELD_PREP(HISTB_VDP_PWR_VPW,
			 histb_vdp_minus_one(timing->vpw));
	histb_vdp_write(vdp, HISTB_VDP_DHD0_PWR, val);

	/*
	 * Raster line the frame interrupt fires on. The vendor driver puts it
	 * at 80% of the vertical period: late enough that the frame is done,
	 * early enough to leave slack before the next one starts. Threshold
	 * mode 0 means one interrupt per frame rather than per field.
	 */
	val = (timing->vfb + timing->vbb + timing->vact) * 8 / 10;
	histb_vdp_write(vdp, HISTB_VDP_DHD0_VTTHD,
			FIELD_PREP(HISTB_VDP_VTTHD_VTMGTHD1, val));

	return 0;
}

/* Mixer block */
static void histb_vdp_mixer_apply_defaults(struct histb_vdp *vdp,
					   const struct histb_vdp_timing *timing)
{
	u32 val;

	histb_vdp_configure_cbm_path(vdp);

	vdp->cbar_enabled = true;
	val = histb_vdp_read(vdp, HISTB_VDP_G0_CTRL);
	val &= ~HISTB_VDP_G0_CTRL_SURFACE_EN;
	histb_vdp_write(vdp, HISTB_VDP_G0_CTRL, val);
	histb_vdp_gfx_regup(vdp);

	/* Keep video path fully disabled until an overlay plane is programmed. */
	val = histb_vdp_read(vdp, HISTB_VDP_V0_16REGIONENL);
	val &= ~HISTB_VDP_V0_16REGIONENL_P0_EN;
	histb_vdp_write(vdp, HISTB_VDP_V0_16REGIONENL, val);

	val = histb_vdp_read(vdp, HISTB_VDP_V0_CTRL);
	val &= ~(HISTB_VDP_V0_CTRL_SURFACE_EN |
		 HISTB_VDP_V0_CTRL_SURFACE_C_EN);
	val |= HISTB_VDP_V0_CTRL_MUTE_EN;
	vdp->video_cfg_valid = false;
	histb_vdp_write(vdp, HISTB_VDP_V0_CTRL, val);

	val = histb_vdp_read(vdp, HISTB_VDP_VP0_CTRL);
	val |= HISTB_VDP_VP0_CTRL_MUTE_EN;
	histb_vdp_write(vdp, HISTB_VDP_VP0_CTRL, val);

	histb_vdp_vid_regup(vdp);
	histb_vdp_vp0_regup(vdp);

	histb_vdp_gp0_setup(vdp, timing->hact, timing->vact);
}

/* GFX block */
static int histb_vdp_gfx_validate_cfg(const struct histb_vdp_gfx_cfg *cfg)
{
	u32 w;
	u32 h;
	u32 stride_hw;

	if (!cfg->width || !cfg->height || (cfg->stride & 0xf))
		return -EINVAL;

	w = histb_vdp_minus_one(cfg->width);
	h = histb_vdp_minus_one(cfg->height);
	stride_hw = cfg->stride >> 4;

	if (!FIELD_FIT(HISTB_VDP_G0_IRESO_IW, w) ||
	    !FIELD_FIT(HISTB_VDP_G0_IRESO_IH, h) ||
	    !FIELD_FIT(HISTB_VDP_G0_STRIDE_SURFACE, stride_hw) ||
	    !FIELD_FIT(HISTB_VDP_G0_CTRL_IFMT, cfg->ifmt))
		return -EINVAL;

	return 0;
}

static void histb_vdp_gfx_apply_cfg(struct histb_vdp *vdp,
				    const struct histb_vdp_gfx_cfg *cfg)
{
	u32 val;
	u32 stride_hw = cfg->stride >> 4;

	histb_vdp_gfx_set_rect(vdp, cfg->width, cfg->height);

	val = histb_vdp_read(vdp, HISTB_VDP_G0_STRIDE);
	val &= ~HISTB_VDP_G0_STRIDE_SURFACE;
	val |= FIELD_PREP(HISTB_VDP_G0_STRIDE_SURFACE, stride_hw);
	histb_vdp_write(vdp, HISTB_VDP_G0_STRIDE, val);

	val = histb_vdp_read(vdp, HISTB_VDP_G0_CTRL);
	val &= ~(HISTB_VDP_G0_CTRL_IFMT | HISTB_VDP_G0_CTRL_MUTE_EN);
	val |= FIELD_PREP(HISTB_VDP_G0_CTRL_IFMT, cfg->ifmt) |
	       HISTB_VDP_G0_CTRL_READ_MODE |
	       HISTB_VDP_G0_CTRL_NOSEC_FLAG;
	histb_vdp_write(vdp, HISTB_VDP_G0_CTRL, val);

	histb_vdp_gfx_set_blend_defaults(vdp);
	histb_vdp_gfx_regup(vdp);
}

static void histb_vdp_gfx_set_addr_locked(struct histb_vdp *vdp, u32 addr)
{
	histb_vdp_write(vdp, HISTB_VDP_G0_ADDR, addr);
	histb_vdp_write(vdp, HISTB_VDP_G0_NADDR, addr);
	histb_vdp_gfx_regup(vdp);
}

static void histb_vdp_gfx_set_enable_locked(struct histb_vdp *vdp, bool enable)
{
	u32 val;

	val = histb_vdp_read(vdp, HISTB_VDP_G0_CTRL);
	if (enable)
		val |= HISTB_VDP_G0_CTRL_SURFACE_EN | HISTB_VDP_G0_CTRL_NOSEC_FLAG;
	else
		val &= ~HISTB_VDP_G0_CTRL_SURFACE_EN;
	histb_vdp_write(vdp, HISTB_VDP_G0_CTRL, val);
	histb_vdp_gfx_regup(vdp);

	if (enable) {
		vdp->cbar_enabled = false;
		histb_vdp_set_output(vdp, true);
	}
}

static int histb_vdp_gfx_set_blend_locked(struct histb_vdp *vdp,
					  u16 alpha, u16 pixel_blend_mode)
{
	u8 alpha_hw = histb_vdp_alpha_to_hw(alpha);
	u32 val;

	if (pixel_blend_mode != DRM_MODE_BLEND_PIXEL_NONE &&
	    pixel_blend_mode != DRM_MODE_BLEND_COVERAGE &&
	    pixel_blend_mode != DRM_MODE_BLEND_PREMULTI)
		return -EINVAL;

	val = histb_vdp_read(vdp, HISTB_VDP_G0_CBMPARA);
	val &= ~(HISTB_VDP_G0_CBMPARA_GALPHA |
		 HISTB_VDP_G0_CBMPARA_PALPHA_RANGE |
		 HISTB_VDP_G0_CBMPARA_PALPHA_EN |
		 HISTB_VDP_G0_CBMPARA_PREMULT_EN);
	val |= FIELD_PREP(HISTB_VDP_G0_CBMPARA_GALPHA, alpha_hw);

	switch (pixel_blend_mode) {
	case DRM_MODE_BLEND_PREMULTI:
		val |= HISTB_VDP_G0_CBMPARA_PALPHA_EN |
		       HISTB_VDP_G0_CBMPARA_PREMULT_EN;
		break;
	case DRM_MODE_BLEND_COVERAGE:
		val |= HISTB_VDP_G0_CBMPARA_PALPHA_EN;
		break;
	default:
		break;
	}

	histb_vdp_write(vdp, HISTB_VDP_G0_CBMPARA, val);
	histb_vdp_gfx_regup(vdp);

	return 0;
}

/* Video block */
static int histb_vdp_video_validate_cfg(const struct histb_vdp_video_cfg *cfg)
{
	u32 src_w;
	u32 src_h;
	u32 dst_w;
	u32 dst_h;

	if (!cfg->src_width || !cfg->src_height ||
	    !cfg->dst_width || !cfg->dst_height ||
	    !cfg->luma_stride || !cfg->chroma_stride)
		return -EINVAL;

	/*
	 * The stride registers count 16-byte units, not bytes - the same as the
	 * graphics layer's. Writing a byte count makes the hardware step
	 * sixteen times too far per line, which shows as a sliver of correct
	 * picture followed by whatever else is in memory.
	 */
	if ((cfg->luma_stride & 0xf) || (cfg->chroma_stride & 0xf))
		return -EINVAL;

	/* NV21 path: source geometry is 4:2:0, keep it even. */
	if ((cfg->src_x & 1) || (cfg->src_y & 1) ||
	    (cfg->src_width & 1) || (cfg->src_height & 1))
		return -EINVAL;

	src_w = histb_vdp_minus_one(cfg->src_width);
	src_h = histb_vdp_minus_one(cfg->src_height);
	dst_w = histb_vdp_minus_one(cfg->dst_width);
	dst_h = histb_vdp_minus_one(cfg->dst_height);

	if (!FIELD_FIT(HISTB_VDP_V0_IRESO_IW, src_w) ||
	    !FIELD_FIT(HISTB_VDP_V0_IRESO_IH, src_h) ||
	    !FIELD_FIT(HISTB_VDP_V0_ORESO_OW, dst_w) ||
	    !FIELD_FIT(HISTB_VDP_V0_ORESO_OH, dst_h) ||
	    !FIELD_FIT(HISTB_VDP_V0_P0RESO_W, dst_w) ||
	    !FIELD_FIT(HISTB_VDP_V0_P0STRIDE_LUMA, cfg->luma_stride >> 4) ||
	    !FIELD_FIT(HISTB_VDP_V0_P0STRIDE_CHROMA, cfg->chroma_stride >> 4) ||
	    !FIELD_FIT(HISTB_VDP_VP0_IRESO_IW, src_w) ||
	    !FIELD_FIT(HISTB_VDP_VP0_IRESO_IH, src_h))
		return -EINVAL;

	if (!histb_vdp_rect_fits(cfg->dst_x, cfg->dst_y,
				 cfg->dst_width, cfg->dst_height,
				 HISTB_VDP_V0_DFPOS_X, HISTB_VDP_V0_DFPOS_Y,
				 HISTB_VDP_V0_DLPOS_X, HISTB_VDP_V0_DLPOS_Y))
		return -EINVAL;

	if (!histb_vdp_rect_fits(cfg->dst_x, cfg->dst_y,
				 cfg->dst_width, cfg->dst_height,
				 HISTB_VDP_V0_VFPOS_X, HISTB_VDP_V0_VFPOS_Y,
				 HISTB_VDP_V0_VLPOS_X, HISTB_VDP_V0_VLPOS_Y))
		return -EINVAL;

	if (!histb_vdp_rect_fits(cfg->dst_x, cfg->dst_y,
				 cfg->dst_width, cfg->dst_height,
				 HISTB_VDP_V0_P0VFPOS_X, HISTB_VDP_V0_P0VFPOS_Y,
				 HISTB_VDP_V0_P0VLPOS_X, HISTB_VDP_V0_P0VLPOS_Y))
		return -EINVAL;

	if (!histb_vdp_rect_fits(cfg->dst_x, cfg->dst_y,
				 cfg->dst_width, cfg->dst_height,
				 HISTB_VDP_VP0_DFPOS_X, HISTB_VDP_VP0_DFPOS_Y,
				 HISTB_VDP_VP0_DLPOS_X, HISTB_VDP_VP0_DLPOS_Y))
		return -EINVAL;

	if (!histb_vdp_rect_fits(cfg->dst_x, cfg->dst_y,
				 cfg->dst_width, cfg->dst_height,
				 HISTB_VDP_VP0_VFPOS_X, HISTB_VDP_VP0_VFPOS_Y,
				 HISTB_VDP_VP0_VLPOS_X, HISTB_VDP_VP0_VLPOS_Y))
		return -EINVAL;

	return 0;
}

static bool histb_vdp_video_color_supported(enum drm_color_encoding encoding,
					    enum drm_color_range range)
{
	return encoding == DRM_COLOR_YCBCR_BT601 &&
	       range == DRM_COLOR_YCBCR_LIMITED_RANGE;
}

/*
 * Colour conversion for the video layer: it delivers YCbCr while the mixer
 * blends full-range RGB, and without this the layer emits a constant colour.
 *
 * Standard BT.601 limited-range YCbCr to full-range RGB, input order (Y, Cb,
 * Cr). Fixed point as in the vendor's vdp_func_pq_csc.c: coefficients scaled
 * by 1<<10 and 15-bit, DC offsets by 4 and 11-bit, both signed - hence masked
 * into their fields, since FIELD_PREP rejects sign-extended negatives.
 */
struct histb_vdp_csc {
	s32 coef[3][3];
	s32 in_dc[3];
	s32 out_dc[3];
};

/*
 * Pre-multiplied so the kernel never sees a floating-point constant:
 * coefficient = round(value * 1024), DC offset = value * 4.
 */
static const struct histb_vdp_csc histb_vdp_csc_bt601_limited = {
	.coef = {
		{  1192,     0,  1634 },	/*  1.164,  0.000,  1.596 */
		{  1192,  -401,  -832 },	/*  1.164, -0.392, -0.813 */
		{  1192,  2065,     0 },	/*  1.164,  2.017,  0.000 */
	},
	.in_dc = { -64, -512, -512 },		/* -16, -128, -128 */
	.out_dc = { 0, 0, 0 },
};

static u32 histb_vdp_csc_pair(s32 lo, s32 hi)
{
	return ((u32)lo & HISTB_VDP_V0_CSC_LO) |
	       (((u32)hi << 16) & HISTB_VDP_V0_CSC_HI);
}

static void histb_vdp_video_apply_csc(struct histb_vdp *vdp,
				      const struct histb_vdp_csc *csc)
{
	u32 val;

	histb_vdp_write(vdp, HISTB_VDP_V0_CSC_P0,
			histb_vdp_csc_pair(csc->coef[0][0], csc->coef[0][1]));
	histb_vdp_write(vdp, HISTB_VDP_V0_CSC_P1,
			histb_vdp_csc_pair(csc->coef[0][2], csc->coef[1][0]));
	histb_vdp_write(vdp, HISTB_VDP_V0_CSC_P2,
			histb_vdp_csc_pair(csc->coef[1][1], csc->coef[1][2]));
	histb_vdp_write(vdp, HISTB_VDP_V0_CSC_P3,
			histb_vdp_csc_pair(csc->coef[2][0], csc->coef[2][1]));
	histb_vdp_write(vdp, HISTB_VDP_V0_CSC_P4,
			(u32)csc->coef[2][2] & HISTB_VDP_V0_CSC_LO);

	histb_vdp_write(vdp, HISTB_VDP_V0_CSC_ODC,
			(((u32)csc->out_dc[0]) & HISTB_VDP_V0_CSC_ODC_DC0) |
			((((u32)csc->out_dc[1]) << 11) &
			 HISTB_VDP_V0_CSC_ODC_DC1));

	histb_vdp_write(vdp, HISTB_VDP_V0_CSC_IODC,
			(((u32)csc->in_dc[2]) & HISTB_VDP_V0_CSC_IODC_IDC2) |
			((((u32)csc->out_dc[2]) << 11) &
			 HISTB_VDP_V0_CSC_IODC_ODC2));

	/* The enable bit lives here, so this register goes last. */
	val = (((u32)csc->in_dc[0]) & HISTB_VDP_V0_CSC_IDC_DC0) |
	      ((((u32)csc->in_dc[1]) << 11) & HISTB_VDP_V0_CSC_IDC_DC1) |
	      HISTB_VDP_V0_CSC_IDC_EN;
	histb_vdp_write(vdp, HISTB_VDP_V0_CSC_IDC, val);

	histb_vdp_vid_regup(vdp);
}

static int histb_vdp_video_set_colorspace_locked(struct histb_vdp *vdp,
						 enum drm_color_encoding encoding,
						 enum drm_color_range range)
{
	if (!histb_vdp_video_color_supported(encoding, range))
		return -EINVAL;

	if (vdp->video_color_encoding == encoding &&
	    vdp->video_color_range == range)
		return 0;

	vdp->video_color_encoding = encoding;
	vdp->video_color_range = range;

	histb_vdp_video_apply_csc(vdp, &histb_vdp_csc_bt601_limited);

	return 0;
}

static void histb_vdp_video_apply_cfg(struct histb_vdp *vdp,
				      const struct histb_vdp_video_cfg *cfg)
{
	u32 val;

	histb_vdp_v0_set_rect(vdp,
			      cfg->src_width, cfg->src_height,
			      cfg->dst_x, cfg->dst_y,
			      cfg->dst_width, cfg->dst_height);
	histb_vdp_vp0_set_rect(vdp,
			       cfg->src_width, cfg->src_height,
			       cfg->dst_x, cfg->dst_y,
			       cfg->dst_width, cfg->dst_height);

	val = histb_vdp_read(vdp, HISTB_VDP_V0_P0STRIDE);
	val &= ~(HISTB_VDP_V0_P0STRIDE_LUMA | HISTB_VDP_V0_P0STRIDE_CHROMA);
	val |= FIELD_PREP(HISTB_VDP_V0_P0STRIDE_LUMA, cfg->luma_stride >> 4) |
	       FIELD_PREP(HISTB_VDP_V0_P0STRIDE_CHROMA, cfg->chroma_stride >> 4);
	histb_vdp_write(vdp, HISTB_VDP_V0_P0STRIDE, val);

	val = histb_vdp_read(vdp, HISTB_VDP_V0_CTRL);
	val &= ~(HISTB_VDP_V0_CTRL_IFMT | HISTB_VDP_V0_CTRL_MUTE_EN);
	val |= FIELD_PREP(HISTB_VDP_V0_CTRL_IFMT, HISTB_VDP_VID_IFMT_SP_420) |
	       HISTB_VDP_V0_CTRL_UV_ORDER |
	       HISTB_VDP_V0_CTRL_NOSEC_FLAG;
	histb_vdp_write(vdp, HISTB_VDP_V0_CTRL, val);

	val = histb_vdp_read(vdp, HISTB_VDP_V0_MULTI_MODE);
	val &= ~HISTB_VDP_V0_MULTI_MODE_MRG_MODE;
	histb_vdp_write(vdp, HISTB_VDP_V0_MULTI_MODE, val);

	val = histb_vdp_read(vdp, HISTB_VDP_V0_CBMPARA);
	val &= ~HISTB_VDP_V0_CBMPARA_GALPHA;
	val |= FIELD_PREP(HISTB_VDP_V0_CBMPARA_GALPHA, 0xff);
	histb_vdp_write(vdp, HISTB_VDP_V0_CBMPARA, val);

	val = histb_vdp_read(vdp, HISTB_VDP_V0_ALPHA);
	val &= ~HISTB_VDP_V0_ALPHA_VBK_ALPHA;
	val |= FIELD_PREP(HISTB_VDP_V0_ALPHA_VBK_ALPHA, 0xff);
	histb_vdp_write(vdp, HISTB_VDP_V0_ALPHA, val);

	val = histb_vdp_read(vdp, HISTB_VDP_VP0_CTRL);
	val &= ~(HISTB_VDP_VP0_CTRL_GALPHA | HISTB_VDP_VP0_CTRL_MUTE_EN);
	val |= FIELD_PREP(HISTB_VDP_VP0_CTRL_GALPHA, 0xff);
	histb_vdp_write(vdp, HISTB_VDP_VP0_CTRL, val);

	histb_vdp_vid_regup(vdp);
	histb_vdp_vp0_regup(vdp);
}

static void histb_vdp_video_set_addr_locked(struct histb_vdp *vdp, u32 luma_addr,
					    u32 chroma_addr)
{
	histb_vdp_write(vdp, HISTB_VDP_V0_P0LADDR, luma_addr);
	histb_vdp_write(vdp, HISTB_VDP_V0_P0CADDR, chroma_addr);
	histb_vdp_write(vdp, HISTB_VDP_V0_NADDR, luma_addr);
	histb_vdp_write(vdp, HISTB_VDP_V0_NCADDR, chroma_addr);
	histb_vdp_vid_regup(vdp);
}

/*
 * Diagnostic: read the video layer back, to tell what the hardware holds from
 * what we wrote - a shadow register that never latches reads back differently.
 */
void histb_vdp_pipeline_video_dump(struct histb_vdp *vdp, const char *tag)
{
	if (!vdp)
		return;

	mutex_lock(&vdp->lock);
	if (!vdp->pm_active) {
		mutex_unlock(&vdp->lock);
		return;
	}

	dev_dbg(vdp->dev,
		"vdp[%s]: ctrl=%08x ireso=%08x oreso=%08x stride=%08x laddr=%08x caddr=%08x dfpos=%08x dlpos=%08x region=%08x cscidc=%08x vp0ctrl=%08x\n",
		tag,
		histb_vdp_read(vdp, HISTB_VDP_V0_CTRL),
		histb_vdp_read(vdp, HISTB_VDP_V0_IRESO),
		histb_vdp_read(vdp, HISTB_VDP_V0_ORESO),
		histb_vdp_read(vdp, HISTB_VDP_V0_P0STRIDE),
		histb_vdp_read(vdp, HISTB_VDP_V0_P0LADDR),
		histb_vdp_read(vdp, HISTB_VDP_V0_P0CADDR),
		histb_vdp_read(vdp, HISTB_VDP_V0_DFPOS),
		histb_vdp_read(vdp, HISTB_VDP_V0_DLPOS),
		histb_vdp_read(vdp, HISTB_VDP_V0_16REGIONENL),
		histb_vdp_read(vdp, HISTB_VDP_V0_CSC_IDC),
		histb_vdp_read(vdp, HISTB_VDP_VP0_CTRL));

	/*
	 * The mixer decides which layers reach the output at all. A perfectly
	 * configured layer composited at priority zero, or left out of the
	 * blend, is indistinguishable from a broken one at the layer level.
	 */
	dev_dbg(vdp->dev,
		"vdp[%s]: mixg0=%08x cbm_mix1=%08x cbm_attr=%08x v0_cbmpara=%08x g0_cbmpara=%08x\n",
		tag,
		histb_vdp_read(vdp, HISTB_VDP_MIXG0_MIX),
		histb_vdp_read(vdp, HISTB_VDP_CBM_MIX1),
		histb_vdp_read(vdp, HISTB_VDP_CBM_ATTR),
		histb_vdp_read(vdp, HISTB_VDP_V0_CBMPARA),
		histb_vdp_read(vdp, HISTB_VDP_G0_CBMPARA));

	mutex_unlock(&vdp->lock);
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_video_dump);

static void histb_vdp_video_set_enable_locked(struct histb_vdp *vdp, bool enable)
{
	u32 val;

	val = histb_vdp_read(vdp, HISTB_VDP_V0_16REGIONENL);
	if (enable)
		val |= HISTB_VDP_V0_16REGIONENL_P0_EN;
	else
		val &= ~HISTB_VDP_V0_16REGIONENL_P0_EN;
	histb_vdp_write(vdp, HISTB_VDP_V0_16REGIONENL, val);

	val = histb_vdp_read(vdp, HISTB_VDP_V0_CTRL);
	if (enable)
		val |= HISTB_VDP_V0_CTRL_SURFACE_EN |
		       HISTB_VDP_V0_CTRL_SURFACE_C_EN |
		       HISTB_VDP_V0_CTRL_NOSEC_FLAG;
	else
		val &= ~(HISTB_VDP_V0_CTRL_SURFACE_EN |
			 HISTB_VDP_V0_CTRL_SURFACE_C_EN);
	val &= ~HISTB_VDP_V0_CTRL_MUTE_EN;
	histb_vdp_write(vdp, HISTB_VDP_V0_CTRL, val);

	val = histb_vdp_read(vdp, HISTB_VDP_VP0_CTRL);
	if (enable)
		val &= ~HISTB_VDP_VP0_CTRL_MUTE_EN;
	else
		val |= HISTB_VDP_VP0_CTRL_MUTE_EN;
	histb_vdp_write(vdp, HISTB_VDP_VP0_CTRL, val);

	histb_vdp_vid_regup(vdp);
	histb_vdp_vp0_regup(vdp);

	if (enable) {
		vdp->cbar_enabled = false;
		histb_vdp_set_output(vdp, true);
	}
}

static void histb_vdp_video_set_alpha_locked(struct histb_vdp *vdp, u16 alpha)
{
	u8 alpha_hw = histb_vdp_alpha_to_hw(alpha);
	u32 val;

	val = histb_vdp_read(vdp, HISTB_VDP_V0_CBMPARA);
	val &= ~HISTB_VDP_V0_CBMPARA_GALPHA;
	val |= FIELD_PREP(HISTB_VDP_V0_CBMPARA_GALPHA, alpha_hw);
	histb_vdp_write(vdp, HISTB_VDP_V0_CBMPARA, val);

	val = histb_vdp_read(vdp, HISTB_VDP_V0_ALPHA);
	val &= ~HISTB_VDP_V0_ALPHA_VBK_ALPHA;
	val |= FIELD_PREP(HISTB_VDP_V0_ALPHA_VBK_ALPHA, alpha_hw);
	histb_vdp_write(vdp, HISTB_VDP_V0_ALPHA, val);

	val = histb_vdp_read(vdp, HISTB_VDP_VP0_CTRL);
	val &= ~HISTB_VDP_VP0_CTRL_GALPHA;
	val |= FIELD_PREP(HISTB_VDP_VP0_CTRL_GALPHA, alpha_hw);
	histb_vdp_write(vdp, HISTB_VDP_VP0_CTRL, val);

	histb_vdp_vid_regup(vdp);
	histb_vdp_vp0_regup(vdp);
}

static int histb_vdp_hw_init(struct histb_vdp *vdp,
			     const struct histb_vdp_timing *timing)
{
	int ret;

	histb_vdp_hdmi_path_setup_locked(vdp);

	ret = histb_vdp_timings_apply(vdp, timing);
	if (ret)
		return ret;

	histb_vdp_mixer_apply_defaults(vdp, timing);

	histb_vdp_set_output(vdp, false);

	return 0;
}

static int histb_vdp_runtime_resume(struct device *dev)
{
	struct histb_vdp *vdp = dev_get_drvdata(dev);
	int ret;

	ret = clk_bulk_prepare_enable(HISTB_VDP_NUM_CLKS, vdp->clks);
	if (ret)
		return ret;

	ret = histb_vdp_configure_crg(vdp);
	if (ret)
		goto err_disable_clks;

	ret = histb_vdp_pulse_reset(vdp);
	if (ret)
		goto err_disable_clks;

	mutex_lock(&vdp->lock);
	ret = histb_vdp_hw_init(vdp, &histb_vdp_720p60);
	mutex_unlock(&vdp->lock);
	if (ret)
		goto err_disable_clks;

	return 0;

err_disable_clks:
	reset_control_assert(vdp->rst);
	clk_bulk_disable_unprepare(HISTB_VDP_NUM_CLKS, vdp->clks);
	return ret;
}

static int histb_vdp_runtime_suspend(struct device *dev)
{
	struct histb_vdp *vdp = dev_get_drvdata(dev);

	mutex_lock(&vdp->lock);
	histb_vdp_set_output(vdp, false);
	mutex_unlock(&vdp->lock);
	WRITE_ONCE(vdp->pm_active, false);

	reset_control_assert(vdp->rst);
	clk_bulk_disable_unprepare(HISTB_VDP_NUM_CLKS, vdp->clks);

	return 0;
}

static void histb_vdp_pm_cleanup_action(void *data)
{
	struct histb_vdp *vdp = data;
	struct device *dev = vdp->dev;

	if (pm_runtime_enabled(dev))
		pm_runtime_disable(dev);

	if (!pm_runtime_status_suspended(dev))
		histb_vdp_runtime_suspend(dev);
}

struct histb_vdp *histb_vdp_get_from_node(struct device_node *np)
{
	struct platform_device *pdev;
	struct histb_vdp *vdp;

	if (!np)
		return ERR_PTR(-EINVAL);

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	vdp = platform_get_drvdata(pdev);
	put_device(&pdev->dev);

	if (!vdp)
		return ERR_PTR(-EPROBE_DEFER);

	return vdp;
}
EXPORT_SYMBOL_GPL(histb_vdp_get_from_node);

int histb_vdp_pipeline_prepare(struct histb_vdp *vdp)
{
	if (!vdp)
		return -EPROBE_DEFER;

	return 0;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_prepare);

int histb_vdp_pipeline_mode_valid(const struct drm_display_mode *mode)
{
	struct histb_vdp_timing timing;

	return histb_vdp_mode_to_timing_hdmi(mode, &timing);
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_mode_valid);

int histb_vdp_pipeline_setup_output(struct histb_vdp *vdp)
{
	if (!vdp)
		return -EPROBE_DEFER;

	mutex_lock(&vdp->lock);
	if (vdp->pm_active)
		histb_vdp_hdmi_path_setup_locked(vdp);
	mutex_unlock(&vdp->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_setup_output);

int histb_vdp_pipeline_set_mode(struct histb_vdp *vdp,
				const struct drm_display_mode *mode)
{
	struct histb_vdp_timing timing;
	bool was_active;
	int ret;

	if (!vdp)
		return -EPROBE_DEFER;

	was_active = READ_ONCE(vdp->pm_active);
	ret = histb_vdp_runtime_ensure_active(vdp);
	if (ret)
		return ret;

	mutex_lock(&vdp->lock);
	ret = histb_vdp_mode_to_timing_hdmi(mode, &timing);
	if (ret)
		goto out_unlock;

	ret = histb_vdp_hw_init(vdp, &timing);

out_unlock:
	mutex_unlock(&vdp->lock);

	if (ret && !was_active)
		histb_vdp_runtime_release(vdp);

	return ret;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_set_mode);

/*
 * The frame interrupt is the only one this driver consumes. Registers are
 * touched raw here rather than through histb_vdp_write(): the tracing hook
 * in that helper would fire once per frame, and hard IRQ context must not
 * take vdp->lock.
 */
static irqreturn_t histb_vdp_irq(int irq, void *data)
{
	struct histb_vdp *vdp = data;
	void (*cb)(void *cb_data);
	u32 status;

	status = readl_relaxed(vdp->regs + HISTB_VDP_VOMSKINTSTA);
	if (!(status & HISTB_VDP_INT_DHD0_VTTHD1))
		return IRQ_NONE;

	writel_relaxed(HISTB_VDP_INT_DHD0_VTTHD1,
		       vdp->regs + HISTB_VDP_VOMSKINTSTA);

	cb = READ_ONCE(vdp->vblank_cb);
	if (cb)
		cb(READ_ONCE(vdp->vblank_data));

	return IRQ_HANDLED;
}

/*
 * Called from the DRM side while the interrupt is still disabled, so no
 * synchronisation beyond the publish order is needed.
 */
void histb_vdp_set_vblank_handler(struct histb_vdp *vdp,
				  void (*cb)(void *data), void *data)
{
	if (!vdp)
		return;

	if (cb) {
		WRITE_ONCE(vdp->vblank_data, data);
		/* Publish the data before the callback that consumes it. */
		smp_wmb();
		WRITE_ONCE(vdp->vblank_cb, cb);
	} else {
		WRITE_ONCE(vdp->vblank_cb, NULL);
		synchronize_irq(vdp->irq);
		WRITE_ONCE(vdp->vblank_data, NULL);
	}
}
EXPORT_SYMBOL_GPL(histb_vdp_set_vblank_handler);

/*
 * DRM calls these under dev->vbl_lock with interrupts off, so they may not
 * sleep: only the mask register is touched, and the block is known to be
 * powered because vblank is only armed while the CRTC is on.
 */
void histb_vdp_enable_vblank(struct histb_vdp *vdp)
{
	u32 val;

	if (!vdp || !READ_ONCE(vdp->pm_active))
		return;

	writel_relaxed(HISTB_VDP_INT_DHD0_VTTHD1,
		       vdp->regs + HISTB_VDP_VOMSKINTSTA);
	val = readl_relaxed(vdp->regs + HISTB_VDP_VOINTMSK);
	val |= HISTB_VDP_INT_DHD0_VTTHD1;
	writel_relaxed(val, vdp->regs + HISTB_VDP_VOINTMSK);
}
EXPORT_SYMBOL_GPL(histb_vdp_enable_vblank);

void histb_vdp_disable_vblank(struct histb_vdp *vdp)
{
	u32 val;

	if (!vdp || !READ_ONCE(vdp->pm_active))
		return;

	val = readl_relaxed(vdp->regs + HISTB_VDP_VOINTMSK);
	val &= ~HISTB_VDP_INT_DHD0_VTTHD1;
	writel_relaxed(val, vdp->regs + HISTB_VDP_VOINTMSK);
}
EXPORT_SYMBOL_GPL(histb_vdp_disable_vblank);

int histb_vdp_pipeline_enable(struct histb_vdp *vdp)
{
	int ret;

	if (!vdp)
		return -EPROBE_DEFER;

	ret = histb_vdp_runtime_ensure_active(vdp);
	if (ret)
		return ret;

	mutex_lock(&vdp->lock);
	histb_vdp_set_output(vdp, true);
	/*
	 * Start from a known interrupt state now that the block is powered,
	 * then let the line through. A bootloader splash may have left the
	 * mask open and a status bit pending.
	 */
	histb_vdp_write(vdp, HISTB_VDP_VOINTMSK, 0);
	histb_vdp_write(vdp, HISTB_VDP_VOMSKINTSTA, ~0u);
	if (!vdp->irq_enabled) {
		vdp->irq_enabled = true;
		enable_irq(vdp->irq);
	}
	mutex_unlock(&vdp->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_enable);

void histb_vdp_pipeline_disable(struct histb_vdp *vdp)
{
	if (!vdp)
		return;

	if (!READ_ONCE(vdp->pm_active))
		return;

	mutex_lock(&vdp->lock);
	/*
	 * Safe under the lock: the handler never takes it, so waiting for an
	 * in-flight one to finish cannot deadlock.
	 */
	if (vdp->irq_enabled) {
		vdp->irq_enabled = false;
		disable_irq(vdp->irq);
	}
	histb_vdp_write(vdp, HISTB_VDP_VOINTMSK, 0);
	histb_vdp_set_output(vdp, false);
	mutex_unlock(&vdp->lock);
	histb_vdp_runtime_release(vdp);
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_disable);

int histb_vdp_pipeline_gfx_setup(struct histb_vdp *vdp, u32 width, u32 height,
				 u32 stride, enum histb_vdp_gfx_ifmt ifmt)
{
	struct histb_vdp_gfx_cfg cfg = {
		.width = width,
		.height = height,
		.stride = stride,
		.ifmt = ifmt,
	};
	int ret;

	if (!vdp)
		return -EPROBE_DEFER;

	ret = histb_vdp_gfx_validate_cfg(&cfg);
	if (ret)
		return ret;

	mutex_lock(&vdp->lock);
	if (!vdp->pm_active) {
		mutex_unlock(&vdp->lock);
		return -EPIPE;
	}
	histb_vdp_gfx_apply_cfg(vdp, &cfg);
	mutex_unlock(&vdp->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_gfx_setup);

int histb_vdp_pipeline_gfx_set_addr(struct histb_vdp *vdp, u32 addr)
{
	if (!vdp)
		return -EPROBE_DEFER;

	mutex_lock(&vdp->lock);
	if (!vdp->pm_active) {
		mutex_unlock(&vdp->lock);
		return -EPIPE;
	}
	histb_vdp_gfx_set_addr_locked(vdp, addr);
	mutex_unlock(&vdp->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_gfx_set_addr);

int histb_vdp_pipeline_gfx_enable(struct histb_vdp *vdp, bool enable)
{
	if (!vdp)
		return -EPROBE_DEFER;

	mutex_lock(&vdp->lock);
	if (!vdp->pm_active) {
		mutex_unlock(&vdp->lock);
		return -EPIPE;
	}
	histb_vdp_gfx_set_enable_locked(vdp, enable);
	mutex_unlock(&vdp->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_gfx_enable);

int histb_vdp_pipeline_gfx_set_blend(struct histb_vdp *vdp,
				     u16 alpha, u16 pixel_blend_mode)
{
	int ret;

	if (!vdp)
		return -EPROBE_DEFER;

	mutex_lock(&vdp->lock);
	if (!vdp->pm_active) {
		mutex_unlock(&vdp->lock);
		return -EPIPE;
	}
	ret = histb_vdp_gfx_set_blend_locked(vdp, alpha, pixel_blend_mode);
	mutex_unlock(&vdp->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_gfx_set_blend);

int histb_vdp_pipeline_video_setup(struct histb_vdp *vdp,
				   u32 src_x, u32 src_y,
				   u32 src_width, u32 src_height,
				   u32 dst_x, u32 dst_y,
				   u32 dst_width, u32 dst_height,
				   u32 luma_stride, u32 chroma_stride)
{
	struct histb_vdp_video_cfg cfg = {
		.src_x = src_x,
		.src_y = src_y,
		.src_width = src_width,
		.src_height = src_height,
		.dst_x = dst_x,
		.dst_y = dst_y,
		.dst_width = dst_width,
		.dst_height = dst_height,
		.luma_stride = luma_stride,
		.chroma_stride = chroma_stride,
	};
	int ret;

	if (!vdp)
		return -EPROBE_DEFER;

	ret = histb_vdp_video_validate_cfg(&cfg);
	if (ret)
		return ret;

	mutex_lock(&vdp->lock);
	if (!vdp->pm_active) {
		mutex_unlock(&vdp->lock);
		return -EPIPE;
	}

	if (vdp->video_cfg_valid &&
	    !memcmp(&vdp->video_cfg, &cfg, sizeof(cfg))) {
		mutex_unlock(&vdp->lock);
		return 0;
	}

	histb_vdp_video_apply_cfg(vdp, &cfg);
	vdp->video_cfg = cfg;
	vdp->video_cfg_valid = true;
	mutex_unlock(&vdp->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_video_setup);

int histb_vdp_pipeline_video_set_addr(struct histb_vdp *vdp, u32 luma_addr,
				      u32 chroma_addr)
{
	if (!vdp)
		return -EPROBE_DEFER;

	mutex_lock(&vdp->lock);
	if (!vdp->pm_active) {
		mutex_unlock(&vdp->lock);
		return -EPIPE;
	}
	histb_vdp_video_set_addr_locked(vdp, luma_addr, chroma_addr);
	mutex_unlock(&vdp->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_video_set_addr);

int histb_vdp_pipeline_video_enable(struct histb_vdp *vdp, bool enable)
{
	if (!vdp)
		return -EPROBE_DEFER;

	mutex_lock(&vdp->lock);
	if (!vdp->pm_active) {
		mutex_unlock(&vdp->lock);
		return -EPIPE;
	}
	histb_vdp_video_set_enable_locked(vdp, enable);
	mutex_unlock(&vdp->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_video_enable);

int histb_vdp_pipeline_video_set_alpha(struct histb_vdp *vdp, u16 alpha)
{
	if (!vdp)
		return -EPROBE_DEFER;

	mutex_lock(&vdp->lock);
	if (!vdp->pm_active) {
		mutex_unlock(&vdp->lock);
		return -EPIPE;
	}
	histb_vdp_video_set_alpha_locked(vdp, alpha);
	mutex_unlock(&vdp->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_video_set_alpha);

int histb_vdp_pipeline_video_set_colorspace(struct histb_vdp *vdp,
					    enum drm_color_encoding encoding,
					    enum drm_color_range range)
{
	int ret;

	if (!vdp)
		return -EPROBE_DEFER;

	mutex_lock(&vdp->lock);
	if (!vdp->pm_active) {
		mutex_unlock(&vdp->lock);
		return -EPIPE;
	}
	ret = histb_vdp_video_set_colorspace_locked(vdp, encoding, range);
	mutex_unlock(&vdp->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_video_set_colorspace);

int histb_vdp_pipeline_set_zorder(struct histb_vdp *vdp, bool video_on_top)
{
	if (!vdp)
		return -EPROBE_DEFER;

	mutex_lock(&vdp->lock);
	if (!vdp->pm_active) {
		mutex_unlock(&vdp->lock);
		return -EPIPE;
	}
	histb_vdp_set_mix1_zorder_locked(vdp, video_on_top);
	mutex_unlock(&vdp->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(histb_vdp_pipeline_set_zorder);

static int histb_vdp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_vdp *vdp;
	int i;
	int ret;

	vdp = devm_kzalloc(dev, sizeof(*vdp), GFP_KERNEL);
	if (!vdp)
		return -ENOMEM;

	vdp->dev = dev;
	mutex_init(&vdp->lock);
	vdp->video_color_encoding = DRM_COLOR_YCBCR_BT601;
	vdp->video_color_range = DRM_COLOR_YCBCR_LIMITED_RANGE;

	for (i = 0; i < HISTB_VDP_NUM_CLKS; i++)
		vdp->clks[i].id = histb_vdp_clk_names[i];

	vdp->irq = platform_get_irq(pdev, 0);
	if (vdp->irq < 0)
		return vdp->irq;

	vdp->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vdp->regs))
		return PTR_ERR(vdp->regs);

	vdp->crg = syscon_regmap_lookup_by_phandle(dev->of_node,
						   "hisilicon,crg-syscon");
	if (IS_ERR(vdp->crg))
		return dev_err_probe(dev, PTR_ERR(vdp->crg),
				     "failed to get CRG regmap\n");

	vdp->rst = devm_reset_control_get_exclusive(dev, "vou");
	if (IS_ERR(vdp->rst))
		return dev_err_probe(dev, PTR_ERR(vdp->rst),
				     "failed to get VOU reset\n");

	ret = devm_clk_bulk_get(dev, HISTB_VDP_NUM_CLKS, vdp->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get VO clocks\n");

	/*
	 * Kept disabled until the pipeline is enabled: the block may not be
	 * powered yet, and a bootloader splash can leave the line asserted.
	 */
	ret = devm_request_irq(dev, vdp->irq, histb_vdp_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), vdp);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request VDP irq\n");

	platform_set_drvdata(pdev, vdp);

	ret = histb_vdp_runtime_resume(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize VDP runtime state\n");

	ret = devm_add_action_or_reset(dev, histb_vdp_pm_cleanup_action, vdp);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(dev, 200);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	dev_info(dev,
		 "initialized DHD0 timing, output gated until HDMI pipeline enables it (mixg0=%#x cbm_attr=%#x cbm_mix1=%#x gp0_ctrl=%#x)\n",
		 histb_vdp_read(vdp, HISTB_VDP_MIXG0_MIX),
		 histb_vdp_read(vdp, HISTB_VDP_CBM_ATTR),
		 histb_vdp_read(vdp, HISTB_VDP_CBM_MIX1),
		 histb_vdp_read(vdp, HISTB_VDP_GP0_CTRL));
	pm_runtime_mark_last_busy(dev);
	pm_runtime_idle(dev);

	return 0;
}

static const struct of_device_id histb_vdp_of_match[] = {
	{ .compatible = "hisilicon,hi3798mv100-vdp" },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_vdp_of_match);

static int __maybe_unused histb_vdp_pm_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);
}

static int __maybe_unused histb_vdp_pm_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}

static const struct dev_pm_ops histb_vdp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(histb_vdp_pm_suspend,
				histb_vdp_pm_resume)
	SET_RUNTIME_PM_OPS(histb_vdp_runtime_suspend,
			   histb_vdp_runtime_resume, NULL)
};

static struct platform_driver histb_vdp_platform_driver = {
	.probe = histb_vdp_probe,
	.driver = {
		.name = "histb-vdp",
		.of_match_table = histb_vdp_of_match,
		.pm = pm_ptr(&histb_vdp_pm_ops),
	},
};
module_platform_driver(histb_vdp_platform_driver);

MODULE_AUTHOR("Hisilicon community");
MODULE_DESCRIPTION("HiSilicon STB VDP driver");
MODULE_LICENSE("GPL");
