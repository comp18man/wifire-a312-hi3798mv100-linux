// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon STB HDMI driver for hi3798mv100.
 *
 * Reset/clock/PHY/TX sequencing plus timing programmed from
 * drm_display_mode, with a fixed/EDID fallback before the first modeset.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/string.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_eld.h>
#include <drm/display/drm_hdcp_helper.h>
#include <drm/display/drm_hdmi_cec_helper.h>
#include <drm/display/drm_hdmi_state_helper.h>

#include <media/cec.h>
#include <sound/hdmi-codec.h>

#include "histb_pipeline.h"

#define CREATE_TRACE_POINTS
#include <trace/events/histb_hdmi.h>

#define HISTB_HDMI_TX0_BASE				0x000
#define HISTB_HDMI_TX1_BASE				0x100
#define HISTB_HDMI_PHY_BASE				0x1800

#define HISTB_HDMI_TX_SWRST_ADDR			0x05
#define HISTB_HDMI_TX_SYS_CTRL1_ADDR			0x08
#define HISTB_HDMI_TX_STAT_ADDR				0x09
#define HISTB_HDMI_DATA_CTRL_ADDR			0x0d
#define HISTB_HDMI_HDCP_CTRL_ADDR			0x0f
#define HISTB_HDMI_DE_HSTART_ADDR			0x32
#define HISTB_HDMI_DE_CNTRL_ADDR			0x33
#define HISTB_HDMI_DE_VSTART_ADDR			0x34
#define HISTB_HDMI_DE_HRES_ADDR				0x36
#define HISTB_HDMI_DE_VRES_ADDR				0x38
#define HISTB_HDMI_HTOTAL_ADDR				0x3a
#define HISTB_HDMI_VTOTAL_ADDR				0x3c
#define HISTB_HDMI_INTERLACE_ADJ_MODE_ADDR		0x3e
#define HISTB_HDMI_HBIT_TO_HSYNC_ADDR			0x40
#define HISTB_HDMI_FIELD2_HSYNC_OFFSET_ADDR		0x42
#define HISTB_HDMI_HLENGTH_ADDR				0x44
#define HISTB_HDMI_VBIT_TO_VSYNC_ADDR			0x46
#define HISTB_HDMI_VLENGTH_ADDR				0x47
#define HISTB_HDMI_TX_VID_CTRL_ADDR			0x48
#define HISTB_HDMI_VID_ACEN_ADDR			0x49
#define HISTB_HDMI_TX_VID_MODE_ADDR			0x4a
#define HISTB_HDMI_TX_VID_DITHER_ADDR			0x4f
#define HISTB_HDMI_VID_IN_MODE_ADDR			0x69
#define HISTB_HDMI_HDMI_INT_STATE_ADDR			0x70
#define HISTB_HDMI_HDMI_INT_ADDR			0x71
#define HISTB_HDMI_HDMI_INT_MASK_ADDR			0x76
#define HISTB_HDMI_INT_CNTRL_ADDR			0x7b
#define HISTB_HDMI_DDC_DELAY_CNT			0xf6

#define HISTB_HDMI_AUDP_TXCTRL_ADDR			0x2f
#define HISTB_HDMI_TEST_TX_ADDR				0x3c

#define HISTB_HDMI_TX_AUDIO_CTRL_ADDR			0x1000
#define HISTB_HDMI_AUD_I2S_CTRL_ADDR			0x1004
#define HISTB_HDMI_AUD_CHST_CFG0_ADDR			0x100c
#define HISTB_HDMI_AUD_CHST_CFG1_ADDR			0x1010
#define HISTB_HDMI_AUD_FIFO_CTRL_ADDR			0x1018
#define HISTB_HDMI_AUD_ACR_CTRL_ADDR			0x1040
#define HISTB_HDMI_ACR_N_VAL_SW_ADDR			0x1048
#define HISTB_HDMI_CEA_AUD_CFG_ADDR			0x19ac
/* Legacy TX1 audio page-style registers (vendor Page_A layout). */
#define HISTB_HDMI_AUD_ACR_CTRL_TX1_ADDR		0x01
#define HISTB_HDMI_FREQ_SVAL_TX1_ADDR			0x02
#define HISTB_HDMI_N_SVAL1_TX1_ADDR			0x03
#define HISTB_HDMI_N_SVAL2_TX1_ADDR			0x04
#define HISTB_HDMI_N_SVAL3_TX1_ADDR			0x05
#define HISTB_HDMI_AUD_EN_TX1_ADDR			0x13
#define HISTB_HDMI_AUD_MODE_TX1_ADDR			0x14
#define HISTB_HDMI_I2S_IN_CTRL_TX1_ADDR			0x1d
#define HISTB_HDMI_I2S_CHST4_TX1_ADDR			0x21
#define HISTB_HDMI_I2S_CHST5_TX1_ADDR			0x22
#define HISTB_HDMI_SAMPLE_RATE_CONV_TX1_ADDR		0x23
#define HISTB_HDMI_I2S_IN_SIZE_TX1_ADDR			0x24
#define HISTB_HDMI_AIP_RST_TX1_ADDR			0x2c
#define HISTB_HDMI_INF_CTRL1_TX1_ADDR			0x3e
#define HISTB_HDMI_AVI_IF_TX1_ADDR			0x40
#define HISTB_HDMI_AUD_IF_TX1_ADDR			0x80
#define HISTB_HDMI_MPEG_IF_TX1_ADDR			0xa0
#define HISTB_HDMI_TPI_DOWN_SMPL_CTRL_TX1_ADDR		0x61
#define HISTB_HDMI_TPI_AUD_CONFIG_TX1_ADDR		0x62
#define HISTB_HDMI_TPI_AUD_FS_TX1_ADDR			0x63
#define HISTB_HDMI_TX_PWD_RST_CTRL_ADDR			0x10

#define HISTB_HDMI_BIT_TX_PD				BIT(0)
#define HISTB_HDMI_BIT_TX_FIFO_RST			BIT(1)
#define HISTB_HDMI_BIT_TX_CLOCK_RISING_EDGE		BIT(1)
#define HISTB_HDMI_BIT_BSEL24BITS			BIT(2)
#define HISTB_HDMI_BIT_HPD_PIN				BIT(1)
#define HISTB_HDMI_BIT_AUD_MUTE				BIT(1)
#define HISTB_HDMI_BIT_VID_BLANK			BIT(2)
#define HISTB_HDMI_HDCP_BIT_ENC_EN			BIT(0)
#define HISTB_HDMI_HDCP_BIT_CP_RESET_N			BIT(2)
#define HISTB_HDMI_HDCP_BIT_AN_STOP			BIT(3)
#define HISTB_HDMI_HDCP_BIT_BKSV_ERROR			BIT(5)
#define HISTB_HDMI_HDCP_BIT_ENC_ON			BIT(6)
#define HISTB_HDMI_BIT_DE_ENABLED			BIT(6)
#define HISTB_HDMI_BIT_TXHDMI_MODE			BIT(0)
#define HISTB_HDMI_BIT_AUDP_AUD_MUTE_EN			BIT(7)
#define HISTB_HDMI_BIT_AUDP_LAYOUT			BIT(1)
#define HISTB_HDMI_BIT_DVI_ENC_BYPASS			BIT(3)
#define HISTB_HDMI_BIT_SET_CSCSEL			BIT(4)
#define HISTB_HDMI_BIT_TX_DITHER_EN			BIT(5)
#define HISTB_HDMI_TX_VID_CTRL_ICLK			GENMASK(1, 0)
#define HISTB_HDMI_TX_VID_MODE_CLR			GENMASK(5, 0)
#define HISTB_HDMI_PHY_PLL1_SWING_MASK			GENMASK(1, 0)

#define HISTB_HDMI_VID_IN_MODE_CLR			0xf9
#define HISTB_HDMI_VID_IN_MODE_24BIT			0x06
#define HISTB_HDMI_TX_VID_DITHER_24BIT			0x0e

#define HISTB_HDMI_BIT_INT_HOT_PLUG			BIT(6)
#define HISTB_HDMI_BIT_INT_RSEN				BIT(5)
#define HISTB_HDMI_CLR_MASK				(HISTB_HDMI_BIT_INT_HOT_PLUG | \
							 BIT(4) | BIT(3) | \
							 HISTB_HDMI_BIT_INT_RSEN)
#define HISTB_HDMI_INT_CONTROL				0x02
#define HISTB_HDMI_DDC_DELAY_DEFAULT			0x1f
#define HISTB_HDMI_HPD_STORM_WINDOW_MS			500
#define HISTB_HDMI_HPD_STORM_THRESHOLD			16
#define HISTB_HDMI_HPD_STORM_QUIET_MS			250
#define HISTB_HDMI_DIAG_DUMP_INTERVAL			(5 * HZ)
#define HISTB_HDMI_DIAG_DUMP_BURST			4

#define HISTB_HDMI_PHY_OE_ADDR				0x00
#define HISTB_HDMI_PHY_PWD_ADDR				0x01
#define HISTB_HDMI_PHY_AUD_ADDR				0x02
#define HISTB_HDMI_PHY_PLL1_ADDR			0x03
#define HISTB_HDMI_PHY_PLL2_ADDR			0x04
#define HISTB_HDMI_PHY_DRV_ADDR				0x05
#define HISTB_HDMI_PHY_CLK_ADDR				0x06

#define HISTB_HDMI_RG_TX_RSTB				BIT(0)

#define HISTB_HDMI_CRG67				0x010c
#define HISTB_HDMI_CRG67_CEC_CLK_SEL			BIT(12)
#define HISTB_HDMI_CRG67_AS_CLK_SEL			BIT(14)
#define HISTB_HDMI_PERICTRL_HDMITX_CTRL		0x08b0
#define HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT		0x0150
#define HISTB_HDMI_PERICTRL_AUD_SRC_MASK_STB		GENMASK(4, 0)
#define HISTB_HDMI_PERICTRL_AUD_SRC_I2S_STB		BIT(1)
#define HISTB_HDMI_PERICTRL_AUD_SRC_MASK_BVT		GENMASK(10, 6)
#define HISTB_HDMI_PERICTRL_AUD_SRC_I2S_BVT		BIT(7)

#define HISTB_HDMI_MDDC_BASE				0xec
#define HISTB_HDMI_MDDC_SEGMENT_ADDR			0xee
#define HISTB_HDMI_MDDC_STATUS_ADDR			0xf2
#define HISTB_HDMI_MDDC_COMMAND_ADDR			0xf3
#define HISTB_HDMI_MDDC_FIFO_ADDR			0xf4
#define HISTB_HDMI_MDDC_FIFO_CNT_ADDR			0xf5

#define HISTB_HDMI_MDDC_ST_I2C_LOW			BIT(6)
#define HISTB_HDMI_MDDC_ST_NO_ACK			BIT(5)
#define HISTB_HDMI_MDDC_ST_IN_PROGR			BIT(4)
#define HISTB_HDMI_MDDC_ST_FIFO_FULL			BIT(3)

#define HISTB_HDMI_MDDC_CMD_ABORT			0x0f
#define HISTB_HDMI_MDDC_CMD_CLEAR_FIFO			0x09
#define HISTB_HDMI_MDDC_CMD_CLOCK			0x0a
#define HISTB_HDMI_MDDC_CMD_SEQ_RD			0x02
#define HISTB_HDMI_MDDC_CMD_ENH_RD			0x04

#define HISTB_HDMI_DDC_ADDR				0xa0

#define HISTB_HDMI_AUD_CTRL_IN_EN			BIT(0)
#define HISTB_HDMI_AUD_CTRL_MUTE_EN			BIT(1)
#define HISTB_HDMI_AUD_CTRL_LAYOUT			BIT(2)
#define HISTB_HDMI_AUD_CTRL_I2S_EN			GENMASK(7, 4)
#define HISTB_HDMI_AUD_CTRL_SPDIF_EN			BIT(8)
#define HISTB_HDMI_AUD_CTRL_SRC_EN			BIT(9)
#define HISTB_HDMI_AUD_CTRL_SRC_CTRL			BIT(10)
#define HISTB_HDMI_AUD_CTRL_FIFO0_MAP			GENMASK(13, 12)
#define HISTB_HDMI_AUD_CTRL_FIFO1_MAP			GENMASK(15, 14)
#define HISTB_HDMI_AUD_CTRL_FIFO2_MAP			GENMASK(17, 16)
#define HISTB_HDMI_AUD_CTRL_FIFO3_MAP			GENMASK(19, 18)

#define HISTB_HDMI_AUD_I2S_CTRL_LENGTH			GENMASK(11, 8)
#define HISTB_HDMI_AUD_I2S_CTRL_CH_SWAP			GENMASK(15, 12)
#define HISTB_HDMI_AUD_FIFO_CTRL_TEST			GENMASK(4, 0)
#define HISTB_HDMI_AUD_FIFO_CTRL_HBR_MASK		GENMASK(11, 8)

#define HISTB_HDMI_AUD_CHST_CFG0_FS			GENMASK(27, 24)
#define HISTB_HDMI_AUD_CHST_CFG0_CLK_ACC		GENMASK(31, 28)
#define HISTB_HDMI_AUD_CHST_CFG1_LENGTH			GENMASK(3, 0)
#define HISTB_HDMI_AUD_CHST_CFG1_ORG_FS			GENMASK(7, 4)

#define HISTB_HDMI_AUD_ACR_CTS_REQ_EN			BIT(0)
#define HISTB_HDMI_AUD_ACR_CTS_HW_SW_SEL		BIT(1)
#define HISTB_HDMI_AUD_ACR_CTS_GEN_SEL			BIT(2)
#define HISTB_HDMI_AUD_ACR_CTRL_MASK			GENMASK(5, 0)

#define HISTB_HDMI_CEA_AUD_EN				BIT(0)
#define HISTB_HDMI_CEA_AUD_RPT_EN			BIT(1)

#define HISTB_HDMI_AUD_ACR_TX1_CTS_HW_SW_SEL		BIT(0)
#define HISTB_HDMI_AUD_ACR_TX1_CTS_REQ_EN		BIT(1)
#define HISTB_HDMI_AUD_ACR_TX1_MCLK_EN			BIT(2)
#define HISTB_HDMI_AUD_ACR_TX1_NO_MCLK_CTSGEN_SEL	BIT(3)
#define HISTB_HDMI_AUD_ACR_TX1_MASK			GENMASK(3, 0)

#define HISTB_HDMI_AUD_MODE_TX1_AUDIO_EN		BIT(0)
#define HISTB_HDMI_AUD_MODE_TX1_SPDIF_SEL		BIT(1)
#define HISTB_HDMI_AUD_MODE_TX1_HBRA_ON			BIT(2)
#define HISTB_HDMI_AUD_MODE_TX1_DSD_SEL			BIT(3)
#define HISTB_HDMI_AUD_MODE_TX1_I2S_EN			GENMASK(7, 4)
#define HISTB_HDMI_AUD_MODE_TX1_I2S_SD0			0x1
#define HISTB_HDMI_AUD_MODE_TX1_I2S_ALL			0xf
#define HISTB_HDMI_AUD_EN_TX1_AUD_IN_EN			BIT(0)
#define HISTB_HDMI_AUD_EN_TX1_AUD_SEL_OWRT		BIT(1)
#define HISTB_HDMI_INF_CTRL1_AVI_REPEAT			BIT(0)
#define HISTB_HDMI_INF_CTRL1_AVI_ENABLE			BIT(1)
#define HISTB_HDMI_INF_CTRL1_AUD_REPEAT			BIT(4)
#define HISTB_HDMI_INF_CTRL1_AUD_ENABLE			BIT(5)
#define HISTB_HDMI_INF_CTRL1_MPEG_REPEAT			BIT(6)
#define HISTB_HDMI_INF_CTRL1_MPEG_ENABLE			BIT(7)
#define HISTB_HDMI_TPI_DOWN_SMPL_AUD_HNDL		GENMASK(1, 0)
#define HISTB_HDMI_TPI_DOWN_SMPL_LOOKUP_EN		BIT(2)
#define HISTB_HDMI_TPI_AUD_CONFIG_MUTE			BIT(4)
#define HISTB_HDMI_TPI_AUD_FS_VAL			GENMASK(5, 0)
#define HISTB_HDMI_TPI_AUD_FS_OVRD			BIT(7)
#define HISTB_HDMI_TPI_AUD_FS_44K			0x00
#define HISTB_HDMI_TPI_AUD_FS_48K			0x02
#define HISTB_HDMI_TPI_AUD_FS_32K			0x03
#define HISTB_HDMI_TPI_AUD_FS_88K			0x08
#define HISTB_HDMI_TPI_AUD_FS_96K			0x0a
#define HISTB_HDMI_TPI_AUD_FS_176K			0x0c
#define HISTB_HDMI_TPI_AUD_FS_192K			0x0e

#define HISTB_HDMI_AUD_N_44K				6272
#define HISTB_HDMI_AUD_N_48K				6144
#define HISTB_HDMI_AUD_N_32K				4096
#define HISTB_HDMI_AUD_N_88K				12544
#define HISTB_HDMI_AUD_N_96K				12288
#define HISTB_HDMI_AUD_N_176K				25088
#define HISTB_HDMI_AUD_N_192K				24576
#define HISTB_HDMI_AUD_FREQ_SVAL_256FS			0x01
#define HISTB_HDMI_AUD_SAMPLE_RATE_CONV_BYPASS		0x00

#define HISTB_HDMI_TX_PWD_RST_CTRL_AUD_SRST		BIT(6)
#define HISTB_HDMI_TX_PWD_RST_CTRL_ACR_SRST		BIT(7)
#define HISTB_HDMI_TX_PWD_RST_CTRL_AFIFO_SRST		BIT(8)

#define HISTB_HDMI_AIP_RST_TX1_AUDIO			BIT(0)
#define HISTB_HDMI_AIP_RST_TX1_FIFO			BIT(1)
#define HISTB_HDMI_AIP_RST_TX1_ACR			BIT(2)
#define HISTB_HDMI_I2S_IN_CTRL_TX1_CBIT_ORDER		BIT(5)
#define HISTB_HDMI_I2S_IN_CTRL_TX1_SCK_RISING		BIT(6)

#define HISTB_EDID_BLOCK_SIZE				128
#define HISTB_EDID_CHUNK_SIZE				16
#define HISTB_EDID_EXT_CEA				0x02
#define HISTB_EDID_CEA_TAG_AUDIO				1
#define HISTB_EDID_CEA_TAG_SPEAKER				4

#define HISTB_HDMI_HDCP_MAX_RETRIES			30
#define HISTB_HDMI_HDCP_RETRY_US_MIN			10000
#define HISTB_HDMI_HDCP_RETRY_US_MAX			12000

#if IS_ENABLED(CONFIG_DRM_DISPLAY_HDMI_CEC_HELPER)
#define HISTB_HDMI_CEC_BASE				0x200
#define HISTB_HDMI_CEC_DEBUG_3_ADDR			0x87
#define HISTB_HDMI_CEC_TX_INIT_ADDR			0x88
#define HISTB_HDMI_CEC_TX_DEST_ADDR			0x89
#define HISTB_HDMI_CEC_CONFIG_CPI_ADDR			0x8e
#define HISTB_HDMI_CEC_TX_COMMAND_ADDR			0x8f
#define HISTB_HDMI_CEC_TX_OPERANDS_0_ADDR		0x90
#define HISTB_HDMI_CEC_TRANSMIT_DATA_ADDR		0x9f
#define HISTB_HDMI_CEC_RETRY_LIMIT_ADDR			0xa0
#define HISTB_HDMI_CEC_CAPTURE_ID0_ADDR			0xa2
#define HISTB_HDMI_CEC_INT_ENABLE_0_ADDR		0xa4
#define HISTB_HDMI_CEC_INT_ENABLE_1_ADDR		0xa5
#define HISTB_HDMI_CEC_INT_STATUS_0_ADDR		0xa6
#define HISTB_HDMI_CEC_INT_STATUS_1_ADDR		0xa7
#define HISTB_HDMI_CEC_RX_CONTROL_ADDR			0xac
#define HISTB_HDMI_CEC_RX_COUNT_ADDR			0xad
#define HISTB_HDMI_CEC_RX_CMD_HEADER_ADDR		0xae
#define HISTB_HDMI_CEC_RX_COMMAND_ADDR			0xaf
#define HISTB_HDMI_CEC_RX_OPERAND_0_ADDR		0xb0

#define HISTB_HDMI_CEC_BIT_FLUSH_TX_FIFO		BIT(7)
#define HISTB_HDMI_CEC_BIT_SEND_POLL			BIT(7)
#define HISTB_HDMI_CEC_BIT_TRANSMIT_CMD			BIT(4)

#define HISTB_HDMI_CEC_BIT_TX_MESSAGE_SENT		BIT(5)
#define HISTB_HDMI_CEC_BIT_TX_FIFO_EMPTY			BIT(2)
#define HISTB_HDMI_CEC_BIT_RX_MSG_RECEIVED		BIT(1)

#define HISTB_HDMI_CEC_BIT_RX_FIFO_OVERRUN		BIT(3)
#define HISTB_HDMI_CEC_BIT_FRAME_RETRANSM_OV		BIT(1)
#define HISTB_HDMI_CEC_BIT_SHORT_PULSE_DET		BIT(2)
#define HISTB_HDMI_CEC_BIT_START_IRREGULAR		BIT(0)

#define HISTB_HDMI_CEC_BIT_CLR_RX_FIFO_CUR		BIT(0)
#define HISTB_HDMI_CEC_BIT_CLR_RX_FIFO_ALL		BIT(1)

#define HISTB_HDMI_CEC_BIT_MSG_ERROR			BIT(7)
#define HISTB_HDMI_CEC_RX_FRAME_CNT_MASK		GENMASK(6, 4)
#define HISTB_HDMI_CEC_RX_OPERAND_CNT_MASK		GENMASK(3, 0)
#define HISTB_HDMI_CEC_INT_STATUS_0_MASK		GENMASK(6, 0)

#define HISTB_HDMI_CEC_CONFIG_CPI_DEFAULT		0x04
#define HISTB_HDMI_CEC_AVAILABLE_LAS			1
#endif

#define HISTB_HDMI_ELD_MAX_BYTES				128
#define HISTB_HDMI_ELD_MNL_MAX					16
#define HISTB_HDMI_ELD_MAX_SAD					15

#define HISTB_EDID_SAD_CHANNELS_MASK				GENMASK(2, 0)
#define HISTB_EDID_SAD_FORMAT_MASK				GENMASK(6, 3)
#define HISTB_EDID_SAD_FORMAT_LPCM				1
#define HISTB_EDID_SAD_RATE_32K_MASK				BIT(0)
#define HISTB_EDID_SAD_RATE_44K1_MASK				BIT(1)
#define HISTB_EDID_SAD_RATE_48K_MASK				BIT(2)
#define HISTB_EDID_SAD_RATE_88K2_MASK				BIT(3)
#define HISTB_EDID_SAD_RATE_96K_MASK				BIT(4)
#define HISTB_EDID_SAD_RATE_176K4_MASK				BIT(5)
#define HISTB_EDID_SAD_RATE_192K_MASK				BIT(6)
#define HISTB_EDID_SAD_WIDTH_16_MASK				BIT(0)
#define HISTB_EDID_SAD_WIDTH_20_MASK				BIT(1)
#define HISTB_EDID_SAD_WIDTH_24_MASK				BIT(2)

static int histb_fixed_mode = 720;
module_param_named(fixed_mode, histb_fixed_mode, int, 0644);
MODULE_PARM_DESC(fixed_mode, "Fallback fixed mode: 720 or 1080");

static bool histb_use_edid = true;
module_param_named(use_edid, histb_use_edid, bool, 0644);
MODULE_PARM_DESC(use_edid, "Enable EDID read and mode auto-select");

/*
 * Keep a single production profile matching HDMI 1.4 vendor flow:
 * - AUD_MODE uses SD0 lane plus AUDIO_EN (0x11 equivalent),
 * - runtime path stays tx1-oriented (no A/B switches).
 */

static bool histb_hdmi_tx1_only_audio_path(void);

enum histb_hdmi_clk_id {
	HISTB_HDMI_CLK_BUS,
	HISTB_HDMI_CLK_CEC,
	HISTB_HDMI_CLK_ID,
	HISTB_HDMI_CLK_MHL,
	HISTB_HDMI_CLK_OS,
	HISTB_HDMI_CLK_AS,
	HISTB_HDMI_CLK_PHY_BUS,
	HISTB_HDMI_NUM_CLKS,
};

struct histb_hdmi_mode {
	u16 hstart;
	u16 vstart;
	u16 hres;
	u16 vres;
	u16 htotal;
	u16 vtotal;
	u16 hbit_to_hsync;
	u16 field2_hsync_offset;
	u16 hlength;
	u8 vbit_to_vsync;
	u8 vlength;
	u8 int_adj_mode;
	u8 de_polarity;
	u8 phy_pll1_swing;
};

static const struct drm_display_mode histb_hdmi_720p60_mode = {
	.clock = 74250,
	.hdisplay = 1280,
	.hsync_start = 1390,
	.hsync_end = 1430,
	.htotal = 1650,
	.vdisplay = 720,
	.vsync_start = 725,
	.vsync_end = 730,
	.vtotal = 750,
	.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
};

static const struct drm_display_mode histb_hdmi_1080p60_mode = {
	.clock = 148500,
	.hdisplay = 1920,
	.hsync_start = 2008,
	.hsync_end = 2052,
	.htotal = 2200,
	.vdisplay = 1080,
	.vsync_start = 1084,
	.vsync_end = 1089,
	.vtotal = 1125,
	.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
};

struct histb_hdmi {
	struct device *dev;
	struct drm_bridge bridge;
	struct drm_connector *connector;
	void __iomem *regs;
	struct regmap *crg;
	struct regmap *perictrl;
	struct reset_control *rst_bus;
	struct reset_control *rst_ctrl;
	struct reset_control *rst_phy;
	struct clk_bulk_data clks[HISTB_HDMI_NUM_CLKS];
	struct mutex lock;
	struct drm_display_mode mode;
	bool mode_from_bridge;
	struct platform_device *audio_pdev;
	bool audio_stream_active;
	bool audio_requested_mute;
	unsigned int audio_sample_rate;
	unsigned int audio_sample_width;
	unsigned int audio_channels;
	struct hdmi_audio_infoframe audio_cea;
	int irq;
	bool hpd_active;
	bool pm_ref_active;
	bool system_suspended;
	unsigned long hpd_storm_window_start;
	unsigned long hpd_storm_ignore_until;
	u16 hpd_storm_events;
	u32 stats_irq_total;
	u32 stats_irq_hpd_rsen;
	u32 stats_hpd_in;
	u32 stats_hpd_out;
	u32 stats_hpd_storm;
	u32 stats_recover_attempts;
	u32 stats_recover_success;
	u32 stats_recover_fail;
	u32 stats_mode_apply_fail;
	u32 stats_edid_fail;
	u32 stats_runtime_resume_fail;
	struct ratelimit_state diag_dump_rs;
	bool eld_valid;
	u8 eld[HISTB_HDMI_ELD_MAX_BYTES];
	bool hdcp_enabled;
	u8 hdcp_content_type;
#if IS_ENABLED(CONFIG_DRM_DISPLAY_HDMI_CEC_HELPER)
	bool cec_enabled;
	bool cec_tx_in_progress;
	u8 cec_logical_addr;
	u8 cec_attempts;
#endif
};

static int histb_hdmi_read_edid_block(struct histb_hdmi *hdmi, u8 block, u8 *buf);
static bool histb_hdmi_edid_checksum_ok(const u8 *block);
static bool histb_hdmi_edid_header_ok(const u8 *block);
static int histb_hdmi_apply_mode(struct histb_hdmi *hdmi,
				 const struct drm_display_mode *mode);
static int histb_hdmi_handle_hpd_event(struct histb_hdmi *hdmi);
static int histb_hdmi_link_recover_locked(struct histb_hdmi *hdmi,
					  const struct drm_display_mode *mode,
					  const char *reason);
static void histb_hdmi_link_quiesce_locked(struct histb_hdmi *hdmi, bool clear_eld);
static void histb_hdmi_dump_status_ratelimited(struct histb_hdmi *hdmi,
					       const char *tag, int err,
					       bool regs_valid);
static void histb_hdmi_trace_error_locked(struct histb_hdmi *hdmi,
					  const char *where, int err,
					  bool regs_valid);

static inline struct histb_hdmi *bridge_to_histb_hdmi(struct drm_bridge *bridge)
{
	return container_of(bridge, struct histb_hdmi, bridge);
}

static int histb_hdmi_runtime_get(struct histb_hdmi *hdmi)
{
	int ret;

	ret = pm_runtime_resume_and_get(hdmi->dev);
	if (ret < 0)
		return ret;

	return 0;
}

static void histb_hdmi_runtime_put(struct histb_hdmi *hdmi)
{
	pm_runtime_mark_last_busy(hdmi->dev);
	pm_runtime_put_autosuspend(hdmi->dev);
}

static const char * const histb_hdmi_clk_names[HISTB_HDMI_NUM_CLKS] = {
	[HISTB_HDMI_CLK_BUS] = "bus",
	[HISTB_HDMI_CLK_CEC] = "cec",
	[HISTB_HDMI_CLK_ID] = "id",
	[HISTB_HDMI_CLK_MHL] = "mhl",
	[HISTB_HDMI_CLK_OS] = "os",
	[HISTB_HDMI_CLK_AS] = "as",
	[HISTB_HDMI_CLK_PHY_BUS] = "phy_bus",
};

static inline u32 histb_hdmi_read_tx0(struct histb_hdmi *hdmi, u32 reg)
{
	return readl_relaxed(hdmi->regs + (HISTB_HDMI_TX0_BASE + reg) * 4);
}

static inline void histb_hdmi_write_tx0(struct histb_hdmi *hdmi, u32 reg, u32 val)
{
	writel_relaxed(val, hdmi->regs + (HISTB_HDMI_TX0_BASE + reg) * 4);
}

static inline u32 histb_hdmi_read_tx1(struct histb_hdmi *hdmi, u32 reg)
{
	return readl_relaxed(hdmi->regs + (HISTB_HDMI_TX1_BASE + reg) * 4);
}

static inline void histb_hdmi_write_tx1(struct histb_hdmi *hdmi, u32 reg, u32 val)
{
	writel_relaxed(val, hdmi->regs + (HISTB_HDMI_TX1_BASE + reg) * 4);
}

static inline void histb_hdmi_write_word_tx0(struct histb_hdmi *hdmi, u32 reg, u16 val)
{
	histb_hdmi_write_tx0(hdmi, reg, val & 0xff);
	histb_hdmi_write_tx0(hdmi, reg + 1, (val >> 8) & 0xff);
}

static inline u32 histb_hdmi_read(struct histb_hdmi *hdmi, u32 reg)
{
	/*
	 * This IP exposes 8/32-bit register addresses in an indexed space where
	 * each register slot is spaced by 4 bytes in MMIO.
	 */
	return readl_relaxed(hdmi->regs + reg * 4);
}

static inline void histb_hdmi_write(struct histb_hdmi *hdmi, u32 reg, u32 val)
{
	writel_relaxed(val, hdmi->regs + reg * 4);
}

static inline u32 histb_hdmi_read_linear(struct histb_hdmi *hdmi, u32 reg)
{
	return readl_relaxed(hdmi->regs + reg);
}

static inline void histb_hdmi_write_linear(struct histb_hdmi *hdmi, u32 reg, u32 val)
{
	writel_relaxed(val, hdmi->regs + reg);
}

static inline u32 histb_hdmi_audio_read_prefer_linear(struct histb_hdmi *hdmi,
						      u32 reg)
{
	return histb_hdmi_read(hdmi, reg);
}

static inline void histb_hdmi_audio_write_both(struct histb_hdmi *hdmi, u32 reg,
						u32 val)
{
	histb_hdmi_write(hdmi, reg, val);
}

static inline void histb_hdmi_write_phy(struct histb_hdmi *hdmi, u32 reg, u32 val)
{
	writel_relaxed(val, hdmi->regs + HISTB_HDMI_PHY_BASE + reg * 4);
}

static inline u32 histb_hdmi_read_phy(struct histb_hdmi *hdmi, u32 reg)
{
	return readl_relaxed(hdmi->regs + HISTB_HDMI_PHY_BASE + reg * 4);
}

static void histb_hdmi_dump_status_ratelimited(struct histb_hdmi *hdmi,
					       const char *tag, int err,
					       bool regs_valid)
{
	u32 tx_stat = 0;
	u32 int_state = 0;
	u32 int_raw = 0;
	u32 data_ctrl = 0;
	u32 tx_swrst = 0;
	u32 phy_oe = 0;
	u32 mddc_status = 0;
	const char *reason = tag ?: "unknown";

	if (!__ratelimit(&hdmi->diag_dump_rs))
		return;

	if (regs_valid) {
		tx_stat = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_TX_STAT_ADDR);
		int_state = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_HDMI_INT_STATE_ADDR);
		int_raw = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_HDMI_INT_ADDR);
		data_ctrl = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_DATA_CTRL_ADDR);
		tx_swrst = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_TX_SWRST_ADDR);
		phy_oe = histb_hdmi_read_phy(hdmi, HISTB_HDMI_PHY_OE_ADDR);
		mddc_status = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_MDDC_STATUS_ADDR);
	}

	dev_warn(hdmi->dev,
		 "diag[%s]: err=%d mode=%s hpd=%d irq_total=%u hpd_irq=%u in=%u out=%u storm=%u rec=%u/%u/%u mode_fail=%u edid_fail=%u pm_fail=%u\n",
		 reason, err, hdmi->mode.name, hdmi->hpd_active,
		 hdmi->stats_irq_total, hdmi->stats_irq_hpd_rsen,
		 hdmi->stats_hpd_in, hdmi->stats_hpd_out, hdmi->stats_hpd_storm,
		 hdmi->stats_recover_attempts, hdmi->stats_recover_success,
		 hdmi->stats_recover_fail, hdmi->stats_mode_apply_fail,
		 hdmi->stats_edid_fail, hdmi->stats_runtime_resume_fail);

	if (!regs_valid) {
		dev_warn(hdmi->dev,
			 "diag[%s]: register dump skipped (runtime inactive)\n",
			 reason);
		return;
	}

	dev_warn(hdmi->dev,
		 "diag[%s]: tx_stat=0x%02x int_state=0x%02x int_raw=0x%02x data_ctrl=0x%02x tx_swrst=0x%02x phy_oe=0x%02x mddc=0x%02x\n",
		 reason, tx_stat, int_state, int_raw, data_ctrl, tx_swrst,
		 phy_oe, mddc_status);
}

static void histb_hdmi_trace_error_locked(struct histb_hdmi *hdmi,
					  const char *where, int err,
					  bool regs_valid)
{
	trace_histb_hdmi_error(where, err);
	histb_hdmi_dump_status_ratelimited(hdmi, where, err, regs_valid);
}

#if IS_ENABLED(CONFIG_DRM_DISPLAY_HDMI_CEC_HELPER)
static inline u32 histb_hdmi_read_cec(struct histb_hdmi *hdmi, u32 reg)
{
	return histb_hdmi_read(hdmi, HISTB_HDMI_CEC_BASE + reg);
}

static inline void histb_hdmi_write_cec(struct histb_hdmi *hdmi, u32 reg, u32 val)
{
	histb_hdmi_write(hdmi, HISTB_HDMI_CEC_BASE + reg, val);
}
#endif

static bool histb_hdmi_hpd_status(struct histb_hdmi *hdmi)
{
	return !!(histb_hdmi_read_tx0(hdmi, HISTB_HDMI_TX_STAT_ADDR) &
		  HISTB_HDMI_BIT_HPD_PIN);
}

static int histb_hdmi_configure_crg(struct histb_hdmi *hdmi)
{
	return regmap_update_bits(hdmi->crg, HISTB_HDMI_CRG67,
				  HISTB_HDMI_CRG67_CEC_CLK_SEL |
				  HISTB_HDMI_CRG67_AS_CLK_SEL,
				  0);
}

static const struct drm_display_mode *histb_hdmi_fixed_mode_from_param(void)
{
	if (histb_fixed_mode == 1080 || histb_fixed_mode == 1)
		return &histb_hdmi_1080p60_mode;

	return &histb_hdmi_720p60_mode;
}

static int histb_hdmi_bridge_attach(struct drm_bridge *bridge,
				    struct drm_encoder *encoder,
				    enum drm_bridge_attach_flags flags)
{
	(void)bridge;
	(void)encoder;

	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR))
		return -EINVAL;

	return 0;
}

static enum drm_connector_status
histb_hdmi_bridge_detect(struct drm_bridge *bridge,
			 struct drm_connector *connector)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	int ret;
	bool hpd;

	ret = histb_hdmi_runtime_get(hdmi);
	if (ret)
		return connector_status_disconnected;

	mutex_lock(&hdmi->lock);
	hdmi->connector = connector;
	hpd = histb_hdmi_hpd_status(hdmi);
	mutex_unlock(&hdmi->lock);
	histb_hdmi_runtime_put(hdmi);

	if (hpd)
		return connector_status_connected;

	return connector_status_disconnected;
}

static int histb_hdmi_bridge_edid_read_block(void *context, u8 *buf,
					     unsigned int block, size_t len)
{
	struct histb_hdmi *hdmi = context;

	if (len != HISTB_EDID_BLOCK_SIZE || block > 0xff)
		return -EINVAL;

	return histb_hdmi_read_edid_block(hdmi, block, buf);
}

static const struct drm_edid *
histb_hdmi_bridge_edid_read(struct drm_bridge *bridge,
			    struct drm_connector *connector)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	const struct drm_edid *drm_edid = NULL;
	int ret;

	ret = histb_hdmi_runtime_get(hdmi);
	if (ret)
		return NULL;

	mutex_lock(&hdmi->lock);
	hdmi->connector = connector;

	if (histb_hdmi_hpd_status(hdmi))
		drm_edid = drm_edid_read_custom(connector,
						histb_hdmi_bridge_edid_read_block,
						hdmi);

	mutex_unlock(&hdmi->lock);
	histb_hdmi_runtime_put(hdmi);

	return drm_edid;
}

static bool histb_hdmi_mode_clock_supported(unsigned int clock)
{
	return clock == 74250 || clock == 148500;
}

static int histb_hdmi_mode_to_hw(const struct drm_display_mode *mode,
				 struct histb_hdmi_mode *hw_mode)
{
	u32 hfront;
	u32 hsync;
	u32 hback;
	u32 vfront;
	u32 vsync;
	u32 vback;

	if (!mode || !hw_mode)
		return -EINVAL;

	if (histb_vdp_pipeline_mode_valid(mode))
		return -EINVAL;

	if (!histb_hdmi_mode_clock_supported(mode->clock))
		return -EINVAL;

	hfront = mode->hsync_start - mode->hdisplay;
	hsync = mode->hsync_end - mode->hsync_start;
	hback = mode->htotal - mode->hsync_end;
	vfront = mode->vsync_start - mode->vdisplay;
	vsync = mode->vsync_end - mode->vsync_start;
	vback = mode->vtotal - mode->vsync_end;

	if (!hfront || !hsync || !hback || !vfront || !vsync || !vback)
		return -EINVAL;

	hw_mode->hstart = hsync + hback;
	hw_mode->vstart = vsync + vback;
	hw_mode->hres = mode->hdisplay;
	hw_mode->vres = mode->vdisplay;
	hw_mode->htotal = mode->htotal;
	hw_mode->vtotal = mode->vtotal;
	hw_mode->hbit_to_hsync = hfront;
	hw_mode->field2_hsync_offset = 0;
	hw_mode->hlength = hsync;
	hw_mode->vbit_to_vsync = vfront;
	hw_mode->vlength = vsync;
	hw_mode->int_adj_mode = 0;
	hw_mode->de_polarity = 0;
	hw_mode->phy_pll1_swing = 1;

	if (hw_mode->hstart > 0x3ff || hw_mode->vstart > 0xff)
		return -EINVAL;

	return 0;
}

static bool histb_hdmi_mode_is_supported(const struct drm_display_mode *mode)
{
	struct histb_hdmi_mode hw_mode;

	return !histb_hdmi_mode_to_hw(mode, &hw_mode);
}

static enum drm_mode_status
histb_hdmi_bridge_mode_valid(struct drm_bridge *bridge,
			     const struct drm_display_info *info,
			     const struct drm_display_mode *mode)
{
	(void)bridge;
	(void)info;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		return MODE_NO_DBLESCAN;

	if (mode->hdisplay > 1920)
		return MODE_BAD_HVALUE;

	if (mode->vdisplay > 1080)
		return MODE_BAD_VVALUE;

	if (!histb_hdmi_mode_clock_supported(mode->clock))
		return MODE_CLOCK_RANGE;

	if (!histb_hdmi_mode_is_supported(mode))
		return MODE_BAD;

	return MODE_OK;
}

static enum drm_mode_status
histb_hdmi_bridge_tmds_char_rate_valid(const struct drm_bridge *bridge,
				       const struct drm_display_mode *mode,
				       unsigned long long tmds_rate)
{
	(void)bridge;

	if (!histb_hdmi_mode_clock_supported(mode->clock))
		return MODE_CLOCK_RANGE;

	/*
	 * Only the 24-bit TMDS datapath is programmed, so do not claim
	 * deep-color rates.
	 */
	if (tmds_rate != mode->clock * 1000ULL)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static int histb_hdmi_bridge_get_modes(struct drm_bridge *bridge,
				       struct drm_connector *connector)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	struct drm_display_mode *mode;

	mutex_lock(&hdmi->lock);
	hdmi->connector = connector;
	mode = drm_mode_duplicate(connector->dev, &hdmi->mode);
	mutex_unlock(&hdmi->lock);

	if (!mode)
		return 0;

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static void histb_hdmi_bridge_mode_set(struct drm_bridge *bridge,
				       const struct drm_display_mode *mode,
				       const struct drm_display_mode *adjusted_mode)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	const struct drm_display_mode *target = adjusted_mode ?: mode;
	const struct drm_display_mode *fixed_mode = histb_hdmi_fixed_mode_from_param();
	struct drm_display_mode prev_mode;
	bool prev_mode_from_bridge;
	struct histb_hdmi_mode hw_mode;
	int ret;

	mutex_lock(&hdmi->lock);
	drm_mode_copy(&prev_mode, &hdmi->mode);
	prev_mode_from_bridge = hdmi->mode_from_bridge;

	ret = histb_hdmi_mode_to_hw(target, &hw_mode);
	if (ret) {
		dev_warn(hdmi->dev,
			 "bridge_mode_set rejected unsupported mode %ux%u@%d (clock=%d)\n",
			 target->hdisplay, target->vdisplay,
			 drm_mode_vrefresh(target), target->clock);
		mutex_unlock(&hdmi->lock);
		return;
	}

	drm_mode_copy(&hdmi->mode, target);
	drm_mode_set_name(&hdmi->mode);
	hdmi->mode_from_bridge = true;

	if (hdmi->hpd_active) {
		ret = histb_hdmi_apply_mode(hdmi, &hdmi->mode);
		if (ret) {
			dev_warn(hdmi->dev,
				 "bridge_mode_set apply failed: %d, trying recovery\n",
				 ret);
			ret = histb_hdmi_link_recover_locked(hdmi, &prev_mode,
							     "bridge-mode");
			if (!ret) {
				drm_mode_copy(&hdmi->mode, &prev_mode);
				drm_mode_set_name(&hdmi->mode);
				hdmi->mode_from_bridge = prev_mode_from_bridge;
			} else {
				ret = histb_hdmi_link_recover_locked(hdmi, fixed_mode,
							     "bridge-fallback");
				if (!ret) {
					drm_mode_copy(&hdmi->mode, fixed_mode);
					drm_mode_set_name(&hdmi->mode);
					hdmi->mode_from_bridge = false;
				} else {
					dev_warn(hdmi->dev,
						 "bridge_mode_set recovery failed: %d\n",
						 ret);
				}
			}
		}
	}

	mutex_unlock(&hdmi->lock);
}

static bool histb_hdmi_colorspace_supported(enum drm_colorspace colorspace)
{
	return colorspace == DRM_MODE_COLORIMETRY_DEFAULT;
}

static void histb_hdmi_hdcp_disable_locked(struct histb_hdmi *hdmi)
{
	u32 val;

	val = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_HDCP_CTRL_ADDR);
	val &= ~HISTB_HDMI_HDCP_BIT_ENC_EN;
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_HDCP_CTRL_ADDR, val);
	hdmi->hdcp_enabled = false;
}

static int histb_hdmi_hdcp_enable_locked(struct histb_hdmi *hdmi)
{
	u32 val;
	int i;

	val = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_HDCP_CTRL_ADDR);
	if ((val & HISTB_HDMI_HDCP_BIT_ENC_EN) && (val & HISTB_HDMI_HDCP_BIT_ENC_ON)) {
		hdmi->hdcp_enabled = true;
		return 0;
	}

	val |= HISTB_HDMI_HDCP_BIT_CP_RESET_N;
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_HDCP_CTRL_ADDR, val);

	val &= ~HISTB_HDMI_HDCP_BIT_AN_STOP;
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_HDCP_CTRL_ADDR, val);
	usleep_range(HISTB_HDMI_HDCP_RETRY_US_MIN, HISTB_HDMI_HDCP_RETRY_US_MAX);
	val |= HISTB_HDMI_HDCP_BIT_AN_STOP;
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_HDCP_CTRL_ADDR, val);

	val |= HISTB_HDMI_HDCP_BIT_ENC_EN;
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_HDCP_CTRL_ADDR, val);

	for (i = 0; i < HISTB_HDMI_HDCP_MAX_RETRIES; i++) {
		val = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_HDCP_CTRL_ADDR);
		if (val & HISTB_HDMI_HDCP_BIT_BKSV_ERROR)
			break;
		if (val & HISTB_HDMI_HDCP_BIT_ENC_ON) {
			hdmi->hdcp_enabled = true;
			return 0;
		}

		usleep_range(HISTB_HDMI_HDCP_RETRY_US_MIN,
			     HISTB_HDMI_HDCP_RETRY_US_MAX);
	}

	histb_hdmi_hdcp_disable_locked(hdmi);
	return -ETIMEDOUT;
}

static void histb_hdmi_bridge_atomic_pre_enable(struct drm_bridge *bridge,
						 struct drm_atomic_state *state)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	int ret;

	(void)state;

	ret = histb_hdmi_runtime_get(hdmi);
	if (ret) {
		dev_warn(hdmi->dev, "runtime PM resume failed: %d\n", ret);
		return;
	}

	mutex_lock(&hdmi->lock);
	if (hdmi->pm_ref_active) {
		mutex_unlock(&hdmi->lock);
		histb_hdmi_runtime_put(hdmi);
		return;
	}
	hdmi->pm_ref_active = true;
	ret = histb_hdmi_handle_hpd_event(hdmi);
	if (ret)
		dev_warn(hdmi->dev, "runtime re-init HPD handling failed: %d\n",
			 ret);
	mutex_unlock(&hdmi->lock);
}

static int histb_hdmi_bridge_atomic_check(struct drm_bridge *bridge,
					  struct drm_bridge_state *bridge_state,
					  struct drm_crtc_state *crtc_state,
					  struct drm_connector_state *conn_state)
{
	struct drm_connector_state *old_conn_state;
	u64 old_cp;
	u64 new_cp;

	(void)bridge;

	if (!bridge_state || !conn_state)
		return 0;

	if (!histb_hdmi_colorspace_supported(conn_state->colorspace))
		return -EINVAL;

	old_conn_state = drm_atomic_get_old_connector_state(bridge_state->base.state,
							     conn_state->connector);
	if (!old_conn_state)
		return 0;

	old_cp = old_conn_state->content_protection;
	new_cp = conn_state->content_protection;

	if (conn_state->crtc && old_conn_state->colorspace != conn_state->colorspace)
		crtc_state->mode_changed = true;

	if (conn_state->crtc &&
	    old_conn_state->hdmi.broadcast_rgb != conn_state->hdmi.broadcast_rgb)
		crtc_state->mode_changed = true;

	/* HDR static metadata is neither advertised nor supported. */
	if (conn_state->hdr_output_metadata)
		return -EINVAL;

	if (new_cp != DRM_MODE_CONTENT_PROTECTION_UNDESIRED &&
	    conn_state->hdcp_content_type != DRM_MODE_HDCP_CONTENT_TYPE0)
		return -EINVAL;

	if (old_conn_state->hdcp_content_type != conn_state->hdcp_content_type &&
	    new_cp != DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
		conn_state->content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
		crtc_state->mode_changed = true;
		return 0;
	}

	if (!conn_state->crtc) {
		if (old_cp == DRM_MODE_CONTENT_PROTECTION_ENABLED)
			conn_state->content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
		return 0;
	}

	if (old_cp == new_cp ||
	    (old_cp == DRM_MODE_CONTENT_PROTECTION_DESIRED &&
	     new_cp == DRM_MODE_CONTENT_PROTECTION_ENABLED))
		return 0;

	crtc_state->mode_changed = true;
	return 0;
}

static void histb_hdmi_bridge_atomic_enable(struct drm_bridge *bridge,
					    struct drm_atomic_state *state)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	struct drm_connector *connector;
	struct drm_connector_state *conn_state;
	int ret = -EIO;

	connector = drm_atomic_get_new_connector_for_encoder(state, bridge->encoder);
	if (!connector)
		return;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (!conn_state)
		return;

	if (IS_ENABLED(CONFIG_DRM_DISPLAY_HELPER)) {
		int if_ret;

		if_ret = drm_atomic_helper_connector_hdmi_update_infoframes(connector, state);
		if (if_ret)
			dev_warn_ratelimited(hdmi->dev,
					     "HDMI infoframe update failed: %d\n",
					     if_ret);
	}

	mutex_lock(&hdmi->lock);
	hdmi->connector = connector;
	hdmi->hdcp_content_type = conn_state->hdcp_content_type;

	if (conn_state->content_protection == DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
		histb_hdmi_hdcp_disable_locked(hdmi);
		goto out_unlock;
	}

	if (conn_state->hdcp_content_type != DRM_MODE_HDCP_CONTENT_TYPE0)
		goto report_desired;

	if (!hdmi->hpd_active || !histb_hdmi_hpd_status(hdmi))
		goto report_desired;

	ret = histb_hdmi_hdcp_enable_locked(hdmi);
	if (!ret) {
#if IS_ENABLED(CONFIG_DRM_DISPLAY_HDCP_HELPER)
		drm_hdcp_update_content_protection(connector,
						   DRM_MODE_CONTENT_PROTECTION_ENABLED);
#endif
	} else {
		goto report_desired;
	}

	goto out_unlock;

report_desired:
#if IS_ENABLED(CONFIG_DRM_DISPLAY_HDCP_HELPER)
	drm_hdcp_update_content_protection(connector,
					   DRM_MODE_CONTENT_PROTECTION_DESIRED);
#endif
out_unlock:
	mutex_unlock(&hdmi->lock);
}

static void histb_hdmi_bridge_atomic_disable(struct drm_bridge *bridge,
					     struct drm_atomic_state *state)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	struct drm_connector *connector;
	struct drm_connector_state *conn_state;
	bool was_enabled;

	connector = drm_atomic_get_new_connector_for_encoder(state, bridge->encoder);
	if (!connector)
		return;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (!conn_state)
		return;

	mutex_lock(&hdmi->lock);
	was_enabled = hdmi->hdcp_enabled;
	histb_hdmi_hdcp_disable_locked(hdmi);

#if IS_ENABLED(CONFIG_DRM_DISPLAY_HDCP_HELPER)
	if (was_enabled &&
	    conn_state->content_protection != DRM_MODE_CONTENT_PROTECTION_UNDESIRED)
		drm_hdcp_update_content_protection(connector,
						   DRM_MODE_CONTENT_PROTECTION_DESIRED);
#else
	(void)was_enabled;
#endif

	mutex_unlock(&hdmi->lock);
}

static void histb_hdmi_bridge_atomic_post_disable(struct drm_bridge *bridge,
						  struct drm_atomic_state *state)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	bool release;

	(void)state;

	mutex_lock(&hdmi->lock);
	release = hdmi->pm_ref_active;
	hdmi->pm_ref_active = false;
	mutex_unlock(&hdmi->lock);

	if (release)
		histb_hdmi_runtime_put(hdmi);
}

#if IS_ENABLED(CONFIG_DRM_DISPLAY_HDMI_CEC_HELPER)
static void histb_hdmi_cec_clear_rx_fifo_locked(struct histb_hdmi *hdmi, bool all)
{
	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_RX_CONTROL_ADDR,
			     all ? HISTB_HDMI_CEC_BIT_CLR_RX_FIFO_ALL :
				   HISTB_HDMI_CEC_BIT_CLR_RX_FIFO_CUR);
}

static void histb_hdmi_cec_clear_tx_fifo_locked(struct histb_hdmi *hdmi)
{
	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_DEBUG_3_ADDR,
			     HISTB_HDMI_CEC_BIT_FLUSH_TX_FIFO);
}

static void histb_hdmi_cec_clear_int_status_locked(struct histb_hdmi *hdmi)
{
	u8 int0 = histb_hdmi_read_cec(hdmi, HISTB_HDMI_CEC_INT_STATUS_0_ADDR);
	u8 int1 = histb_hdmi_read_cec(hdmi, HISTB_HDMI_CEC_INT_STATUS_1_ADDR);

	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_INT_STATUS_0_ADDR, int0);
	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_INT_STATUS_1_ADDR, int1);
}

static int histb_hdmi_cec_set_log_addr_locked(struct histb_hdmi *hdmi, u8 logical_addr)
{
	u8 cap0 = 0;
	u8 cap1 = 0;

	hdmi->cec_logical_addr = logical_addr;
	if (logical_addr == CEC_LOG_ADDR_INVALID) {
		histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_CAPTURE_ID0_ADDR, 0);
		histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_CAPTURE_ID0_ADDR + 1, 0);
		return 0;
	}

	if (logical_addr > 0xf)
		return -EINVAL;

	if (logical_addr < 8)
		cap0 = BIT(logical_addr);
	else
		cap1 = BIT(logical_addr - 8);

	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_CAPTURE_ID0_ADDR, cap0);
	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_CAPTURE_ID0_ADDR + 1, cap1);
	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_TX_INIT_ADDR, logical_addr);

	return 0;
}

static int histb_hdmi_cec_enable_locked(struct histb_hdmi *hdmi, bool enable)
{
	if (enable) {
		histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_CONFIG_CPI_ADDR,
				     HISTB_HDMI_CEC_CONFIG_CPI_DEFAULT);
		histb_hdmi_cec_clear_tx_fifo_locked(hdmi);
		histb_hdmi_cec_clear_rx_fifo_locked(hdmi, true);
		histb_hdmi_cec_clear_int_status_locked(hdmi);
		histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_INT_ENABLE_0_ADDR, 0xff);
		histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_INT_ENABLE_1_ADDR, 0xff);
		hdmi->cec_enabled = true;
		hdmi->cec_tx_in_progress = false;
		hdmi->cec_attempts = 1;

		if (hdmi->cec_logical_addr != CEC_LOG_ADDR_INVALID)
			return histb_hdmi_cec_set_log_addr_locked(hdmi,
								   hdmi->cec_logical_addr);

		return 0;
	}

	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_INT_ENABLE_0_ADDR, 0x00);
	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_INT_ENABLE_1_ADDR, 0x00);
	histb_hdmi_cec_clear_int_status_locked(hdmi);
	histb_hdmi_cec_clear_tx_fifo_locked(hdmi);
	histb_hdmi_cec_clear_rx_fifo_locked(hdmi, true);
	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_CAPTURE_ID0_ADDR, 0);
	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_CAPTURE_ID0_ADDR + 1, 0);
	hdmi->cec_enabled = false;
	hdmi->cec_tx_in_progress = false;

	return 0;
}

static int histb_hdmi_cec_read_msg_locked(struct histb_hdmi *hdmi, struct cec_msg *msg)
{
	u8 rx_count;
	u8 operand_count;
	int i;

	rx_count = histb_hdmi_read_cec(hdmi, HISTB_HDMI_CEC_RX_COUNT_ADDR);
	if (!(rx_count & HISTB_HDMI_CEC_RX_FRAME_CNT_MASK))
		return -ENODATA;

	if (rx_count & HISTB_HDMI_CEC_BIT_MSG_ERROR) {
		histb_hdmi_cec_clear_rx_fifo_locked(hdmi, false);
		return -EIO;
	}

	operand_count = FIELD_GET(HISTB_HDMI_CEC_RX_OPERAND_CNT_MASK, rx_count);
	msg->msg[0] = histb_hdmi_read_cec(hdmi, HISTB_HDMI_CEC_RX_CMD_HEADER_ADDR);
	msg->len = 1;

	if (!operand_count)
		goto pop_rx;

	msg->msg[1] = histb_hdmi_read_cec(hdmi, HISTB_HDMI_CEC_RX_COMMAND_ADDR);
	msg->len = 2;

	for (i = 0; i < operand_count && msg->len < CEC_MAX_MSG_SIZE; i++)
		msg->msg[msg->len++] = histb_hdmi_read_cec(
			hdmi, HISTB_HDMI_CEC_RX_OPERAND_0_ADDR + i);

pop_rx:
	histb_hdmi_cec_clear_rx_fifo_locked(hdmi, false);
	return 0;
}

static bool histb_hdmi_cec_irq_pending(struct histb_hdmi *hdmi)
{
	u8 int0;
	u8 int1;
	u8 rx_count;

	if (!hdmi->cec_enabled)
		return false;

	int0 = histb_hdmi_read_cec(hdmi, HISTB_HDMI_CEC_INT_STATUS_0_ADDR);
	int1 = histb_hdmi_read_cec(hdmi, HISTB_HDMI_CEC_INT_STATUS_1_ADDR);
	rx_count = histb_hdmi_read_cec(hdmi, HISTB_HDMI_CEC_RX_COUNT_ADDR);

	return (int0 & HISTB_HDMI_CEC_INT_STATUS_0_MASK) || int1 ||
	       (rx_count & HISTB_HDMI_CEC_RX_FRAME_CNT_MASK);
}

static bool histb_hdmi_cec_irq_process_locked(struct histb_hdmi *hdmi)
{
	struct drm_connector *connector = hdmi->connector;
	u8 int0;
	u8 int1;
	u8 rx_count;
	bool handled = false;

	if (!hdmi->cec_enabled)
		return false;

	int0 = histb_hdmi_read_cec(hdmi, HISTB_HDMI_CEC_INT_STATUS_0_ADDR);
	int1 = histb_hdmi_read_cec(hdmi, HISTB_HDMI_CEC_INT_STATUS_1_ADDR);
	rx_count = histb_hdmi_read_cec(hdmi, HISTB_HDMI_CEC_RX_COUNT_ADDR);

	if (!(int0 & HISTB_HDMI_CEC_INT_STATUS_0_MASK) && !int1 &&
	    !(rx_count & HISTB_HDMI_CEC_RX_FRAME_CNT_MASK))
		return false;

	handled = true;
	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_INT_STATUS_0_ADDR, int0);
	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_INT_STATUS_1_ADDR, int1);

	if (int1 & HISTB_HDMI_CEC_BIT_RX_FIFO_OVERRUN)
		histb_hdmi_cec_clear_rx_fifo_locked(hdmi, true);

	if (hdmi->cec_tx_in_progress) {
		if (int0 & HISTB_HDMI_CEC_BIT_TX_MESSAGE_SENT) {
			hdmi->cec_tx_in_progress = false;
			if (connector)
				drm_connector_hdmi_cec_transmit_done(
					connector, CEC_TX_STATUS_OK, 0, 0, 0, 0);
		} else if (int1 & HISTB_HDMI_CEC_BIT_FRAME_RETRANSM_OV) {
			u8 nack_cnt = hdmi->cec_attempts ? hdmi->cec_attempts : 1;

			hdmi->cec_tx_in_progress = false;
			if (connector)
				drm_connector_hdmi_cec_transmit_done(
					connector,
					CEC_TX_STATUS_MAX_RETRIES |
					CEC_TX_STATUS_NACK,
					0, nack_cnt, 0, 0);
		}
	}

	if (!connector) {
		if (rx_count & HISTB_HDMI_CEC_RX_FRAME_CNT_MASK)
			histb_hdmi_cec_clear_rx_fifo_locked(hdmi, true);
		return handled;
	}

	while (rx_count & HISTB_HDMI_CEC_RX_FRAME_CNT_MASK) {
		struct cec_msg msg = {};

		if (!histb_hdmi_cec_read_msg_locked(hdmi, &msg)) {
			u8 initiator = msg.msg[0] >> 4;

			if (hdmi->cec_logical_addr != CEC_LOG_ADDR_INVALID &&
			    initiator == hdmi->cec_logical_addr)
				goto next_msg;

			drm_connector_hdmi_cec_received_msg(connector, &msg);
		}

next_msg:
		rx_count = histb_hdmi_read_cec(hdmi, HISTB_HDMI_CEC_RX_COUNT_ADDR);
	}

	return handled;
}

static int histb_hdmi_bridge_cec_init(struct drm_bridge *bridge,
				      struct drm_connector *connector)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);

	mutex_lock(&hdmi->lock);
	hdmi->connector = connector;
	hdmi->cec_tx_in_progress = false;
	mutex_unlock(&hdmi->lock);

	return 0;
}

static int histb_hdmi_bridge_cec_enable(struct drm_bridge *bridge, bool enable)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	int ret;

	mutex_lock(&hdmi->lock);
	ret = histb_hdmi_cec_enable_locked(hdmi, enable);
	mutex_unlock(&hdmi->lock);

	return ret;
}

static int histb_hdmi_bridge_cec_log_addr(struct drm_bridge *bridge, u8 logical_addr)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	int ret = 0;

	mutex_lock(&hdmi->lock);

	hdmi->cec_logical_addr = logical_addr;
	if (!hdmi->cec_enabled && logical_addr != CEC_LOG_ADDR_INVALID) {
		ret = -EIO;
		goto out_unlock;
	}

	ret = histb_hdmi_cec_set_log_addr_locked(hdmi, logical_addr);

out_unlock:
	mutex_unlock(&hdmi->lock);
	return ret;
}

static int histb_hdmi_bridge_cec_transmit(struct drm_bridge *bridge, u8 attempts,
					  u32 signal_free_time, struct cec_msg *msg)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	u8 retries;
	u8 operand_count;
	u8 header;
	u8 init;
	u8 dest;
	int i;
	int ret = 0;

	(void)signal_free_time;

	if (!msg || !msg->len || msg->len > CEC_MAX_MSG_SIZE)
		return -EINVAL;

	header = msg->msg[0];
	init = (header >> 4) & 0xf;
	dest = header & 0xf;
	retries = attempts ? attempts - 1 : 0;
	operand_count = msg->len > 2 ? msg->len - 2 : 0;

	mutex_lock(&hdmi->lock);

	if (!hdmi->cec_enabled) {
		ret = -EIO;
		goto out_unlock;
	}

	if (hdmi->cec_tx_in_progress) {
		ret = -EBUSY;
		goto out_unlock;
	}

	histb_hdmi_cec_clear_int_status_locked(hdmi);
	histb_hdmi_cec_clear_tx_fifo_locked(hdmi);
	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_RETRY_LIMIT_ADDR, retries & 0x0f);
	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_TX_INIT_ADDR, init);
	hdmi->cec_attempts = attempts ? attempts : 1;

	if (msg->len == 1) {
		histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_TX_DEST_ADDR,
				     HISTB_HDMI_CEC_BIT_SEND_POLL | dest);
		hdmi->cec_tx_in_progress = true;
		goto out_unlock;
	}

	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_TX_DEST_ADDR, dest);
	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_TX_COMMAND_ADDR, msg->msg[1]);

	for (i = 0; i < operand_count; i++)
		histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_TX_OPERANDS_0_ADDR + i,
				     msg->msg[2 + i]);

	histb_hdmi_write_cec(hdmi, HISTB_HDMI_CEC_TRANSMIT_DATA_ADDR,
			     HISTB_HDMI_CEC_BIT_TRANSMIT_CMD | operand_count);
	hdmi->cec_tx_in_progress = true;

out_unlock:
	mutex_unlock(&hdmi->lock);
	return ret;
}
#endif

static bool histb_hdmi_bridge_infoframe_layout(enum hdmi_infoframe_type type,
					       u32 *if_addr, u32 *repeat_bit,
					       u32 *enable_bit)
{
	switch (type) {
	case HDMI_INFOFRAME_TYPE_AVI:
		*if_addr = HISTB_HDMI_AVI_IF_TX1_ADDR;
		*repeat_bit = HISTB_HDMI_INF_CTRL1_AVI_REPEAT;
		*enable_bit = HISTB_HDMI_INF_CTRL1_AVI_ENABLE;
		return true;
	case HDMI_INFOFRAME_TYPE_AUDIO:
		*if_addr = HISTB_HDMI_AUD_IF_TX1_ADDR;
		*repeat_bit = HISTB_HDMI_INF_CTRL1_AUD_REPEAT;
		*enable_bit = HISTB_HDMI_INF_CTRL1_AUD_ENABLE;
		return true;
	case HDMI_INFOFRAME_TYPE_VENDOR:
	case HDMI_INFOFRAME_TYPE_SPD:
		*if_addr = HISTB_HDMI_MPEG_IF_TX1_ADDR;
		*repeat_bit = HISTB_HDMI_INF_CTRL1_MPEG_REPEAT;
		*enable_bit = HISTB_HDMI_INF_CTRL1_MPEG_ENABLE;
		return true;
	default:
		return false;
	}
}

static int histb_hdmi_bridge_hdmi_clear_infoframe(struct drm_bridge *bridge,
						   enum hdmi_infoframe_type type)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	u32 if_addr;
	u32 repeat_bit;
	u32 enable_bit;
	u32 val;

	if (type == HDMI_INFOFRAME_TYPE_DRM)
		return 0;

	if (!histb_hdmi_bridge_infoframe_layout(type, &if_addr, &repeat_bit, &enable_bit))
		return 0;

	(void)if_addr;

	mutex_lock(&hdmi->lock);
	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_INF_CTRL1_TX1_ADDR);
	val &= ~(repeat_bit | enable_bit);
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_INF_CTRL1_TX1_ADDR, val);
	mutex_unlock(&hdmi->lock);

	return 0;
}

static int histb_hdmi_bridge_hdmi_write_infoframe(struct drm_bridge *bridge,
						   enum hdmi_infoframe_type type,
						   const u8 *buffer, size_t len)
{
	struct histb_hdmi *hdmi = bridge_to_histb_hdmi(bridge);
	u32 if_addr;
	u32 repeat_bit;
	u32 enable_bit;
	u32 val;
	size_t i;

	if (type == HDMI_INFOFRAME_TYPE_DRM)
		return 0;

	if (!buffer || !len)
		return -EINVAL;

	if (!histb_hdmi_bridge_infoframe_layout(type, &if_addr, &repeat_bit, &enable_bit))
		return 0;

	mutex_lock(&hdmi->lock);

	for (i = 0; i < len; i++)
		histb_hdmi_write_tx1(hdmi, if_addr + i, buffer[i]);

	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_INF_CTRL1_TX1_ADDR);
	val &= ~(repeat_bit | enable_bit);
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_INF_CTRL1_TX1_ADDR, val);
	udelay(1);
	val |= repeat_bit | enable_bit;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_INF_CTRL1_TX1_ADDR, val);

	mutex_unlock(&hdmi->lock);

	return 0;
}

static const struct drm_bridge_funcs histb_hdmi_bridge_funcs = {
	.attach = histb_hdmi_bridge_attach,
	.detect = histb_hdmi_bridge_detect,
	.edid_read = histb_hdmi_bridge_edid_read,
	.mode_valid = histb_hdmi_bridge_mode_valid,
	.hdmi_tmds_char_rate_valid = histb_hdmi_bridge_tmds_char_rate_valid,
	.hdmi_clear_infoframe = histb_hdmi_bridge_hdmi_clear_infoframe,
	.hdmi_write_infoframe = histb_hdmi_bridge_hdmi_write_infoframe,
	.atomic_check = histb_hdmi_bridge_atomic_check,
	.atomic_pre_enable = histb_hdmi_bridge_atomic_pre_enable,
	.atomic_enable = histb_hdmi_bridge_atomic_enable,
	.atomic_disable = histb_hdmi_bridge_atomic_disable,
	.atomic_post_disable = histb_hdmi_bridge_atomic_post_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.get_modes = histb_hdmi_bridge_get_modes,
	.mode_set = histb_hdmi_bridge_mode_set,
#if IS_ENABLED(CONFIG_DRM_DISPLAY_HDMI_CEC_HELPER)
	.hdmi_cec_init = histb_hdmi_bridge_cec_init,
	.hdmi_cec_enable = histb_hdmi_bridge_cec_enable,
	.hdmi_cec_log_addr = histb_hdmi_bridge_cec_log_addr,
	.hdmi_cec_transmit = histb_hdmi_bridge_cec_transmit,
#endif
};

static int histb_hdmi_perictrl_mmio_update_bits_at(struct histb_hdmi *hdmi, u32 reg,
						    u32 mask, u32 set,
						    u32 *before, u32 *after);
static int histb_hdmi_perictrl_mmio_update_bits(struct histb_hdmi *hdmi, u32 mask,
						 u32 set, u32 *before, u32 *after);
static u32 histb_hdmi_perictrl_mmio_read(struct histb_hdmi *hdmi, u32 reg);

static inline bool histb_hdmi_audio_source_is_i2s(u32 val)
{
	return !!(val & (HISTB_HDMI_PERICTRL_AUD_SRC_I2S_STB |
			 HISTB_HDMI_PERICTRL_AUD_SRC_I2S_BVT));
}

static void histb_hdmi_audio_select_i2s_source_locked(struct histb_hdmi *hdmi)
{
	int ret;
	u32 before;
	u32 after;
	u32 alt_before;
	u32 mmio_before = 0;
	u32 mmio_after = 0;
	u32 mmio_alt_before = 0;
	u32 mmio_alt_after = 0;
	bool mmio_tried = false;
	bool mmio_alt_tried = false;

	if (!hdmi->perictrl)
		return;

	ret = regmap_read(hdmi->perictrl, HISTB_HDMI_PERICTRL_HDMITX_CTRL, &before);
	if (ret) {
		dev_dbg(hdmi->dev, "failed to read HDMI source status: %d\n", ret);
		return;
	}

	if (histb_hdmi_audio_source_is_i2s(before))
		return;

	alt_before = histb_hdmi_perictrl_mmio_read(hdmi,
						    HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT);
	if (histb_hdmi_audio_source_is_i2s(alt_before))
		return;

	/* First try STB layout (bits [4:0]). */
	ret = regmap_update_bits(hdmi->perictrl, HISTB_HDMI_PERICTRL_HDMITX_CTRL,
				 HISTB_HDMI_PERICTRL_AUD_SRC_MASK_STB,
				 HISTB_HDMI_PERICTRL_AUD_SRC_I2S_STB);
	if (ret)
		dev_dbg(hdmi->dev, "PERI_HDMITX_CTRL STB select update failed: %d\n", ret);
	if (!ret &&
	    !regmap_read(hdmi->perictrl, HISTB_HDMI_PERICTRL_HDMITX_CTRL, &after) &&
	    (after & HISTB_HDMI_PERICTRL_AUD_SRC_I2S_STB))
		return;

	/* Otherwise the BVT layout, bits [10:6]. */
	ret = regmap_update_bits(hdmi->perictrl, HISTB_HDMI_PERICTRL_HDMITX_CTRL,
				 HISTB_HDMI_PERICTRL_AUD_SRC_MASK_BVT,
				 HISTB_HDMI_PERICTRL_AUD_SRC_I2S_BVT);
	if (ret)
		dev_dbg(hdmi->dev, "PERI_HDMITX_CTRL BVT select update failed: %d\n", ret);
	if (!ret &&
	    !regmap_read(hdmi->perictrl, HISTB_HDMI_PERICTRL_HDMITX_CTRL, &after) &&
	    (after & HISTB_HDMI_PERICTRL_AUD_SRC_I2S_BVT))
		return;

	/*
	 * Fallback: retry source-select via direct MMIO like vendor SDK.
	 */
	ret = histb_hdmi_perictrl_mmio_update_bits(hdmi,
						   HISTB_HDMI_PERICTRL_AUD_SRC_MASK_STB,
						   HISTB_HDMI_PERICTRL_AUD_SRC_I2S_STB,
						   &mmio_before, &mmio_after);
	if (ret > 0) {
		dev_dbg(hdmi->dev,
			"PERI_HDMITX_CTRL selected I2S via direct MMIO STB fallback (before=0x%08x after=0x%08x)\n",
			mmio_before, mmio_after);
		return;
	}
	if (ret < 0)
		dev_dbg(hdmi->dev, "PERI_HDMITX_CTRL direct-MMIO STB fallback unavailable: %d\n",
			ret);
	mmio_tried = ret >= 0;

	ret = histb_hdmi_perictrl_mmio_update_bits(hdmi,
						   HISTB_HDMI_PERICTRL_AUD_SRC_MASK_BVT,
						   HISTB_HDMI_PERICTRL_AUD_SRC_I2S_BVT,
						   &mmio_before, &mmio_after);
	if (ret > 0) {
		dev_dbg(hdmi->dev,
			"PERI_HDMITX_CTRL selected I2S via direct MMIO BVT fallback (before=0x%08x after=0x%08x)\n",
			mmio_before, mmio_after);
		return;
	}
	if (ret < 0)
		dev_dbg(hdmi->dev, "PERI_HDMITX_CTRL direct-MMIO BVT fallback unavailable: %d\n",
			ret);
	mmio_tried = mmio_tried || ret >= 0;

	/*
	 * Some SDK branches use a low-offset PERI_HDMITX_CTRL layout
	 * (base + 0x150). Try it as a last-resort fallback.
	 */
	ret = histb_hdmi_perictrl_mmio_update_bits_at(
		hdmi,
		HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT,
		HISTB_HDMI_PERICTRL_AUD_SRC_MASK_STB |
			HISTB_HDMI_PERICTRL_AUD_SRC_MASK_BVT,
		HISTB_HDMI_PERICTRL_AUD_SRC_I2S_STB |
			HISTB_HDMI_PERICTRL_AUD_SRC_I2S_BVT,
		&mmio_alt_before, &mmio_alt_after);
	if (ret > 0) {
		dev_dbg(hdmi->dev,
			"PERI_HDMITX_CTRL selected I2S via alt-reg direct MMIO STB+BVT fallback (reg=0x%03x before=0x%08x after=0x%08x)\n",
			HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT,
			mmio_alt_before, mmio_alt_after);
		return;
	}
	if (ret < 0)
		dev_dbg(hdmi->dev,
			"PERI_HDMITX_CTRL alt-reg direct-MMIO STB+BVT fallback unavailable (reg=0x%03x): %d\n",
			HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT, ret);
	mmio_alt_tried = ret >= 0;

	ret = histb_hdmi_perictrl_mmio_update_bits_at(hdmi,
						      HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT,
						      HISTB_HDMI_PERICTRL_AUD_SRC_MASK_STB,
						      HISTB_HDMI_PERICTRL_AUD_SRC_I2S_STB,
						      &mmio_alt_before, &mmio_alt_after);
	if (ret > 0) {
		dev_dbg(hdmi->dev,
			"PERI_HDMITX_CTRL selected I2S via alt-reg direct MMIO STB fallback (reg=0x%03x before=0x%08x after=0x%08x)\n",
			HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT,
			mmio_alt_before, mmio_alt_after);
		return;
	}
	if (ret < 0)
		dev_dbg(hdmi->dev,
			"PERI_HDMITX_CTRL alt-reg direct-MMIO STB fallback unavailable (reg=0x%03x): %d\n",
			HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT, ret);
	mmio_alt_tried = mmio_alt_tried || ret >= 0;

	ret = histb_hdmi_perictrl_mmio_update_bits_at(hdmi,
						      HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT,
						      HISTB_HDMI_PERICTRL_AUD_SRC_MASK_BVT,
						      HISTB_HDMI_PERICTRL_AUD_SRC_I2S_BVT,
						      &mmio_alt_before, &mmio_alt_after);
	if (ret > 0) {
		dev_dbg(hdmi->dev,
			"PERI_HDMITX_CTRL selected I2S via alt-reg direct MMIO BVT fallback (reg=0x%03x before=0x%08x after=0x%08x)\n",
			HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT,
			mmio_alt_before, mmio_alt_after);
		return;
	}
	if (ret < 0)
		dev_dbg(hdmi->dev,
			"PERI_HDMITX_CTRL alt-reg direct-MMIO BVT fallback unavailable (reg=0x%03x): %d\n",
			HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT, ret);
	mmio_alt_tried = mmio_alt_tried || ret >= 0;

	if (!regmap_read(hdmi->perictrl, HISTB_HDMI_PERICTRL_HDMITX_CTRL, &after)) {
		if (mmio_tried || mmio_alt_tried) {
			dev_warn(hdmi->dev,
				 "PERI_HDMITX_CTRL remains non-I2S (regmap before=0x%08x after=0x%08x, mmio before=0x%08x after=0x%08x, mmio alt@0x%03x before=0x%08x after=0x%08x)\n",
				 before, after, mmio_before, mmio_after,
				 HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT,
				 mmio_alt_before, mmio_alt_after);
		} else {
			dev_warn(hdmi->dev,
				 "PERI_HDMITX_CTRL remains non-I2S (before=0x%08x after=0x%08x)\n",
				 before, after);
		}
	}
}

static u32 histb_hdmi_audio_source_status(struct histb_hdmi *hdmi)
{
	u32 val = 0;

	if (!hdmi->perictrl)
		return 0;

	if (regmap_read(hdmi->perictrl, HISTB_HDMI_PERICTRL_HDMITX_CTRL, &val))
		return 0;

	return val;
}

static u32 histb_hdmi_perictrl_mmio_read(struct histb_hdmi *hdmi, u32 reg)
{
	struct device_node *perictrl_np;
	void __iomem *perictrl_regs;
	u32 val;

	perictrl_np = of_parse_phandle(hdmi->dev->of_node,
				       "hisilicon,peripheral-syscon", 0);
	if (!perictrl_np)
		return 0;

	perictrl_regs = of_iomap(perictrl_np, 0);
	of_node_put(perictrl_np);
	if (!perictrl_regs)
		return 0;

	val = readl_relaxed(perictrl_regs + reg);
	iounmap(perictrl_regs);
	return val;
}

static u32 histb_hdmi_audio_source_status_alt(struct histb_hdmi *hdmi)
{
	return histb_hdmi_perictrl_mmio_read(hdmi,
					     HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT);
}

static int histb_hdmi_perictrl_mmio_update_bits_at(struct histb_hdmi *hdmi, u32 reg,
						    u32 mask, u32 set,
						    u32 *before, u32 *after)
{
	struct device_node *perictrl_np;
	void __iomem *perictrl_regs;
	u32 val;

	perictrl_np = of_parse_phandle(hdmi->dev->of_node,
				       "hisilicon,peripheral-syscon", 0);
	if (!perictrl_np)
		return -ENODEV;

	perictrl_regs = of_iomap(perictrl_np, 0);
	of_node_put(perictrl_np);
	if (!perictrl_regs)
		return -ENOMEM;

	val = readl_relaxed(perictrl_regs + reg);
	if (before)
		*before = val;

	val &= ~mask;
	val |= set;
	writel(val, perictrl_regs + reg);

	val = readl(perictrl_regs + reg);
	if (after)
		*after = val;

	iounmap(perictrl_regs);
	return (val & set) == set ? 1 : 0;
}

static int histb_hdmi_perictrl_mmio_update_bits(struct histb_hdmi *hdmi, u32 mask,
						 u32 set, u32 *before, u32 *after)
{
	return histb_hdmi_perictrl_mmio_update_bits_at(hdmi,
							HISTB_HDMI_PERICTRL_HDMITX_CTRL,
							mask, set, before, after);
}

static void histb_hdmi_audio_mute_locked(struct histb_hdmi *hdmi, bool mute)
{
	u32 tx_audio_ctrl;
	u32 audp_txctrl;

	tx_audio_ctrl = histb_hdmi_audio_read_prefer_linear(hdmi,
							     HISTB_HDMI_TX_AUDIO_CTRL_ADDR);
	if (mute)
		tx_audio_ctrl |= HISTB_HDMI_AUD_CTRL_MUTE_EN;
	else
		tx_audio_ctrl &= ~HISTB_HDMI_AUD_CTRL_MUTE_EN;
	histb_hdmi_audio_write_both(hdmi, HISTB_HDMI_TX_AUDIO_CTRL_ADDR,
				    tx_audio_ctrl);

	audp_txctrl = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUDP_TXCTRL_ADDR);
	if (mute)
		audp_txctrl |= HISTB_HDMI_BIT_AUDP_AUD_MUTE_EN;
	else
		audp_txctrl &= ~HISTB_HDMI_BIT_AUDP_AUD_MUTE_EN;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_AUDP_TXCTRL_ADDR, audp_txctrl);
}

static void histb_hdmi_audio_set_hdmi_output_locked(struct histb_hdmi *hdmi, bool enable)
{
	u32 data_ctrl;

	data_ctrl = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_DATA_CTRL_ADDR);
	if (enable) {
		/* Match SDK SI_SetHdmiAudio(TRUE): release FIFO reset before unmute. */
		histb_hdmi_write_tx0(hdmi, HISTB_HDMI_TX_SWRST_ADDR, 0x00);
		data_ctrl &= ~HISTB_HDMI_BIT_AUD_MUTE;
	} else {
		/* Match SDK SI_SetHdmiAudio(FALSE): mute output then flush FIFO. */
		data_ctrl |= HISTB_HDMI_BIT_AUD_MUTE;
		histb_hdmi_write_tx0(hdmi, HISTB_HDMI_DATA_CTRL_ADDR, data_ctrl);
		histb_hdmi_write_tx0(hdmi, HISTB_HDMI_TX_SWRST_ADDR,
				     HISTB_HDMI_BIT_TX_FIFO_RST);
		return;
	}

	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_DATA_CTRL_ADDR, data_ctrl);
}

static void histb_hdmi_tx1_disable_infoframe_locked(struct histb_hdmi *hdmi,
						     u32 repeat_bit, u32 enable_bit)
{
	u32 val;

	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_INF_CTRL1_TX1_ADDR);
	val &= ~(repeat_bit | enable_bit);
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_INF_CTRL1_TX1_ADDR, val);
}

static void histb_hdmi_tx1_rearm_infoframe_locked(struct histb_hdmi *hdmi,
						   u32 repeat_bit, u32 enable_bit)
{
	u32 val;

	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_INF_CTRL1_TX1_ADDR);
	val &= ~(repeat_bit | enable_bit);
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_INF_CTRL1_TX1_ADDR, val);
	udelay(1);
	val |= repeat_bit | enable_bit;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_INF_CTRL1_TX1_ADDR, val);
}

static void histb_hdmi_tx1_program_infoframe_locked(struct histb_hdmi *hdmi,
						     u32 if_addr,
						     const u8 *packet, ssize_t len,
						     u32 repeat_bit, u32 enable_bit)
{
	size_t i;

	if (!packet || len <= 0) {
		histb_hdmi_tx1_disable_infoframe_locked(hdmi, repeat_bit, enable_bit);
		return;
	}

	for (i = 0; i < (size_t)len; i++)
		histb_hdmi_write_tx1(hdmi, if_addr + i, packet[i]);

	histb_hdmi_tx1_rearm_infoframe_locked(hdmi, repeat_bit, enable_bit);
}

static void histb_hdmi_avi_infoframe_update_locked(struct histb_hdmi *hdmi,
						    const struct drm_display_mode *mode)
{
	struct hdmi_avi_infoframe frame;
	const struct drm_connector *connector = hdmi->connector;
	u8 packet[HDMI_INFOFRAME_SIZE(AVI)];
	ssize_t len;
	int ret;

	ret = drm_hdmi_avi_infoframe_from_display_mode(&frame, connector, mode);
	if (ret) {
		dev_warn_ratelimited(hdmi->dev,
				     "AVI infoframe compose failed: %d\n", ret);
		histb_hdmi_tx1_disable_infoframe_locked(
			hdmi,
			HISTB_HDMI_INF_CTRL1_AVI_REPEAT,
			HISTB_HDMI_INF_CTRL1_AVI_ENABLE);
		return;
	}

	len = hdmi_avi_infoframe_pack_only(&frame, packet, sizeof(packet));
	if (len <= 0) {
		dev_warn_ratelimited(hdmi->dev,
				     "AVI infoframe pack failed: %zd\n", len);
		histb_hdmi_tx1_disable_infoframe_locked(
			hdmi,
			HISTB_HDMI_INF_CTRL1_AVI_REPEAT,
			HISTB_HDMI_INF_CTRL1_AVI_ENABLE);
		return;
	}

	histb_hdmi_tx1_program_infoframe_locked(hdmi, HISTB_HDMI_AVI_IF_TX1_ADDR,
						packet, len,
						HISTB_HDMI_INF_CTRL1_AVI_REPEAT,
						HISTB_HDMI_INF_CTRL1_AVI_ENABLE);
}

static void histb_hdmi_vendor_infoframe_update_locked(struct histb_hdmi *hdmi,
						       const struct drm_display_mode *mode)
{
	struct hdmi_vendor_infoframe frame;
	const struct drm_connector *connector = hdmi->connector;
	u8 packet[HDMI_INFOFRAME_SIZE(VENDOR)];
	ssize_t len;
	int ret;

	ret = drm_hdmi_vendor_infoframe_from_display_mode(&frame, connector, mode);
	if (ret == -EINVAL) {
		histb_hdmi_tx1_disable_infoframe_locked(
			hdmi,
			HISTB_HDMI_INF_CTRL1_MPEG_REPEAT,
			HISTB_HDMI_INF_CTRL1_MPEG_ENABLE);
		return;
	}

	if (ret) {
		dev_warn_ratelimited(hdmi->dev,
				     "vendor infoframe compose failed: %d\n", ret);
		histb_hdmi_tx1_disable_infoframe_locked(
			hdmi,
			HISTB_HDMI_INF_CTRL1_MPEG_REPEAT,
			HISTB_HDMI_INF_CTRL1_MPEG_ENABLE);
		return;
	}

	/* Regular video only: no 3D and no 4K vendor packet. */
	if (!frame.vic && frame.s3d_struct == HDMI_3D_STRUCTURE_INVALID) {
		histb_hdmi_tx1_disable_infoframe_locked(
			hdmi,
			HISTB_HDMI_INF_CTRL1_MPEG_REPEAT,
			HISTB_HDMI_INF_CTRL1_MPEG_ENABLE);
		return;
	}

	len = hdmi_vendor_infoframe_pack_only(&frame, packet, sizeof(packet));
	if (len <= 0) {
		dev_warn_ratelimited(hdmi->dev,
				     "vendor infoframe pack failed: %zd\n", len);
		histb_hdmi_tx1_disable_infoframe_locked(
			hdmi,
			HISTB_HDMI_INF_CTRL1_MPEG_REPEAT,
			HISTB_HDMI_INF_CTRL1_MPEG_ENABLE);
		return;
	}

	histb_hdmi_tx1_program_infoframe_locked(hdmi, HISTB_HDMI_MPEG_IF_TX1_ADDR,
						packet, len,
						HISTB_HDMI_INF_CTRL1_MPEG_REPEAT,
						HISTB_HDMI_INF_CTRL1_MPEG_ENABLE);
}

static void histb_hdmi_video_infoframes_update_locked(struct histb_hdmi *hdmi,
						       const struct drm_display_mode *mode)
{
	histb_hdmi_avi_infoframe_update_locked(hdmi, mode);
	histb_hdmi_vendor_infoframe_update_locked(hdmi, mode);
}

static void histb_hdmi_audio_infoframe_rearm_locked(
	struct histb_hdmi *hdmi, const struct hdmi_audio_infoframe *cea,
	unsigned int sample_width, unsigned int sample_rate,
	unsigned int channels)
{
	struct hdmi_audio_infoframe frame;
	u8 packet[HDMI_INFOFRAME_SIZE(AUDIO)];
	ssize_t len;
	u32 val;

	if (cea) {
		frame = *cea;
	} else {
		hdmi_audio_infoframe_init(&frame);
		frame.channels = 1; /* 2ch in CEA encoding */
	}

	if (hdmi_audio_infoframe_check(&frame)) {
		hdmi_audio_infoframe_init(&frame);
		frame.channels = 1; /* 2ch in CEA encoding */
	}

	/*
	 * Force explicit PCM infoframe fields (instead of STREAM placeholders)
	 * to match legacy sink expectations on this HDMI 1.4 block.
	 */
	frame.coding_type = HDMI_AUDIO_CODING_TYPE_PCM;
	frame.channels = channels;
	switch (sample_rate) {
	case 32000:
		frame.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_32000;
		break;
	case 44100:
		frame.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_44100;
		break;
	case 88200:
		frame.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_88200;
		break;
	case 96000:
		frame.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_96000;
		break;
	case 176400:
		frame.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_176400;
		break;
	case 192000:
		frame.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_192000;
		break;
	case 48000:
	default:
		frame.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_48000;
		break;
	}
	switch (sample_width) {
	case 24:
		frame.sample_size = HDMI_AUDIO_SAMPLE_SIZE_24;
		break;
	case 20:
		frame.sample_size = HDMI_AUDIO_SAMPLE_SIZE_20;
		break;
	case 16:
	default:
		frame.sample_size = HDMI_AUDIO_SAMPLE_SIZE_16;
		break;
	}

	len = hdmi_audio_infoframe_pack_only(&frame, packet, sizeof(packet));
	histb_hdmi_tx1_program_infoframe_locked(
		hdmi,
		HISTB_HDMI_AUD_IF_TX1_ADDR,
		packet, len,
		HISTB_HDMI_INF_CTRL1_AUD_REPEAT,
		HISTB_HDMI_INF_CTRL1_AUD_ENABLE);

	/*
	 * Mirror the same disable->enable pulse in indexed CEA audio gate.
	 * Skip this in tx1-only mode to avoid fighting legacy packetizer flow.
	 */
	if (!histb_hdmi_tx1_only_audio_path()) {
		val = histb_hdmi_read(hdmi, HISTB_HDMI_CEA_AUD_CFG_ADDR);
		val &= ~(HISTB_HDMI_CEA_AUD_EN | HISTB_HDMI_CEA_AUD_RPT_EN);
		histb_hdmi_write(hdmi, HISTB_HDMI_CEA_AUD_CFG_ADDR, val);
		udelay(1);
		val |= HISTB_HDMI_CEA_AUD_EN | HISTB_HDMI_CEA_AUD_RPT_EN;
		histb_hdmi_write(hdmi, HISTB_HDMI_CEA_AUD_CFG_ADDR, val);
	}
}

static u8 histb_hdmi_audio_mode_value(void)
{
	return FIELD_PREP(HISTB_HDMI_AUD_MODE_TX1_I2S_EN,
			  HISTB_HDMI_AUD_MODE_TX1_I2S_SD0) |
	       HISTB_HDMI_AUD_MODE_TX1_AUDIO_EN;
}

static void histb_hdmi_audio_tx1_clocking_setup_locked(struct histb_hdmi *hdmi)
{
	/*
	 * Keep tx1 in 256fs MCLK mode and explicitly bypass sample-rate
	 * conversion, matching SDK LPCM I2S flow.
	 */
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_FREQ_SVAL_TX1_ADDR,
			     HISTB_HDMI_AUD_FREQ_SVAL_256FS);
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_SAMPLE_RATE_CONV_TX1_ADDR,
			     HISTB_HDMI_AUD_SAMPLE_RATE_CONV_BYPASS);
}

static void histb_hdmi_audio_tx1_input_enable_locked(struct histb_hdmi *hdmi,
						      bool enable)
{
	u32 val;

	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_EN_TX1_ADDR);
	val |= HISTB_HDMI_AUD_EN_TX1_AUD_SEL_OWRT;
	if (enable)
		val |= HISTB_HDMI_AUD_EN_TX1_AUD_IN_EN;
	else
		val &= ~HISTB_HDMI_AUD_EN_TX1_AUD_IN_EN;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_AUD_EN_TX1_ADDR, val);
}

static int histb_hdmi_audio_rate_cfg(unsigned int sample_rate, u8 *fs_cfg,
				     u32 *n_val)
{
	u8 fs;
	u32 n;

	switch (sample_rate) {
	case 32000:
		fs = HISTB_HDMI_TPI_AUD_FS_32K;
		n = HISTB_HDMI_AUD_N_32K;
		break;
	case 44100:
		fs = HISTB_HDMI_TPI_AUD_FS_44K;
		n = HISTB_HDMI_AUD_N_44K;
		break;
	case 88200:
		fs = HISTB_HDMI_TPI_AUD_FS_88K;
		n = HISTB_HDMI_AUD_N_88K;
		break;
	case 96000:
		fs = HISTB_HDMI_TPI_AUD_FS_96K;
		n = HISTB_HDMI_AUD_N_96K;
		break;
	case 176400:
		fs = HISTB_HDMI_TPI_AUD_FS_176K;
		n = HISTB_HDMI_AUD_N_176K;
		break;
	case 192000:
		fs = HISTB_HDMI_TPI_AUD_FS_192K;
		n = HISTB_HDMI_AUD_N_192K;
		break;
	case 48000:
		fs = HISTB_HDMI_TPI_AUD_FS_48K;
		n = HISTB_HDMI_AUD_N_48K;
		break;
	default:
		return -EINVAL;
	}

	if (fs_cfg)
		*fs_cfg = fs;
	if (n_val)
		*n_val = n;

	return 0;
}

static bool histb_hdmi_tx1_only_audio_path(void)
{
	return true;
}

static void histb_hdmi_audio_reset_locked(struct histb_hdmi *hdmi)
{
	u32 rst_mask = HISTB_HDMI_TX_PWD_RST_CTRL_AUD_SRST |
		       HISTB_HDMI_TX_PWD_RST_CTRL_ACR_SRST |
		       HISTB_HDMI_TX_PWD_RST_CTRL_AFIFO_SRST;
	u32 rst_ctrl;
	u32 rst_set;

	rst_ctrl = histb_hdmi_read(hdmi, HISTB_HDMI_TX_PWD_RST_CTRL_ADDR);
	histb_hdmi_write(hdmi, HISTB_HDMI_TX_PWD_RST_CTRL_ADDR,
				rst_ctrl | rst_mask);
	udelay(10);
	rst_set = histb_hdmi_read(hdmi, HISTB_HDMI_TX_PWD_RST_CTRL_ADDR);
	histb_hdmi_write(hdmi, HISTB_HDMI_TX_PWD_RST_CTRL_ADDR,
				rst_ctrl & ~rst_mask);

	/* If the control block did not take the reset bits, pulse tx1 instead. */
	if ((rst_set & rst_mask) != rst_mask) {
		histb_hdmi_write_tx1(hdmi, HISTB_HDMI_AIP_RST_TX1_ADDR,
				     HISTB_HDMI_AIP_RST_TX1_AUDIO |
					     HISTB_HDMI_AIP_RST_TX1_FIFO |
					     HISTB_HDMI_AIP_RST_TX1_ACR);
		udelay(10);
		histb_hdmi_write_tx1(hdmi, HISTB_HDMI_AIP_RST_TX1_ADDR, 0x00);
	}
}

static void histb_hdmi_audio_path_setup_locked(struct histb_hdmi *hdmi,
					       unsigned int sample_width,
					       unsigned int sample_rate)
{
	u32 val;
	u32 n_val = HISTB_HDMI_AUD_N_48K;
	u8 aud_mode;
	u8 chst_fs_cfg = HISTB_HDMI_TPI_AUD_FS_48K;
	u32 chst_len_cfg = sample_width == 16 ? 0x2 : 0xb;
	u32 i2s_in_length_cfg = chst_len_cfg;
	u8 chst_org_fs_cfg;

	if (histb_hdmi_audio_rate_cfg(sample_rate, &chst_fs_cfg, &n_val))
		histb_hdmi_audio_rate_cfg(48000, &chst_fs_cfg, &n_val);
	chst_org_fs_cfg = chst_fs_cfg;

	histb_hdmi_audio_select_i2s_source_locked(hdmi);

	if (!histb_hdmi_tx1_only_audio_path()) {
		/*
		 * Use the dedicated audio-path register block
		 * (SDK BASE_ADDR_audio_path_reg) instead of tx1 byte registers
		 * for input path/ACR programming.
		 */
		val = histb_hdmi_audio_read_prefer_linear(hdmi,
							  HISTB_HDMI_TX_AUDIO_CTRL_ADDR);
		val &= ~(HISTB_HDMI_AUD_CTRL_LAYOUT |
			 HISTB_HDMI_AUD_CTRL_I2S_EN |
			 HISTB_HDMI_AUD_CTRL_SPDIF_EN |
			 HISTB_HDMI_AUD_CTRL_SRC_EN |
			 HISTB_HDMI_AUD_CTRL_SRC_CTRL |
			 HISTB_HDMI_AUD_CTRL_FIFO0_MAP |
			 HISTB_HDMI_AUD_CTRL_FIFO1_MAP |
			 HISTB_HDMI_AUD_CTRL_FIFO2_MAP |
			 HISTB_HDMI_AUD_CTRL_FIFO3_MAP |
			 HISTB_HDMI_AUD_CTRL_MUTE_EN);
		val |= HISTB_HDMI_AUD_CTRL_IN_EN |
		       FIELD_PREP(HISTB_HDMI_AUD_CTRL_I2S_EN, HISTB_HDMI_AUD_MODE_TX1_I2S_ALL) |
		       FIELD_PREP(HISTB_HDMI_AUD_CTRL_FIFO0_MAP, 0x0) |
		       FIELD_PREP(HISTB_HDMI_AUD_CTRL_FIFO1_MAP, 0x1) |
		       FIELD_PREP(HISTB_HDMI_AUD_CTRL_FIFO2_MAP, 0x2) |
		       FIELD_PREP(HISTB_HDMI_AUD_CTRL_FIFO3_MAP, 0x3);
		histb_hdmi_audio_write_both(hdmi, HISTB_HDMI_TX_AUDIO_CTRL_ADDR, val);

		val = histb_hdmi_audio_read_prefer_linear(hdmi,
							  HISTB_HDMI_AUD_I2S_CTRL_ADDR);
		val &= ~(HISTB_HDMI_AUD_I2S_CTRL_LENGTH |
			 HISTB_HDMI_AUD_I2S_CTRL_CH_SWAP |
			 GENMASK(5, 0));
		val |= FIELD_PREP(HISTB_HDMI_AUD_I2S_CTRL_LENGTH, i2s_in_length_cfg);
		histb_hdmi_audio_write_both(hdmi, HISTB_HDMI_AUD_I2S_CTRL_ADDR, val);

		/* Keep FIFO in normal mode (no test injection, no HBR mask) for LPCM I2S. */
		val = histb_hdmi_audio_read_prefer_linear(hdmi,
							  HISTB_HDMI_AUD_FIFO_CTRL_ADDR);
		val &= ~(HISTB_HDMI_AUD_FIFO_CTRL_TEST | HISTB_HDMI_AUD_FIFO_CTRL_HBR_MASK);
		histb_hdmi_audio_write_both(hdmi, HISTB_HDMI_AUD_FIFO_CTRL_ADDR, val);

		val = FIELD_PREP(HISTB_HDMI_AUD_CHST_CFG0_FS, chst_fs_cfg) |
		      FIELD_PREP(HISTB_HDMI_AUD_CHST_CFG0_CLK_ACC, 0x0);
		histb_hdmi_audio_write_both(hdmi, HISTB_HDMI_AUD_CHST_CFG0_ADDR, val);

		val = FIELD_PREP(HISTB_HDMI_AUD_CHST_CFG1_ORG_FS, chst_org_fs_cfg) |
		      FIELD_PREP(HISTB_HDMI_AUD_CHST_CFG1_LENGTH, chst_len_cfg);
		histb_hdmi_audio_write_both(hdmi, HISTB_HDMI_AUD_CHST_CFG1_ADDR, val);

		/* Enable hardware CTS generation and request ACR updates. */
		val = histb_hdmi_audio_read_prefer_linear(hdmi,
							  HISTB_HDMI_AUD_ACR_CTRL_ADDR);
		val &= ~HISTB_HDMI_AUD_ACR_CTRL_MASK;
		val |= HISTB_HDMI_AUD_ACR_CTS_REQ_EN | HISTB_HDMI_AUD_ACR_CTS_GEN_SEL;
		val &= ~HISTB_HDMI_AUD_ACR_CTS_HW_SW_SEL;
		histb_hdmi_audio_write_both(hdmi, HISTB_HDMI_AUD_ACR_CTRL_ADDR, val);
		histb_hdmi_audio_write_both(hdmi, HISTB_HDMI_ACR_N_VAL_SW_ADDR, n_val);

		/* Keep CEA audio infoframe generation enabled while stream is active. */
		val = histb_hdmi_audio_read_prefer_linear(hdmi,
							  HISTB_HDMI_CEA_AUD_CFG_ADDR);
		val |= HISTB_HDMI_CEA_AUD_EN | HISTB_HDMI_CEA_AUD_RPT_EN;
		histb_hdmi_audio_write_both(hdmi, HISTB_HDMI_CEA_AUD_CFG_ADDR, val);
	}

	/*
	 * Audio controls are split between the dedicated block and the tx1
	 * packetizer, so program both.
	 */
	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_ACR_CTRL_TX1_ADDR);
	val &= ~HISTB_HDMI_AUD_ACR_TX1_MASK;
	val |= HISTB_HDMI_AUD_ACR_TX1_CTS_REQ_EN |
	       HISTB_HDMI_AUD_ACR_TX1_MCLK_EN;
	val &= ~HISTB_HDMI_AUD_ACR_TX1_CTS_HW_SW_SEL;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_AUD_ACR_CTRL_TX1_ADDR, val);

	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_I2S_IN_CTRL_TX1_ADDR,
			     HISTB_HDMI_I2S_IN_CTRL_TX1_CBIT_ORDER |
			     HISTB_HDMI_I2S_IN_CTRL_TX1_SCK_RISING);
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_I2S_IN_SIZE_TX1_ADDR, i2s_in_length_cfg);
	/*
	 * Match SDK si_audio.c for I2S:
	 *   CHST4(0x21) = Fs
	 *   CHST5(0x22) = Length | (OrgFs << 4)
	 */
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_I2S_CHST4_TX1_ADDR, chst_fs_cfg);
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_I2S_CHST5_TX1_ADDR,
			     (chst_org_fs_cfg << 4) | chst_len_cfg);
	histb_hdmi_audio_reset_locked(hdmi);

	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_AUD_ACR_CTRL_TX1_ADDR, val);
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_N_SVAL1_TX1_ADDR, n_val & 0xff);
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_N_SVAL2_TX1_ADDR, (n_val >> 8) & 0xff);
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_N_SVAL3_TX1_ADDR, (n_val >> 16) & 0x0f);
	histb_hdmi_audio_tx1_clocking_setup_locked(hdmi);
	histb_hdmi_audio_tx1_input_enable_locked(hdmi, true);
	aud_mode = histb_hdmi_audio_mode_value();
	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_MODE_TX1_ADDR);
	val &= ~(HISTB_HDMI_AUD_MODE_TX1_AUDIO_EN |
		 HISTB_HDMI_AUD_MODE_TX1_SPDIF_SEL |
		 HISTB_HDMI_AUD_MODE_TX1_HBRA_ON |
		 HISTB_HDMI_AUD_MODE_TX1_DSD_SEL |
		 HISTB_HDMI_AUD_MODE_TX1_I2S_EN);
	val |= aud_mode;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_AUD_MODE_TX1_ADDR, val);

	/* Keep TPI audio path unmuted and force negotiated sample-rate signaling. */
	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_TPI_AUD_CONFIG_TX1_ADDR);
	val &= ~HISTB_HDMI_TPI_AUD_CONFIG_MUTE;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_TPI_AUD_CONFIG_TX1_ADDR, val);
	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_TPI_AUD_FS_TX1_ADDR);
	val &= ~(HISTB_HDMI_TPI_AUD_FS_VAL | HISTB_HDMI_TPI_AUD_FS_OVRD);
	val |= FIELD_PREP(HISTB_HDMI_TPI_AUD_FS_VAL, chst_fs_cfg) |
	       HISTB_HDMI_TPI_AUD_FS_OVRD;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_TPI_AUD_FS_TX1_ADDR, val);

	if (!histb_hdmi_tx1_only_audio_path()) {
		/* The AIP reset drops part of TX_AUDIO_CTRL, so re-apply it. */
		val = histb_hdmi_audio_read_prefer_linear(hdmi,
							  HISTB_HDMI_TX_AUDIO_CTRL_ADDR);
		val &= ~(HISTB_HDMI_AUD_CTRL_MUTE_EN | HISTB_HDMI_AUD_CTRL_SPDIF_EN |
			 HISTB_HDMI_AUD_CTRL_SRC_EN | HISTB_HDMI_AUD_CTRL_SRC_CTRL |
			 HISTB_HDMI_AUD_CTRL_LAYOUT | HISTB_HDMI_AUD_CTRL_I2S_EN |
			 HISTB_HDMI_AUD_CTRL_FIFO0_MAP | HISTB_HDMI_AUD_CTRL_FIFO1_MAP |
			 HISTB_HDMI_AUD_CTRL_FIFO2_MAP | HISTB_HDMI_AUD_CTRL_FIFO3_MAP);
		val |= HISTB_HDMI_AUD_CTRL_IN_EN |
		       FIELD_PREP(HISTB_HDMI_AUD_CTRL_I2S_EN, HISTB_HDMI_AUD_MODE_TX1_I2S_ALL) |
		       FIELD_PREP(HISTB_HDMI_AUD_CTRL_FIFO0_MAP, 0x0) |
		       FIELD_PREP(HISTB_HDMI_AUD_CTRL_FIFO1_MAP, 0x1) |
		       FIELD_PREP(HISTB_HDMI_AUD_CTRL_FIFO2_MAP, 0x2) |
		       FIELD_PREP(HISTB_HDMI_AUD_CTRL_FIFO3_MAP, 0x3);
		histb_hdmi_audio_write_both(hdmi, HISTB_HDMI_TX_AUDIO_CTRL_ADDR, val);
	}

	/* Layout0 for 2ch and SD0 I2S lane enabled. */
	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUDP_TXCTRL_ADDR);
	val &= ~HISTB_HDMI_BIT_AUDP_LAYOUT;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_AUDP_TXCTRL_ADDR, val);

	/* Enable audio infoframe repeat in the legacy tx1 control page. */
	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_INF_CTRL1_TX1_ADDR);
	val |= HISTB_HDMI_INF_CTRL1_AUD_REPEAT | HISTB_HDMI_INF_CTRL1_AUD_ENABLE;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_INF_CTRL1_TX1_ADDR, val);
}

static void histb_hdmi_audio_runtime_reassert_locked(struct histb_hdmi *hdmi,
						     unsigned int sample_rate)
{
	u32 val;
	u8 aud_mode;
	u8 tpi_fs = HISTB_HDMI_TPI_AUD_FS_48K;

	if (histb_hdmi_audio_rate_cfg(sample_rate, &tpi_fs, NULL))
		histb_hdmi_audio_rate_cfg(48000, &tpi_fs, NULL);

	histb_hdmi_audio_tx1_clocking_setup_locked(hdmi);
	histb_hdmi_audio_tx1_input_enable_locked(hdmi, true);

	aud_mode = histb_hdmi_audio_mode_value();
	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_MODE_TX1_ADDR);
	val &= ~(HISTB_HDMI_AUD_MODE_TX1_AUDIO_EN |
		 HISTB_HDMI_AUD_MODE_TX1_SPDIF_SEL |
		 HISTB_HDMI_AUD_MODE_TX1_HBRA_ON |
		 HISTB_HDMI_AUD_MODE_TX1_DSD_SEL |
		 HISTB_HDMI_AUD_MODE_TX1_I2S_EN);
	val |= aud_mode;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_AUD_MODE_TX1_ADDR, val);

	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_TPI_AUD_CONFIG_TX1_ADDR);
	val &= ~HISTB_HDMI_TPI_AUD_CONFIG_MUTE;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_TPI_AUD_CONFIG_TX1_ADDR, val);

	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_TPI_AUD_FS_TX1_ADDR);
	val &= ~(HISTB_HDMI_TPI_AUD_FS_VAL | HISTB_HDMI_TPI_AUD_FS_OVRD);
	val |= FIELD_PREP(HISTB_HDMI_TPI_AUD_FS_VAL, tpi_fs) |
	       HISTB_HDMI_TPI_AUD_FS_OVRD;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_TPI_AUD_FS_TX1_ADDR, val);
}

static void histb_hdmi_audio_warmup_locked(struct histb_hdmi *hdmi)
{
	/*
	 * video artifacts until tx1 audio-side packetizer state
	 * is initialized once. Prime the tx1 audio block on HPD, but keep audio
	 * input disabled and muted until a real stream starts.
	 */
	histb_hdmi_audio_path_setup_locked(hdmi, 16, 48000);
	histb_hdmi_audio_runtime_reassert_locked(hdmi, 48000);
	histb_hdmi_audio_mute_locked(hdmi, true);
	histb_hdmi_audio_tx1_input_enable_locked(hdmi, false);
}

static void histb_hdmi_audio_cache_hw_params_locked(
	struct histb_hdmi *hdmi, const struct hdmi_codec_params *params)
{
	hdmi->audio_sample_rate = params->sample_rate;
	hdmi->audio_sample_width = params->sample_width;
	hdmi->audio_channels = params->channels;
	hdmi->audio_cea = params->cea;
}

static void histb_hdmi_audio_restore_stream_locked(struct histb_hdmi *hdmi)
{
	if (!hdmi->audio_stream_active)
		return;

	/*
	 * Mode reprogramming may reset parts of TX1 audio packetizer state.
	 * Re-apply cached hw_params so active streams survive modeset/replug.
	 */
	histb_hdmi_audio_mute_locked(hdmi, true);
	histb_hdmi_audio_tx1_input_enable_locked(hdmi, false);
	histb_hdmi_audio_path_setup_locked(hdmi, hdmi->audio_sample_width,
					   hdmi->audio_sample_rate);
	histb_hdmi_audio_runtime_reassert_locked(hdmi, hdmi->audio_sample_rate);
	histb_hdmi_audio_infoframe_rearm_locked(hdmi, &hdmi->audio_cea,
						hdmi->audio_sample_width,
						hdmi->audio_sample_rate,
						hdmi->audio_channels);
	histb_hdmi_audio_set_hdmi_output_locked(hdmi, true);
	histb_hdmi_audio_mute_locked(hdmi, hdmi->audio_requested_mute);
}

static u8 histb_hdmi_audio_eld_rate_mask(unsigned int sample_rate)
{
	switch (sample_rate) {
	case 32000:
		return HISTB_EDID_SAD_RATE_32K_MASK;
	case 44100:
		return HISTB_EDID_SAD_RATE_44K1_MASK;
	case 48000:
		return HISTB_EDID_SAD_RATE_48K_MASK;
	case 88200:
		return HISTB_EDID_SAD_RATE_88K2_MASK;
	case 96000:
		return HISTB_EDID_SAD_RATE_96K_MASK;
	case 176400:
		return HISTB_EDID_SAD_RATE_176K4_MASK;
	case 192000:
		return HISTB_EDID_SAD_RATE_192K_MASK;
	default:
		return 0;
	}
}

static int histb_hdmi_audio_validate_eld_locked(struct histb_hdmi *hdmi,
						 unsigned int sample_rate,
						 unsigned int sample_width,
						 unsigned int channels,
						 const char **reason)
{
	const u8 *sad;
	int sad_count;
	u8 req_rate_mask;
	u8 req_width_mask;
	bool have_lpcm_sad = false;
	bool have_channel_match = false;
	bool have_rate_match = false;
	int i;

	if (!hdmi->eld_valid) {
		if (reason)
			*reason = "ELD unavailable (strict policy)";
		return -EINVAL;
	}

	req_rate_mask = histb_hdmi_audio_eld_rate_mask(sample_rate);
	if (!req_rate_mask) {
		if (reason)
			*reason = "unsupported sample rate mapping";
		return -EINVAL;
	}

	switch (sample_width) {
	case 16:
		req_width_mask = HISTB_EDID_SAD_WIDTH_16_MASK;
		break;
	case 24:
		req_width_mask = HISTB_EDID_SAD_WIDTH_24_MASK;
		break;
	default:
		if (reason)
			*reason = "unsupported sample width";
		return -EINVAL;
	}

	sad = drm_eld_sad(hdmi->eld);
	sad_count = drm_eld_sad_count(hdmi->eld);
	if (!sad || sad_count <= 0) {
		if (reason)
			*reason = "ELD has no SAD";
		return -EINVAL;
	}

	for (i = 0; i < sad_count; i++, sad += 3) {
		u8 format;
		u8 max_channels;

		format = FIELD_GET(HISTB_EDID_SAD_FORMAT_MASK, sad[0]);
		if (format != HISTB_EDID_SAD_FORMAT_LPCM)
			continue;

		have_lpcm_sad = true;

		max_channels = FIELD_GET(HISTB_EDID_SAD_CHANNELS_MASK, sad[0]) + 1;
		if (max_channels < channels)
			continue;
		have_channel_match = true;

		if (!(sad[1] & req_rate_mask))
			continue;
		have_rate_match = true;

		if (!(sad[2] & req_width_mask))
			continue;

		return 0;
	}

	if (reason) {
		if (!have_lpcm_sad)
			*reason = "ELD has no LPCM SAD";
		else if (!have_channel_match)
			*reason = "LPCM SAD channel mismatch";
		else if (!have_rate_match)
			*reason = "LPCM SAD rate mismatch";
		else
			*reason = "LPCM SAD sample-width mismatch";
	}

	return -EINVAL;
}

static void histb_hdmi_clear_eld(struct histb_hdmi *hdmi)
{
	memset(hdmi->eld, 0, sizeof(hdmi->eld));
	hdmi->eld_valid = false;
}

static void histb_hdmi_build_fallback_eld(struct histb_hdmi *hdmi)
{
	u8 *eld = hdmi->eld;
	u8 *sad;
	u8 mnl = 0;

	histb_hdmi_clear_eld(hdmi);

	eld[DRM_ELD_VER] = DRM_ELD_VER_CEA861D;
	eld[DRM_ELD_CEA_EDID_VER_MNL] =
		(3 << DRM_ELD_CEA_EDID_VER_SHIFT) | (mnl & DRM_ELD_MNL_MASK);
	eld[DRM_ELD_SAD_COUNT_CONN_TYPE] =
		(1 << DRM_ELD_SAD_COUNT_SHIFT) | DRM_ELD_CONN_TYPE_HDMI;

	sad = &eld[DRM_ELD_CEA_SAD(mnl, 0)];
	sad[0] = FIELD_PREP(HISTB_EDID_SAD_FORMAT_MASK, HISTB_EDID_SAD_FORMAT_LPCM) |
		 FIELD_PREP(HISTB_EDID_SAD_CHANNELS_MASK, 1);
	sad[1] = HISTB_EDID_SAD_RATE_32K_MASK |
		 HISTB_EDID_SAD_RATE_44K1_MASK |
		 HISTB_EDID_SAD_RATE_48K_MASK |
		 HISTB_EDID_SAD_RATE_88K2_MASK |
		 HISTB_EDID_SAD_RATE_96K_MASK |
		 HISTB_EDID_SAD_RATE_176K4_MASK |
		 HISTB_EDID_SAD_RATE_192K_MASK;
	sad[2] = HISTB_EDID_SAD_WIDTH_16_MASK |
		 HISTB_EDID_SAD_WIDTH_24_MASK;

	eld[DRM_ELD_BASELINE_ELD_LEN] =
		DIV_ROUND_UP(drm_eld_calc_baseline_block_size(eld), 4);
	hdmi->eld_valid = true;
}

static void histb_hdmi_build_eld(struct histb_hdmi *hdmi, const u8 *block0,
				 const u8 *block1)
{
	char monitor_name[HISTB_HDMI_ELD_MNL_MAX + 1];
	u8 *eld = hdmi->eld;
	u8 cea_rev = 0;
	u8 mnl;
	u8 end;
	u8 idx;
	int sad_count = 0;

	memset(monitor_name, 0, sizeof(monitor_name));
	drm_edid_get_monitor_name((const struct edid *)block0, monitor_name,
				  sizeof(monitor_name));

	mnl = min_t(size_t, strnlen(monitor_name, sizeof(monitor_name)),
		    HISTB_HDMI_ELD_MNL_MAX);

	histb_hdmi_clear_eld(hdmi);

	eld[DRM_ELD_VER] = DRM_ELD_VER_CEA861D;
	eld[DRM_ELD_MANUFACTURER_NAME0] = block0[8];
	eld[DRM_ELD_MANUFACTURER_NAME1] = block0[9];
	eld[DRM_ELD_PRODUCT_CODE0] = block0[10];
	eld[DRM_ELD_PRODUCT_CODE1] = block0[11];
	memcpy(&eld[DRM_ELD_MONITOR_NAME_STRING], monitor_name, mnl);

	if (block1 && block1[0] == HISTB_EDID_EXT_CEA) {
		cea_rev = block1[1] & 0x7;
		end = block1[2];
		if (end < 4 || end > 127)
			end = 127;

		idx = 4;
		while (idx < end) {
			const u8 *data;
			u8 tag;
			u8 len;
			int copy_sads;

			tag = block1[idx] >> 5;
			len = block1[idx] & 0x1f;
			if (idx + 1 + len > end)
				break;

			data = block1 + idx + 1;
			switch (tag) {
			case HISTB_EDID_CEA_TAG_AUDIO:
				copy_sads = min_t(int, len / 3,
						  HISTB_HDMI_ELD_MAX_SAD - sad_count);
				if (copy_sads > 0) {
					memcpy(&eld[DRM_ELD_CEA_SAD(mnl, sad_count)], data,
					       copy_sads * 3);
					sad_count += copy_sads;
				}
				break;
			case HISTB_EDID_CEA_TAG_SPEAKER:
				if (len >= 1)
					eld[DRM_ELD_SPEAKER] = data[0] & DRM_ELD_SPEAKER_MASK;
				break;
			default:
				break;
			}

			idx += 1 + len;
		}
	}

	eld[DRM_ELD_CEA_EDID_VER_MNL] = (cea_rev << DRM_ELD_CEA_EDID_VER_SHIFT) |
					(mnl & DRM_ELD_MNL_MASK);
	eld[DRM_ELD_SAD_COUNT_CONN_TYPE] =
		(sad_count << DRM_ELD_SAD_COUNT_SHIFT) | DRM_ELD_CONN_TYPE_HDMI;
	eld[DRM_ELD_BASELINE_ELD_LEN] =
		DIV_ROUND_UP(drm_eld_calc_baseline_block_size(eld), 4);
	hdmi->eld_valid = true;
}

static int histb_hdmi_refresh_eld(struct histb_hdmi *hdmi)
{
	u8 block0[HISTB_EDID_BLOCK_SIZE];
	u8 block1[HISTB_EDID_BLOCK_SIZE];
	u8 old_eld[HISTB_HDMI_ELD_MAX_BYTES];
	bool had_old_eld = hdmi->eld_valid;
	bool have_cea = false;
	u8 ext_blocks;
	int ret;

	if (had_old_eld)
		memcpy(old_eld, hdmi->eld, sizeof(old_eld));

	ret = histb_hdmi_read_edid_block(hdmi, 0, block0);
	if (ret)
		goto clear;

	if (!histb_hdmi_edid_header_ok(block0) || !histb_hdmi_edid_checksum_ok(block0)) {
		ret = -EINVAL;
		goto clear;
	}

	ext_blocks = block0[126];
	if (ext_blocks > 0) {
		ret = histb_hdmi_read_edid_block(hdmi, 1, block1);
		if (!ret && histb_hdmi_edid_checksum_ok(block1) &&
		    block1[0] == HISTB_EDID_EXT_CEA)
			have_cea = true;
	}

	histb_hdmi_build_eld(hdmi, block0, have_cea ? block1 : NULL);
	return 0;

clear:
	hdmi->stats_edid_fail++;
	histb_hdmi_trace_error_locked(hdmi, "refresh-eld", ret, true);

	if (had_old_eld) {
		/*
		 * Keep previous ELD on transient DDC/EDID failures while the
		 * sink is still connected. This avoids hw_params
		 * rejection during mode changes.
		 */
		memcpy(hdmi->eld, old_eld, sizeof(old_eld));
		hdmi->eld_valid = true;
	} else {
		histb_hdmi_clear_eld(hdmi);
	}

	return ret;
}

static int histb_hdmi_audio_hw_params(struct device *dev, void *data,
				      struct hdmi_codec_daifmt *daifmt,
				      struct hdmi_codec_params *params)
{
	struct histb_hdmi *hdmi = dev_get_drvdata(dev);
	const char *eld_reason = NULL;
	int eld_check;

	(void)data;

	if (!hdmi)
		return -ENODEV;

	if (daifmt->fmt != HDMI_I2S)
		return -EINVAL;

	if (daifmt->bit_clk_inv || daifmt->frame_clk_inv ||
	    daifmt->bit_clk_provider || daifmt->frame_clk_provider)
		return -EINVAL;

	if (params->channels != 2)
		return -EINVAL;

	if (params->sample_rate != 32000 &&
	    params->sample_rate != 44100 &&
	    params->sample_rate != 48000 &&
	    params->sample_rate != 88200 &&
	    params->sample_rate != 96000 &&
	    params->sample_rate != 176400 &&
	    params->sample_rate != 192000)
		return -EINVAL;

	if (params->sample_width != 16 && params->sample_width != 24)
		return -EINVAL;

	mutex_lock(&hdmi->lock);
	eld_check = histb_hdmi_audio_validate_eld_locked(hdmi,
							  params->sample_rate,
							  params->sample_width,
							  params->channels,
							  &eld_reason);
	if (eld_check) {
		dev_warn_ratelimited(dev,
				     "audio hw_params ELD mismatch, using fallback policy: rate=%u width=%u ch=%u (%s)\n",
				     params->sample_rate, params->sample_width,
				     params->channels,
				     eld_reason ? eld_reason : "unknown");
	}

	/*
	 * Production sequence: keep sink muted while reprogramming CHST/ACR/N/IF
	 * to avoid audible pops and transient packet-state mismatch.
	 */
	histb_hdmi_audio_mute_locked(hdmi, true);
	histb_hdmi_audio_tx1_input_enable_locked(hdmi, false);

	histb_hdmi_audio_cache_hw_params_locked(hdmi, params);
	histb_hdmi_audio_path_setup_locked(hdmi, params->sample_width,
					   params->sample_rate);
	histb_hdmi_audio_runtime_reassert_locked(hdmi, params->sample_rate);
	histb_hdmi_audio_infoframe_rearm_locked(hdmi, &params->cea,
						params->sample_width,
						params->sample_rate,
						params->channels);
	histb_hdmi_audio_set_hdmi_output_locked(hdmi, true);
	hdmi->audio_stream_active = true;
	hdmi->audio_requested_mute = false;
	histb_hdmi_audio_mute_locked(hdmi, false);
	dev_dbg_ratelimited(dev,
		"audio hw_params: data_ctrl=0x%02x tx_swrst=0x%02x audp_txctrl=0x%02x peri_hdmitx=0x%08x peri_hdmitx_alt=0x%08x tx_audio_idx=0x%08x tx_audio_lin=0x%08x i2s_idx=0x%08x i2s_lin=0x%08x fifo_idx=0x%08x fifo_lin=0x%08x chst1_idx=0x%08x chst1_lin=0x%08x acr_idx=0x%08x acr_lin=0x%08x n_idx=0x%08x n_lin=0x%08x cea_idx=0x%08x cea_lin=0x%08x tx_pwd_rst_idx=0x%08x tx_pwd_rst_lin=0x%08x inf_ctrl1=0x%02x aud_if0=0x%02x aud_if1=0x%02x aud_if2=0x%02x aud_if3=0x%02x aud_if4=0x%02x aud_if5=0x%02x tx1_acr=0x%02x tx1_i2s_ctrl=0x%02x tx1_chst4=0x%02x tx1_chst5=0x%02x aud_mode=0x%02x aud_en=0x%02x tpi_aud_cfg=0x%02x tpi_aud_fs=0x%02x n1=0x%02x n2=0x%02x n3=0x%02x rate=%u ch=%u width=%u\n",
		histb_hdmi_read_tx0(hdmi, HISTB_HDMI_DATA_CTRL_ADDR),
		histb_hdmi_read_tx0(hdmi, HISTB_HDMI_TX_SWRST_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUDP_TXCTRL_ADDR),
		histb_hdmi_audio_source_status(hdmi),
		histb_hdmi_audio_source_status_alt(hdmi),
		histb_hdmi_read(hdmi, HISTB_HDMI_TX_AUDIO_CTRL_ADDR),
		histb_hdmi_read_linear(hdmi, HISTB_HDMI_TX_AUDIO_CTRL_ADDR),
		histb_hdmi_read(hdmi, HISTB_HDMI_AUD_I2S_CTRL_ADDR),
		histb_hdmi_read_linear(hdmi, HISTB_HDMI_AUD_I2S_CTRL_ADDR),
		histb_hdmi_read(hdmi, HISTB_HDMI_AUD_FIFO_CTRL_ADDR),
		histb_hdmi_read_linear(hdmi, HISTB_HDMI_AUD_FIFO_CTRL_ADDR),
		histb_hdmi_read(hdmi, HISTB_HDMI_AUD_CHST_CFG1_ADDR),
		histb_hdmi_read_linear(hdmi, HISTB_HDMI_AUD_CHST_CFG1_ADDR),
		histb_hdmi_read(hdmi, HISTB_HDMI_AUD_ACR_CTRL_ADDR),
		histb_hdmi_read_linear(hdmi, HISTB_HDMI_AUD_ACR_CTRL_ADDR),
		histb_hdmi_read(hdmi, HISTB_HDMI_ACR_N_VAL_SW_ADDR),
		histb_hdmi_read_linear(hdmi, HISTB_HDMI_ACR_N_VAL_SW_ADDR),
		histb_hdmi_read(hdmi, HISTB_HDMI_CEA_AUD_CFG_ADDR),
		histb_hdmi_read_linear(hdmi, HISTB_HDMI_CEA_AUD_CFG_ADDR),
		histb_hdmi_read(hdmi, HISTB_HDMI_TX_PWD_RST_CTRL_ADDR),
		histb_hdmi_read_linear(hdmi, HISTB_HDMI_TX_PWD_RST_CTRL_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_INF_CTRL1_TX1_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_IF_TX1_ADDR + 0),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_IF_TX1_ADDR + 1),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_IF_TX1_ADDR + 2),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_IF_TX1_ADDR + 3),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_IF_TX1_ADDR + 4),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_IF_TX1_ADDR + 5),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_ACR_CTRL_TX1_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_I2S_IN_CTRL_TX1_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_I2S_CHST4_TX1_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_I2S_CHST5_TX1_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_MODE_TX1_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_EN_TX1_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_TPI_AUD_CONFIG_TX1_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_TPI_AUD_FS_TX1_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_N_SVAL1_TX1_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_N_SVAL2_TX1_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_N_SVAL3_TX1_ADDR),
		params->sample_rate, params->channels, params->sample_width);

	mutex_unlock(&hdmi->lock);
	return 0;
}

static void histb_hdmi_audio_shutdown(struct device *dev, void *data)
{
	struct histb_hdmi *hdmi = dev_get_drvdata(dev);

	(void)data;

	if (!hdmi)
		return;

	mutex_lock(&hdmi->lock);

	/*
	 * Keep the audio packetizer configured across stream stop/start and
	 * only drop incoming samples, so the sink does not retrain visibly.
	 */
	if (!histb_hdmi_tx1_only_audio_path()) {
		u32 tx_audio_ctrl;

		tx_audio_ctrl = histb_hdmi_audio_read_prefer_linear(hdmi,
								     HISTB_HDMI_TX_AUDIO_CTRL_ADDR);
		tx_audio_ctrl &= ~HISTB_HDMI_AUD_CTRL_IN_EN;
		histb_hdmi_audio_write_both(hdmi, HISTB_HDMI_TX_AUDIO_CTRL_ADDR,
					    tx_audio_ctrl);
	}

	histb_hdmi_audio_mute_locked(hdmi, true);
	/*
	 * Keep HDMI packetizer path alive across stop/start and only stop new
	 * input samples. This minimizes sink-side re-training side effects.
	 */
	histb_hdmi_audio_tx1_input_enable_locked(hdmi, false);
	hdmi->audio_stream_active = false;
	hdmi->audio_requested_mute = true;
	dev_dbg_ratelimited(dev,
		"audio shutdown: data_ctrl=0x%02x tx_swrst=0x%02x tx_audio_idx=0x%08x audp_txctrl=0x%02x aud_en=0x%02x tpi_aud_cfg=0x%02x tpi_aud_fs=0x%02x\n",
		histb_hdmi_read_tx0(hdmi, HISTB_HDMI_DATA_CTRL_ADDR),
		histb_hdmi_read_tx0(hdmi, HISTB_HDMI_TX_SWRST_ADDR),
		histb_hdmi_read(hdmi, HISTB_HDMI_TX_AUDIO_CTRL_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUDP_TXCTRL_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUD_EN_TX1_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_TPI_AUD_CONFIG_TX1_ADDR),
		histb_hdmi_read_tx1(hdmi, HISTB_HDMI_TPI_AUD_FS_TX1_ADDR));
	mutex_unlock(&hdmi->lock);
}

static int histb_hdmi_audio_mute_stream(struct device *dev, void *data,
					bool enable, int direction)
{
	struct histb_hdmi *hdmi = dev_get_drvdata(dev);

	(void)data;
	(void)direction;

	if (!hdmi)
		return -ENODEV;

	mutex_lock(&hdmi->lock);
	hdmi->audio_requested_mute = enable;
	histb_hdmi_audio_mute_locked(hdmi, enable);
	mutex_unlock(&hdmi->lock);

	return 0;
}

static int histb_hdmi_audio_get_eld(struct device *dev, void *data,
				    u8 *buf, size_t len)
{
	struct histb_hdmi *hdmi = dev_get_drvdata(dev);
	size_t copy_len;

	(void)data;

	if (!hdmi)
		return -ENODEV;

	mutex_lock(&hdmi->lock);
	if (!hdmi->eld_valid)
		histb_hdmi_build_fallback_eld(hdmi);
	if (!hdmi->hpd_active)
		dev_dbg_ratelimited(dev, "ELD request without active HPD, returning fallback/cached ELD\n");

	copy_len = min_t(size_t, len, sizeof(hdmi->eld));
	memcpy(buf, hdmi->eld, copy_len);

	mutex_unlock(&hdmi->lock);
	return 0;
}

static const struct hdmi_codec_ops histb_hdmi_audio_codec_ops = {
	.hw_params = histb_hdmi_audio_hw_params,
	.audio_shutdown = histb_hdmi_audio_shutdown,
	.mute_stream = histb_hdmi_audio_mute_stream,
	.get_eld = histb_hdmi_audio_get_eld,
};

static void histb_hdmi_audio_unregister_action(void *data)
{
	struct histb_hdmi *hdmi = data;

	if (IS_ERR_OR_NULL(hdmi->audio_pdev))
		return;

	platform_device_unregister(hdmi->audio_pdev);
	hdmi->audio_pdev = NULL;
}

static int histb_hdmi_audio_register(struct histb_hdmi *hdmi)
{
	struct hdmi_codec_pdata codec_data = {
		.ops = &histb_hdmi_audio_codec_ops,
		.max_i2s_channels = 2,
		.i2s = 1,
		.no_i2s_capture = 1,
		.no_spdif_capture = 1,
		.data = hdmi,
	};
	int ret;

	hdmi->audio_pdev = platform_device_register_data(hdmi->dev,
							 HDMI_CODEC_DRV_NAME,
							 PLATFORM_DEVID_AUTO,
							 &codec_data,
							 sizeof(codec_data));
	if (IS_ERR(hdmi->audio_pdev))
		return PTR_ERR(hdmi->audio_pdev);

	ret = devm_add_action_or_reset(hdmi->dev, histb_hdmi_audio_unregister_action,
				       hdmi);
	if (ret)
		return ret;

	return 0;
}

static void histb_hdmi_phy_init(struct histb_hdmi *hdmi)
{
	/*
	 * hi3798mv100 sequence from SDK (si_phy.c):
	 * program analog defaults, then power-up PHY.
	 */
	histb_hdmi_write_phy(hdmi, HISTB_HDMI_PHY_OE_ADDR, 0x00);
	histb_hdmi_write_phy(hdmi, HISTB_HDMI_PHY_AUD_ADDR, 0x02);
	histb_hdmi_write_phy(hdmi, HISTB_HDMI_PHY_PLL1_ADDR, 0x02);
	histb_hdmi_write_phy(hdmi, HISTB_HDMI_PHY_PLL2_ADDR, 0x09);
	histb_hdmi_write_phy(hdmi, HISTB_HDMI_PHY_DRV_ADDR, 0x01);
	histb_hdmi_write_phy(hdmi, HISTB_HDMI_PHY_CLK_ADDR, 0x00);
	histb_hdmi_write_phy(hdmi, HISTB_HDMI_PHY_PWD_ADDR, 0x01);
}

static void histb_hdmi_set_av_blank(struct histb_hdmi *hdmi, bool blank)
{
	u32 val;

	val = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_DATA_CTRL_ADDR);
	if (blank)
		val |= HISTB_HDMI_BIT_VID_BLANK | HISTB_HDMI_BIT_AUD_MUTE;
	else
		val &= ~(HISTB_HDMI_BIT_VID_BLANK | HISTB_HDMI_BIT_AUD_MUTE);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_DATA_CTRL_ADDR, val);
}

static void histb_hdmi_set_phy_output(struct histb_hdmi *hdmi, bool enable)
{
	u32 val;

	val = histb_hdmi_read_phy(hdmi, HISTB_HDMI_PHY_OE_ADDR);
	if (enable)
		val |= HISTB_HDMI_RG_TX_RSTB;
	else
		val &= ~HISTB_HDMI_RG_TX_RSTB;
	histb_hdmi_write_phy(hdmi, HISTB_HDMI_PHY_OE_ADDR, val);
}

static void histb_hdmi_tx_program_mode(struct histb_hdmi *hdmi,
				       const struct histb_hdmi_mode *mode)
{
	u32 val;

	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_INT_CNTRL_ADDR,
			     HISTB_HDMI_INT_CONTROL);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_DDC_DELAY_CNT,
			     HISTB_HDMI_DDC_DELAY_DEFAULT);

	val = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_TX_SYS_CTRL1_ADDR);
	val &= ~(HISTB_HDMI_BIT_TX_PD |
		 HISTB_HDMI_BIT_BSEL24BITS |
		 HISTB_HDMI_BIT_TX_CLOCK_RISING_EDGE);
	val |= HISTB_HDMI_BIT_TX_PD |
	       HISTB_HDMI_BIT_BSEL24BITS |
	       HISTB_HDMI_BIT_TX_CLOCK_RISING_EDGE;
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_TX_SYS_CTRL1_ADDR, val);

	val = histb_hdmi_read_tx1(hdmi, HISTB_HDMI_AUDP_TXCTRL_ADDR);
	/*
	 * Force 24-bit/1x TMDS packet mode like SDK SI_SetDeepColor(24bpp).
	 * Leaving these bits stale may drive an unexpected TMDS multiplier and
	 * produce "out of range" on some sinks.
	 */
	val &= ~GENMASK(6, 3);
	val |= BIT(5) | HISTB_HDMI_BIT_TXHDMI_MODE;
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_AUDP_TXCTRL_ADDR, val);

	val = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_VID_IN_MODE_ADDR);
	val = (val & HISTB_HDMI_VID_IN_MODE_CLR) | HISTB_HDMI_VID_IN_MODE_24BIT;
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_VID_IN_MODE_ADDR, val);

	val = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_TX_VID_MODE_ADDR);
	val &= ~HISTB_HDMI_TX_VID_MODE_CLR;
	val |= HISTB_HDMI_BIT_TX_DITHER_EN;
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_TX_VID_MODE_ADDR, val);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_TX_VID_DITHER_ADDR,
			     HISTB_HDMI_TX_VID_DITHER_24BIT);

	/* Match SDK InitTestCtrlReg(): controller DVI encoder enabled. */
	histb_hdmi_write_tx1(hdmi, HISTB_HDMI_TEST_TX_ADDR, 0x00);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_VID_ACEN_ADDR, 0x00);

	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_DE_HSTART_ADDR, mode->hstart & 0xff);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_DE_CNTRL_ADDR,
			     (mode->hstart >> 8) & 0x3);
	histb_hdmi_write_word_tx0(hdmi, HISTB_HDMI_DE_HRES_ADDR, mode->hres);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_DE_VSTART_ADDR, mode->vstart);
	histb_hdmi_write_word_tx0(hdmi, HISTB_HDMI_DE_VRES_ADDR, mode->vres);

	val = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_DE_CNTRL_ADDR);
	val |= HISTB_HDMI_BIT_DE_ENABLED;
	val &= ~GENMASK(5, 4);
	val |= (mode->de_polarity & 0x3) << 4;
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_DE_CNTRL_ADDR, val);

	val = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_INTERLACE_ADJ_MODE_ADDR);
	val &= GENMASK(7, 3);
	val |= mode->int_adj_mode & 0x7;
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_INTERLACE_ADJ_MODE_ADDR, val);
	histb_hdmi_write_word_tx0(hdmi, HISTB_HDMI_HBIT_TO_HSYNC_ADDR,
				  mode->hbit_to_hsync);
	histb_hdmi_write_word_tx0(hdmi, HISTB_HDMI_FIELD2_HSYNC_OFFSET_ADDR,
				  mode->field2_hsync_offset);
	histb_hdmi_write_word_tx0(hdmi, HISTB_HDMI_HLENGTH_ADDR, mode->hlength);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_VBIT_TO_VSYNC_ADDR,
			     mode->vbit_to_vsync);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_VLENGTH_ADDR, mode->vlength);
	histb_hdmi_write_word_tx0(hdmi, HISTB_HDMI_HTOTAL_ADDR, mode->htotal);
	histb_hdmi_write_word_tx0(hdmi, HISTB_HDMI_VTOTAL_ADDR, mode->vtotal);

	val = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_TX_VID_CTRL_ADDR);
	val &= ~(HISTB_HDMI_TX_VID_CTRL_ICLK | HISTB_HDMI_BIT_SET_CSCSEL);
	val |= HISTB_HDMI_BIT_SET_CSCSEL;
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_TX_VID_CTRL_ADDR, val);

	val = histb_hdmi_read_phy(hdmi, HISTB_HDMI_PHY_PLL1_ADDR);
	val &= ~HISTB_HDMI_PHY_PLL1_SWING_MASK;
	val |= mode->phy_pll1_swing & 0x3;
	histb_hdmi_write_phy(hdmi, HISTB_HDMI_PHY_PLL1_ADDR, val);

	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_HDMI_INT_ADDR, HISTB_HDMI_CLR_MASK);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_HDMI_INT_MASK_ADDR, HISTB_HDMI_CLR_MASK);
}

static int histb_hdmi_mddc_read_chunk(struct histb_hdmi *hdmi, u16 addr, u8 len,
				      u8 *buf)
{
	u8 header[7];
	u8 seg = addr >> 8;
	u8 reg = addr & 0xff;
	u32 status;
	int i;

	if (!len || len > HISTB_EDID_CHUNK_SIZE)
		return -EINVAL;

	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_MDDC_SEGMENT_ADDR, seg);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_MDDC_COMMAND_ADDR,
			     HISTB_HDMI_MDDC_CMD_CLEAR_FIFO);

	header[0] = HISTB_HDMI_DDC_ADDR;
	header[1] = seg;
	header[2] = reg;
	header[3] = len;
	header[4] = 0;
	header[5] = 0;
	header[6] = seg ? HISTB_HDMI_MDDC_CMD_ENH_RD : HISTB_HDMI_MDDC_CMD_SEQ_RD;

	for (i = 0; i < ARRAY_SIZE(header); i++)
		histb_hdmi_write_tx0(hdmi, HISTB_HDMI_MDDC_BASE + 1 + i, header[i]);

	for (i = 0; i < 64; i++) {
		status = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_MDDC_STATUS_ADDR);
		if (!(status & HISTB_HDMI_MDDC_ST_IN_PROGR) &&
		    histb_hdmi_read_tx0(hdmi, HISTB_HDMI_MDDC_FIFO_CNT_ADDR) >= len)
			break;
		usleep_range(1000, 2000);
	}
	if (i == 64) {
		histb_hdmi_write_tx0(hdmi, HISTB_HDMI_MDDC_COMMAND_ADDR,
				     HISTB_HDMI_MDDC_CMD_ABORT);
		return -ETIMEDOUT;
	}

	for (i = 0; i < len; i++)
		buf[i] = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_MDDC_FIFO_ADDR) & 0xff;

	status = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_MDDC_STATUS_ADDR);
	if (status & (HISTB_HDMI_MDDC_ST_I2C_LOW |
		      HISTB_HDMI_MDDC_ST_NO_ACK |
		      HISTB_HDMI_MDDC_ST_FIFO_FULL)) {
		histb_hdmi_write_tx0(hdmi, HISTB_HDMI_MDDC_COMMAND_ADDR,
				     HISTB_HDMI_MDDC_CMD_ABORT);
		histb_hdmi_write_tx0(hdmi, HISTB_HDMI_MDDC_COMMAND_ADDR,
				     HISTB_HDMI_MDDC_CMD_CLOCK);
		return -EIO;
	}

	return 0;
}

static int histb_hdmi_read_edid_block(struct histb_hdmi *hdmi, u8 block, u8 *buf)
{
	u16 base = block * HISTB_EDID_BLOCK_SIZE;
	int ret;
	int off;

	for (off = 0; off < HISTB_EDID_BLOCK_SIZE; off += HISTB_EDID_CHUNK_SIZE) {
		ret = histb_hdmi_mddc_read_chunk(hdmi, base + off,
						 HISTB_EDID_CHUNK_SIZE,
						 buf + off);
		if (ret)
			return ret;
	}

	return 0;
}

static bool histb_hdmi_edid_checksum_ok(const u8 *block)
{
	u32 sum = 0;
	int i;

	for (i = 0; i < HISTB_EDID_BLOCK_SIZE; i++)
		sum += block[i];

	return !(sum & 0xff);
}

static bool histb_hdmi_edid_header_ok(const u8 *block)
{
	static const u8 header[] = { 0x00, 0xff, 0xff, 0xff,
				     0xff, 0xff, 0xff, 0x00 };

	return !memcmp(block, header, sizeof(header));
}

static void histb_hdmi_edid_check_dtd(const u8 *dtd, bool *has_720, bool *has_1080)
{
	u16 hact;
	u16 vact;

	if (!dtd[0] && !dtd[1])
		return;

	hact = dtd[2] | ((dtd[4] & 0xf0) << 4);
	vact = dtd[5] | ((dtd[7] & 0xf0) << 4);

	/* Interlaced modes are not supported. */
	if (dtd[17] & BIT(7))
		return;

	if (hact == 1920 && vact == 1080)
		*has_1080 = true;
	else if (hact == 1280 && vact == 720)
		*has_720 = true;
}

static void histb_hdmi_edid_parse_cea(const u8 *block, bool *has_720, bool *has_1080)
{
	u8 end;
	u8 idx;

	if (block[0] != HISTB_EDID_EXT_CEA)
		return;

	end = block[2];
	if (end < 4 || end > 127)
		end = 127;

	idx = 4;
	while (idx < end) {
		u8 tag = block[idx] >> 5;
		u8 len = block[idx] & 0x1f;
		u8 j;

		if (idx + 1 + len > end)
			break;

		if (tag == 2) { /* Video Data Block */
			for (j = 0; j < len; j++) {
				u8 vic = block[idx + 1 + j] & 0x7f;

				if (vic == 16)
					*has_1080 = true;
				else if (vic == 4)
					*has_720 = true;
			}
		}

		idx += 1 + len;
	}

	for (idx = end; idx + 18 <= 126; idx += 18)
		histb_hdmi_edid_check_dtd(block + idx, has_720, has_1080);
}

static int histb_hdmi_pick_mode_from_edid(struct histb_hdmi *hdmi,
					  const struct drm_display_mode *fallback,
					  struct drm_display_mode *mode)
{
	u8 block0[HISTB_EDID_BLOCK_SIZE];
	u8 block1[HISTB_EDID_BLOCK_SIZE];
	const struct drm_display_mode *selected = fallback;
	bool has_720 = false;
	bool has_1080 = false;
	bool have_cea = false;
	u8 ext_blocks;
	int ret;
	int dtd;

	ret = histb_hdmi_read_edid_block(hdmi, 0, block0);
	if (ret)
		goto clear_eld;

	if (!histb_hdmi_edid_header_ok(block0) || !histb_hdmi_edid_checksum_ok(block0))
		goto clear_eld_einval;

	for (dtd = 54; dtd <= 108; dtd += 18)
		histb_hdmi_edid_check_dtd(block0 + dtd, &has_720, &has_1080);

	ext_blocks = block0[126];
	if (ext_blocks > 0) {
		ret = histb_hdmi_read_edid_block(hdmi, 1, block1);
		if (!ret && histb_hdmi_edid_checksum_ok(block1)) {
			if (block1[0] == HISTB_EDID_EXT_CEA)
				have_cea = true;
			histb_hdmi_edid_parse_cea(block1, &has_720, &has_1080);
		}
	}

	histb_hdmi_build_eld(hdmi, block0, have_cea ? block1 : NULL);

	if (has_1080)
		selected = &histb_hdmi_1080p60_mode;
	else if (has_720)
		selected = &histb_hdmi_720p60_mode;

	drm_mode_copy(mode, selected);
	drm_mode_set_name(mode);

	return 0;

clear_eld_einval:
	ret = -EINVAL;
clear_eld:
	hdmi->stats_edid_fail++;
	histb_hdmi_trace_error_locked(hdmi, "pick-mode-edid", ret, true);
	histb_hdmi_clear_eld(hdmi);
	return ret;
}

static int histb_hdmi_apply_mode(struct histb_hdmi *hdmi,
				 const struct drm_display_mode *mode)
{
	struct histb_hdmi_mode hw_mode;
	int ret;

	ret = histb_hdmi_mode_to_hw(mode, &hw_mode);
	if (ret) {
		hdmi->stats_mode_apply_fail++;
		histb_hdmi_trace_error_locked(hdmi, "apply-mode", ret,
					      pm_runtime_active(hdmi->dev));
		return ret;
	}

	histb_hdmi_set_av_blank(hdmi, true);

	histb_hdmi_tx_program_mode(hdmi, &hw_mode);
	histb_hdmi_video_infoframes_update_locked(hdmi, mode);
	histb_hdmi_set_phy_output(hdmi, true);

	histb_hdmi_set_av_blank(hdmi, false);

	drm_mode_copy(&hdmi->mode, mode);
	drm_mode_set_name(&hdmi->mode);
	histb_hdmi_audio_restore_stream_locked(hdmi);
	return 0;
}

static bool histb_hdmi_hpd_storm_detect_locked(struct histb_hdmi *hdmi)
{
	unsigned long now = jiffies;
	unsigned long window = msecs_to_jiffies(HISTB_HDMI_HPD_STORM_WINDOW_MS);

	if (!hdmi->hpd_storm_window_start ||
	    time_after_eq(now, hdmi->hpd_storm_window_start + window)) {
		hdmi->hpd_storm_window_start = now;
		hdmi->hpd_storm_events = 1;
		return false;
	}

	hdmi->hpd_storm_events++;
	if (hdmi->hpd_storm_events <= HISTB_HDMI_HPD_STORM_THRESHOLD)
		return false;

	hdmi->hpd_storm_ignore_until =
		now + msecs_to_jiffies(HISTB_HDMI_HPD_STORM_QUIET_MS);
	hdmi->hpd_storm_events = 0;
	return true;
}

static void histb_hdmi_link_quiesce_locked(struct histb_hdmi *hdmi, bool clear_eld)
{
	histb_hdmi_hdcp_disable_locked(hdmi);
	histb_hdmi_audio_mute_locked(hdmi, true);
	histb_hdmi_audio_tx1_input_enable_locked(hdmi, false);
	histb_hdmi_set_av_blank(hdmi, true);
	histb_hdmi_set_phy_output(hdmi, false);
	if (clear_eld)
		histb_hdmi_clear_eld(hdmi);
	hdmi->hpd_active = false;
}

static int histb_hdmi_link_recover_locked(struct histb_hdmi *hdmi,
					  const struct drm_display_mode *mode,
					  const char *reason)
{
	int ret;
	u32 hdisplay = mode ? mode->hdisplay : 0;
	u32 vdisplay = mode ? mode->vdisplay : 0;
	u32 vrefresh = mode ? drm_mode_vrefresh(mode) : 0;

	hdmi->stats_recover_attempts++;

	if (!mode) {
		ret = -EINVAL;
		goto out_fail;
	}

	if (!histb_hdmi_hpd_status(hdmi)) {
		histb_hdmi_link_quiesce_locked(hdmi, true);
		ret = -ENODEV;
		goto out_fail;
	}

	dev_warn_ratelimited(hdmi->dev,
			     "recovering HDMI link (%s), mode=%ux%u@%d\n",
			     reason ?: "unspecified",
			     mode->hdisplay, mode->vdisplay,
			     drm_mode_vrefresh(mode));

	histb_hdmi_link_quiesce_locked(hdmi, false);
	usleep_range(1000, 2000);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_TX_SWRST_ADDR, 0x00);
	histb_hdmi_phy_init(hdmi);
	usleep_range(1000, 2000);
	histb_hdmi_audio_select_i2s_source_locked(hdmi);

	ret = histb_hdmi_apply_mode(hdmi, mode);
	if (ret)
		histb_hdmi_link_quiesce_locked(hdmi, true);

	if (!ret) {
		hdmi->stats_recover_success++;
		trace_histb_hdmi_recovery(reason, 0, hdisplay, vdisplay,
					  vrefresh);
		return 0;
	}

out_fail:
	hdmi->stats_recover_fail++;
	trace_histb_hdmi_recovery(reason, ret, hdisplay, vdisplay, vrefresh);
	histb_hdmi_trace_error_locked(hdmi, "link-recover", ret,
				      pm_runtime_active(hdmi->dev));
	return ret;
}

static int histb_hdmi_handle_hpd_event(struct histb_hdmi *hdmi)
{
	const struct drm_display_mode *fixed_mode = histb_hdmi_fixed_mode_from_param();
	struct drm_display_mode target_mode;
	const char *mode_source = "cached";
	bool hpd;
	int ret;

	hpd = histb_hdmi_hpd_status(hdmi);
	if (!hpd) {
		if (!hdmi->hpd_active)
			return 0;

		hdmi->stats_hpd_out++;
		histb_hdmi_link_quiesce_locked(hdmi, true);
		dev_info(hdmi->dev, "hotplug out: pipeline disabled\n");
		return 0;
	}

	/*
	 * Some sinks/controllers generate repeated HPD/RSEN IRQs while link is
	 * already up. Ignore duplicates to avoid constant mode reprogramming.
	 */
	if (hdmi->hpd_active)
		return 0;

	drm_mode_copy(&target_mode, &hdmi->mode);
	if (!histb_hdmi_mode_is_supported(&target_mode)) {
		drm_mode_copy(&target_mode, fixed_mode);
		mode_source = "fixed";
	}

	if (hdmi->mode_from_bridge) {
		ret = histb_hdmi_refresh_eld(hdmi);
		if (ret)
			dev_warn(hdmi->dev,
				 "ELD refresh failed (%d), keeping previous ELD if available\n",
				 ret);
		mode_source = "bridge-cached";
	} else if (histb_use_edid) {
		ret = histb_hdmi_pick_mode_from_edid(hdmi, fixed_mode, &target_mode);
		if (ret) {
			drm_mode_copy(&target_mode, fixed_mode);
			mode_source = "fixed-fallback";
			dev_warn(hdmi->dev,
				 "EDID read failed (%d), keeping fixed mode\n", ret);
		} else {
			mode_source = "edid/fallback";
		}
	} else {
		ret = histb_hdmi_refresh_eld(hdmi);
		if (ret)
			dev_warn(hdmi->dev,
				 "ELD refresh failed (%d), keeping previous ELD if available\n",
				 ret);
		mode_source = "fixed";
		drm_mode_copy(&target_mode, fixed_mode);
	}

	drm_mode_set_name(&target_mode);

	ret = histb_hdmi_apply_mode(hdmi, &target_mode);
	if (ret) {
		dev_warn(hdmi->dev,
			 "mode apply failed (%d), trying fixed-mode recovery\n",
			 ret);
		ret = histb_hdmi_link_recover_locked(hdmi, fixed_mode,
						     "hpd-mode-apply");
		if (ret)
			return ret;

		drm_mode_copy(&target_mode, fixed_mode);
		drm_mode_set_name(&target_mode);
		mode_source = "recovery-fixed";
		hdmi->mode_from_bridge = false;
	}

	if (!hdmi->audio_stream_active)
		histb_hdmi_audio_warmup_locked(hdmi);

	hdmi->hpd_active = true;
	hdmi->stats_hpd_in++;
	dev_info(hdmi->dev, "hotplug in: mode=%s (%s)\n",
		 hdmi->mode.name, mode_source);

	return 0;
}

static int histb_hdmi_hw_power_on(struct histb_hdmi *hdmi)
{
	int ret;

	ret = clk_bulk_prepare_enable(HISTB_HDMI_NUM_CLKS, hdmi->clks);
	if (ret)
		return ret;

	ret = histb_hdmi_configure_crg(hdmi);
	if (ret)
		goto err_disable_clks;

	reset_control_assert(hdmi->rst_bus);
	reset_control_assert(hdmi->rst_ctrl);
	reset_control_assert(hdmi->rst_phy);
	usleep_range(1000, 2000);

	ret = reset_control_deassert(hdmi->rst_phy);
	if (ret)
		goto err_assert_reset;

	usleep_range(1000, 2000);
	histb_hdmi_phy_init(hdmi);
	usleep_range(1000, 2000);

	ret = reset_control_deassert(hdmi->rst_bus);
	if (ret)
		goto err_assert_reset;

	ret = reset_control_deassert(hdmi->rst_ctrl);
	if (ret)
		goto err_assert_reset;

	usleep_range(10000, 11000);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_TX_SWRST_ADDR, 0x00);

	mutex_lock(&hdmi->lock);
	histb_hdmi_audio_select_i2s_source_locked(hdmi);
#if IS_ENABLED(CONFIG_DRM_DISPLAY_HDMI_CEC_HELPER)
	histb_hdmi_cec_enable_locked(hdmi, false);
#endif
	hdmi->hpd_active = false;
	mutex_unlock(&hdmi->lock);

	return 0;

err_assert_reset:
	reset_control_assert(hdmi->rst_ctrl);
	reset_control_assert(hdmi->rst_bus);
	reset_control_assert(hdmi->rst_phy);
err_disable_clks:
	clk_bulk_disable_unprepare(HISTB_HDMI_NUM_CLKS, hdmi->clks);
	return ret;
}

static void histb_hdmi_hw_power_off(struct histb_hdmi *hdmi)
{
	mutex_lock(&hdmi->lock);
	histb_hdmi_link_quiesce_locked(hdmi, true);
	hdmi->pm_ref_active = false;
	mutex_unlock(&hdmi->lock);

	reset_control_assert(hdmi->rst_ctrl);
	reset_control_assert(hdmi->rst_bus);
	reset_control_assert(hdmi->rst_phy);
	clk_bulk_disable_unprepare(HISTB_HDMI_NUM_CLKS, hdmi->clks);
}

static int histb_hdmi_runtime_resume(struct device *dev)
{
	struct histb_hdmi *hdmi = dev_get_drvdata(dev);
	int ret;

	ret = histb_hdmi_hw_power_on(hdmi);
	if (ret) {
		mutex_lock(&hdmi->lock);
		hdmi->stats_runtime_resume_fail++;
		histb_hdmi_trace_error_locked(hdmi, "runtime-resume", ret,
					      false);
		mutex_unlock(&hdmi->lock);
	}

	return ret;
}

static int histb_hdmi_runtime_suspend(struct device *dev)
{
	struct histb_hdmi *hdmi = dev_get_drvdata(dev);

	histb_hdmi_hw_power_off(hdmi);
	return 0;
}

static void histb_hdmi_pm_cleanup_action(void *data)
{
	struct histb_hdmi *hdmi = data;
	struct device *dev = hdmi->dev;

	if (pm_runtime_enabled(dev))
		pm_runtime_disable(dev);

	if (!pm_runtime_status_suspended(dev))
		histb_hdmi_runtime_suspend(dev);
}

static int histb_hdmi_check_vdp_link(struct device *dev)
{
	/* Scanout is sequenced by the VDP driver, not from here. */
	return 0;
}

static irqreturn_t histb_hdmi_irq_handler(int irq, void *data)
{
	struct histb_hdmi *hdmi = data;

	(void)irq;

	if (READ_ONCE(hdmi->system_suspended))
		return IRQ_NONE;

	if (!histb_hdmi_read_tx0(hdmi, HISTB_HDMI_HDMI_INT_STATE_ADDR)
#if IS_ENABLED(CONFIG_DRM_DISPLAY_HDMI_CEC_HELPER)
	    && !histb_hdmi_cec_irq_pending(hdmi)
#endif
	)
		return IRQ_NONE;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t histb_hdmi_irq_thread(int irq, void *data)
{
	struct histb_hdmi *hdmi = data;
	enum drm_connector_status hpd_status = connector_status_unknown;
	bool hpd_level;
	bool storm;
	bool hpd_notify = false;
	bool was_active;
	u8 ints[4];
	int ret;

	(void)irq;

	if (READ_ONCE(hdmi->system_suspended))
		return IRQ_HANDLED;

	mutex_lock(&hdmi->lock);

	ints[0] = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_HDMI_INT_ADDR);
	ints[1] = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_HDMI_INT_ADDR + 1);
	ints[2] = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_HDMI_INT_ADDR + 2);
	ints[3] = histb_hdmi_read_tx0(hdmi, HISTB_HDMI_HDMI_INT_ADDR + 3);

	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_HDMI_INT_ADDR, ints[0]);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_HDMI_INT_ADDR + 1, ints[1]);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_HDMI_INT_ADDR + 2, ints[2]);
	histb_hdmi_write_tx0(hdmi, HISTB_HDMI_HDMI_INT_ADDR + 3, ints[3]);
	hdmi->stats_irq_total++;

#if IS_ENABLED(CONFIG_DRM_DISPLAY_HDMI_CEC_HELPER)
	histb_hdmi_cec_irq_process_locked(hdmi);
#endif

	if (!(ints[0] & (HISTB_HDMI_BIT_INT_HOT_PLUG | HISTB_HDMI_BIT_INT_RSEN))) {
		mutex_unlock(&hdmi->lock);
		return IRQ_HANDLED;
	}

	hdmi->stats_irq_hpd_rsen++;
	hpd_level = histb_hdmi_hpd_status(hdmi);

	if (time_before(jiffies, hdmi->hpd_storm_ignore_until)) {
		trace_histb_hdmi_hpd_irq(ints[0], hpd_level, hdmi->hpd_active, true);
		mutex_unlock(&hdmi->lock);
		return IRQ_HANDLED;
	}

	was_active = hdmi->hpd_active;
	storm = histb_hdmi_hpd_storm_detect_locked(hdmi);
	trace_histb_hdmi_hpd_irq(ints[0], hpd_level, hdmi->hpd_active, storm);
	if (storm) {
		hdmi->stats_hpd_storm++;
		dev_warn_ratelimited(hdmi->dev,
				     "HPD/RSEN storm detected, entering quiet period\n");
		if (hdmi->hpd_active)
			histb_hdmi_link_quiesce_locked(hdmi, true);

		if (histb_hdmi_hpd_status(hdmi)) {
			ret = histb_hdmi_link_recover_locked(hdmi,
					histb_hdmi_fixed_mode_from_param(),
					"hpd-storm");
			if (ret)
				dev_warn_ratelimited(hdmi->dev,
						     "storm recovery failed: %d\n",
						     ret);
			else
				hdmi->mode_from_bridge = false;
		}

		if (was_active != hdmi->hpd_active) {
			hpd_notify = true;
			hpd_status = hdmi->hpd_active ?
				     connector_status_connected :
				     connector_status_disconnected;
		}

		mutex_unlock(&hdmi->lock);
		if (hpd_notify)
			drm_bridge_hpd_notify(&hdmi->bridge, hpd_status);
		return IRQ_HANDLED;
	}

	ret = histb_hdmi_handle_hpd_event(hdmi);
	if (ret) {
		dev_warn(hdmi->dev, "HPD handling failed: %d, trying recovery\n",
			 ret);
		ret = histb_hdmi_link_recover_locked(hdmi,
				histb_hdmi_fixed_mode_from_param(),
				"irq-hpd");
		if (ret)
			dev_warn_ratelimited(hdmi->dev,
					     "HPD recovery failed: %d\n", ret);
		else
			hdmi->mode_from_bridge = false;
	}

	if (was_active != hdmi->hpd_active) {
		hpd_notify = true;
		hpd_status = hdmi->hpd_active ?
			     connector_status_connected :
			     connector_status_disconnected;
	}

	mutex_unlock(&hdmi->lock);

	if (hpd_notify)
		drm_bridge_hpd_notify(&hdmi->bridge, hpd_status);

	return IRQ_HANDLED;
}

static int histb_hdmi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_hdmi *hdmi;
	int irq;
	int i;
	int ret;

	hdmi = devm_drm_bridge_alloc(dev, struct histb_hdmi, bridge,
				     &histb_hdmi_bridge_funcs);
	if (IS_ERR(hdmi))
		return PTR_ERR(hdmi);

	hdmi->dev = dev;
	mutex_init(&hdmi->lock);
	ratelimit_state_init(&hdmi->diag_dump_rs,
			     HISTB_HDMI_DIAG_DUMP_INTERVAL,
			     HISTB_HDMI_DIAG_DUMP_BURST);
	drm_mode_copy(&hdmi->mode, histb_hdmi_fixed_mode_from_param());
	drm_mode_set_name(&hdmi->mode);
	hdmi_audio_infoframe_init(&hdmi->audio_cea);
	hdmi->audio_cea.channels = 1; /* 2ch in CEA encoding */
	hdmi->audio_sample_rate = 48000;
	hdmi->audio_sample_width = 16;
	hdmi->audio_channels = 2;
	hdmi->audio_stream_active = false;
	hdmi->audio_requested_mute = true;
	hdmi->mode_from_bridge = false;
	hdmi->hdcp_content_type = DRM_MODE_HDCP_CONTENT_TYPE0;
#if IS_ENABLED(CONFIG_DRM_DISPLAY_HDMI_CEC_HELPER)
	hdmi->cec_logical_addr = CEC_LOG_ADDR_INVALID;
	hdmi->cec_attempts = 1;
#endif
	histb_hdmi_clear_eld(hdmi);
	hdmi->bridge.of_node = dev->of_node;
	hdmi->bridge.type = DRM_MODE_CONNECTOR_HDMIA;
	hdmi->bridge.interlace_allowed = false;
	hdmi->bridge.support_hdcp = true;
	hdmi->bridge.vendor = "HiSilcn";
	hdmi->bridge.product = "HI3798MV100";
	hdmi->bridge.supported_formats = BIT(HDMI_COLORSPACE_RGB);
	/* Keep HDR metadata property disabled (drmm_connector_hdmi_init gate). */
	hdmi->bridge.max_bpc = 8;
	hdmi->bridge.ops = DRM_BRIDGE_OP_EDID |
			   DRM_BRIDGE_OP_MODES |
			   DRM_BRIDGE_OP_DETECT |
			   DRM_BRIDGE_OP_HPD |
			   DRM_BRIDGE_OP_HDMI;
#if IS_ENABLED(CONFIG_DRM_DISPLAY_HDMI_CEC_HELPER)
	hdmi->bridge.ops |= DRM_BRIDGE_OP_HDMI_CEC_ADAPTER;
	hdmi->bridge.hdmi_cec_dev = dev;
	hdmi->bridge.hdmi_cec_adapter_name = dev_name(dev);
	hdmi->bridge.hdmi_cec_available_las = HISTB_HDMI_CEC_AVAILABLE_LAS;
#endif

	ret = histb_hdmi_check_vdp_link(dev);
	if (ret)
		return ret;

	for (i = 0; i < HISTB_HDMI_NUM_CLKS; i++)
		hdmi->clks[i].id = histb_hdmi_clk_names[i];

	hdmi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hdmi->regs))
		return PTR_ERR(hdmi->regs);

	hdmi->crg = syscon_regmap_lookup_by_phandle(dev->of_node,
						    "hisilicon,crg-syscon");
	if (IS_ERR(hdmi->crg))
		return dev_err_probe(dev, PTR_ERR(hdmi->crg),
				     "failed to get CRG regmap\n");

	hdmi->perictrl = syscon_regmap_lookup_by_phandle(
		dev->of_node, "hisilicon,peripheral-syscon");
	if (IS_ERR(hdmi->perictrl)) {
		ret = PTR_ERR(hdmi->perictrl);
		if (ret != -EINVAL && ret != -ENODEV && ret != -ENOENT)
			return dev_err_probe(dev, ret,
					     "failed to get peripheral syscon\n");
		hdmi->perictrl = NULL;
	}

	if (hdmi->perictrl)
		dev_dbg(dev, "peripheral syscon ready (HDMI source status diagnostics)\n");
	else
		dev_dbg(dev, "peripheral syscon unavailable; skipping source-status diagnostics\n");

	hdmi->rst_bus = devm_reset_control_get_exclusive(dev, "bus");
	if (IS_ERR(hdmi->rst_bus))
		return dev_err_probe(dev, PTR_ERR(hdmi->rst_bus),
				     "failed to get bus reset\n");

	hdmi->rst_ctrl = devm_reset_control_get_exclusive(dev, "ctrl");
	if (IS_ERR(hdmi->rst_ctrl))
		return dev_err_probe(dev, PTR_ERR(hdmi->rst_ctrl),
				     "failed to get ctrl reset\n");

	hdmi->rst_phy = devm_reset_control_get_exclusive(dev, "phy");
	if (IS_ERR(hdmi->rst_phy))
		return dev_err_probe(dev, PTR_ERR(hdmi->rst_phy),
				     "failed to get phy reset\n");

	ret = devm_clk_bulk_get(dev, HISTB_HDMI_NUM_CLKS, hdmi->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get HDMI clocks\n");

	platform_set_drvdata(pdev, hdmi);

	ret = histb_hdmi_runtime_resume(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize HDMI runtime state\n");

	ret = devm_add_action_or_reset(dev, histb_hdmi_pm_cleanup_action, hdmi);
	if (ret)
		return ret;

	/* Diagnostics: capture the source select status once running. */
	mutex_lock(&hdmi->lock);
	dev_dbg(dev, "initial PERI_HDMITX_CTRL=0x%08x alt(0x%03x)=0x%08x\n",
		histb_hdmi_audio_source_status(hdmi),
		HISTB_HDMI_PERICTRL_HDMITX_CTRL_ALT,
		histb_hdmi_audio_source_status_alt(hdmi));
	mutex_unlock(&hdmi->lock);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get HDMI irq\n");
	hdmi->irq = irq;

	ret = devm_request_threaded_irq(dev, irq, histb_hdmi_irq_handler,
					histb_hdmi_irq_thread, IRQF_ONESHOT,
					dev_name(dev), hdmi);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request HDMI irq\n");

	mutex_lock(&hdmi->lock);
	ret = histb_hdmi_handle_hpd_event(hdmi);
	mutex_unlock(&hdmi->lock);
	if (ret)
		return dev_err_probe(dev, ret, "failed to arm initial HDMI mode\n");

	ret = histb_hdmi_audio_register(hdmi);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register hdmi-audio-codec\n");

	ret = devm_drm_bridge_add(dev, &hdmi->bridge);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register DRM bridge\n");

	pm_runtime_set_autosuspend_delay(dev, 200);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_idle(dev);

	dev_info(dev, "pipeline armed, mode control: fixed=%d, edid=%s\n",
		 histb_fixed_mode, histb_use_edid ? "on" : "off");

	return 0;
}

static const struct of_device_id histb_hdmi_of_match[] = {
	{ .compatible = "hisilicon,hi3798mv100-hdmi" },
	{ }
};
MODULE_DEVICE_TABLE(of, histb_hdmi_of_match);

static int __maybe_unused histb_hdmi_pm_suspend(struct device *dev)
{
	struct histb_hdmi *hdmi = dev_get_drvdata(dev);
	int ret;

	WRITE_ONCE(hdmi->system_suspended, true);
	disable_irq(hdmi->irq);

	ret = pm_runtime_force_suspend(dev);
	if (ret) {
		enable_irq(hdmi->irq);
		WRITE_ONCE(hdmi->system_suspended, false);
	}

	return ret;
}

static int __maybe_unused histb_hdmi_pm_resume(struct device *dev)
{
	struct histb_hdmi *hdmi = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	WRITE_ONCE(hdmi->system_suspended, false);
	enable_irq(hdmi->irq);

	return 0;
}

static const struct dev_pm_ops histb_hdmi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(histb_hdmi_pm_suspend,
				histb_hdmi_pm_resume)
	SET_RUNTIME_PM_OPS(histb_hdmi_runtime_suspend,
			   histb_hdmi_runtime_resume, NULL)
};

static struct platform_driver histb_hdmi_platform_driver = {
	.probe = histb_hdmi_probe,
	.driver = {
		.name = "histb-hdmi",
		.of_match_table = histb_hdmi_of_match,
		.pm = pm_ptr(&histb_hdmi_pm_ops),
	},
};
module_platform_driver(histb_hdmi_platform_driver);

MODULE_AUTHOR("Hisilicon community");
MODULE_DESCRIPTION("HiSilicon STB HDMI driver");
MODULE_LICENSE("GPL");
