// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3798MV100 AIAO I2S CPU DAI driver
 *
 * Stereo playback only, through the AIAO TX ring buffer.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/limits.h>
#include <linux/spinlock.h>
#include <linux/wordpart.h>
#include <sound/memalloc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define HI3798MV100_I2S_TX_IF_ATTRI			0x00
#define HI3798MV100_I2S_TX_DSP_CTRL			0x04
#define HI3798MV100_I2S_TX_WS_CNT			0x20
#define HI3798MV100_I2S_TX_BCLK_CNT			0x24
#define HI3798MV100_I2S_TX_BUFF_SADDR			0x80
#define HI3798MV100_I2S_TX_BUFF_SIZE			0x84
#define HI3798MV100_I2S_TX_BUFF_WPTR			0x88
#define HI3798MV100_I2S_TX_BUFF_RPTR			0x8c
#define HI3798MV100_I2S_TX_BUFF_ALEMPTY_TH		0x90
#define HI3798MV100_I2S_TX_TRANS_SIZE			0x94
#define HI3798MV100_I2S_TX_INT_ENA			0xa0
#define HI3798MV100_I2S_TX_INT_STATUS			0xa8
#define HI3798MV100_I2S_TX_INT_CLR			0xac

#define HI3798MV100_AIAO_COM_INT_ENA			0x00
#define HI3798MV100_AIAO_COM_SWITCH_TX_BCLK		0x2c
#define HI3798MV100_AIAO_COM_I2S_TX_CRG_BASE		0x140
#define HI3798MV100_AIAO_COM_I2S_TX_CRG_STRIDE		0x8
#define HI3798MV100_AIAO_COM_MAP_SIZE			0x200
#define HI3798MV100_AIAO_MAP_SIZE			0x10000
#define HI3798MV100_AIAO_TX_PORT_OFFSET			0x2000
#define HI3798MV100_AIAO_TX_PORT_STRIDE			0x100
#define HI3798MV100_AIAO_TX_PORT_MAX			8
#define HI3798MV100_AIAO_TX_HDMI_PORT_ID		3

#define HI3798MV100_I2S_TX_MODE_MASK			GENMASK(1, 0)
#define HI3798MV100_I2S_TX_PRECISION_MASK		GENMASK(3, 2)
#define HI3798MV100_I2S_TX_CH_NUM_MASK			GENMASK(5, 4)
#define HI3798MV100_I2S_TX_SD_OFFSET_MASK		GENMASK(15, 8)
#define HI3798MV100_I2S_TX_TRACK_MODE_MASK		GENMASK(18, 16)
#define HI3798MV100_I2S_TX_SPD_I2S_SEL			BIT(19)
#define HI3798MV100_I2S_TX_SD_SOURCE_SEL_MASK		GENMASK(23, 20)
#define HI3798MV100_I2S_TX_SD0_SEL_MASK			GENMASK(25, 24)
#define HI3798MV100_I2S_TX_SD1_SEL_MASK			GENMASK(27, 26)
#define HI3798MV100_I2S_TX_SD2_SEL_MASK			GENMASK(29, 28)
#define HI3798MV100_I2S_TX_SD3_SEL_MASK			GENMASK(31, 30)

#define HI3798MV100_I2S_TX_MODE_I2S			0
#define HI3798MV100_I2S_TX_MODE_PCM			1

#define HI3798MV100_I2S_TX_PRECISION_16BIT		1
#define HI3798MV100_I2S_TX_PRECISION_24BIT		2

#define HI3798MV100_I2S_TX_CH_NUM_2			1
#define HI3798MV100_I2S_TX_TRACK_MODE_STEREO		0
#define HI3798MV100_I2S_TX_SD0_SEL_SD0			0
#define HI3798MV100_I2S_TX_SD1_SEL_SD1			1
#define HI3798MV100_I2S_TX_SD2_SEL_SD2			2
#define HI3798MV100_I2S_TX_SD3_SEL_SD3			3

#define HI3798MV100_I2S_TX_ENABLE			BIT(28)
#define HI3798MV100_I2S_TX_MUTE_EN			BIT(0)
#define HI3798MV100_I2S_TX_MUTE_FADE_EN			BIT(1)
#define HI3798MV100_I2S_TX_VOLUME_MASK			GENMASK(14, 8)
#define HI3798MV100_I2S_TX_FADE_IN_RATE_MASK		GENMASK(19, 16)
#define HI3798MV100_I2S_TX_FADE_OUT_RATE_MASK		GENMASK(23, 20)
#define HI3798MV100_I2S_TX_BYPASS_EN			BIT(27)
#define HI3798MV100_I2S_TX_DISABLE_DONE			BIT(29)
#define HI3798MV100_I2S_TX_BUF_PTR_MASK			GENMASK(23, 0)
#define HI3798MV100_I2S_TX_INT_TRANSFINISH		BIT(0)
#define HI3798MV100_I2S_TX_INT_CLR_ALL			GENMASK(7, 0)

#define HI3798MV100_I2S_BUFFER_ALIGN			128
#define HI3798MV100_I2S_BUFFER_BYTES_MAX		(512 * 1024)

#define HI3798MV100_I2S_DSP_VOLUME_0DB			0x79
#define HI3798MV100_I2S_DSP_VOLUME_MAX			0x7f
#define HI3798MV100_I2S_DSP_FADE_IN_DEF			4
#define HI3798MV100_I2S_DSP_FADE_OUT_DEF		3
#define HI3798MV100_I2S_SD_OFFSET_DEF			1
#define HI3798MV100_I2S_PCM_RATES			(SNDRV_PCM_RATE_8000 | \
							 SNDRV_PCM_RATE_11025 | \
							 SNDRV_PCM_RATE_12000 | \
							 SNDRV_PCM_RATE_16000 | \
							 SNDRV_PCM_RATE_22050 | \
							 SNDRV_PCM_RATE_24000 | \
							 SNDRV_PCM_RATE_32000 | \
							 SNDRV_PCM_RATE_44100 | \
							 SNDRV_PCM_RATE_48000 | \
							 SNDRV_PCM_RATE_88200 | \
							 SNDRV_PCM_RATE_96000 | \
							 SNDRV_PCM_RATE_176400 | \
							 SNDRV_PCM_RATE_192000)

#define HI3798MV100_AIAO_I2S_TX_CRG_BCLK_DIV_MASK	GENMASK(3, 0)
#define HI3798MV100_AIAO_I2S_TX_CRG_FSCLK_DIV_MASK	GENMASK(6, 4)
#define HI3798MV100_AIAO_I2S_TX_CRG_CKEN		BIT(8)
#define HI3798MV100_AIAO_I2S_TX_CRG_SRST_REQ		BIT(9)
#define HI3798MV100_AIAO_I2S_TX_CRG_BCLK_OEN		BIT(10)
#define HI3798MV100_AIAO_I2S_TX_CRG_BCLK_SEL		BIT(11)
#define HI3798MV100_AIAO_I2S_TX_CRG_BCLKIN_PCTRL	BIT(12)
#define HI3798MV100_AIAO_I2S_TX_CRG_BCLKOUT_PCTRL	BIT(13)
#define HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_8K_256FS		0x00088888
#define HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_11K_256FS		0x000bc28f
#define HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_12K_256FS		0x000ccccc
#define HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_16K_256FS		0x00111111
#define HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_22K_256FS		0x0017851e
#define HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_24K_256FS		0x00199999
#define HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_32K_256FS		0x00222222
#define HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_44K_256FS	0x002f0a3d
#define HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_48K_256FS	0x00333333
#define HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_88K_256FS	0x005e147b
#define HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_96K_256FS	0x00666666
#define HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_176K_256FS	0x00bc28f6
#define HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_192K_256FS	0x00cccccd
#define HI3798MV100_AIAO_I2S_TX_CRG_BCLK_DIV_4		3
#define HI3798MV100_AIAO_I2S_TX_CRG_FSCLK_DIV_64	3
#define HI3798MV100_AIAO_I2S_TX_CRG_SEL_BASE		8
#define HI3798MV100_AIAO_TOP_INT_TX_SHIFT		16

struct hi3798mv100_i2s_state {
	u32 tx_if_attr;
	u32 tx_dsp_ctrl;
	u32 tx_buff_saddr;
	u32 tx_buff_size;
	u32 tx_buff_wptr;
	u32 tx_buff_rptr;
	u32 tx_buff_alempty_th;
	u32 tx_trans_size;
	u32 tx_int_ena;
	u32 com_switch_tx_bclk;
	u32 com_tx_crg_cfg0;
	u32 com_tx_crg_cfg1;
	bool valid;
};

struct hi3798mv100_i2s {
	void __iomem *base;
	void __iomem *aiao_base;
	void __iomem *aiao_com_base;
	struct clk *mclk;
	struct reset_control *rst;
	int irq;
	spinlock_t lock;
	unsigned int dai_fmt;
	unsigned int sample_rate;
	u8 dsp_volume;
	bool dsp_mute;
	struct snd_pcm_substream *playback_substream;
	unsigned int last_period;
	unsigned int tx_id;
	struct hi3798mv100_i2s_state state;
};

static unsigned int hi3798mv100_i2s_rptr_bytes(struct hi3798mv100_i2s *i2s,
						unsigned int buffer_bytes);

static const struct snd_pcm_hardware hi3798mv100_i2s_pcm_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
		   SNDRV_PCM_FMTBIT_S24_LE,
	.rates = HI3798MV100_I2S_PCM_RATES,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min = 2,
	.channels_max = 2,
	.period_bytes_min = HI3798MV100_I2S_BUFFER_ALIGN,
	.period_bytes_max = HI3798MV100_I2S_BUFFER_BYTES_MAX / 2,
	.periods_min = 2,
	.periods_max = 128,
	.buffer_bytes_max = HI3798MV100_I2S_BUFFER_BYTES_MAX,
};

static inline u32 hi3798mv100_i2s_readl(struct hi3798mv100_i2s *i2s,
					u32 reg)
{
	return readl_relaxed(i2s->base + reg);
}

static inline void hi3798mv100_i2s_writel(struct hi3798mv100_i2s *i2s,
					  u32 reg, u32 val)
{
	writel_relaxed(val, i2s->base + reg);
}

static inline u32 hi3798mv100_i2s_com_readl(struct hi3798mv100_i2s *i2s, u32 reg)
{
	return readl_relaxed(i2s->aiao_com_base + reg);
}

static inline void hi3798mv100_i2s_com_writel(struct hi3798mv100_i2s *i2s,
					      u32 reg, u32 val)
{
	writel_relaxed(val, i2s->aiao_com_base + reg);
}

static u32 hi3798mv100_i2s_top_int_mask(const struct hi3798mv100_i2s *i2s)
{
	if (i2s->tx_id >= HI3798MV100_AIAO_TX_PORT_MAX)
		return 0;

	return BIT(HI3798MV100_AIAO_TOP_INT_TX_SHIFT + i2s->tx_id);
}

static void hi3798mv100_i2s_enable_top_int_locked(struct hi3798mv100_i2s *i2s)
{
	u32 mask;
	u32 val;

	if (!i2s->aiao_com_base)
		return;

	mask = hi3798mv100_i2s_top_int_mask(i2s);
	if (!mask)
		return;

	val = hi3798mv100_i2s_com_readl(i2s, HI3798MV100_AIAO_COM_INT_ENA);
	val |= mask;
	hi3798mv100_i2s_com_writel(i2s, HI3798MV100_AIAO_COM_INT_ENA, val);
}

static u32 hi3798mv100_i2s_dsp_ctrl_default(struct hi3798mv100_i2s *i2s)
{
	u32 val = 0;

	if (i2s->dsp_mute)
		val |= HI3798MV100_I2S_TX_MUTE_EN;
	val |= HI3798MV100_I2S_TX_MUTE_FADE_EN;
	val |= FIELD_PREP(HI3798MV100_I2S_TX_VOLUME_MASK, i2s->dsp_volume);
	val |= FIELD_PREP(HI3798MV100_I2S_TX_FADE_IN_RATE_MASK,
			  HI3798MV100_I2S_DSP_FADE_IN_DEF);
	val |= FIELD_PREP(HI3798MV100_I2S_TX_FADE_OUT_RATE_MASK,
			  HI3798MV100_I2S_DSP_FADE_OUT_DEF);

	return val;
}

static void hi3798mv100_i2s_program_dsp_ctrl_locked(struct hi3798mv100_i2s *i2s)
{
	u32 val;

	val = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_DSP_CTRL);
	val &= ~(HI3798MV100_I2S_TX_MUTE_EN |
		 HI3798MV100_I2S_TX_MUTE_FADE_EN |
		 HI3798MV100_I2S_TX_VOLUME_MASK |
		 HI3798MV100_I2S_TX_FADE_IN_RATE_MASK |
		 HI3798MV100_I2S_TX_FADE_OUT_RATE_MASK |
		 HI3798MV100_I2S_TX_BYPASS_EN);
	if (i2s->dsp_mute)
		val |= HI3798MV100_I2S_TX_MUTE_EN;
	val |= HI3798MV100_I2S_TX_MUTE_FADE_EN;
	val |= FIELD_PREP(HI3798MV100_I2S_TX_VOLUME_MASK, i2s->dsp_volume);
	val |= FIELD_PREP(HI3798MV100_I2S_TX_FADE_IN_RATE_MASK,
			  HI3798MV100_I2S_DSP_FADE_IN_DEF);
	val |= FIELD_PREP(HI3798MV100_I2S_TX_FADE_OUT_RATE_MASK,
			  HI3798MV100_I2S_DSP_FADE_OUT_DEF);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_DSP_CTRL, val);
}

static int hi3798mv100_i2s_ctl_volume_get(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct hi3798mv100_i2s *i2s = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = i2s->dsp_volume;
	return 0;
}

static int hi3798mv100_i2s_ctl_volume_put(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct hi3798mv100_i2s *i2s = snd_soc_component_get_drvdata(component);
	unsigned long flags;
	unsigned int volume = ucontrol->value.integer.value[0];

	if (volume > HI3798MV100_I2S_DSP_VOLUME_MAX)
		return -EINVAL;

	spin_lock_irqsave(&i2s->lock, flags);
	if (i2s->dsp_volume == volume) {
		spin_unlock_irqrestore(&i2s->lock, flags);
		return 0;
	}

	i2s->dsp_volume = volume;
	hi3798mv100_i2s_program_dsp_ctrl_locked(i2s);
	spin_unlock_irqrestore(&i2s->lock, flags);

	return 1;
}

static int hi3798mv100_i2s_ctl_switch_get(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct hi3798mv100_i2s *i2s = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = !i2s->dsp_mute;
	return 0;
}

static int hi3798mv100_i2s_ctl_switch_put(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct hi3798mv100_i2s *i2s = snd_soc_component_get_drvdata(component);
	unsigned long flags;
	bool unmuted = !!ucontrol->value.integer.value[0];
	bool new_mute = !unmuted;

	spin_lock_irqsave(&i2s->lock, flags);
	if (i2s->dsp_mute == new_mute) {
		spin_unlock_irqrestore(&i2s->lock, flags);
		return 0;
	}

	i2s->dsp_mute = new_mute;
	hi3798mv100_i2s_program_dsp_ctrl_locked(i2s);
	spin_unlock_irqrestore(&i2s->lock, flags);

	return 1;
}

static int hi3798mv100_i2s_ctl_output_delay_info(struct snd_kcontrol *kcontrol,
						 struct snd_ctl_elem_info *uinfo)
{
	(void)kcontrol;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1000;

	return 0;
}

static int hi3798mv100_i2s_ctl_output_delay_get(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct hi3798mv100_i2s *i2s = snd_soc_component_get_drvdata(component);
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int sample_rate;
	unsigned int buffer_bytes;
	unsigned int rptr;
	unsigned int wptr;
	unsigned int pending;
	unsigned long flags;
	int width;
	u64 bytes_per_sec;
	u64 delay_ms;

	substream = READ_ONCE(i2s->playback_substream);
	if (!substream || !substream->runtime)
		goto out_zero;

	runtime = substream->runtime;
	sample_rate = READ_ONCE(i2s->sample_rate);
	if (!sample_rate)
		goto out_zero;

	width = snd_pcm_format_physical_width(runtime->format);
	if (width <= 0 || !runtime->channels)
		goto out_zero;

	bytes_per_sec = (u64)sample_rate * runtime->channels * width;
	bytes_per_sec = div_u64(bytes_per_sec, 8);
	if (!bytes_per_sec)
		goto out_zero;

	buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
	if (!buffer_bytes)
		goto out_zero;

	spin_lock_irqsave(&i2s->lock, flags);
	rptr = hi3798mv100_i2s_rptr_bytes(i2s, buffer_bytes);
	wptr = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_BUFF_WPTR);
	wptr &= HI3798MV100_I2S_TX_BUF_PTR_MASK;
	if (wptr >= buffer_bytes)
		wptr %= buffer_bytes;
	pending = (wptr + buffer_bytes - rptr) % buffer_bytes;
	spin_unlock_irqrestore(&i2s->lock, flags);

	delay_ms = div_u64((u64)pending * 1000, bytes_per_sec);
	if (delay_ms > INT_MAX)
		delay_ms = INT_MAX;

	ucontrol->value.integer.value[0] = delay_ms;
	return 0;

out_zero:
	ucontrol->value.integer.value[0] = 0;
	return 0;
}

static int hi3798mv100_i2s_ctl_output_delay_put(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	(void)kcontrol;
	(void)ucontrol;

	return 0;
}

static const struct snd_kcontrol_new hi3798mv100_i2s_master_controls[] = {
	SOC_SINGLE_EXT("Master Playback Volume", SND_SOC_NOPM, 0,
		       HI3798MV100_I2S_DSP_VOLUME_MAX, 0,
		       hi3798mv100_i2s_ctl_volume_get,
		       hi3798mv100_i2s_ctl_volume_put),
	SOC_SINGLE_EXT("Master Playback Switch", SND_SOC_NOPM, 0, 1, 0,
		       hi3798mv100_i2s_ctl_switch_get,
		       hi3798mv100_i2s_ctl_switch_put),
	SOC_SINGLE_EXT("ALL Playback Switch", SND_SOC_NOPM, 0, 1, 0,
		       hi3798mv100_i2s_ctl_switch_get,
		       hi3798mv100_i2s_ctl_switch_put),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Output Delay",
		.info = hi3798mv100_i2s_ctl_output_delay_info,
		.get = hi3798mv100_i2s_ctl_output_delay_get,
		.put = hi3798mv100_i2s_ctl_output_delay_put,
	},
};

static const struct snd_kcontrol_new hi3798mv100_i2s_hdmi_controls[] = {
	SOC_SINGLE_EXT("Hdmi Playback Volume", SND_SOC_NOPM, 0,
		       HI3798MV100_I2S_DSP_VOLUME_MAX, 0,
		       hi3798mv100_i2s_ctl_volume_get,
		       hi3798mv100_i2s_ctl_volume_put),
	SOC_SINGLE_EXT("Hdmi Playback Switch", SND_SOC_NOPM, 0, 1, 0,
		       hi3798mv100_i2s_ctl_switch_get,
		       hi3798mv100_i2s_ctl_switch_put),
	SOC_SINGLE_EXT("ALL Playback Switch", SND_SOC_NOPM, 0, 1, 0,
		       hi3798mv100_i2s_ctl_switch_get,
		       hi3798mv100_i2s_ctl_switch_put),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Output Delay",
		.info = hi3798mv100_i2s_ctl_output_delay_info,
		.get = hi3798mv100_i2s_ctl_output_delay_get,
		.put = hi3798mv100_i2s_ctl_output_delay_put,
	},
};

static int hi3798mv100_i2s_component_probe(struct snd_soc_component *component)
{
	struct hi3798mv100_i2s *i2s = snd_soc_component_get_drvdata(component);
	const struct snd_kcontrol_new *controls;
	unsigned int num_controls;

	if (i2s->tx_id == HI3798MV100_AIAO_TX_HDMI_PORT_ID) {
		controls = hi3798mv100_i2s_hdmi_controls;
		num_controls = ARRAY_SIZE(hi3798mv100_i2s_hdmi_controls);
	} else {
		controls = hi3798mv100_i2s_master_controls;
		num_controls = ARRAY_SIZE(hi3798mv100_i2s_master_controls);
	}

	return snd_soc_add_component_controls(component, controls, num_controls);
}

static u32 hi3798mv100_i2s_mclk_div_for_rate(unsigned int sample_rate)
{
	switch (sample_rate) {
	case 8000:
		return HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_8K_256FS;
	case 11025:
		return HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_11K_256FS;
	case 12000:
		return HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_12K_256FS;
	case 16000:
		return HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_16K_256FS;
	case 22050:
		return HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_22K_256FS;
	case 24000:
		return HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_24K_256FS;
	case 32000:
		return HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_32K_256FS;
	case 44100:
		return HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_44K_256FS;
	case 88200:
		return HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_88K_256FS;
	case 96000:
		return HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_96K_256FS;
	case 176400:
		return HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_176K_256FS;
	case 192000:
		return HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_192K_256FS;
	case 48000:
	default:
		return HI3798MV100_AIAO_I2S_TX_CRG_MCLK_DIV_48K_256FS;
	}
}

static void hi3798mv100_i2s_setup_com_clk_txid(struct hi3798mv100_i2s *i2s,
						unsigned int tx_id,
						unsigned int sample_rate)
{
	u32 tx_crg_cfg0;
	u32 tx_crg_cfg1;
	u32 switch_tx;
	u32 shift;
	u32 reg;

	if (!i2s->aiao_com_base || tx_id >= HI3798MV100_AIAO_TX_PORT_MAX)
		return;

	reg = HI3798MV100_AIAO_COM_I2S_TX_CRG_BASE +
	      tx_id * HI3798MV100_AIAO_COM_I2S_TX_CRG_STRIDE;

	tx_crg_cfg0 = hi3798mv100_i2s_mclk_div_for_rate(sample_rate);
	tx_crg_cfg1 = FIELD_PREP(HI3798MV100_AIAO_I2S_TX_CRG_BCLK_DIV_MASK,
				 HI3798MV100_AIAO_I2S_TX_CRG_BCLK_DIV_4);
	tx_crg_cfg1 |= FIELD_PREP(HI3798MV100_AIAO_I2S_TX_CRG_FSCLK_DIV_MASK,
				  HI3798MV100_AIAO_I2S_TX_CRG_FSCLK_DIV_64);
	tx_crg_cfg1 |= HI3798MV100_AIAO_I2S_TX_CRG_CKEN;
	/* The SDK sets these explicitly for TX master mode. */
	tx_crg_cfg1 &= ~(HI3798MV100_AIAO_I2S_TX_CRG_SRST_REQ |
			 HI3798MV100_AIAO_I2S_TX_CRG_BCLK_OEN |
			 HI3798MV100_AIAO_I2S_TX_CRG_BCLK_SEL |
			 HI3798MV100_AIAO_I2S_TX_CRG_BCLKIN_PCTRL |
			 HI3798MV100_AIAO_I2S_TX_CRG_BCLKOUT_PCTRL);

	hi3798mv100_i2s_com_writel(i2s, reg, tx_crg_cfg0);
	hi3798mv100_i2s_com_writel(i2s, reg + 4, tx_crg_cfg1);

	switch_tx = hi3798mv100_i2s_com_readl(i2s,
					      HI3798MV100_AIAO_COM_SWITCH_TX_BCLK);
	shift = tx_id * 4;
	switch_tx &= ~(0xfU << shift);
	switch_tx |= ((HI3798MV100_AIAO_I2S_TX_CRG_SEL_BASE + tx_id) & 0xf) <<
		     shift;
	hi3798mv100_i2s_com_writel(i2s, HI3798MV100_AIAO_COM_SWITCH_TX_BCLK,
				   switch_tx);
}

static void hi3798mv100_i2s_setup_com_clk(struct hi3798mv100_i2s *i2s,
					  unsigned int sample_rate)
{
	hi3798mv100_i2s_setup_com_clk_txid(i2s, i2s->tx_id, sample_rate);
}

static void hi3798mv100_i2s_save_state_locked(struct hi3798mv100_i2s *i2s)
{
	struct hi3798mv100_i2s_state *state = &i2s->state;
	u32 reg;

	state->tx_if_attr = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_IF_ATTRI);
	state->tx_dsp_ctrl = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_DSP_CTRL);
	state->tx_buff_saddr = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_BUFF_SADDR);
	state->tx_buff_size = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_BUFF_SIZE);
	state->tx_buff_wptr = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_BUFF_WPTR);
	state->tx_buff_rptr = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_BUFF_RPTR);
	state->tx_buff_alempty_th = hi3798mv100_i2s_readl(i2s,
							  HI3798MV100_I2S_TX_BUFF_ALEMPTY_TH);
	state->tx_trans_size = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_TRANS_SIZE);
	state->tx_int_ena = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_INT_ENA);

	if (i2s->tx_id >= HI3798MV100_AIAO_TX_PORT_MAX)
		goto done;

	reg = HI3798MV100_AIAO_COM_I2S_TX_CRG_BASE +
	      i2s->tx_id * HI3798MV100_AIAO_COM_I2S_TX_CRG_STRIDE;
	state->com_tx_crg_cfg0 = hi3798mv100_i2s_com_readl(i2s, reg);
	state->com_tx_crg_cfg1 = hi3798mv100_i2s_com_readl(i2s, reg + 4);
	state->com_switch_tx_bclk = hi3798mv100_i2s_com_readl(i2s,
							      HI3798MV100_AIAO_COM_SWITCH_TX_BCLK);
done:
	state->valid = true;
}

static void hi3798mv100_i2s_restore_state_locked(struct hi3798mv100_i2s *i2s)
{
	const struct hi3798mv100_i2s_state *state = &i2s->state;
	u32 reg;

	if (!state->valid)
		return;

	hi3798mv100_i2s_enable_top_int_locked(i2s);

	if (i2s->tx_id < HI3798MV100_AIAO_TX_PORT_MAX) {
		u32 shift = i2s->tx_id * 4;
		u32 mask = 0xfU << shift;
		u32 switch_tx;

		reg = HI3798MV100_AIAO_COM_I2S_TX_CRG_BASE +
		      i2s->tx_id * HI3798MV100_AIAO_COM_I2S_TX_CRG_STRIDE;
		hi3798mv100_i2s_com_writel(i2s, reg, state->com_tx_crg_cfg0);
		hi3798mv100_i2s_com_writel(i2s, reg + 4, state->com_tx_crg_cfg1);
		switch_tx = hi3798mv100_i2s_com_readl(i2s,
						      HI3798MV100_AIAO_COM_SWITCH_TX_BCLK);
		switch_tx &= ~mask;
		switch_tx |= state->com_switch_tx_bclk & mask;
		hi3798mv100_i2s_com_writel(i2s, HI3798MV100_AIAO_COM_SWITCH_TX_BCLK,
					   switch_tx);
	}

	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_IF_ATTRI, state->tx_if_attr);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_DSP_CTRL, state->tx_dsp_ctrl);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_BUFF_SADDR,
			       state->tx_buff_saddr);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_BUFF_SIZE, state->tx_buff_size);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_BUFF_WPTR, state->tx_buff_wptr);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_BUFF_RPTR, state->tx_buff_rptr);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_BUFF_ALEMPTY_TH,
			       state->tx_buff_alempty_th);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_TRANS_SIZE, state->tx_trans_size);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_INT_CLR,
			       HI3798MV100_I2S_TX_INT_CLR_ALL);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_INT_ENA,
			       state->tx_int_ena & HI3798MV100_I2S_TX_INT_TRANSFINISH);
}

static void hi3798mv100_i2s_dump_regs(struct hi3798mv100_i2s *i2s,
				      struct device *dev, const char *tag)
{
	u32 if_attr;
	u32 dsp_ctrl;
	u32 bclk_cnt;
	u32 ws_cnt;
	u32 size;
	u32 wptr;
	u32 rptr;
	u32 tx_crg_cfg0 = 0;
	u32 tx_crg_cfg1 = 0;
	u32 switch_tx = 0;
	u32 reg;

	if_attr = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_IF_ATTRI);
	dsp_ctrl = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_DSP_CTRL);
	bclk_cnt = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_BCLK_CNT);
	ws_cnt = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_WS_CNT);
	size = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_BUFF_SIZE);
	wptr = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_BUFF_WPTR);
	rptr = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_BUFF_RPTR);

	if (i2s->aiao_com_base && i2s->tx_id < HI3798MV100_AIAO_TX_PORT_MAX) {
		reg = HI3798MV100_AIAO_COM_I2S_TX_CRG_BASE +
		      i2s->tx_id * HI3798MV100_AIAO_COM_I2S_TX_CRG_STRIDE;
		tx_crg_cfg0 = hi3798mv100_i2s_com_readl(i2s, reg);
		tx_crg_cfg1 = hi3798mv100_i2s_com_readl(i2s, reg + 4);
		switch_tx = hi3798mv100_i2s_com_readl(i2s,
						      HI3798MV100_AIAO_COM_SWITCH_TX_BCLK);
	}

	dev_dbg(dev,
		"%s: if=0x%08x dsp=0x%08x ws=0x%08x bclk=0x%08x size=0x%08x wptr=0x%08x rptr=0x%08x crg0=0x%08x crg1=0x%08x sw=0x%08x tx=%u\n",
		tag, if_attr, dsp_ctrl, ws_cnt, bclk_cnt, size, wptr, rptr,
		tx_crg_cfg0, tx_crg_cfg1, switch_tx, i2s->tx_id);
}

static unsigned int hi3798mv100_i2s_rptr_bytes(struct hi3798mv100_i2s *i2s,
						unsigned int buffer_bytes)
{
	u32 rptr;

	rptr = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_BUFF_RPTR);
	rptr &= HI3798MV100_I2S_TX_BUF_PTR_MASK;

	if (buffer_bytes && rptr >= buffer_bytes)
		rptr %= buffer_bytes;

	return rptr;
}

static unsigned int hi3798mv100_i2s_guard_wptr(struct hi3798mv100_i2s *i2s,
						unsigned int wptr,
						unsigned int buffer_bytes)
{
	unsigned int rptr;
	unsigned int guard = HI3798MV100_I2S_BUFFER_ALIGN;

	if (!buffer_bytes)
		return wptr;

	if (guard >= buffer_bytes)
		guard = buffer_bytes / 2;
	if (!guard)
		return wptr;

	rptr = hi3798mv100_i2s_rptr_bytes(i2s, buffer_bytes);
	if (wptr == rptr)
		wptr = (wptr + buffer_bytes - guard) % buffer_bytes;

	return wptr;
}

static void hi3798mv100_i2s_set_period_irq_locked(struct hi3798mv100_i2s *i2s,
						   bool enable)
{
	if (enable) {
		hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_INT_CLR,
				       HI3798MV100_I2S_TX_INT_CLR_ALL);
		hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_INT_ENA,
				       HI3798MV100_I2S_TX_INT_TRANSFINISH);
	} else {
		hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_INT_ENA, 0);
		hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_INT_CLR,
				       HI3798MV100_I2S_TX_INT_CLR_ALL);
	}
}

static void hi3798mv100_i2s_period_elapsed(struct hi3798mv100_i2s *i2s)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	unsigned int period_bytes;
	unsigned int buffer_bytes;
	unsigned int periods;
	unsigned int cur_period;
	unsigned int last_period;
	unsigned int delta;
	unsigned int i;

	substream = READ_ONCE(i2s->playback_substream);
	if (!substream)
		return;

	runtime = substream->runtime;
	if (!runtime)
		return;

	period_bytes = snd_pcm_lib_period_bytes(substream);
	buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
	if (!period_bytes || !buffer_bytes)
		return;

	periods = max(1U, runtime->periods);
	cur_period = hi3798mv100_i2s_rptr_bytes(i2s, buffer_bytes) / period_bytes;
	last_period = READ_ONCE(i2s->last_period);
	delta = (cur_period + periods - last_period) % periods;
	if (delta) {
		WRITE_ONCE(i2s->last_period, cur_period);
		for (i = 0; i < delta; i++)
			snd_pcm_period_elapsed(substream);
	}
}

static irqreturn_t hi3798mv100_i2s_irq(int irq, void *dev_id)
{
	struct hi3798mv100_i2s *i2s = dev_id;
	u32 int_status;

	(void)irq;

	int_status = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_INT_STATUS);
	int_status &= HI3798MV100_I2S_TX_INT_CLR_ALL;
	if (!int_status)
		return IRQ_NONE;

	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_INT_CLR, int_status);
	if (int_status & HI3798MV100_I2S_TX_INT_TRANSFINISH)
		hi3798mv100_i2s_period_elapsed(i2s);

	return IRQ_HANDLED;
}

static int hi3798mv100_i2s_startup(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *cpu_dai)
{
	struct hi3798mv100_i2s *i2s = dev_get_drvdata(cpu_dai->dev);
	int ret;

	(void)substream;

	ret = pm_runtime_resume_and_get(cpu_dai->dev);
	if (ret < 0)
		return ret;

	dev_dbg(cpu_dai->dev, "startup: mclk=%lu\n",
		clk_get_rate(i2s->mclk));

	return 0;
}

static void hi3798mv100_i2s_shutdown(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *cpu_dai)
{
	struct hi3798mv100_i2s *i2s = dev_get_drvdata(cpu_dai->dev);
	unsigned long flags;
	u32 val;

	(void)substream;

	WRITE_ONCE(i2s->playback_substream, NULL);
	spin_lock_irqsave(&i2s->lock, flags);
	hi3798mv100_i2s_set_period_irq_locked(i2s, false);
	val = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_DSP_CTRL);
	val &= ~HI3798MV100_I2S_TX_ENABLE;
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_DSP_CTRL, val);
	spin_unlock_irqrestore(&i2s->lock, flags);

	pm_runtime_mark_last_busy(cpu_dai->dev);
	pm_runtime_put_autosuspend(cpu_dai->dev);
}

static int hi3798mv100_i2s_set_fmt(struct snd_soc_dai *cpu_dai,
				   unsigned int fmt)
{
	struct hi3798mv100_i2s *i2s = dev_get_drvdata(cpu_dai->dev);

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BC_FC:
	case SND_SOC_DAIFMT_BP_FP:
		break;
	default:
		return -EINVAL;
	}

	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		return -EINVAL;
	}

	i2s->dai_fmt = fmt;

	return 0;
}

static int hi3798mv100_i2s_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params,
				     struct snd_soc_dai *cpu_dai)
{
	struct hi3798mv100_i2s *i2s = dev_get_drvdata(cpu_dai->dev);
	unsigned int precision;
	unsigned int mode = HI3798MV100_I2S_TX_MODE_I2S;
	unsigned long flags;
	u32 val;

	(void)substream;

	switch (params_rate(params)) {
	case 8000:
	case 11025:
	case 12000:
	case 16000:
	case 22050:
	case 24000:
	case 32000:
	case 44100:
	case 48000:
	case 88200:
	case 96000:
	case 176400:
	case 192000:
		break;
	default:
		return -EINVAL;
	}

	if (params_channels(params) != 2)
		return -EINVAL;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		precision = HI3798MV100_I2S_TX_PRECISION_16BIT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		precision = HI3798MV100_I2S_TX_PRECISION_24BIT;
		break;
	default:
		return -EINVAL;
	}

	if ((i2s->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S)
		mode = HI3798MV100_I2S_TX_MODE_PCM;

	spin_lock_irqsave(&i2s->lock, flags);
	i2s->sample_rate = params_rate(params);
	val = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_IF_ATTRI);
	val &= ~(HI3798MV100_I2S_TX_MODE_MASK |
		 HI3798MV100_I2S_TX_PRECISION_MASK |
		 HI3798MV100_I2S_TX_CH_NUM_MASK |
		 HI3798MV100_I2S_TX_SD_OFFSET_MASK |
		 HI3798MV100_I2S_TX_TRACK_MODE_MASK |
		 HI3798MV100_I2S_TX_SPD_I2S_SEL |
		 HI3798MV100_I2S_TX_SD_SOURCE_SEL_MASK |
		 HI3798MV100_I2S_TX_SD0_SEL_MASK |
		 HI3798MV100_I2S_TX_SD1_SEL_MASK |
		 HI3798MV100_I2S_TX_SD2_SEL_MASK |
		 HI3798MV100_I2S_TX_SD3_SEL_MASK);
	val |= FIELD_PREP(HI3798MV100_I2S_TX_MODE_MASK, mode);
	val |= FIELD_PREP(HI3798MV100_I2S_TX_PRECISION_MASK, precision);
	val |= FIELD_PREP(HI3798MV100_I2S_TX_CH_NUM_MASK,
			  HI3798MV100_I2S_TX_CH_NUM_2);
	val |= FIELD_PREP(HI3798MV100_I2S_TX_SD_OFFSET_MASK,
			  HI3798MV100_I2S_SD_OFFSET_DEF);
	val |= FIELD_PREP(HI3798MV100_I2S_TX_TRACK_MODE_MASK,
			  HI3798MV100_I2S_TX_TRACK_MODE_STEREO);
	val |= HI3798MV100_I2S_TX_SPD_I2S_SEL;
	val |= FIELD_PREP(HI3798MV100_I2S_TX_SD_SOURCE_SEL_MASK,
			  i2s->tx_id);
	val |= FIELD_PREP(HI3798MV100_I2S_TX_SD0_SEL_MASK,
			  HI3798MV100_I2S_TX_SD0_SEL_SD0);
	val |= FIELD_PREP(HI3798MV100_I2S_TX_SD1_SEL_MASK,
			  HI3798MV100_I2S_TX_SD1_SEL_SD1);
	val |= FIELD_PREP(HI3798MV100_I2S_TX_SD2_SEL_MASK,
			  HI3798MV100_I2S_TX_SD2_SEL_SD2);
	val |= FIELD_PREP(HI3798MV100_I2S_TX_SD3_SEL_MASK,
			  HI3798MV100_I2S_TX_SD3_SEL_SD3);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_IF_ATTRI, val);
	hi3798mv100_i2s_program_dsp_ctrl_locked(i2s);
	hi3798mv100_i2s_setup_com_clk(i2s, i2s->sample_rate);
	spin_unlock_irqrestore(&i2s->lock, flags);

	hi3798mv100_i2s_dump_regs(i2s, cpu_dai->dev, "hw_params");

	return 0;
}

static int hi3798mv100_i2s_trigger(struct snd_pcm_substream *substream,
				   int cmd, struct snd_soc_dai *cpu_dai)
{
	struct hi3798mv100_i2s *i2s = dev_get_drvdata(cpu_dai->dev);
	unsigned long flags;
	bool enable;
	u32 val;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		enable = true;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		enable = false;
		break;
	default:
		return -EINVAL;
	}

	if (enable && substream->runtime) {
		unsigned int buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
		unsigned int wptr = 0;
		snd_pcm_uframes_t appl_ptr = substream->runtime->control->appl_ptr;

		if (buffer_bytes)
			wptr = frames_to_bytes(substream->runtime, appl_ptr) %
			       buffer_bytes;
		wptr = hi3798mv100_i2s_guard_wptr(i2s, wptr, buffer_bytes);

		spin_lock_irqsave(&i2s->lock, flags);
		hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_BUFF_WPTR, wptr);
		spin_unlock_irqrestore(&i2s->lock, flags);
		dev_dbg(cpu_dai->dev, "trigger-start: appl_ptr=%llu wptr=0x%x\n",
			(unsigned long long)appl_ptr, wptr);

		WRITE_ONCE(i2s->playback_substream, substream);
		if (snd_pcm_lib_period_bytes(substream))
			WRITE_ONCE(i2s->last_period,
				   hi3798mv100_i2s_rptr_bytes(i2s, buffer_bytes) /
				   snd_pcm_lib_period_bytes(substream));
	}
	if (!enable)
		WRITE_ONCE(i2s->playback_substream, NULL);

	spin_lock_irqsave(&i2s->lock, flags);
	val = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_DSP_CTRL);
	if (enable) {
		hi3798mv100_i2s_set_period_irq_locked(i2s, true);
		val |= HI3798MV100_I2S_TX_ENABLE;
	} else {
		hi3798mv100_i2s_set_period_irq_locked(i2s, false);
		val &= ~HI3798MV100_I2S_TX_ENABLE;
	}
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_DSP_CTRL, val);
	spin_unlock_irqrestore(&i2s->lock, flags);

	hi3798mv100_i2s_dump_regs(i2s, cpu_dai->dev,
				  enable ? "trigger-start" : "trigger-stop");

	return 0;
}

static const struct snd_soc_dai_ops hi3798mv100_i2s_dai_ops = {
	.startup = hi3798mv100_i2s_startup,
	.shutdown = hi3798mv100_i2s_shutdown,
	.set_fmt = hi3798mv100_i2s_set_fmt,
	.hw_params = hi3798mv100_i2s_hw_params,
	.trigger = hi3798mv100_i2s_trigger,
};

static struct snd_soc_dai_driver hi3798mv100_i2s_dai = {
	.name = "hi3798mv100-i2s-dai",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = HI3798MV100_I2S_PCM_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE,
	},
	.ops = &hi3798mv100_i2s_dai_ops,
};

static int hi3798mv100_i2s_pcm_open(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	(void)component;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	snd_soc_set_runtime_hwparams(substream, &hi3798mv100_i2s_pcm_hw);

	ret = snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
					 HI3798MV100_I2S_BUFFER_ALIGN);
	if (ret)
		return ret;

	ret = snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
					 HI3798MV100_I2S_BUFFER_ALIGN);
	if (ret)
		return ret;

	return snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
}

static int hi3798mv100_i2s_pcm_construct(struct snd_soc_component *component,
					 struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm_substream *substream;

	substream = rtd->pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	if (!substream)
		return 0;

	return snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, component->dev,
				   HI3798MV100_I2S_BUFFER_BYTES_MAX,
				   &substream->dma_buffer);
}

static void hi3798mv100_i2s_pcm_destruct(struct snd_soc_component *component,
					 struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;

	(void)component;

	substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	if (substream)
		snd_dma_free_pages(&substream->dma_buffer);
}

static int hi3798mv100_i2s_pcm_hw_params(struct snd_soc_component *component,
					 struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params)
{
	(void)component;
	(void)params;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	return 0;
}

static int hi3798mv100_i2s_pcm_hw_free(struct snd_soc_component *component,
				       struct snd_pcm_substream *substream)
{
	(void)component;

	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int hi3798mv100_i2s_pcm_prepare(struct snd_soc_component *component,
				       struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct hi3798mv100_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned int buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
	unsigned int period_bytes = snd_pcm_lib_period_bytes(substream);
	unsigned long flags;

	(void)component;

	if (!buffer_bytes || !period_bytes)
		return -EINVAL;

	if (!IS_ALIGNED(buffer_bytes, HI3798MV100_I2S_BUFFER_ALIGN) ||
	    !IS_ALIGNED(period_bytes, HI3798MV100_I2S_BUFFER_ALIGN))
		return -EINVAL;

	if (upper_32_bits(substream->dma_buffer.addr))
		return -EADDRNOTAVAIL;

	switch (substream->runtime->format) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S24_LE:
		break;
	default:
		return -EINVAL;
	}

	WRITE_ONCE(i2s->playback_substream, NULL);
	WRITE_ONCE(i2s->last_period, 0);

	spin_lock_irqsave(&i2s->lock, flags);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_BUFF_SADDR,
			       lower_32_bits(substream->dma_buffer.addr));
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_BUFF_SIZE, buffer_bytes);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_BUFF_ALEMPTY_TH,
			       period_bytes);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_TRANS_SIZE, period_bytes);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_BUFF_RPTR, 0);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_BUFF_WPTR, 0);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_INT_CLR,
			       HI3798MV100_I2S_TX_INT_CLR_ALL);
	hi3798mv100_i2s_set_period_irq_locked(i2s, false);
	spin_unlock_irqrestore(&i2s->lock, flags);

	hi3798mv100_i2s_dump_regs(i2s, cpu_dai->dev, "prepare");

	return 0;
}

static snd_pcm_uframes_t
hi3798mv100_i2s_pcm_pointer(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct hi3798mv100_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned int buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
	unsigned int rptr;

	(void)component;

	rptr = hi3798mv100_i2s_rptr_bytes(i2s, buffer_bytes);
	return bytes_to_frames(substream->runtime, rptr);
}

static int hi3798mv100_i2s_pcm_ack(struct snd_soc_component *component,
				   struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct hi3798mv100_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
	unsigned int wptr;
	unsigned long flags;

	(void)component;

	if (!buffer_bytes)
		return -EINVAL;

	wptr = frames_to_bytes(runtime, runtime->control->appl_ptr) % buffer_bytes;
	if (wptr == hi3798mv100_i2s_rptr_bytes(i2s, buffer_bytes)) {
		unsigned int old_wptr = wptr;

		wptr = hi3798mv100_i2s_guard_wptr(i2s, wptr, buffer_bytes);
		if (old_wptr != wptr)
			dev_dbg_ratelimited(cpu_dai->dev,
					    "ack: guarded full-ring wptr 0x%x -> 0x%x (appl_ptr=%llu)\n",
					    old_wptr, wptr,
					    (unsigned long long)runtime->control->appl_ptr);
	}

	spin_lock_irqsave(&i2s->lock, flags);
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_BUFF_WPTR, wptr);
	spin_unlock_irqrestore(&i2s->lock, flags);

	return 0;
}

static const struct snd_soc_component_driver hi3798mv100_i2s_component = {
	.name = "hi3798mv100-i2s",
	.probe = hi3798mv100_i2s_component_probe,
	.open = hi3798mv100_i2s_pcm_open,
	.pcm_construct = hi3798mv100_i2s_pcm_construct,
	.pcm_destruct = hi3798mv100_i2s_pcm_destruct,
	.hw_params = hi3798mv100_i2s_pcm_hw_params,
	.hw_free = hi3798mv100_i2s_pcm_hw_free,
	.prepare = hi3798mv100_i2s_pcm_prepare,
	.pointer = hi3798mv100_i2s_pcm_pointer,
	.ack = hi3798mv100_i2s_pcm_ack,
};

static int __maybe_unused hi3798mv100_i2s_runtime_suspend(struct device *dev)
{
	struct hi3798mv100_i2s *i2s = dev_get_drvdata(dev);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&i2s->lock, flags);
	hi3798mv100_i2s_save_state_locked(i2s);
	hi3798mv100_i2s_set_period_irq_locked(i2s, false);
	val = hi3798mv100_i2s_readl(i2s, HI3798MV100_I2S_TX_DSP_CTRL);
	val &= ~HI3798MV100_I2S_TX_ENABLE;
	hi3798mv100_i2s_writel(i2s, HI3798MV100_I2S_TX_DSP_CTRL, val);
	spin_unlock_irqrestore(&i2s->lock, flags);

	clk_disable_unprepare(i2s->mclk);
	if (i2s->rst)
		reset_control_assert(i2s->rst);

	return 0;
}

static int __maybe_unused hi3798mv100_i2s_runtime_resume(struct device *dev)
{
	struct hi3798mv100_i2s *i2s = dev_get_drvdata(dev);
	unsigned long flags;
	int ret;

	if (i2s->rst) {
		ret = reset_control_deassert(i2s->rst);
		if (ret)
			return ret;
	}

	ret = clk_prepare_enable(i2s->mclk);
	if (ret)
		return ret;

	spin_lock_irqsave(&i2s->lock, flags);
	hi3798mv100_i2s_restore_state_locked(i2s);
	spin_unlock_irqrestore(&i2s->lock, flags);

	return 0;
}

static const struct dev_pm_ops hi3798mv100_i2s_pm_ops = {
	SET_RUNTIME_PM_OPS(hi3798mv100_i2s_runtime_suspend,
			   hi3798mv100_i2s_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static int hi3798mv100_i2s_probe(struct platform_device *pdev)
{
	struct hi3798mv100_i2s *i2s;
	struct platform_device *parent_pdev;
	struct resource *res;
	struct resource *parent_res;
	resource_size_t tx_phys;
	resource_size_t tx_off;
	resource_size_t aiao_phys;
	resource_size_t aiao_map_size;
	int ret;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to set DMA mask\n");

	i2s->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2s->base))
		return PTR_ERR(i2s->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	tx_phys = res->start;
	parent_res = NULL;
	if (pdev->dev.parent) {
		parent_pdev = to_platform_device(pdev->dev.parent);
		parent_res = platform_get_resource(parent_pdev, IORESOURCE_MEM,
						   0);
	}
	if (parent_res)
		aiao_phys = parent_res->start;
	else
		aiao_phys = tx_phys - HI3798MV100_AIAO_TX_PORT_OFFSET;
	if (parent_res)
		aiao_map_size = resource_size(parent_res);
	else
		aiao_map_size = HI3798MV100_AIAO_MAP_SIZE;

	if (tx_phys < aiao_phys + HI3798MV100_AIAO_TX_PORT_OFFSET)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "invalid i2s offset\n");

	tx_off = tx_phys - aiao_phys;
	if (tx_off < HI3798MV100_AIAO_TX_PORT_OFFSET ||
	    (tx_off - HI3798MV100_AIAO_TX_PORT_OFFSET) %
		    HI3798MV100_AIAO_TX_PORT_STRIDE)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "unsupported i2s resource offset\n");

	i2s->tx_id = (tx_off - HI3798MV100_AIAO_TX_PORT_OFFSET) /
		     HI3798MV100_AIAO_TX_PORT_STRIDE;
	if (i2s->tx_id >= HI3798MV100_AIAO_TX_PORT_MAX)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "invalid tx channel id %u\n", i2s->tx_id);

	i2s->aiao_base = devm_ioremap(&pdev->dev, aiao_phys, aiao_map_size);
	if (!i2s->aiao_base)
		return dev_err_probe(&pdev->dev, -ENOMEM,
				     "failed to map aiao\n");
	i2s->aiao_com_base = i2s->aiao_base;

	i2s->rst = devm_reset_control_get_optional_shared(&pdev->dev, "core");
	if (IS_ERR(i2s->rst))
		return dev_err_probe(&pdev->dev, PTR_ERR(i2s->rst),
				     "failed to get core reset\n");

	i2s->mclk = devm_clk_get(&pdev->dev, "mclk");
	if (IS_ERR(i2s->mclk))
		return dev_err_probe(&pdev->dev, PTR_ERR(i2s->mclk),
				     "failed to get mclk\n");

	i2s->irq = platform_get_irq(pdev, 0);
	if (i2s->irq < 0)
		return i2s->irq;

	spin_lock_init(&i2s->lock);
	i2s->dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		       SND_SOC_DAIFMT_BP_FP;
	i2s->sample_rate = 48000;
	i2s->dsp_volume = HI3798MV100_I2S_DSP_VOLUME_0DB;
	i2s->dsp_mute = false;
	i2s->state.tx_dsp_ctrl = hi3798mv100_i2s_dsp_ctrl_default(i2s);
	i2s->state.valid = true;

	ret = devm_request_irq(&pdev->dev, i2s->irq, hi3798mv100_i2s_irq,
			       IRQF_SHARED, dev_name(&pdev->dev), i2s);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to request irq\n");

	dev_set_drvdata(&pdev->dev, i2s);

	pm_runtime_set_autosuspend_delay(&pdev->dev, 500);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	ret = devm_pm_runtime_enable(&pdev->dev);
	if (ret)
		return ret;

	return devm_snd_soc_register_component(&pdev->dev,
					       &hi3798mv100_i2s_component,
					       &hi3798mv100_i2s_dai, 1);
}

static const struct of_device_id hi3798mv100_i2s_of_match[] = {
	{ .compatible = "hisilicon,hi3798mv100-i2s" },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3798mv100_i2s_of_match);

static struct platform_driver hi3798mv100_i2s_driver = {
	.probe = hi3798mv100_i2s_probe,
	.driver = {
		.name = "hi3798mv100-i2s",
		.of_match_table = hi3798mv100_i2s_of_match,
		.pm = pm_ptr(&hi3798mv100_i2s_pm_ops),
	},
};
module_platform_driver(hi3798mv100_i2s_driver);

MODULE_DESCRIPTION("HiSilicon Hi3798MV100 I2S ASoC CPU DAI driver");
MODULE_AUTHOR("HiSilicon STB community");
MODULE_LICENSE("GPL");
