// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Rockchip Electronics Co. Ltd.
 *
 * Author: Dingxian Wen <shawn.wen@rock-chips.com>
 */

// #define DEBUG
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <video/videomode.h>
#include <linux/debugfs.h>

#include "rk628_csi.h"
#include "rk628.h"
#include "rk628_hdmirx.h"
#include "rk628_cru.h"
#include "rk628_combrxphy.h"
#include "rk628_combtxphy.h"
#include "rk628_mipi_dphy.h"

#undef dev_dbg
#define dev_dbg(dev, format, arg...)           \
	dev_printk(KERN_INFO, dev, format, ##arg)

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-1)");

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x0, 0x6)

#define EDID_NUM_BLOCKS_MAX 		2
#define EDID_BLOCK_SIZE 		128

#define MIPI_DATARATE_MBPS_LOW		750
#define MIPI_DATARATE_MBPS_HIGH		1250

#define POLL_INTERVAL_MS		1000
#define MODETCLK_CNT_NUM		1000
#define MODETCLK_HZ			49500000
#define RXPHY_CFG_MAX_TIMES		15
#define CSITX_ERR_RETRY_TIMES		3

#define USE_4_LANES			4
#define YUV422_8BIT			0x1e

struct rk628_csi {
	struct device *dev;
	struct rk628 *parent;
	struct class *rk628_class;
	struct i2c_client *i2c_client;
	struct rk628 *rk628;
	struct v4l2_dv_timings src_timings;
	struct v4l2_dv_timings timings;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *power_gpio;
	struct gpio_desc *plugin_det_gpio;
	struct clk *soc_24M;
	struct clk *clk_hdmirx_aud;
	struct clk *clk_vop;
	struct clk *clk_rx_read;
	struct delayed_work delayed_work_enable_hotplug;
	struct delayed_work delayed_work_res_change;
	struct timer_list timer;
	struct work_struct work_i2c_poll;
	struct mutex confctl_mutex;
	const struct rk628_csi_mode *cur_mode;
	u8 edid_blocks_written;
	u64 lane_mbps;
	u8 csi_lanes_in_use;
	u32 mbus_fmt_code;
	u8 fps;
	u32 stream_state;
	int hdmirx_irq;
	int plugin_irq;
	bool nosignal;
	bool rxphy_pwron;
	bool txphy_pwron;
	bool enable_hdcp;
	bool scaler_en;
	bool hpd_output_inverted;
	bool avi_rcv_rdy;
	bool vid_ints_en;
	struct rk628_hdcp hdcp;
	bool i2s_enable_default;
	HAUDINFO audio_info;
	struct rk628_combtxphy *txphy;
};

struct rk628_csi_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
};

static u8 edid_init_data[] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x49, 0x73, 0x8D, 0x62, 0x00, 0x88, 0x88, 0x88,
	0x08, 0x1E, 0x01, 0x03, 0x80, 0x00, 0x00, 0x78,
	0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,
	0x12, 0x48, 0x4C, 0x00, 0x00, 0x00, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3A,
	0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C,
	0x45, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x1E,
	0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20,
	0x6E, 0x28, 0x55, 0x00, 0xC4, 0x8E, 0x21, 0x00,
	0x00, 0x1E, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x54,
	0x37, 0x34, 0x39, 0x2D, 0x66, 0x48, 0x44, 0x37,
	0x32, 0x30, 0x0A, 0x20, 0x00, 0x00, 0x00, 0xFD,
	0x00, 0x14, 0x78, 0x01, 0xFF, 0x1D, 0x00, 0x0A,
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x18,

	0x02, 0x03, 0x1A, 0x71, 0x47, 0x5F, 0x90, 0x22,
	0x04, 0x11, 0x02, 0x01, 0x23, 0x09, 0x07, 0x01,
	0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0C, 0x00,
	0x10, 0x00, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38,
	0x2D, 0x40, 0x58, 0x2C, 0x45, 0x00, 0x20, 0xC2,
	0x31, 0x00, 0x00, 0x1E, 0x01, 0x1D, 0x00, 0x72,
	0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00,
	0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x02, 0x3A,
	0x80, 0xD0, 0x72, 0x38, 0x2D, 0x40, 0x10, 0x2C,
	0x45, 0x80, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E,
	0x01, 0x1D, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40,
	0x58, 0x2C, 0x45, 0x00, 0xC0, 0x6C, 0x00, 0x00,
	0x00, 0x18, 0x01, 0x1D, 0x80, 0x18, 0x71, 0x1C,
	0x16, 0x20, 0x58, 0x2C, 0x25, 0x00, 0xC0, 0x6C,
	0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC1,
};

static const struct rk628_csi_mode supported_modes[] = {
	{
		.width = 3840,
		.height = 2160,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.hts_def = 4400,
		.vts_def = 2250,
	}, {
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 2200,
		.vts_def = 1125,
	}, {
		.width = 1280,
		.height = 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 1650,
		.vts_def = 750,
	}, {
		.width = 720,
		.height = 576,
		.max_fps = {
			.numerator = 10000,
			.denominator = 500000,
		},
		.hts_def = 864,
		.vts_def = 625,
	}, {
		.width = 720,
		.height = 480,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 858,
		.vts_def = 525,
	},
};

static struct v4l2_dv_timings dst_timing = {
	.type = V4L2_DV_BT_656_1120,
	.bt = {
		.interlaced = V4L2_DV_PROGRESSIVE,
		.width = 1920,
		.height = 1080,
		.hfrontporch = 88,
		.hsync = 44,
		.hbackporch = 148,
		.vfrontporch = 4,
		.vsync = 5,
		.vbackporch = 36,
		.pixelclock = 148500000,
	},
};

static void rk628_post_process_setup(struct rk628_csi *csi);
static void rk628_csi_enable_interrupts(struct rk628_csi *csi, bool en);
static int rk628_csi_s_dv_timings(struct rk628_csi *csi,
				 struct v4l2_dv_timings *timings);
static int rk628_csi_s_edid(struct rk628_csi *csi,
				struct v4l2_subdev_edid *edid);
static int mipi_dphy_power_on(struct rk628_csi *csi);
static void mipi_dphy_power_off(struct rk628_csi *csi);
static int rk628_hdmirx_phy_power_on(struct rk628_csi *csi);
static int rk628_hdmirx_phy_power_off(struct rk628_csi *csi);
static int rk628_hdmirx_phy_setup(struct rk628_csi *csi);
static void rk628_csi_format_change(struct rk628_csi *csi);
static void enable_stream(struct rk628_csi *csi, bool enable);
static void rk628_hdmirx_vid_enable(struct rk628_csi *csi, bool en);
static void rk628_csi_set_csi(struct rk628_csi *csi);
static void rk628_hdmirx_hpd_ctrl(struct rk628_csi *csi, bool en);
static bool tx_5v_power_present(struct rk628_csi *csi);
static void rk628_hdmirx_controller_reset(struct rk628_csi *csi);
static struct rk628_csi *g_csi;

#ifndef KERNEL_VERSION_4_19
static ssize_t rk628_resolution_read(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct rk628_csi *csi = g_csi;
	u32 width, height, fps, fs_audio;
	static u8 cnt;

	if (csi->nosignal && tx_5v_power_present(csi)) {
		if (cnt++ >= 60) {
			cnt = 0;
			dev_info(csi->dev, "no signal but 5v_det, recfg hdmirx!\n");
			schedule_delayed_work(&csi->delayed_work_enable_hotplug,
					HZ / 20);
		}
	} else {
		cnt = 0;
	}

	width = csi->timings.bt.width;
	height = csi->timings.bt.height;
	fps = csi->fps;
	fs_audio = rk628_hdmirx_audio_fs(csi->audio_info);
	if(!rk628_hdmirx_audio_present(csi->audio_info)) {
		fs_audio = 0;
	}

	/* update resolution for userspace only if signal is stable */
	if (csi->nosignal) {
		dev_dbg(csi->dev, "%s: nosignal\n", __func__);
		return sprintf(buf, "unsupported\n");
	}

	dev_dbg(csi->dev, "%s %dx%dP%d@%d\n", __func__, width, height, fps,
			fs_audio);
	return sprintf(buf, "%dx%dP%d@%d\n", width, height, fps, fs_audio);
}

static ssize_t rk628_stream_state_read(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct rk628_csi *csi = g_csi;

	dev_dbg(csi->dev, "%s: state: %d\n", __func__, csi->stream_state);
	return sprintf(buf, "%d\n", csi->stream_state);
}

static ssize_t rk628_stream_state_write(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct rk628_csi *csi = g_csi;
	u32 state = 0;
	int ret;

	if (csi->nosignal) {
		dev_dbg(csi->dev, "%s: nosignal, no need to set stream!\n", __func__);
		return count;
	}

	ret = kstrtouint(buf, 2, &state);
	if (!ret) {
		dev_dbg(csi->dev, "%s: state: %d\n", __func__, state);
		csi->stream_state = state;
		if (csi->stream_state == 0)
			enable_stream(csi, false);
		else
			enable_stream(csi, true);
	} else {
		dev_err(csi->dev, "%s: write stream state failed!!!\n", __func__);
	}

	return count;
}

static CLASS_ATTR(resolution, 0444, rk628_resolution_read, NULL);
static CLASS_ATTR(streamen, 0664, rk628_stream_state_read, rk628_stream_state_write);
#endif

static bool tx_5v_power_present(struct rk628_csi *csi)
{
	bool ret;
	int val, i, cnt;

	/* Direct Mode */
	if (!csi->plugin_det_gpio)
		return true;

	cnt = 0;
	for (i = 0; i < 5; i++) {
		val = gpiod_get_value(csi->plugin_det_gpio);
		if (val > 0)
			cnt++;
		usleep_range(500, 600);
	}

	ret = (cnt >= 3) ? true : false;
	dev_dbg(csi->dev, "%s: %d\n", __func__, ret);

	return ret;
}

static inline bool no_signal(struct rk628_csi *csi)
{
	dev_dbg(csi->dev, "%s no signal:%d\n", __func__, csi->nosignal);

	return csi->nosignal;
}

#if 0
static inline bool audio_present(struct rk628_csi *csi)
{
	return rk628_hdmirx_audio_present(csi->audio_info);
}

static int get_audio_sampling_rate(struct rk628_csi *csi)
{
	if (no_signal(csi))
		return 0;

	return rk628_hdmirx_audio_fs(csi->audio_info);
}
#endif

static void rk628_hdmirx_ctrl_enable(struct rk628_csi *csi, int en)
{
	u32 mask;

	if (en) {
		/* don't enable audio until N CTS updated */
		mask = HDMI_ENABLE_MASK;
		dev_dbg(csi->dev, "%s: %#x %d\n", __func__, mask, en);
		rk628_i2c_update_bits(csi->rk628, HDMI_RX_DMI_DISABLE_IF,
				   mask, HDMI_ENABLE(1) | AUD_ENABLE(1));
	} else {
		mask = AUD_ENABLE_MASK | HDMI_ENABLE_MASK;
		dev_dbg(csi->dev, "%s: %#x %d\n", __func__, mask, en);
		rk628_i2c_update_bits(csi->rk628, HDMI_RX_DMI_DISABLE_IF,
				   mask, HDMI_ENABLE(0) | AUD_ENABLE(0));
	}
}

static int rk628_csi_get_detected_timings(struct rk628_csi *csi,
				     struct v4l2_dv_timings *timings)
{
	struct v4l2_bt_timings *bt = &timings->bt;
	u32 hact, vact, htotal, vtotal, fps, status;
	u32 val;
	u32 modetclk_cnt_hs, modetclk_cnt_vs, hs, vs;
	u32 hofs_pix, hbp, hfp, vbp, vfp;
	u32 tmds_clk, tmdsclk_cnt;
	u64 tmp_data;
	int retry = 0;

__retry:
	memset(timings, 0, sizeof(struct v4l2_dv_timings));
	timings->type = V4L2_DV_BT_656_1120;
	rk628_i2c_read(csi->rk628, HDMI_RX_SCDC_REGS1, &val);
	status = val;

	rk628_i2c_read(csi->rk628, HDMI_RX_MD_STS, &val);
	bt->interlaced = val & ILACE_STS ?
		V4L2_DV_INTERLACED : V4L2_DV_PROGRESSIVE;

	rk628_i2c_read(csi->rk628, HDMI_RX_MD_HACT_PX, &val);
	hact = val & 0xffff;
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_VAL, &val);
	vact = val & 0xffff;
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_HT1, &val);
	htotal = (val >> 16) & 0xffff;
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_VTL, &val);
	vtotal = val & 0xffff;
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_HT1, &val);
	hofs_pix = val & 0xffff;
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_VOL, &val);
	vbp = (val & 0xffff) + 1;

	rk628_i2c_read(csi->rk628, HDMI_RX_HDMI_CKM_RESULT, &val);
	tmdsclk_cnt = val & 0xffff;
	tmp_data = tmdsclk_cnt;
	tmp_data = ((tmp_data * MODETCLK_HZ) + MODETCLK_CNT_NUM / 2);
	do_div(tmp_data, MODETCLK_CNT_NUM);
	tmds_clk = tmp_data;
	if (!(htotal * vtotal)) {
		dev_err(csi->dev, "timing err, htotal:%d, vtotal:%d\n",
				htotal, vtotal);
		if (retry++ < 5)
			goto __retry;

		goto TIMING_ERR;
	}
	fps = (tmds_clk + (htotal * vtotal) / 2) / (htotal * vtotal);

	rk628_i2c_read(csi->rk628, HDMI_RX_MD_HT0, &val);
	modetclk_cnt_hs = val & 0xffff;
	hs = (tmdsclk_cnt * modetclk_cnt_hs + MODETCLK_CNT_NUM / 2) /
		MODETCLK_CNT_NUM;

	rk628_i2c_read(csi->rk628, HDMI_RX_MD_VSC, &val);
	modetclk_cnt_vs = val & 0xffff;
	vs = (tmdsclk_cnt * modetclk_cnt_vs + MODETCLK_CNT_NUM / 2) /
		MODETCLK_CNT_NUM;
	vs = (vs + htotal / 2) / htotal;

	if ((hofs_pix < hs) || (htotal < (hact + hofs_pix)) ||
			(vtotal < (vact + vs + vbp))) {
		dev_err(csi->dev, "timing err, total:%dx%d, act:%dx%d, hofs:%d, "
				"hs:%d, vs:%d, vbp:%d\n", htotal, vtotal, hact,
				vact, hofs_pix, hs, vs, vbp);
		goto TIMING_ERR;
	}
	hbp = hofs_pix - hs;
	hfp = htotal - hact - hofs_pix;
	vfp = vtotal - vact - vs - vbp;

	dev_dbg(csi->dev, "cnt_num:%d, tmds_cnt:%d, hs_cnt:%d, vs_cnt:%d, hofs:%d\n",
			MODETCLK_CNT_NUM, tmdsclk_cnt, modetclk_cnt_hs,
			modetclk_cnt_vs, hofs_pix);

	bt->width = hact;
	bt->height = vact;
	bt->hfrontporch = hfp;
	bt->hsync = hs;
	bt->hbackporch = hbp;
	bt->vfrontporch = vfp;
	bt->vsync = vs;
	bt->vbackporch = vbp;
	bt->pixelclock = htotal * vtotal * fps;
	csi->fps = fps;

	if (bt->interlaced == V4L2_DV_INTERLACED) {
		bt->height *= 2;
		bt->il_vsync = bt->vsync + 1;
		bt->pixelclock /= 2;
	}

	dev_info(csi->dev,
		"SCDC_REGS1:%#x, act:%dx%d, total:%dx%d, fps:%d, pixclk:%llu\n",
		status, hact, vact, htotal, vtotal, fps, bt->pixelclock);
	dev_info(csi->dev,
		"hfp:%d, hs:%d, hbp:%d, vfp:%d, vs:%d, vbp:%d, interlace:%d\n",
		bt->hfrontporch, bt->hsync, bt->hbackporch, bt->vfrontporch,
		bt->vsync, bt->vbackporch, bt->interlaced);

	csi->src_timings = *timings;
	if (csi->scaler_en)
		*timings = csi->timings;

	return 0;

TIMING_ERR:
	return -ENOLCK;
}

static void rk628_hdmirx_config_all(struct rk628_csi *csi)
{
	int ret;

	rk628_hdmirx_controller_setup(csi->rk628);
	ret = rk628_hdmirx_phy_setup(csi);
	if (ret >= 0) {
		rk628_csi_format_change(csi);
		csi->nosignal = false;
	}
}

static void rk628_csi_delayed_work_enable_hotplug(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct rk628_csi *csi = container_of(dwork, struct rk628_csi,
			delayed_work_enable_hotplug);
	bool plugin;

	mutex_lock(&csi->confctl_mutex);
	csi->avi_rcv_rdy = false;
	plugin = tx_5v_power_present(csi);
	dev_dbg(csi->dev, "%s: 5v_det:%d\n", __func__, plugin);
	if (plugin) {
		rk628_csi_enable_interrupts(csi, false);
		rk628_hdmirx_audio_setup(csi->audio_info);
		rk628_hdmirx_set_hdcp(csi->rk628, &csi->hdcp, csi->enable_hdcp);
		rk628_hdmirx_hpd_ctrl(csi, true);
		rk628_hdmirx_config_all(csi);
		rk628_csi_enable_interrupts(csi, true);
		rk628_i2c_update_bits(csi->rk628, GRF_SYSTEM_CON0,
				SW_I2S_DATA_OEN_MASK, SW_I2S_DATA_OEN(0));
	} else {
		rk628_csi_enable_interrupts(csi, false);
		enable_stream(csi, false);
		cancel_delayed_work(&csi->delayed_work_res_change);
		rk628_hdmirx_audio_cancel_work_audio(csi->audio_info, true);
		rk628_hdmirx_hpd_ctrl(csi, false);
		rk628_hdmirx_phy_power_off(csi);
		rk628_hdmirx_controller_reset(csi);
		csi->nosignal = true;
	}
	mutex_unlock(&csi->confctl_mutex);
}

static int rk628_check_resulotion_change(struct rk628_csi *csi)
{
	u32 val;
	u32 htotal, vtotal;
	u32 old_htotal, old_vtotal;
	struct v4l2_bt_timings *bt = &csi->src_timings.bt;

	rk628_i2c_read(csi->rk628, HDMI_RX_MD_HT1, &val);
	htotal = (val >> 16) & 0xffff;
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_VTL, &val);
	vtotal = val & 0xffff;

	old_htotal = bt->hfrontporch + bt->hsync + bt->width + bt->hbackporch;
	old_vtotal = bt->vfrontporch + bt->vsync + bt->height + bt->vbackporch;

	dev_dbg(csi->rk628->dev, "new mode: %d x %d\n", htotal, vtotal);
	dev_dbg(csi->rk628->dev, "old mode: %d x %d\n", old_htotal, old_vtotal);

	if (htotal != old_htotal || vtotal != old_vtotal)
		return 1;

	return 0;
}

static void rk628_delayed_work_res_change(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct rk628_csi *csi = container_of(dwork, struct rk628_csi,
			delayed_work_res_change);
	bool plugin;

	mutex_lock(&csi->confctl_mutex);
	csi->avi_rcv_rdy = false;
	plugin = tx_5v_power_present(csi);
	dev_dbg(csi->dev, "%s: 5v_det:%d\n", __func__, plugin);
	if (plugin) {
		if (rk628_check_resulotion_change(csi)) {
			dev_dbg(csi->dev, "res change, recfg ctrler and phy!\n");
			rk628_hdmirx_audio_cancel_work_audio(csi->audio_info, true);
			rk628_hdmirx_phy_power_off(csi);
			rk628_hdmirx_controller_reset(csi);
			rk628_hdmirx_audio_setup(csi->audio_info);
			rk628_hdmirx_set_hdcp(csi->rk628, &csi->hdcp, csi->enable_hdcp);
			rk628_hdmirx_hpd_ctrl(csi, true);
			rk628_hdmirx_config_all(csi);
			rk628_csi_enable_interrupts(csi, true);
			rk628_i2c_update_bits(csi->rk628, GRF_SYSTEM_CON0,
					      SW_I2S_DATA_OEN_MASK,
					      SW_I2S_DATA_OEN(0));
		} else {
			rk628_csi_format_change(csi);
			csi->nosignal = false;
			rk628_csi_enable_interrupts(csi, true);
		}
	}
	mutex_unlock(&csi->confctl_mutex);
}

static void rk628_hdmirx_hpd_ctrl(struct rk628_csi *csi, bool en)
{
	u8 en_level, set_level;

	dev_dbg(csi->dev, "%s: %sable, hpd invert:%d\n", __func__,
			en ? "en" : "dis", csi->hpd_output_inverted);
	en_level = csi->hpd_output_inverted ? 0 : 1;
	set_level = en ? en_level : !en_level;
	rk628_i2c_update_bits(csi->rk628, HDMI_RX_HDMI_SETUP_CTRL,
			HOT_PLUG_DETECT_MASK, HOT_PLUG_DETECT(set_level));
}

static void rk62_csi_reset(struct rk628_csi *csi)
{
	rk628_control_assert(csi->rk628, RGU_CSI);
	udelay(10);
	rk628_control_deassert(csi->rk628, RGU_CSI);

	rk628_i2c_write(csi->rk628, CSITX_SYS_CTRL0_IMD, 0x1);
	usleep_range(1000, 1000);
	rk628_i2c_write(csi->rk628, CSITX_SYS_CTRL0_IMD, 0x0);
}

static void enable_csitx(struct rk628_csi *csi)
{
	u32 i, ret, val;

	for (i = 0; i < CSITX_ERR_RETRY_TIMES; i++) {
		rk628_csi_set_csi(csi);
		rk628_i2c_update_bits(csi->rk628, CSITX_CSITX_EN,
					DPHY_EN_MASK |
					CSITX_EN_MASK,
					DPHY_EN(1) |
					CSITX_EN(1));
		rk628_i2c_write(csi->rk628, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
		msleep(40);
		rk628_i2c_write(csi->rk628, CSITX_ERR_INTR_CLR_IMD, 0xffffffff);
		rk628_i2c_update_bits(csi->rk628, CSITX_SYS_CTRL1,
				BYPASS_SELECT_MASK, BYPASS_SELECT(0));
		rk628_i2c_write(csi->rk628, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
		msleep(40);
		ret = rk628_i2c_read(csi->rk628, CSITX_ERR_INTR_RAW_STATUS_IMD, &val);
		if (!ret && !val)
			break;

		dev_err(csi->dev, "%s csitx err, retry:%d, err status:%#x, ret:%d\n",
				__func__, i, val, ret);
	}

}

static void enable_stream(struct rk628_csi *csi, bool en)
{

	dev_dbg(csi->dev, "%s: %sable\n", __func__, en ? "en" : "dis");
	if (en) {
		rk628_hdmirx_vid_enable(csi, true);
		enable_csitx(csi);
	} else {
		rk628_hdmirx_vid_enable(csi, false);
		rk628_i2c_update_bits(csi->rk628, CSITX_CSITX_EN,
					DPHY_EN_MASK |
					CSITX_EN_MASK,
					DPHY_EN(0) |
					CSITX_EN(0));
		rk628_i2c_write(csi->rk628, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
	}
}

static void rk628_post_process_setup(struct rk628_csi *csi)
{
	struct v4l2_bt_timings *bt = &csi->src_timings.bt;
	struct v4l2_bt_timings *dst_bt = &csi->timings.bt;
	struct videomode src, dst;
	u64 dst_pclk;

	src.hactive = bt->width;
	src.hfront_porch = bt->hfrontporch;
	src.hsync_len = bt->hsync;
	src.hback_porch = bt->hbackporch;
	src.vactive = bt->height;
	src.vfront_porch = bt->vfrontporch;
	src.vsync_len = bt->vsync;
	src.vback_porch = bt->vbackporch;
	src.pixelclock = bt->pixelclock;
	src.flags = 0;
	if (bt->interlaced == V4L2_DV_INTERLACED)
		src.flags |= DISPLAY_FLAGS_INTERLACED;
	if (!src.pixelclock) {
		enable_stream(csi, false);
		csi->nosignal = true;
		schedule_delayed_work(&csi->delayed_work_enable_hotplug, HZ / 20);
		return;
	}

	dst.hactive = dst_bt->width;
	dst.hfront_porch = dst_bt->hfrontporch;
	dst.hsync_len = dst_bt->hsync;
	dst.hback_porch = dst_bt->hbackporch;
	dst.vactive = dst_bt->height;
	dst.vfront_porch = dst_bt->vfrontporch;
	dst.vsync_len = dst_bt->vsync;
	dst.vback_porch = dst_bt->vbackporch;
	dst.pixelclock = dst_bt->pixelclock;

	rk628_post_process_en(csi->rk628, &src, &dst, &dst_pclk);
	dst_bt->pixelclock = dst_pclk;
}

static void rk628_csi_set_csi(struct rk628_csi *csi)
{
	u8 video_fmt;
	u8 lanes = csi->csi_lanes_in_use;
	u8 lane_num;
	u8 dphy_lane_en;
	u32 wc_usrdef, val;
	int avi_rdy;

	lane_num = lanes - 1;
	dphy_lane_en = (1 << (lanes + 1)) - 1;
	wc_usrdef = csi->timings.bt.width * 2;

	rk62_csi_reset(csi);
	rk628_post_process_setup(csi);

	if (csi->txphy_pwron) {
		dev_dbg(csi->dev, "%s: txphy already power on, power off\n",
			__func__);
		mipi_dphy_power_off(csi);
		csi->txphy_pwron = false;
	}

	mipi_dphy_power_on(csi);
	csi->txphy_pwron = true;
	dev_dbg(csi->dev, "%s: txphy power on!\n", __func__);
	usleep_range(1000, 1500);

	rk628_i2c_update_bits(csi->rk628, CSITX_CSITX_EN,
			VOP_UV_SWAP_MASK |
			VOP_YUV422_EN_MASK |
			VOP_P2_EN_MASK |
			LANE_NUM_MASK |
			DPHY_EN_MASK |
			CSITX_EN_MASK,
			VOP_UV_SWAP(1) |
			VOP_YUV422_EN(1) |
			VOP_P2_EN(1) |
			LANE_NUM(lane_num) |
			DPHY_EN(0) |
			CSITX_EN(0));
	rk628_i2c_update_bits(csi->rk628, CSITX_SYS_CTRL1,
			BYPASS_SELECT_MASK,
			BYPASS_SELECT(1));
	rk628_i2c_write(csi->rk628, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
	rk628_i2c_write(csi->rk628, CSITX_SYS_CTRL2, VOP_WHOLE_FRM_EN | VSYNC_ENABLE);
	rk628_i2c_update_bits(csi->rk628, CSITX_SYS_CTRL3_IMD,
			CONT_MODE_CLK_CLR_MASK |
			CONT_MODE_CLK_SET_MASK |
			NON_CONTINOUS_MODE_MASK,
			CONT_MODE_CLK_CLR(0) |
			CONT_MODE_CLK_SET(0) |
			NON_CONTINOUS_MODE(1));

	rk628_i2c_write(csi->rk628, CSITX_VOP_PATH_CTRL,
			VOP_WC_USERDEFINE(wc_usrdef) |
			VOP_DT_USERDEFINE(YUV422_8BIT) |
			VOP_PIXEL_FORMAT(0) |
			VOP_WC_USERDEFINE_EN(1) |
			VOP_DT_USERDEFINE_EN(1) |
			VOP_PATH_EN(1));
	rk628_i2c_update_bits(csi->rk628, CSITX_DPHY_CTRL,
				CSI_DPHY_EN_MASK,
				CSI_DPHY_EN(dphy_lane_en));
	rk628_i2c_write(csi->rk628, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
	dev_dbg(csi->dev, "%s csi cofig done\n", __func__);

	mutex_lock(&csi->confctl_mutex);
	avi_rdy = rk628_is_avi_ready(csi->rk628, csi->avi_rcv_rdy);
	mutex_unlock(&csi->confctl_mutex);

	rk628_i2c_read(csi->rk628, HDMI_RX_PDEC_AVI_PB, &val);
	video_fmt = (val & VIDEO_FORMAT_MASK) >> 5;
	dev_dbg(csi->dev, "%s PDEC_AVI_PB:%#x, video format:%d\n",
			__func__, val, video_fmt);
	if (video_fmt) {
		/* yuv data: cfg SW_YUV2VYU_SWP */
		rk628_i2c_write(csi->rk628, GRF_CSC_CTRL_CON,
				SW_YUV2VYU_SWP(1) |
				SW_R2Y_EN(0));
	} else {
		/* rgb data: cfg SW_R2Y_EN */
		rk628_i2c_write(csi->rk628, GRF_CSC_CTRL_CON,
				SW_YUV2VYU_SWP(0) |
				SW_R2Y_EN(1));
	}

	/* if avi packet is not stable, reset ctrl*/
	if (!avi_rdy)
		schedule_delayed_work(&csi->delayed_work_enable_hotplug, HZ / 20);
}

static int rk628_hdmirx_phy_power_on(struct rk628_csi *csi)
{
	int ret, f;

	/* Bit31 is used to distinguish HDMI cable mode and direct connection
	 * mode in the rk628_combrxphy driver.
	 * Bit31: 0 -direct connection mode;
	 *        1 -cable mode;
	 * The cable mode is to know the input clock frequency through cdr_mode
	 * in the rk628_combrxphy driver, and the cable mode supports up to
	 * 297M, so 297M is passed uniformly here.
	 */
	f = 297000 | BIT(31);

	if (csi->rxphy_pwron) {
		dev_dbg(csi->dev, "rxphy already power on, power off!\n");
		ret = rk628_rxphy_power_off(csi->rk628);
		if (ret)
			dev_err(csi->dev, "hdmi rxphy power off failed!\n");
		else
			csi->rxphy_pwron = false;
		usleep_range(100, 100);
	}

	if (csi->rxphy_pwron == false) {
		rk628_hdmirx_ctrl_enable(csi, 0);
		ret = rk628_rxphy_power_on(csi->rk628, f);
		if (ret) {
			csi->rxphy_pwron = false;
			dev_err(csi->dev, "hdmi rxphy power on failed\n");
		} else {
			csi->rxphy_pwron = true;
		}
		rk628_hdmirx_ctrl_enable(csi, 1);
		msleep(100);
	}

	return ret;
}

static int rk628_hdmirx_phy_power_off(struct rk628_csi *csi)
{
	if (csi->rxphy_pwron) {
		dev_dbg(csi->dev, "rxphy power off!\n");
		rk628_rxphy_power_off(csi->rk628);
		csi->rxphy_pwron = false;
	}
	usleep_range(100, 100);
	return 0;
}

static void rk628_hdmirx_vid_enable(struct rk628_csi *csi, bool en)
{
	dev_dbg(csi->dev, "%s: %sable\n", __func__, en ? "en" : "dis");
	if (en) {
		if (!csi->i2s_enable_default)
			rk628_hdmirx_audio_i2s_ctrl(csi->audio_info, true);
		rk628_i2c_update_bits(csi->rk628, HDMI_RX_DMI_DISABLE_IF,
				      VID_ENABLE_MASK, VID_ENABLE(1));
	} else {
		if (!csi->i2s_enable_default)
			rk628_hdmirx_audio_i2s_ctrl(csi->audio_info, false);
		rk628_i2c_update_bits(csi->rk628, HDMI_RX_DMI_DISABLE_IF,
				      VID_ENABLE_MASK, VID_ENABLE(0));
	}
}

static void rk628_hdmirx_controller_reset(struct rk628_csi *csi)
{
	rk628_control_assert(csi->rk628, RGU_HDMIRX_PON);
	udelay(10);
	rk628_control_deassert(csi->rk628, RGU_HDMIRX_PON);
	udelay(10);
	rk628_i2c_write(csi->rk628, HDMI_RX_DMI_SW_RST, 0x000101ff);
	rk628_i2c_write(csi->rk628, HDMI_RX_DMI_DISABLE_IF, 0x00000000);
	rk628_i2c_write(csi->rk628, HDMI_RX_DMI_DISABLE_IF, 0x0000017f);
	rk628_i2c_write(csi->rk628, HDMI_RX_DMI_DISABLE_IF, 0x0001017f);
}

static bool rk628_rcv_supported_res(struct rk628_csi *csi, u32 width,
		u32 height)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if ((supported_modes[i].width == width) &&
		    (supported_modes[i].height == height)) {
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes)) {
		dev_err(csi->dev, "%s do not support res wxh: %dx%d\n",
				__func__, width, height);
		return false;
	} else {
		return true;
	}
}

static int rk628_hdmirx_phy_setup(struct rk628_csi *csi)
{
	u32 i, cnt, val;
	u32 width, height, frame_width, frame_height, status;
	int ret;

	for ( i = 0; i < RXPHY_CFG_MAX_TIMES; i++) {
		ret = rk628_hdmirx_phy_power_on(csi);
		if (ret < 0) {
			msleep(50);
			continue;
		}
		cnt = 0;

		do {
			cnt++;
			rk628_i2c_read(csi->rk628, HDMI_RX_MD_HACT_PX, &val);
			width = val & 0xffff;
			rk628_i2c_read(csi->rk628, HDMI_RX_MD_VAL, &val);
			height = val & 0xffff;
			rk628_i2c_read(csi->rk628, HDMI_RX_MD_HT1, &val);
			frame_width = (val >> 16) & 0xffff;
			rk628_i2c_read(csi->rk628, HDMI_RX_MD_VTL, &val);
			frame_height = val & 0xffff;
			rk628_i2c_read(csi->rk628, HDMI_RX_SCDC_REGS1, &val);
			status = val;
			dev_dbg(csi->dev,
				"%s read wxh:%dx%d, total:%dx%d, SCDC_REGS1:%#x, cnt:%d\n",
				__func__, width, height, frame_width,
				frame_height, status, cnt);

			rk628_i2c_read(csi->rk628, HDMI_RX_PDEC_STS, &val);
			if (val & DVI_DET)
				dev_info(csi->dev, "DVI mode detected\n");

			if (!tx_5v_power_present(csi)) {
				dev_info(csi->dev, "HDMI pull out, return!\n");
				return -1;
			}

			if (cnt >= 15)
				break;
		} while(((status & 0xfff) != 0xf00) ||
				(!rk628_rcv_supported_res(csi, width, height)));

		if (((status & 0xfff) != 0xf00) ||
				(!rk628_rcv_supported_res(csi, width, height))) {
			dev_err(csi->dev, "%s hdmi rxphy lock failed, retry:%d\n",
					__func__, i);
			continue;
		} else {
			break;
		}
	}

	if (i == RXPHY_CFG_MAX_TIMES) {
		return -1;
	}

	return 0;
}

static void rk628_csi_initial_setup(struct rk628_csi *csi)
{
	struct v4l2_subdev_edid def_edid;

	// TODO: clk reset
	rk628_clk_mux_testout(csi->rk628, CGU_CLK_HDMIRX_AUD);
	rk628_i2c_write(csi->rk628, GRF_GPIO1AB_SEL_CON, HIWORD_UPDATE(0x1, 0, 0)); /* enable gpio1a0 TESTclkout */
	/* selete int io function */
	rk628_i2c_write(csi->rk628, GRF_GPIO3AB_SEL_CON, 0x30002000);
	rk628_i2c_write(csi->rk628, GRF_GPIO1AB_SEL_CON, HIWORD_UPDATE(0x7, 10, 8));
	/* I2S_SCKM0 */
	rk628_i2c_write(csi->rk628, GRF_GPIO0AB_SEL_CON, HIWORD_UPDATE(0x1, 2, 2));
	/* I2SLR_M0 */
	rk628_i2c_write(csi->rk628, GRF_GPIO0AB_SEL_CON, HIWORD_UPDATE(0x1, 3, 3));
	/* I2SM0D0 */
	rk628_i2c_write(csi->rk628, GRF_GPIO0AB_SEL_CON, HIWORD_UPDATE(0x1, 5, 4));
	/* hdmirx int en */
	rk628_i2c_write(csi->rk628, GRF_INTR0_EN, 0x01000100);

	udelay(10);
	rk628_control_assert(csi->rk628, RGU_HDMIRX);
	rk628_control_assert(csi->rk628, RGU_HDMIRX_PON);
	rk628_control_assert(csi->rk628, RGU_CSI);
	udelay(10);
	rk628_control_deassert(csi->rk628, RGU_HDMIRX);
	rk628_control_deassert(csi->rk628, RGU_HDMIRX_PON);
	rk628_control_deassert(csi->rk628, RGU_CSI);
	udelay(10);

	rk628_i2c_update_bits(csi->rk628, GRF_SYSTEM_CON0,
			SW_INPUT_MODE_MASK |
			SW_OUTPUT_MODE_MASK |
			SW_EFUSE_HDCP_EN_MASK |
			SW_HSYNC_POL_MASK |
			SW_VSYNC_POL_MASK,
			SW_INPUT_MODE(INPUT_MODE_HDMI) |
			SW_OUTPUT_MODE(OUTPUT_MODE_CSI) |
			SW_EFUSE_HDCP_EN(0) |
			SW_HSYNC_POL(1) |
			SW_VSYNC_POL(1));
	rk628_hdmirx_controller_reset(csi);

	def_edid.pad = 0;
	def_edid.start_block = 0;
	def_edid.blocks = 2;
	def_edid.edid = edid_init_data;
	rk628_csi_s_edid(csi, &def_edid);
	rk628_hdmirx_set_hdcp(csi->rk628, &csi->hdcp, false);

	mipi_dphy_reset(csi->rk628);
	mipi_dphy_power_on(csi);
	csi->txphy_pwron = true;
	if (tx_5v_power_present(csi))
		schedule_delayed_work(&csi->delayed_work_enable_hotplug, 1000);
}

static void rk628_csi_format_change(struct rk628_csi *csi)
{
	struct v4l2_dv_timings timings;

	if (rk628_csi_get_detected_timings(csi, &timings) == 0) {
		rk628_csi_s_dv_timings(csi, &timings);
	}
}

static void rk628_csi_enable_interrupts(struct rk628_csi *csi, bool en)
{
	u32 pdec_ien, md_ien;
	u32 pdec_mask = 0, md_mask = 0;

	pdec_mask |= AVI_RCV_ENSET;
	md_mask = VACT_LIN_ENSET | HACT_PIX_ENSET | HS_CLK_ENSET |
		  DE_ACTIVITY_ENSET | VS_ACT_ENSET | HS_ACT_ENSET;
	dev_dbg(csi->dev, "%s: %sable\n", __func__, en ? "en" : "dis");
	/* clr irq */
	rk628_i2c_write(csi->rk628, HDMI_RX_MD_ICLR, md_mask);
	rk628_i2c_write(csi->rk628, HDMI_RX_PDEC_ICLR, pdec_mask);
	if (en) {
		rk628_i2c_write(csi->rk628, HDMI_RX_MD_IEN_SET, md_mask);
		rk628_i2c_write(csi->rk628, HDMI_RX_PDEC_IEN_SET, pdec_mask);
		csi->vid_ints_en = true;
	} else {
		rk628_i2c_write(csi->rk628, HDMI_RX_MD_IEN_CLR, md_mask);
		rk628_i2c_write(csi->rk628, HDMI_RX_PDEC_IEN_CLR, pdec_mask);
		rk628_i2c_write(csi->rk628, HDMI_RX_AUD_FIFO_IEN_CLR, 0x1f);
		csi->vid_ints_en = false;
	}
	usleep_range(5000, 5000);
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_IEN, &md_ien);
	rk628_i2c_read(csi->rk628, HDMI_RX_PDEC_IEN, &pdec_ien);
	dev_dbg(csi->dev, "%s MD_IEN:%#x, PDEC_IEN:%#x\n", __func__, md_ien, pdec_ien);
}

static int rk628_csi_isr(struct rk628_csi *csi, u32 status, bool *handled)
{
	u32 md_ints, pdec_ints, fifo_ints, hact, vact;
	bool plugin;
	void *audio_info = csi->audio_info;

	if (handled == NULL) {
		dev_err(csi->dev, "handled NULL, err return!\n");
		return -EINVAL;
	}
	rk628_i2c_read(csi->rk628, HDMI_RX_PDEC_ISTS, &pdec_ints);
	if (rk628_audio_ctsnints_enabled(audio_info)) {
		if (pdec_ints & (ACR_N_CHG_ICLR | ACR_CTS_CHG_ICLR)) {
			rk628_csi_isr_ctsn(audio_info, pdec_ints);
			pdec_ints &= ~(ACR_CTS_CHG_ICLR | ACR_CTS_CHG_ICLR);
			*handled = true;
		}
	}
	if (rk628_audio_fifoints_enabled(audio_info)) {
		rk628_i2c_read(csi->rk628, HDMI_RX_AUD_FIFO_ISTS, &fifo_ints);
		if (fifo_ints & 0x18) {
			rk628_csi_isr_fifoints(audio_info, fifo_ints);
			*handled = true;
		}
	}
	if (csi->vid_ints_en) {
		rk628_i2c_read(csi->rk628, HDMI_RX_MD_ISTS, &md_ints);
		plugin = tx_5v_power_present(csi);
		dev_dbg(csi->dev, "%s: md_ints: %#x, pdec_ints:%#x, plugin: %d\n",
			__func__, md_ints, pdec_ints, plugin);

		if ((md_ints & (VACT_LIN_ISTS | HACT_PIX_ISTS |
				HS_CLK_ISTS | DE_ACTIVITY_ISTS |
				VS_ACT_ISTS | HS_ACT_ISTS))
				&& plugin) {

			rk628_i2c_read(csi->rk628, HDMI_RX_MD_HACT_PX, &hact);
			rk628_i2c_read(csi->rk628, HDMI_RX_MD_VAL, &vact);
			dev_dbg(csi->dev, "%s: HACT:%#x, VACT:%#x\n",
					__func__, hact, vact);

			rk628_csi_enable_interrupts(csi, false);
			enable_stream(csi, false);
			csi->nosignal = true;
			schedule_delayed_work(&csi->delayed_work_res_change, HZ / 2);

			dev_dbg(csi->dev, "%s: hact/vact change, md_ints: %#x\n",
					__func__, (u32)(md_ints & (VACT_LIN_ISTS | HACT_PIX_ISTS)));
			*handled = true;
		}

		if ((pdec_ints & AVI_RCV_ISTS) && plugin) {
			dev_dbg(csi->dev, "%s: AVI RCV INT!\n", __func__);
			csi->avi_rcv_rdy = true;
			/* After get the AVI_RCV interrupt state, disable interrupt. */
			rk628_i2c_write(csi->rk628, HDMI_RX_PDEC_IEN_CLR, AVI_RCV_ISTS);

			*handled = true;
		}
	}
	if (*handled != true)
		dev_dbg(csi->dev, "%s: unhandled interrupt!\n", __func__);

	/* clear interrupts */
	rk628_i2c_write(csi->rk628, HDMI_RX_MD_ICLR, 0xffffffff);
	rk628_i2c_write(csi->rk628, HDMI_RX_PDEC_ICLR, 0xffffffff);
	rk628_i2c_write(csi->rk628, GRF_INTR0_CLR_EN, 0x01000100);

	return 0;
}

static irqreturn_t rk628_csi_irq_handler(int irq, void *dev_id)
{
	struct rk628_csi *csi = dev_id;
	bool handled = false;

	rk628_csi_isr(csi, 0, &handled);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}
#ifdef KERNEL_VERSION_4_19
static void rk628_csi_irq_poll_timer(struct timer_list *t)
{
	struct rk628_csi *csi = from_timer(csi, t, timer);

	schedule_work(&csi->work_i2c_poll);
	mod_timer(&csi->timer, jiffies + msecs_to_jiffies(POLL_INTERVAL_MS));
}
#else
static void rk628_csi_irq_poll_timer(unsigned long arg)
{
	struct rk628_csi *csi = (struct rk628_csi *)arg;

	schedule_work(&csi->work_i2c_poll);
	mod_timer(&csi->timer, jiffies + msecs_to_jiffies(POLL_INTERVAL_MS));
}
#endif

static void rk628_csi_work_i2c_poll(struct work_struct *work)
{
	struct rk628_csi *csi = container_of(work, struct rk628_csi,
			work_i2c_poll);

	rk628_csi_format_change(csi);
}

#if 0
static int rk628_csi_g_input_status(struct rk628_csi *csi, u32 *status)
{
	*status = 0;
	*status |= no_signal(csi) ? V4L2_IN_ST_NO_SIGNAL : 0;

	dev_dbg(csi->dev, "%s: status = 0x%x\n", __func__, *status);

	return 0;
}
#endif

static int rk628_csi_s_dv_timings(struct rk628_csi *csi,
		struct v4l2_dv_timings *timings)
{
	csi->timings = *timings;
	enable_stream(csi, false);

	return 0;
}

static int rk628_csi_s_edid(struct rk628_csi *csi,
				struct v4l2_subdev_edid *edid)
{
	u16 edid_len = edid->blocks * EDID_BLOCK_SIZE;
	u32 i, val;

	dev_dbg(csi->dev, "%s, pad %d, start block %d, blocks %d\n",
		 __func__, edid->pad, edid->start_block, edid->blocks);

	memset(edid->reserved, 0, sizeof(edid->reserved));

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->start_block != 0)
		return -EINVAL;

	if (edid->blocks > EDID_NUM_BLOCKS_MAX) {
		edid->blocks = EDID_NUM_BLOCKS_MAX;
		return -E2BIG;
	}

	rk628_hdmirx_hpd_ctrl(csi, false);

	if (edid->blocks == 0) {
		csi->edid_blocks_written = 0;
		return 0;
	}

	/* edid access by apb when write, i2c slave addr: 0x0 */
	rk628_i2c_update_bits(csi->rk628, GRF_SYSTEM_CON0,
			SW_ADAPTER_I2CSLADR_MASK |
			SW_EDID_MODE_MASK,
			SW_ADAPTER_I2CSLADR(0) |
			SW_EDID_MODE(1));

	for (i = 0; i < edid_len; i++) {
		rk628_i2c_write(csi->rk628, EDID_BASE + i * 4, edid->edid[i]);
	}

	/* read out for debug */
	if (debug) {
		printk("%s: Read EDID: ======\n", __func__);
		for (i = 0; i < edid_len; i++) {
			rk628_i2c_read(csi->rk628, EDID_BASE + i * 4, &val);
			printk("0x%02x ", val);
			if ((i + 1) % 8 == 0)
				printk("\n");
		}
		printk("%s: ======\n", __func__);
	}

	/* edid access by RX's i2c, i2c slave addr: 0x0 */
	rk628_i2c_update_bits(csi->rk628, GRF_SYSTEM_CON0,
			SW_ADAPTER_I2CSLADR_MASK |
			SW_EDID_MODE_MASK,
			SW_ADAPTER_I2CSLADR(0) |
			SW_EDID_MODE(0));
	csi->edid_blocks_written = edid->blocks;
	udelay(100);

	if (tx_5v_power_present(csi))
		rk628_hdmirx_hpd_ctrl(csi, true);

	return 0;
}

static int mipi_dphy_power_on(struct rk628_csi *csi)
{
	unsigned int val;
	u32 bus_width, mask;

	if ((csi->timings.bt.width == 3840) &&
			(csi->timings.bt.height == 2160)) {
		csi->lane_mbps = MIPI_DATARATE_MBPS_HIGH;
	} else {
		csi->lane_mbps = MIPI_DATARATE_MBPS_LOW;
	}

	bus_width =  csi->lane_mbps << 8;
	bus_width |= COMBTXPHY_MODULEA_EN;
	dev_dbg(csi->dev, "%s mipi bitrate:%llu mbps\n", __func__,
			csi->lane_mbps);
	rk628_txphy_set_bus_width(csi->rk628, bus_width);
	rk628_txphy_set_mode(csi->rk628, PHY_MODE_VIDEO_MIPI);

	mipi_dphy_init_hsfreqrange(csi->rk628, csi->lane_mbps);
	usleep_range(1500, 2000);
	rk628_txphy_power_on(csi->rk628);

	usleep_range(1500, 2000);
	mask = DPHY_PLL_LOCK;
	rk628_i2c_read(csi->rk628, CSITX_CSITX_STATUS1, &val);
	if ((val & mask) != mask) {
		dev_err(csi->dev, "PHY is not locked\n");
		return -1;
	}

	udelay(10);

	return 0;
}

static void mipi_dphy_power_off(struct rk628_csi *csi)
{
	rk628_txphy_power_off(csi->rk628);
}

static irqreturn_t plugin_detect_irq(int irq, void *dev_id)
{
	struct rk628_csi *csi = dev_id;

	/* control hpd after 50ms */
	schedule_delayed_work(&csi->delayed_work_enable_hotplug, HZ / 20);
	tx_5v_power_present(csi);

	return IRQ_HANDLED;
}

static int rk628_csi_probe_of(struct rk628_csi *csi)
{
	struct device *dev = csi->dev;
	int ret = -EINVAL;
	bool hdcp1x_enable = false, i2s_enable_default = false;
	bool scaler_en = false;

	csi->soc_24M = devm_clk_get(dev, "soc_24M");
	if (csi->soc_24M == ERR_PTR(-ENOENT))
		csi->soc_24M = NULL;
	if (IS_ERR(csi->soc_24M)) {
		ret = PTR_ERR(csi->soc_24M);
		dev_err(dev, "Unable to get soc_24M: %d\n", ret);
	}
	clk_prepare_enable(csi->soc_24M);

	csi->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						     GPIOD_OUT_LOW);
	if (IS_ERR(csi->enable_gpio)) {
		ret = PTR_ERR(csi->enable_gpio);
		dev_err(dev, "failed to request enable GPIO: %d\n", ret);
		return ret;
	}

	csi->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(csi->reset_gpio)) {
		ret = PTR_ERR(csi->reset_gpio);
		dev_err(dev, "failed to request reset GPIO: %d\n", ret);
		return ret;
	}

	csi->power_gpio = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_HIGH);
	if (IS_ERR(csi->power_gpio)) {
		dev_err(csi->dev, "failed to get power gpio\n");
		ret = PTR_ERR(csi->power_gpio);
		return ret;
	}

	csi->plugin_det_gpio = devm_gpiod_get_optional(dev, "plugin-det",
						    GPIOD_IN);
	if (IS_ERR(csi->plugin_det_gpio)) {
		dev_err(csi->dev, "failed to get hdmirx det gpio\n");
		ret = PTR_ERR(csi->plugin_det_gpio);
		return ret;
	}
	csi->hpd_output_inverted = of_property_read_bool(dev->of_node,
			"hpd-output-inverted");

	gpiod_set_value(csi->enable_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value(csi->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value(csi->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value(csi->reset_gpio, 0);
	usleep_range(10000, 11000);

	if (csi->power_gpio) {
		gpiod_set_value(csi->power_gpio, 1);
		usleep_range(500, 510);
	}

	if (of_property_read_bool(dev->of_node, "hdcp-enable"))
		hdcp1x_enable = true;

	if (of_property_read_bool(dev->of_node, "i2s-enable-default"))
		i2s_enable_default = true;

	if (of_property_read_bool(dev->of_node, "scaler-en"))
		scaler_en = true;

	csi->csi_lanes_in_use = USE_4_LANES;
	csi->enable_hdcp = hdcp1x_enable;
	csi->i2s_enable_default = i2s_enable_default;
	csi->scaler_en = scaler_en;
	if (csi->scaler_en)
		csi->timings = dst_timing;

	csi->rxphy_pwron = false;
	csi->txphy_pwron = false;
	csi->nosignal = true;
	csi->stream_state = 0;
	csi->avi_rcv_rdy = false;

	ret = 0;

	return ret;
}

#ifndef KERNEL_VERSION_4_19
static int rk628_create_class_attr(struct rk628_csi *csi)
{
	int ret = -1;

	csi->rk628_class = class_create(THIS_MODULE, "rk628csi");
	if (IS_ERR(csi->rk628_class)) {
		ret = -ENOMEM;
		dev_err(csi->dev, "failed to create rk628csi class!\n");
		return ret;
	}

	ret = class_create_file(csi->rk628_class, &class_attr_resolution);
	if (ret) {
		dev_err(csi->dev, "failed to create attr resolution\n");
		goto err_res;
	}

	ret = class_create_file(csi->rk628_class, &class_attr_streamen);
	if (ret) {
		dev_err(csi->dev, "failed to create attr streamen\n");
		goto err_stream;
	}

	return ret;

err_stream:
	class_remove_file(csi->rk628_class, &class_attr_resolution);
err_res:
	class_destroy(csi->rk628_class);
	return ret;
}
#endif


static int rk628_csi_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct rk628_csi *csi;
	struct device *dev = &client->dev;
	int err;
	u32 val;
	struct rk628 *rk628;
	unsigned long irq_flags;

	dev_info(dev, "RK628 I2C driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	if (!of_device_is_available(dev->of_node))
		return -ENODEV;

	csi = devm_kzalloc(dev, sizeof(*csi), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;

	csi->dev = dev;
	csi->i2c_client = client;
	rk628 = rk628_i2c_register(client);
	if (!rk628)
		return -ENOMEM;
	csi->rk628 = rk628;
	csi->cur_mode = &supported_modes[0];
	csi->hdmirx_irq = client->irq;

	err = rk628_csi_probe_of(csi);
	if (err) {
		dev_err(csi->dev, "rk628_csi_probe_of failed! err:%d\n", err);
		return err;
	}

	rk628_cru_initialize(csi->rk628);

	/* i2c access, read chip id*/
	err = rk628_i2c_read(csi->rk628, CSITX_CSITX_VERSION, &val);
	if (err) {
		dev_err(csi->dev, "i2c access failed! err:%d\n", err);
		return -ENODEV;
	}
	dev_dbg(csi->dev, "CSITX VERSION: %#x\n", val);
 	csi->mbus_fmt_code = MEDIA_BUS_FMT_UYVY8_2X8;

	mutex_init(&csi->confctl_mutex);

	csi->txphy = rk628_txphy_register(rk628);
	if (!csi->txphy) {
		dev_err(csi->dev, "register txphy failed\n");
		return -ENOMEM;
	}

#ifndef KERNEL_VERSION_4_19
	err = rk628_create_class_attr(csi);
	if (err) {
		dev_err(csi->dev, "create class attr failed! err:%d\n", err);
		return -ENODEV;
	}
#endif

	INIT_DELAYED_WORK(&csi->delayed_work_enable_hotplug,
			rk628_csi_delayed_work_enable_hotplug);
	INIT_DELAYED_WORK(&csi->delayed_work_res_change,
			rk628_delayed_work_res_change);
	csi->audio_info = rk628_hdmirx_audioinfo_alloc(dev,
						       &csi->confctl_mutex,
						       rk628,
						       csi->i2s_enable_default);
	if (!csi->audio_info) {
		dev_err(csi->dev, "request audio info fail\n");
		goto err_work_queues;
	}
	rk628_csi_initial_setup(csi);

	if (csi->hdmirx_irq) {
		irq_flags = irqd_get_trigger_type(irq_get_irq_data(csi->hdmirx_irq));
		dev_dbg(csi->dev, "cfg hdmirx irq, flags: %lu!\n", irq_flags);
		err = devm_request_threaded_irq(dev, csi->hdmirx_irq, NULL,
				rk628_csi_irq_handler, irq_flags |
				IRQF_ONESHOT, "rk628_csi", csi);
		if (err) {
			dev_err(csi->dev, "request rk628-csi irq failed! err:%d\n",
					err);
			goto err_work_queues;
		}
	} else {
		dev_dbg(csi->dev, "no irq, cfg poll!\n");
		INIT_WORK(&csi->work_i2c_poll,
			  rk628_csi_work_i2c_poll);
#ifdef KERNEL_VERSION_4_19
		timer_setup(&csi->timer, rk628_csi_irq_poll_timer, 0);
#else
		csi->timer.data = (unsigned long)csi;
		csi->timer.function = rk628_csi_irq_poll_timer;
#endif
		csi->timer.expires = jiffies +
			msecs_to_jiffies(POLL_INTERVAL_MS);
		add_timer(&csi->timer);
	}

	if (csi->plugin_det_gpio) {
		csi->plugin_irq = gpiod_to_irq(csi->plugin_det_gpio);
		if (csi->plugin_irq < 0) {
			dev_err(csi->dev, "failed to get plugin det irq\n");
			err = csi->plugin_irq;
			goto err_work_queues;
		}

		err = devm_request_threaded_irq(dev, csi->plugin_irq, NULL,
				plugin_detect_irq, IRQF_TRIGGER_FALLING |
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, "rk628_csi", csi);
		if (err) {
			dev_err(csi->dev, "failed to register plugin det irq (%d)\n", err);
			goto err_work_queues;
		}
	}

	dev_info(csi->dev, "%s found @ 0x%x (%s)\n", client->name,
		  client->addr << 1, client->adapter->name);
	g_csi = csi;

	return 0;

err_work_queues:
	if (!csi->hdmirx_irq)
		flush_work(&csi->work_i2c_poll);
	cancel_delayed_work(&csi->delayed_work_enable_hotplug);
	cancel_delayed_work(&csi->delayed_work_res_change);
	rk628_hdmirx_audio_destroy(csi->audio_info);
	mutex_destroy(&csi->confctl_mutex);
	return err;
}

static int rk628_csi_remove(struct i2c_client *client)
{
	struct rk628_csi *csi = i2c_get_clientdata(client);

	if (!csi->hdmirx_irq) {
		del_timer_sync(&csi->timer);
		flush_work(&csi->work_i2c_poll);
	}
	rk628_hdmirx_audio_cancel_work_audio(csi->audio_info, true);
	rk628_hdmirx_audio_cancel_work_rate_change(csi->audio_info, true);
	cancel_delayed_work_sync(&csi->delayed_work_enable_hotplug);
	cancel_delayed_work_sync(&csi->delayed_work_res_change);

	if (csi->rxphy_pwron)
		rk628_rxphy_power_off(csi->rk628);
	if (csi->txphy_pwron)
		mipi_dphy_power_off(csi);

	mutex_destroy(&csi->confctl_mutex);

	rk628_control_assert(csi->rk628, RGU_HDMIRX);
	rk628_control_assert(csi->rk628, RGU_HDMIRX_PON);
	rk628_control_assert(csi->rk628, RGU_DECODER);
	rk628_control_assert(csi->rk628, RGU_CLK_RX);
	rk628_control_assert(csi->rk628, RGU_VOP);
	rk628_control_assert(csi->rk628, RGU_CSI);

	return 0;
}

static const struct i2c_device_id rk628_csi_i2c_id[] = {
        { "rk628-hdmi2csi", 0 },
        { }
};

MODULE_DEVICE_TABLE(i2c, rk628_csi_i2c_id);

static const struct of_device_id rk628_csi_of_match[] = {
	{ .compatible = "rockchip,rk628-hdmi2csi" },
	{}
};
MODULE_DEVICE_TABLE(of, rk628_csi_of_match);

static struct i2c_driver rk628_csi_i2c_driver = {
	.driver = {
		.name = "rk628-hdmi2csi",
		.of_match_table = of_match_ptr(rk628_csi_of_match),
	},
	.id_table = rk628_csi_i2c_id,
	.probe	= rk628_csi_probe,
	.remove = rk628_csi_remove,
};

module_i2c_driver(rk628_csi_i2c_driver);

MODULE_DESCRIPTION("Rockchip RK628 HDMI to MIPI CSI-2 bridge I2C driver");
MODULE_AUTHOR("Dingxian Wen <shawn.wen@rock-chips.com>");
MODULE_AUTHOR("Shunqing Chen <csq@rock-chips.com>");
MODULE_LICENSE("GPL v2");
