/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for common functions
 *
 *  Copyright (C) 2019 Himax Corporation.
 *
 *  This software is licensed under the terms of the GNU General Public
 *  License version 2,  as published by the Free Software Foundation,  and
 *  may be copied,  distributed,  and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include "himax_ic_HX83102.h"
#include "himax_ic_incell_core.h"

#define SUPPORT_FINGER_DATA_CHECKSUM 0x0F
#define TS_WAKE_LOCK_TIMEOUT (5000)
#define FRAME_COUNT 5

#if defined(HX_TP_PROC_GUEST_INFO)
struct hx_guest_info *g_guest_info_data;
EXPORT_SYMBOL(g_guest_info_data);

char *g_guest_info_item[] = {
	"projectID", "CGColor",	 "BarCode", "Reserve1",	 "Reserve2", "Reserve3",
	"Reserve4",  "Reserve5", "VCOM",    "Vcom-3Gar", NULL,
};
#endif

/* enable debug info */
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
int g_ts_dbg = 1;
#endif

int g_mmi_refcnt;
EXPORT_SYMBOL(g_mmi_refcnt);

/*ts_work about start*/
struct himax_target_report_data *g_target_report_data;
EXPORT_SYMBOL(g_target_report_data);

static void himax_report_all_leave_event(struct himax_ts_data *ts);
/*ts_work about end*/

static bool chip_test_r_flag;
u8 HX_HW_RESET_ACTIVATE;

static uint8_t AA_press;
static uint8_t EN_NoiseFilter;
static uint8_t Last_EN_NoiseFilter;

static int p_point_num = 0xFFFF;
static int probe_fail_flag;

void himax_parse_assign_cmd(uint32_t addr, uint8_t *cmd, int len)
{
	/*D("%s: Entering!\n", __func__);*/

	switch (len) {
	case 1:
		cmd[0] = addr;
		/*D("%s: cmd[0] = 0x%02X\n", __func__, cmd[0]);*/
		break;

	case 2:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		/*D("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X\n",*/
		/*	__func__, cmd[0], cmd[1]);*/
		break;

	case 4:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		cmd[2] = (addr >> 16) % 0x100;
		cmd[3] = addr / 0x1000000;
		/*  D("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X,*/
		/*cmd[2] = 0x%02X,cmd[3] = 0x%02X\n", */
		/* __func__, cmd[0], cmd[1], cmd[2], cmd[3]);*/
		break;

	default:
		E("%s: input length fault,len = %d!\n", __func__, len);
	}
}
EXPORT_SYMBOL(himax_parse_assign_cmd);

int himax_input_register(struct himax_ts_data *ts)
{
	int ret = 0;

	ret = himax_dev_set(ts);
	if (ret < 0) {
		E("%s: input device register fail!\n", __func__);
		ret = INPUT_REGISTER_FAIL;
		goto input_device_fail;
	}

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);

	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	/* protocol type is set by OF himax,report_type: <0> => A or <1> => B */
	if (ts->protocol_type == PROTOCOL_TYPE_A) {
		/*ts->input_dev->mtsize = ts->nFinger_support;*/
		input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 3, 0,
				     0);
	} else {
		set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
		if (ts->protocol_type == PROTOCOL_TYPE_B_3PA) {
			input_mt_init_slots(ts->input_dev, ts->nFinger_support,
					    INPUT_MT_DIRECT | INPUT_MT_POINTER);
		} else {
			input_mt_init_slots(ts->input_dev, ts->nFinger_support,
					    INPUT_MT_POINTER);
		}
	}

	D("%s: input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
	  __func__, ts->pdata->abs_x_min, ts->pdata->abs_x_max,
	  ts->pdata->abs_y_min, ts->pdata->abs_y_max);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			     ts->pdata->abs_x_min, ts->pdata->abs_x_max,
			     ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			     ts->pdata->abs_y_min, ts->pdata->abs_y_max,
			     ts->pdata->abs_y_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
			     ts->pdata->abs_pressure_min,
			     ts->pdata->abs_pressure_max,
			     ts->pdata->abs_pressure_fuzz, 0);

	if (ts->pdata->protocol_type != PROTOCOL_TYPE_A) {
		input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,
				     ts->pdata->abs_pressure_min,
				     ts->pdata->abs_pressure_max,
				     ts->pdata->abs_pressure_fuzz, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
				     ts->pdata->abs_width_min,
				     ts->pdata->abs_width_max,
				     ts->pdata->abs_pressure_fuzz, 0);
	}

	if (himax_input_register_device(ts->input_dev)) {
		E("%s: input register fail\n", __func__);
		ret = INPUT_REGISTER_FAIL;
		goto input_device_fail;
	}

	if (!ts->ic_data->HX_PEN_FUNC) {
		W("%s: No PEN function, skip pen operation\n", __func__);
		goto skip_pen_operation;
	}

	set_bit(EV_SYN, ts->hx_pen_dev->evbit);
	set_bit(EV_ABS, ts->hx_pen_dev->evbit);
	set_bit(EV_KEY, ts->hx_pen_dev->evbit);

	set_bit(INPUT_PROP_DIRECT, ts->hx_pen_dev->propbit);

	input_set_abs_params(ts->hx_pen_dev, ABS_PRESSURE, 0, 4095, 0, 0);
	input_set_abs_params(ts->hx_pen_dev, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_abs_params(ts->hx_pen_dev, ABS_TILT_X, -60, 60, 0, 0);
	input_set_abs_params(ts->hx_pen_dev, ABS_TILT_Y, -60, 60, 0, 0);
	/*input_set_capability(ts->hx_pen_dev, EV_SW, SW_PEN_INSERT);*/
	input_set_capability(ts->hx_pen_dev, EV_KEY, BTN_TOUCH);
	input_set_capability(ts->hx_pen_dev, EV_KEY, BTN_STYLUS);
	input_set_capability(ts->hx_pen_dev, EV_KEY, BTN_STYLUS2);

	input_set_abs_params(ts->hx_pen_dev, ABS_X, ts->pdata->abs_x_min,
			     ts->pdata->abs_x_max, ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->hx_pen_dev, ABS_Y, ts->pdata->abs_y_min,
			     ts->pdata->abs_y_max, ts->pdata->abs_y_fuzz, 0);

	if (himax_input_register_device(ts->hx_pen_dev) == 0) {
		ret = NO_ERR;
	} else {
		E("%s: input register pen fail\n", __func__);
		ret = INPUT_REGISTER_FAIL;
		goto input_device_fail;
	}

skip_pen_operation:
	D("%s, input device registered.\n", __func__);

input_device_fail:
	return ret;
}
EXPORT_SYMBOL(himax_input_register);

static void calcDataSize(struct himax_ts_data *ts)
{
	ts->x_channel = ts->ic_data->HX_RX_NUM;
	ts->y_channel = ts->ic_data->HX_TX_NUM;
	ts->nFinger_support = ts->ic_data->HX_MAX_PT;

	ts->coord_data_size = 4 * ts->nFinger_support;
	ts->area_data_size = ((ts->nFinger_support / 4) +
			      (ts->nFinger_support % 4 ? 1 : 0)) *
			     4;
	ts->coordInfoSize = ts->coord_data_size + ts->area_data_size + 4;
	ts->raw_data_frame_size =
		128 - ts->coord_data_size - ts->area_data_size - 4 - 4 - 1;

	if (ts->raw_data_frame_size == 0) {
		E("%s: could NOT calculate!\n", __func__);
		return;
	}

	ts->raw_data_nframes =
		((uint32_t)ts->x_channel * ts->y_channel + ts->x_channel +
		 ts->y_channel) / ts->raw_data_frame_size +
				(((uint32_t)ts->x_channel * ts->y_channel +
				  ts->x_channel + ts->y_channel) %
				 ts->raw_data_frame_size) ?
			1 :
			0;

	D("%s: coord_dsz:%d,area_dsz:%d,raw_data_fsz:%d,raw_data_nframes:%d",
	  __func__, ts->coord_data_size, ts->area_data_size,
	  ts->raw_data_frame_size, ts->raw_data_nframes);
}

static void calculate_point_number(struct himax_ts_data *ts)
{
	ts->HX_TOUCH_INFO_POINT_CNT = ts->ic_data->HX_MAX_PT * 4;

	if ((ts->ic_data->HX_MAX_PT % 4) == 0)
		ts->HX_TOUCH_INFO_POINT_CNT += (ts->ic_data->HX_MAX_PT / 4) * 4;
	else
		ts->HX_TOUCH_INFO_POINT_CNT +=
			((ts->ic_data->HX_MAX_PT / 4) + 1) * 4;
}

#if defined(HX_ESD_RECOVERY)
static void himax_esd_hw_reset(struct himax_ts_data *ts)
{
	if (g_ts_dbg != 0)
		D("%s: Entering\n", __func__);

	D("%s: START_Himax TP: ESD - Reset\n", __func__);

	if (ts->in_self_test == 1) {
		D("%s: In self test , not  TP: ESD - Reset\n", __func__);
		return;
	}

	himax_mcu_esd_ic_reset(ts);

	D("%s: END_Himax TP: ESD - Reset\n", __func__);
}
#endif

int himax_report_data_init(struct himax_ts_data *ts)
{
	if (ts->hx_touch_data->hx_coord_buf != NULL) {
		kfree(ts->hx_touch_data->hx_coord_buf);
		ts->hx_touch_data->hx_coord_buf = NULL;
	}

	if (ts->hx_touch_data->hx_rawdata_buf != NULL) {
		kfree(ts->hx_touch_data->hx_rawdata_buf);
		ts->hx_touch_data->hx_rawdata_buf = NULL;
	}

	ts->hx_touch_data->touch_all_size = himax_mcu_get_touch_data_size();
	ts->hx_touch_data->raw_cnt_max = ts->ic_data->HX_MAX_PT / 4;
	ts->hx_touch_data->raw_cnt_rmd = ts->ic_data->HX_MAX_PT % 4;
	/* more than 4 fingers */
	if (ts->hx_touch_data->raw_cnt_rmd != 0x00) {
		ts->hx_touch_data->rawdata_size =
			himax_mcu_cal_data_len(ts->hx_touch_data->raw_cnt_rmd,
					       ts->ic_data->HX_MAX_PT,
					       ts->hx_touch_data->raw_cnt_max);

		ts->hx_touch_data->touch_info_size =
			(ts->ic_data->HX_MAX_PT +
			 ts->hx_touch_data->raw_cnt_max + 2) *
			4;
	} else { /* less than 4 fingers */
		ts->hx_touch_data->rawdata_size =
			himax_mcu_cal_data_len(ts->hx_touch_data->raw_cnt_rmd,
					       ts->ic_data->HX_MAX_PT,
					       ts->hx_touch_data->raw_cnt_max);

		ts->hx_touch_data->touch_info_size =
			(ts->ic_data->HX_MAX_PT +
			 ts->hx_touch_data->raw_cnt_max + 1) *
			4;
	}

	if (ts->ic_data->HX_PEN_FUNC) {
		ts->hx_touch_data->touch_info_size += PEN_INFO_SZ;
		ts->hx_touch_data->rawdata_size -= PEN_INFO_SZ;
	}

	if ((ts->ic_data->HX_TX_NUM * ts->ic_data->HX_RX_NUM +
	     ts->ic_data->HX_TX_NUM + ts->ic_data->HX_RX_NUM) %
		    ts->hx_touch_data->rawdata_size ==
	    0)
		ts->hx_touch_data->rawdata_frame_size =
			(ts->ic_data->HX_TX_NUM * ts->ic_data->HX_RX_NUM +
			 ts->ic_data->HX_TX_NUM + ts->ic_data->HX_RX_NUM) /
			ts->hx_touch_data->rawdata_size;
	else
		ts->hx_touch_data->rawdata_frame_size =
			(ts->ic_data->HX_TX_NUM * ts->ic_data->HX_RX_NUM +
			 ts->ic_data->HX_TX_NUM + ts->ic_data->HX_RX_NUM) /
				ts->hx_touch_data->rawdata_size +
			1;

	D("%s: rawdata_fsz = %d,HX_MAX_PT:%d,hx_raw_cnt_max:%d\n", __func__,
	  ts->hx_touch_data->rawdata_frame_size, ts->ic_data->HX_MAX_PT,
	  ts->hx_touch_data->raw_cnt_max);
	D("%s: hx_raw_cnt_rmd:%d,g_hx_rawdata_size:%d,touch_info_size:%d\n",
	  __func__, ts->hx_touch_data->raw_cnt_rmd,
	  ts->hx_touch_data->rawdata_size, ts->hx_touch_data->touch_info_size);

	ts->hx_touch_data->hx_coord_buf =
		kzalloc(sizeof(uint8_t) * (ts->hx_touch_data->touch_info_size),
			GFP_KERNEL);

	if (ts->hx_touch_data->hx_coord_buf == NULL)
		goto mem_alloc_fail_coord_buf;

	ts->hx_touch_data->hx_rawdata_buf =
		kzalloc(sizeof(uint8_t) * (ts->hx_touch_data->touch_all_size -
					   ts->hx_touch_data->touch_info_size),
			GFP_KERNEL);
	if (ts->hx_touch_data->hx_rawdata_buf == NULL)
		goto mem_alloc_fail_rawdata_buf;

	if (g_target_report_data == NULL) {
		g_target_report_data = kzalloc(
			sizeof(struct himax_target_report_data), GFP_KERNEL);
		if (g_target_report_data == NULL)
			goto mem_alloc_fail_report_data;

		g_target_report_data->x = kzalloc(
			sizeof(int) * (ts->ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->x == NULL)
			goto mem_alloc_fail_report_data_x;

		g_target_report_data->y = kzalloc(
			sizeof(int) * (ts->ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->y == NULL)
			goto mem_alloc_fail_report_data_y;

		g_target_report_data->w = kzalloc(
			sizeof(int) * (ts->ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->w == NULL)
			goto mem_alloc_fail_report_data_w;

		g_target_report_data->finger_id = kzalloc(
			sizeof(int) * (ts->ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->finger_id == NULL)
			goto mem_alloc_fail_report_data_fid;

		if (!ts->ic_data->HX_PEN_FUNC)
			goto skip_pen_operation;

		g_target_report_data->p_x =
			kzalloc(sizeof(int) * 2, GFP_KERNEL);
		if (g_target_report_data->p_x == NULL)
			goto mem_alloc_fail_report_data_px;

		g_target_report_data->p_y =
			kzalloc(sizeof(int) * 2, GFP_KERNEL);
		if (g_target_report_data->p_y == NULL)
			goto mem_alloc_fail_report_data_py;

		g_target_report_data->p_w =
			kzalloc(sizeof(int) * 2, GFP_KERNEL);
		if (g_target_report_data->p_w == NULL)
			goto mem_alloc_fail_report_data_pw;

		g_target_report_data->pen_id =
			kzalloc(sizeof(int) * 2, GFP_KERNEL);
		if (g_target_report_data->pen_id == NULL)
			goto mem_alloc_fail_report_data_pid;

		g_target_report_data->p_hover =
			kzalloc(sizeof(int) * 2, GFP_KERNEL);
		if (g_target_report_data->p_hover == NULL)
			goto mem_alloc_fail_report_data_ph;

		g_target_report_data->p_tilt_x =
			kzalloc(sizeof(int) * 2, GFP_KERNEL);
		if (g_target_report_data->p_tilt_x == NULL)
			goto mem_alloc_fail_report_data_ptx;

		g_target_report_data->p_btn =
			kzalloc(sizeof(int) * 2, GFP_KERNEL);
		if (g_target_report_data->p_btn == NULL)
			goto mem_alloc_fail_report_data_pb;

		g_target_report_data->p_btn2 =
			kzalloc(sizeof(int) * 2, GFP_KERNEL);
		if (g_target_report_data->p_btn2 == NULL)
			goto mem_alloc_fail_report_data_pb2;

		g_target_report_data->p_tilt_y =
			kzalloc(sizeof(int) * 2, GFP_KERNEL);
		if (g_target_report_data->p_tilt_y == NULL)
			goto mem_alloc_fail_report_data_pty;

		g_target_report_data->p_on =
			kzalloc(sizeof(int) * 2, GFP_KERNEL);
		if (g_target_report_data->p_on == NULL)
			goto mem_alloc_fail_report_data_pon;
	}

skip_pen_operation:

	return NO_ERR;

mem_alloc_fail_report_data_pon:
	kfree(g_target_report_data->p_tilt_y);
	g_target_report_data->p_tilt_y = NULL;
mem_alloc_fail_report_data_pty:
	kfree(g_target_report_data->p_btn2);
	g_target_report_data->p_btn2 = NULL;
mem_alloc_fail_report_data_pb2:
	kfree(g_target_report_data->p_btn);
	g_target_report_data->p_btn = NULL;
mem_alloc_fail_report_data_pb:
	kfree(g_target_report_data->p_tilt_x);
	g_target_report_data->p_tilt_x = NULL;
mem_alloc_fail_report_data_ptx:
	kfree(g_target_report_data->p_hover);
	g_target_report_data->p_hover = NULL;
mem_alloc_fail_report_data_ph:
	kfree(g_target_report_data->pen_id);
	g_target_report_data->pen_id = NULL;
mem_alloc_fail_report_data_pid:
	kfree(g_target_report_data->p_w);
	g_target_report_data->p_w = NULL;
mem_alloc_fail_report_data_pw:
	kfree(g_target_report_data->p_y);
	g_target_report_data->p_y = NULL;
mem_alloc_fail_report_data_py:
	kfree(g_target_report_data->p_x);
	g_target_report_data->p_x = NULL;
mem_alloc_fail_report_data_px:

	kfree(g_target_report_data->finger_id);
	g_target_report_data->finger_id = NULL;
mem_alloc_fail_report_data_fid:
	kfree(g_target_report_data->w);
	g_target_report_data->w = NULL;
mem_alloc_fail_report_data_w:
	kfree(g_target_report_data->y);
	g_target_report_data->y = NULL;
mem_alloc_fail_report_data_y:
	kfree(g_target_report_data->x);
	g_target_report_data->x = NULL;
mem_alloc_fail_report_data_x:
	kfree(g_target_report_data);
	g_target_report_data = NULL;
mem_alloc_fail_report_data:
	kfree(ts->hx_touch_data->hx_rawdata_buf);
	ts->hx_touch_data->hx_rawdata_buf = NULL;
mem_alloc_fail_rawdata_buf:
#if defined(HX_SMART_WAKEUP)
	kfree(ts->hx_touch_data->hx_event_buf);
	ts->hx_touch_data->hx_event_buf = NULL;
mem_alloc_fail_event_buf:
	kfree(wake_event_buffer);
	wake_event_buffer = NULL;
mem_alloc_fail_smwp:
#endif
	kfree(ts->hx_touch_data->hx_coord_buf);
	ts->hx_touch_data->hx_coord_buf = NULL;
mem_alloc_fail_coord_buf:

	E("%s: Failed to allocate memory\n", __func__);
	return MEM_ALLOC_FAIL;
}
EXPORT_SYMBOL(himax_report_data_init);

void himax_report_data_deinit(struct himax_ts_data *ts)
{
	if (ts->ic_data->HX_PEN_FUNC) {
		kfree(g_target_report_data->p_on);
		g_target_report_data->p_on = NULL;
		kfree(g_target_report_data->p_tilt_y);
		g_target_report_data->p_tilt_y = NULL;
		kfree(g_target_report_data->p_btn2);
		g_target_report_data->p_btn2 = NULL;
		kfree(g_target_report_data->p_btn);
		g_target_report_data->p_btn = NULL;
		kfree(g_target_report_data->p_tilt_x);
		g_target_report_data->p_tilt_x = NULL;
		kfree(g_target_report_data->p_hover);
		g_target_report_data->p_hover = NULL;
		kfree(g_target_report_data->pen_id);
		g_target_report_data->pen_id = NULL;
		kfree(g_target_report_data->p_w);
		g_target_report_data->p_w = NULL;
		kfree(g_target_report_data->p_y);
		g_target_report_data->p_y = NULL;
		kfree(g_target_report_data->p_x);
		g_target_report_data->p_x = NULL;
	}

	kfree(g_target_report_data->finger_id);
	g_target_report_data->finger_id = NULL;
	kfree(g_target_report_data->w);
	g_target_report_data->w = NULL;
	kfree(g_target_report_data->y);
	g_target_report_data->y = NULL;
	kfree(g_target_report_data->x);
	g_target_report_data->x = NULL;
	kfree(g_target_report_data);
	g_target_report_data = NULL;
	kfree(ts->hx_touch_data->hx_rawdata_buf);
	ts->hx_touch_data->hx_rawdata_buf = NULL;
	kfree(ts->hx_touch_data->hx_coord_buf);
	ts->hx_touch_data->hx_coord_buf = NULL;
}

/*start ts_work*/
static int himax_ts_work_status(struct himax_ts_data *ts)
{
	/* 1: normal, 2:SMWP */
	int result = HX_REPORT_COORD;

	ts->hx_touch_data->diag_cmd = ts->diag_cmd;
	if (ts->hx_touch_data->diag_cmd)
		result = HX_REPORT_COORD_RAWDATA;

	return result;
}

static int himax_touch_get(struct himax_ts_data *ts, uint8_t *buf, int ts_path,
			   int ts_status)
{
	if (g_ts_dbg != 0)
		D("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	switch (ts_path) {
	/*normal*/
	case HX_REPORT_COORD:
		if ((HX_HW_RESET_ACTIVATE)
#if defined(HX_ESD_RECOVERY)
		    || (ts->HX_ESD_RESET_ACTIVATE)
#endif
		) {
			/* TODO: himax_mcu_read_event_stack */
			if (!hx83102e_read_event_stack(ts, buf, 128)) {
				E("%s: can't read data from chip!\n", __func__);
				ts_status = HX_TS_GET_DATA_FAIL;
			}
		} else {
			/* TODO: himax_mcu_read_event_stack */
			if (!hx83102e_read_event_stack(
				    ts, buf,
				    ts->hx_touch_data->touch_info_size)) {
				E("%s: can't read data from chip!\n", __func__);
				ts_status = HX_TS_GET_DATA_FAIL;
			}
		}
		break;

	case HX_REPORT_COORD_RAWDATA:
		/* TODO: himax_mcu_read_event_stack */
		if (!hx83102e_read_event_stack(ts, buf, 128)) {
			E("%s: can't read data from chip!\n", __func__);
			ts_status = HX_TS_GET_DATA_FAIL;
		}
		break;
	default:
		break;
	}

	return ts_status;
}

/* start error_control*/
static int himax_checksum_cal(struct himax_ts_data *ts, uint8_t *buf,
			      int ts_path, int ts_status)
{
	uint16_t check_sum_cal = 0;
	int32_t i = 0;
	int length = 0;
	int zero_cnt = 0;
	int raw_data_sel = 0;
	int ret_val = ts_status;

	if (g_ts_dbg != 0)
		D("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	/* Normal */
	switch (ts_path) {
	case HX_REPORT_COORD:
		length = ts->hx_touch_data->touch_info_size;
		break;
	case HX_REPORT_COORD_RAWDATA:
		length = ts->hx_touch_data->touch_info_size;
		break;
	default:
		D("%s: Neither Normal Nor SMWP error!\n", __func__);
		ret_val = HX_PATH_FAIL;
		goto END_FUNCTION;
	}

	for (i = 0; i < length; i++) {
		check_sum_cal += buf[i];
		if (buf[i] == 0x00)
			zero_cnt++;
	}

	if (check_sum_cal % 0x100 != 0) {
		D("%s: point data_checksum not match check_sum_cal: 0x%02X",
		  __func__, check_sum_cal);
		ret_val = HX_CHKSUM_FAIL;
	} else if (zero_cnt == length) {
		if (ts->use_irq)
			D("%s: [HIMAX TP MSG] All Zero event\n", __func__);

		ret_val = HX_CHKSUM_FAIL;
	} else {
		raw_data_sel = buf[ts->HX_TOUCH_INFO_POINT_CNT] >> 4 & 0x0F;
		/*raw data out not match skip it*/
		if ((raw_data_sel != 0x0F) &&
		    (raw_data_sel != ts->hx_touch_data->diag_cmd)) {
			if (!ts->hx_touch_data->diag_cmd) {
				/*Need to clear event stack here*/
				/* TODO: himax_mcu_read_event_stack */
				hx83102e_read_event_stack(
					ts, buf,
					(128 -
					 ts->hx_touch_data->touch_info_size));
			}
			ret_val = HX_READY_SERVE;
		}
	}

END_FUNCTION:
	if (g_ts_dbg != 0)
		D("%s: END, ret_val=%d!\n", __func__, ret_val);
	return ret_val;
}

#if defined(HX_ESD_RECOVERY)
__attribute__((unused)) static int
himax_ts_event_check(struct himax_ts_data *ts, uint8_t *buf, int ts_path,
		     int ts_status)
{
	int hx_EB_event = 0;
	int hx_EC_event = 0;
	int hx_ED_event = 0;
	int hx_esd_event = 0;
	int hx_zero_event = 0;
	int shaking_ret = 0;

	int32_t loop_i = 0;
	int length = 0;
	int ret_val = ts_status;

	if (g_ts_dbg != 0)
		D("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	/* Normal */
	switch (ts_path) {
	case HX_REPORT_COORD:
		length = ts->hx_touch_data->touch_info_size;
		break;
	case HX_REPORT_COORD_RAWDATA:
		length = ts->hx_touch_data->touch_info_size;
		break;
	default:
		W("%s: Neither Normal Nor SMWP error!\n", __func__);
		ret_val = HX_PATH_FAIL;
		goto END_FUNCTION;
	}

	if (g_ts_dbg != 0)
		D("%s: Now Path=%d, Now status=%d, length=%d\n", __func__,
		  ts_path, ts_status, length);

	for (loop_i = 0; loop_i < length; loop_i++) {
		if (ts_path == HX_REPORT_COORD ||
		    ts_path == HX_REPORT_COORD_RAWDATA) {
			/* case 1 ESD recovery flow */
			if (buf[loop_i] == 0xEB) {
				hx_EB_event++;
			} else if (buf[loop_i] == 0xEC) {
				hx_EC_event++;
			} else if (buf[loop_i] == 0xED) {
				hx_ED_event++;

				/* case 2 ESD recovery flow-Disable */
			} else if (buf[loop_i] == 0x00) {
				hx_zero_event++;
			} else {
				hx_EB_event = 0;
				hx_EC_event = 0;
				hx_ED_event = 0;
				hx_zero_event = 0;
				ts->g_zero_event_count = 0;
			}
		}
	}

	if (hx_EB_event == length) {
		hx_esd_event = length;
		ts->hx_EB_event_flag++;
		D("%s: [HIMAX TP MSG]: ESD event checked - ALL 0xEB.\n",
		  __func__);
	} else if (hx_EC_event == length) {
		hx_esd_event = length;
		ts->hx_EC_event_flag++;
		D("%s: [HIMAX TP MSG]: ESD event checked - ALL 0xEC.\n",
		  __func__);
	} else if (hx_ED_event == length) {
		hx_esd_event = length;
		ts->hx_ED_event_flag++;
		D("%s: [HIMAX TP MSG]: ESD event checked - ALL 0xED.\n",
		  __func__);
	} else {
		ts->hx_EB_event_flag = 0;
		ts->hx_EC_event_flag = 0;
		ts->hx_ED_event_flag = 0;
		hx_esd_event = 0;
	}

	if ((hx_esd_event == length || hx_zero_event == length) &&
	    (HX_HW_RESET_ACTIVATE == 0) && (ts->HX_ESD_RESET_ACTIVATE == 0) &&
	    (ts->hx_touch_data->diag_cmd == 0) && (ts->in_self_test == 0)) //
	{
		shaking_ret = himax_mcu_ic_esd_recovery(ts, hx_esd_event,
							hx_zero_event, length);
		if (shaking_ret == HX_ESD_EVENT) {
			himax_esd_hw_reset(ts);
			ret_val = HX_ESD_EVENT;
		} else if (shaking_ret == HX_ZERO_EVENT_COUNT) {
			ret_val = HX_ZERO_EVENT_COUNT;
		} else {
			D("%s: I2C running. Nothing to be done!\n", __func__);
			ret_val = HX_IC_RUNNING;
		}
		/* drop 1st interrupts after chip reset */
	} else if (ts->HX_ESD_RESET_ACTIVATE) {
		ts->HX_ESD_RESET_ACTIVATE = 0;
		D("%s: [HX_ESD_RESET_ACTIVATE] Back from reset,ready to serve.\n",
		  __func__);
		ret_val = HX_ESD_REC_OK;
	}

END_FUNCTION:
	if (g_ts_dbg != 0)
		D("%s: END, ret_val=%d!\n", __func__, ret_val);

	return ret_val;
}
#endif

static int himax_err_ctrl(struct himax_ts_data *ts, uint8_t *buf, int ts_path,
			  int ts_status)
{
#if defined(HX_RST_PIN_FUNC)
	if (HX_HW_RESET_ACTIVATE) {
		/* drop 1st interrupts after chip reset */
		HX_HW_RESET_ACTIVATE = 0;
		D("%s: [HX_HW_RESET_ACTIVATE] Back from reset,ready to serve.\n",
		  __func__);
		ts_status = HX_RST_OK;
		goto END_FUNCTION;
	}
#endif

	ts_status = himax_checksum_cal(ts, buf, ts_path, ts_status);
	if (ts_status == HX_CHKSUM_FAIL) {
		ts_status = HX_REPORT_DATA;
		goto END_FUNCTION;
		//goto CHK_FAIL;
	} else {
#if defined(HX_ESD_RECOVERY)
		/* continuous N times record, not total N times. */
		ts->g_zero_event_count = 0;
#endif
		goto END_FUNCTION;
	}

END_FUNCTION:

	if (g_ts_dbg != 0)
		D("%s: END, ts_status=%d!\n", __func__, ts_status);

	return ts_status;
}
/* end error_control*/

/* start distribute_data*/
static int himax_distribute_touch_data(struct himax_ts_data *ts, uint8_t *buf,
				       int ts_path, int ts_status)
{
	uint8_t hx_state_info_pos = ts->hx_touch_data->touch_info_size - 3;

	if (ts->ic_data->HX_PEN_FUNC)
		hx_state_info_pos -= PEN_INFO_SZ;

	if (g_ts_dbg != 0)
		D("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD) {
		memcpy(ts->hx_touch_data->hx_coord_buf, &buf[0],
		       ts->hx_touch_data->touch_info_size);

		if (buf[hx_state_info_pos] != 0xFF &&
		    buf[hx_state_info_pos + 1] != 0xFF)
			memcpy(ts->hx_touch_data->hx_state_info,
			       &buf[hx_state_info_pos], 2);
		else
			memset(ts->hx_touch_data->hx_state_info, 0x00,
			       sizeof(ts->hx_touch_data->hx_state_info));

		if ((HX_HW_RESET_ACTIVATE)
#if defined(HX_ESD_RECOVERY)
		    || (ts->HX_ESD_RESET_ACTIVATE)
#endif
		) {
			memcpy(ts->hx_touch_data->hx_rawdata_buf,
			       &buf[ts->hx_touch_data->touch_info_size],
			       ts->hx_touch_data->touch_all_size -
				       ts->hx_touch_data->touch_info_size);
		}
	} else if (ts_path == HX_REPORT_COORD_RAWDATA) {
		memcpy(ts->hx_touch_data->hx_coord_buf, &buf[0],
		       ts->hx_touch_data->touch_info_size);

		if (buf[hx_state_info_pos] != 0xFF &&
		    buf[hx_state_info_pos + 1] != 0xFF)
			memcpy(ts->hx_touch_data->hx_state_info,
			       &buf[hx_state_info_pos], 2);
		else
			memset(ts->hx_touch_data->hx_state_info, 0x00,
			       sizeof(ts->hx_touch_data->hx_state_info));

		memcpy(ts->hx_touch_data->hx_rawdata_buf,
		       &buf[ts->hx_touch_data->touch_info_size],
		       ts->hx_touch_data->touch_all_size -
			       ts->hx_touch_data->touch_info_size);
	} else {
		E("%s: Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		D("%s: End, ts_status=%d!\n", __func__, ts_status);

	return ts_status;
}
/* end assign_data*/

/* start parse_report_data*/
int himax_parse_report_points(struct himax_ts_data *ts, int ts_path,
			      int ts_status)
{
	int x = 0, y = 0, w = 0;

	uint8_t p_hover = 0, p_btn = 0, p_btn2 = 0;
	int8_t p_tilt_x = 0, p_tilt_y = 0;
	int p_x = 0, p_y = 0, p_w = 0;
	static uint8_t p_p_on;

	int base = 0;
	int32_t loop_i = 0;

	if (g_ts_dbg != 0)
		D("%s: start!\n", __func__);

	if (!ts->ic_data->HX_PEN_FUNC)
		goto skip_pen_operation;

	p_p_on = 0;
	base = ts->hx_touch_data->touch_info_size - PEN_INFO_SZ;

	p_x = ts->hx_touch_data->hx_coord_buf[base] << 8 |
	      ts->hx_touch_data->hx_coord_buf[base + 1];
	p_y = (ts->hx_touch_data->hx_coord_buf[base + 2] << 8 |
	       ts->hx_touch_data->hx_coord_buf[base + 3]);
	p_w = (ts->hx_touch_data->hx_coord_buf[base + 4] << 8 |
	       ts->hx_touch_data->hx_coord_buf[base + 5]);
	p_tilt_x = (int8_t)ts->hx_touch_data->hx_coord_buf[base + 6];
	p_hover = ts->hx_touch_data->hx_coord_buf[base + 7];
	p_btn = ts->hx_touch_data->hx_coord_buf[base + 8];
	p_btn2 = ts->hx_touch_data->hx_coord_buf[base + 9];
	p_tilt_y = (int8_t)ts->hx_touch_data->hx_coord_buf[base + 10];

	if (g_ts_dbg != 0) {
		D("%s: p_x=%d, p_y=%d, p_w=%d,p_tilt_x=%d, p_hover=%d\n",
		  __func__, p_x, p_y, p_w, p_tilt_x, p_hover);
		D("%s: p_btn=%d, p_btn2=%d, p_tilt_y=%d\n", __func__, p_btn,
		  p_btn2, p_tilt_y);
	}

	if (p_x >= 0 && p_x <= ts->pdata->abs_x_max && p_y >= 0 &&
	    p_y <= ts->pdata->abs_y_max) {
		g_target_report_data->p_x[0] = p_x;
		g_target_report_data->p_y[0] = p_y;
		g_target_report_data->p_w[0] = p_w;
		g_target_report_data->p_hover[0] = p_hover;
		g_target_report_data->pen_id[0] = 1;
		g_target_report_data->p_btn[0] = p_btn;
		g_target_report_data->p_btn2[0] = p_btn2;
		g_target_report_data->p_tilt_x[0] = p_tilt_x;
		g_target_report_data->p_tilt_y[0] = p_tilt_y;
		g_target_report_data->p_on[0] = 1;
		ts->hx_point_num++;
	} else { /* report coordinates */
		g_target_report_data->p_x[0] = 0;
		g_target_report_data->p_y[0] = 0;
		g_target_report_data->p_w[0] = 0;
		g_target_report_data->p_hover[0] = 0;
		g_target_report_data->pen_id[0] = 0;
		g_target_report_data->p_btn[0] = 0;
		g_target_report_data->p_btn2[0] = 0;
		g_target_report_data->p_tilt_x[0] = 0;
		g_target_report_data->p_tilt_y[0] = 0;
		g_target_report_data->p_on[0] = 0;
	}

	if (g_ts_dbg != 0) {
		if (p_p_on != g_target_report_data->p_on[0]) {
			D("%s: p_on[0] = %d, hx_point_num=%d\n", __func__,
			  g_target_report_data->p_on[0], ts->hx_point_num);
			p_p_on = g_target_report_data->p_on[0];
		}
	}
skip_pen_operation:

	ts->old_finger = ts->pre_finger_mask;
	if (ts->hx_point_num == 0) {
		if (g_ts_dbg != 0)
			D("%s: hx_point_num = 0!\n", __func__);
		return ts_status;
	}
	ts->pre_finger_mask = 0;
	ts->hx_touch_data->finger_num =
		ts->hx_touch_data->hx_coord_buf[ts->coordInfoSize - 4] & 0x0F;
	ts->hx_touch_data->finger_on = 1;
	AA_press = 1;

	g_target_report_data->finger_num = ts->hx_touch_data->finger_num;
	g_target_report_data->finger_on = ts->hx_touch_data->finger_on;
	g_target_report_data->ig_count =
		ts->hx_touch_data->hx_coord_buf[ts->coordInfoSize - 5];

	if (g_ts_dbg != 0)
		D("%s:finger_num = 0x%2X, finger_on = %d\n", __func__,
		  g_target_report_data->finger_num,
		  g_target_report_data->finger_on);

	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
		base = loop_i * 4;
		x = ts->hx_touch_data->hx_coord_buf[base] << 8 |
		    ts->hx_touch_data->hx_coord_buf[base + 1];
		y = (ts->hx_touch_data->hx_coord_buf[base + 2] << 8 |
		     ts->hx_touch_data->hx_coord_buf[base + 3]);
		w = ts->hx_touch_data
			    ->hx_coord_buf[(ts->nFinger_support * 4) + loop_i];

		if (g_ts_dbg != 0)
			D("%s: now parsing[%d]:x=%d, y=%d, w=%d\n", __func__,
			  loop_i, x, y, w);

		if (x >= 0 && x <= ts->pdata->abs_x_max && y >= 0 &&
		    y <= ts->pdata->abs_y_max) {
			ts->hx_touch_data->finger_num--;

			g_target_report_data->x[loop_i] = x;
			g_target_report_data->y[loop_i] = y;
			g_target_report_data->w[loop_i] = w;
			g_target_report_data->finger_id[loop_i] = 1;

			if (!ts->first_pressed) {
				ts->first_pressed = 1;
				D("%s: S1@%d, %d\n", __func__, x, y);
			}

			ts->pre_finger_data[loop_i][0] = x;
			ts->pre_finger_data[loop_i][1] = y;

			ts->pre_finger_mask =
				ts->pre_finger_mask + (1 << loop_i);
		} else { /* report coordinates */
			g_target_report_data->x[loop_i] = x;
			g_target_report_data->y[loop_i] = y;
			g_target_report_data->w[loop_i] = w;
			g_target_report_data->finger_id[loop_i] = 0;

			if (loop_i == 0 && ts->first_pressed == 1) {
				ts->first_pressed = 2;
				if (g_ts_dbg != 0)
					D("%s: E1@%d, %d\n", __func__,
					  ts->pre_finger_data[0][0],
					  ts->pre_finger_data[0][1]);
			}
		}
	}

	if (g_ts_dbg != 0) {
		for (loop_i = 0; loop_i < 10; loop_i++) {
			D("%s: DBG X=%d  Y=%d ID=%d\n", __func__,
			  g_target_report_data->x[loop_i],
			  g_target_report_data->y[loop_i],
			  g_target_report_data->finger_id[loop_i]);
		}
		D("%s: finger number %d\n", __func__,
		  g_target_report_data->finger_num);
	}

	if (g_ts_dbg != 0)
		D("%s: end!\n", __func__);
	return ts_status;
}

static int himax_parse_report_data(struct himax_ts_data *ts, int ts_path,
				   int ts_status)
{
	if (g_ts_dbg != 0)
		D("%s: start now_status=%d!\n", __func__, ts_status);

	EN_NoiseFilter =
		(ts->hx_touch_data
			 ->hx_coord_buf[ts->HX_TOUCH_INFO_POINT_CNT + 2] >>
		 3);
	EN_NoiseFilter = EN_NoiseFilter & 0x01;
	p_point_num = ts->hx_point_num;

	if (ts->hx_touch_data->hx_coord_buf[ts->HX_TOUCH_INFO_POINT_CNT] ==
	    0xff)
		ts->hx_point_num = 0;
	else
		ts->hx_point_num =
			ts->hx_touch_data
				->hx_coord_buf[ts->HX_TOUCH_INFO_POINT_CNT] &
			0x0f;

	switch (ts_path) {
	case HX_REPORT_COORD:
		ts_status = himax_parse_report_points(ts, ts_path, ts_status);
		break;
	case HX_REPORT_COORD_RAWDATA:
		/* touch monitor rawdata */
		ts_status = himax_parse_report_points(ts, ts_path, ts_status);
		break;
	default:
		E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
		break;
	}
	if (g_ts_dbg != 0)
		D("%s: end now_status=%d!\n", __func__, ts_status);
	return ts_status;
}

/* end parse_report_data*/

static void himax_report_all_leave_event(struct himax_ts_data *ts)
{
	int loop_i = 0;
	if (ts->pdata->protocol_type == PROTOCOL_TYPE_A) {
		for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
			input_mt_slot(ts->input_dev, loop_i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev,
						   MT_TOOL_FINGER, 0);
		}
	}
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);
}

/* start report_point*/
static void himax_finger_report(struct himax_ts_data *ts)
{
	int i = 0;
	bool valid = false;

	if (g_ts_dbg != 0) {
		D("%s:start ts->hx_touch_data->finger_num=%d\n", __func__,
		  ts->hx_touch_data->finger_num);
	}
	for (i = 0; i < ts->nFinger_support; i++) {
		if (g_target_report_data->x[i] >= 0 &&
		    g_target_report_data->x[i] <= ts->pdata->abs_x_max &&
		    g_target_report_data->y[i] >= 0 &&
		    g_target_report_data->y[i] <= ts->pdata->abs_y_max)
			valid = true;
		else
			valid = false;
		if (g_ts_dbg != 0)
			D("%s: valid=%d\n", __func__, valid);
		if (valid) {
			if (g_ts_dbg != 0) {
				D("%s: report_data->x[i]=%d,y[i]=%d,w[i]=%d",
				  __func__, g_target_report_data->x[i],
				  g_target_report_data->y[i],
				  g_target_report_data->w[i]);
			}

			if (ts->pdata->protocol_type != PROTOCOL_TYPE_A) {
				input_mt_slot(ts->input_dev, i);
			} else {
				input_report_key(ts->input_dev, BTN_TOUCH, 1);
			}
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					 g_target_report_data->w[i]);
			if (ts->pdata->protocol_type != PROTOCOL_TYPE_A) {
				input_report_abs(ts->input_dev,
						 ABS_MT_WIDTH_MAJOR,
						 g_target_report_data->w[i]);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
						 g_target_report_data->w[i]);
			} else {
				input_report_abs(ts->input_dev,
						 ABS_MT_TRACKING_ID, i + 1);
			}
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					 g_target_report_data->x[i]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					 g_target_report_data->y[i]);
			if (ts->pdata->protocol_type != PROTOCOL_TYPE_A) {
				ts->last_slot = i;
				input_mt_report_slot_state(ts->input_dev,
							   MT_TOOL_FINGER, 1);
			} else {
				input_mt_sync(ts->input_dev);
			}
		} else {
			if (ts->pdata->protocol_type != PROTOCOL_TYPE_A) {
				input_mt_slot(ts->input_dev, i);
				input_report_abs(ts->input_dev,
						 ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(ts->input_dev,
						 ABS_MT_WIDTH_MAJOR, 0);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
						 0);
				input_mt_report_slot_state(ts->input_dev,
							   MT_TOOL_FINGER, 0);
			}
		}
	}

	if (ts->pdata->protocol_type != PROTOCOL_TYPE_A) {
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
	}

	input_sync(ts->input_dev);

	if (!ts->ic_data->HX_PEN_FUNC)
		goto skip_pen_operation;

	valid = false;

	if (g_target_report_data->p_x[0] >= 0 &&
	    g_target_report_data->p_x[0] <= ts->pdata->abs_x_max &&
	    g_target_report_data->p_y[0] >= 0 &&
	    g_target_report_data->p_y[0] <= ts->pdata->abs_y_max &&
	    (g_target_report_data->p_on[0] == 1))
		valid = true;
	else
		valid = false;

	if (g_ts_dbg != 0)
		D("%s: pen valid=%d\n", __func__, valid);

	if (valid) { /*Pen down*/
		if (g_ts_dbg != 0)
			D("%s: p_x[i]=%d, p_y[i]=%d, p_w[i]=%d\n", __func__,
			  g_target_report_data->p_x[0],
			  g_target_report_data->p_y[0],
			  g_target_report_data->p_w[0]);

		input_report_abs(ts->hx_pen_dev, ABS_X,
				 g_target_report_data->p_x[0]);
		input_report_abs(ts->hx_pen_dev, ABS_Y,
				 g_target_report_data->p_y[0]);

		if (g_target_report_data->p_btn[0] !=
		    g_target_report_data->pre_p_btn) {
			if (g_ts_dbg != 0)
				D("%s: BTN_STYLUS:%d\n", __func__,
				  g_target_report_data->p_btn[0]);

			input_report_key(ts->hx_pen_dev, BTN_STYLUS,
					 g_target_report_data->p_btn[0]);

			g_target_report_data->pre_p_btn =
				g_target_report_data->p_btn[0];
		} else {
			if (g_ts_dbg != 0)
				D("%s: BTN_STYLUS status no change, value=%d!\n",
				  __func__, g_target_report_data->p_btn[0]);
		}

		if (g_target_report_data->p_btn2[0] !=
		    g_target_report_data->pre_p_btn2) {
			if (g_ts_dbg != 0)
				D("%s: BTN_STYLUS2:%d\n", __func__,
				  g_target_report_data->p_btn2[0]);

			input_report_key(ts->hx_pen_dev, BTN_STYLUS2,
					 g_target_report_data->p_btn2[0]);

			g_target_report_data->pre_p_btn2 =
				g_target_report_data->p_btn2[0];
		} else {
			if (g_ts_dbg != 0)
				D("%s: BTN_STYLUS2 status no change, value=%d!\n",
				  __func__, g_target_report_data->p_btn2[0]);
		}
		input_report_abs(ts->hx_pen_dev, ABS_TILT_X,
				 g_target_report_data->p_tilt_x[0]);

		input_report_abs(ts->hx_pen_dev, ABS_TILT_Y,
				 g_target_report_data->p_tilt_y[0]);

		input_report_key(ts->hx_pen_dev, BTN_TOOL_PEN, 1);

		if (g_target_report_data->p_hover[0] == 0) {
			input_report_key(ts->hx_pen_dev, BTN_TOUCH, 1);
			input_report_abs(ts->hx_pen_dev, ABS_DISTANCE, 0);
			input_report_abs(ts->hx_pen_dev, ABS_PRESSURE,
					 g_target_report_data->p_w[0]);
		} else {
			input_report_key(ts->hx_pen_dev, BTN_TOUCH, 0);
			input_report_abs(ts->hx_pen_dev, ABS_DISTANCE, 1);
			input_report_abs(ts->hx_pen_dev, ABS_PRESSURE, 0);
		}
	} else { /*Pen up*/
		g_target_report_data->pre_p_btn = 0;
		g_target_report_data->pre_p_btn2 = 0;
		input_report_key(ts->hx_pen_dev, BTN_STYLUS, 0);
		input_report_key(ts->hx_pen_dev, BTN_STYLUS2, 0);
		input_report_key(ts->hx_pen_dev, BTN_TOUCH, 0);
		input_report_abs(ts->hx_pen_dev, ABS_PRESSURE, 0);
		input_sync(ts->hx_pen_dev);

		input_report_abs(ts->hx_pen_dev, ABS_DISTANCE, 0);
		input_report_key(ts->hx_pen_dev, BTN_TOOL_RUBBER, 0);
		input_report_key(ts->hx_pen_dev, BTN_TOOL_PEN, 0);
		input_report_abs(ts->hx_pen_dev, ABS_PRESSURE, 0);
	}
	input_sync(ts->hx_pen_dev);

skip_pen_operation:

	if (g_ts_dbg != 0)
		D("%s: end\n", __func__);
}

static void himax_finger_leave(struct himax_ts_data *ts)
{
	int32_t loop_i = 0;

	if (g_ts_dbg != 0)
		D("%s: start!\n", __func__);

	ts->hx_touch_data->finger_on = 0;
	g_target_report_data->finger_on = 0;
	g_target_report_data->finger_num = 0;
	AA_press = 0;

	if (ts->pdata->protocol_type != PROTOCOL_TYPE_A) {
		for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
			input_mt_slot(ts->input_dev, loop_i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev,
						   MT_TOOL_FINGER, 0);
		}
	} else {
		input_mt_sync(ts->input_dev);
	}

	if (ts->pre_finger_mask > 0)
		ts->pre_finger_mask = 0;

	if (ts->first_pressed == 1) {
		ts->first_pressed = 2;
		D("%s: E1@%d, %d\n", __func__, ts->pre_finger_data[0][0],
		  ts->pre_finger_data[0][1]);
	}

	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);

	if (ts->ic_data->HX_PEN_FUNC) {
		input_report_key(ts->hx_pen_dev, BTN_STYLUS, 0);
		input_report_key(ts->hx_pen_dev, BTN_TOUCH, 0);
		input_report_abs(ts->hx_pen_dev, ABS_PRESSURE, 0);
		input_sync(ts->hx_pen_dev);

		input_report_abs(ts->hx_pen_dev, ABS_DISTANCE, 0);
		input_report_abs(ts->hx_pen_dev, ABS_TILT_X, 0);
		input_report_abs(ts->hx_pen_dev, ABS_TILT_Y, 0);
		input_report_key(ts->hx_pen_dev, BTN_TOOL_RUBBER, 0);
		input_report_key(ts->hx_pen_dev, BTN_TOOL_PEN, 0);
		input_sync(ts->hx_pen_dev);
	}

	if (g_ts_dbg != 0)
		D("%s: end!\n", __func__);
}

static void himax_report_points(struct himax_ts_data *ts)
{
	if (g_ts_dbg != 0)
		D("%s: start!\n", __func__);

	if (ts->hx_point_num != 0)
		himax_finger_report(ts);
	else
		himax_finger_leave(ts);
	Last_EN_NoiseFilter = EN_NoiseFilter;

	if (g_ts_dbg != 0)
		D("%s: end!\n", __func__);
}
/* end report_points*/

int himax_report_data(struct himax_ts_data *ts, int ts_path, int ts_status)
{
	if (g_ts_dbg != 0)
		D("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD || ts_path == HX_REPORT_COORD_RAWDATA) {
		/* Touch Point information */
		himax_report_points(ts);
	} else {
		E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		D("%s: END, ts_status=%d!\n", __func__, ts_status);
	return ts_status;
}
/* end report_data */

static int himax_ts_operation(struct himax_ts_data *ts, int ts_path,
			      int ts_status)
{
	uint8_t hw_reset_check[2];

	memset(ts->xfer_buff, 0x00, 128 * sizeof(uint8_t));
	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));

	ts_status = himax_touch_get(ts, ts->xfer_buff, ts_path, ts_status);
	if (ts_status == HX_TS_GET_DATA_FAIL) {
		goto END_FUNCTION;
	}

	ts_status = himax_distribute_touch_data(ts, ts->xfer_buff, ts_path,
						ts_status);
	ts_status = himax_err_ctrl(ts, ts->xfer_buff, ts_path, ts_status);
	if (ts_status == HX_REPORT_DATA || ts_status == HX_TS_NORMAL_END)
		ts_status = himax_parse_report_data(ts, ts_path, ts_status);
	else
		goto END_FUNCTION;

	ts_status = himax_report_data(ts, ts_path, ts_status);

END_FUNCTION:
	return ts_status;
}

void himax_ts_work(struct himax_ts_data *ts)
{
	int ts_status = HX_TS_NORMAL_END;
	int ts_path = 0;

	D("%s: ENTER ****\n", __func__);
	ts_path = himax_ts_work_status(ts);
	switch (ts_path) {
	case HX_REPORT_COORD:
		ts_status = himax_ts_operation(ts, ts_path, ts_status);
		break;
	case HX_REPORT_SMWP_EVENT:
		ts_status = himax_ts_operation(ts, ts_path, ts_status);
		break;
	case HX_REPORT_COORD_RAWDATA:
		ts_status = himax_ts_operation(ts, ts_path, ts_status);
		break;
	default:
		E("%s:Path Fault! value=%d\n", __func__, ts_path);
		goto END_FUNCTION;
	}

	if (ts_status == HX_TS_GET_DATA_FAIL)
		goto GET_TOUCH_FAIL;
	else
		goto END_FUNCTION;

GET_TOUCH_FAIL:
	D("%s: Now reset the Touch chip.\n", __func__);
#if defined(HX_RST_PIN_FUNC)
	himax_mcu_ic_reset(ts, false, true);
#else
	himax_mcu_system_reset(ts);
#endif

END_FUNCTION:
	D("%s: LEAVE ****\n", __func__);
}
/*end ts_work*/
enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts;

	ts = container_of(timer, struct himax_ts_data, timer);
	queue_work(ts->himax_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

int himax_chip_common_init(struct himax_ts_data *ts)
{
	int i = 0, ret = 0, err = PROBE_FAIL;

	D("%s: XFER_BUFF START\n", __func__);
	ts->xfer_buff =
		devm_kzalloc(ts->dev, 128 * sizeof(uint8_t), GFP_KERNEL);
	if (ts->xfer_buff == NULL) {
		err = -ENOMEM;
		goto exit_err_0;
	}

	D("%s: PDATA START\n", __func__);
	ts->pdata = kzalloc(sizeof(struct himax_i2c_platform_data), GFP_KERNEL);
	if (ts->pdata == NULL) { /*Allocate Platform data space*/
		err = -ENOMEM;
		goto exit_err_1;
	}

	/* allocate IC data */
	D("%s: ic_data START\n", __func__);
	ts->ic_data = kzalloc(sizeof(struct himax_ic_data), GFP_KERNEL);
	if (ts->ic_data == NULL) { /*Allocate IC data space*/
		err = -ENOMEM;
		goto exit_err_2;
	}

	/* allocate report data */
	D("%s: report data START\n", __func__);
	ts->hx_touch_data =
		kzalloc(sizeof(struct himax_report_data), GFP_KERNEL);
	if (ts->hx_touch_data == NULL) {
		err = -ENOMEM;
		goto exit_err_3;
	}

	if (himax_parse_dt(ts, ts->pdata) < 0) {
		E("%s: parse DT config failed.\n", __func__);
		goto exit_err_4;
	}

	if (ts->pdata->virtual_key) {
		ts->button = ts->pdata->virtual_key;
	}

#if defined(HX_RST_PIN_FUNC)
	ts->rst_gpio = ts->pdata->gpio_reset;
#endif

	if (himax_gpio_power_config(ts)) {
		E("%s: gpio power config failed\n", __func__);
		goto exit_err_5;
	}

#if !defined(CONFIG_OF)
	if (ts->pdata->power) {
		ret = ts->pdata->power(1);
		if (ret < 0) {
			E("%s: power on failed\n", __func__);
			goto exit_err_4;
		}
	}
#else
	ts->power = ts->pdata->power;
#endif

	if (!hx83102_chip_detect(ts)) {
		E("%s: HX83102-E chip NOT found!\n", __func__);
		goto exit_err_5;
	}

	if (himax_mcu_power_on_init(ts)) {
		E("%s: Chip power on failed!\n", __func__);
		goto exit_err_5;
	}

	/*calculate touch points */
	calculate_point_number(ts);

	/*calculate the i2c data size*/
	calcDataSize(ts);

	ts->suspended = false;
#if defined(HX_HIGH_SENSE)
	ts->HSEN_enable = 0; /* TODO ???? =1 */
#endif
#if defined(CONFIG_OF)
	ts->pdata->abs_pressure_min = 0;
	ts->pdata->abs_pressure_max = 200;
	ts->pdata->abs_width_min = 0;
	ts->pdata->abs_width_max = 200;
	ts->pdata->cable_config[0] = 0xF0;
	ts->pdata->cable_config[1] = 0x00;
#endif

	ret = himax_input_register(ts);
	if (ret) {
		E("%s: Unable to register %s input device\n", __func__,
		  ts->input_dev->name);
		goto exit_err_5;
	}

	spin_lock_init(&ts->irq_lock);

	/*touch data init*/
	err = himax_report_data_init(ts);
	if (err)
		goto exit_err_6;

	err = himax_ts_register_interrupt(ts);
	if (err)
		goto exit_err_7;

	/* chip is ready */
	ts->initialized = true;
	return 0;

exit_err_7:
	himax_report_data_deinit(ts);

exit_err_6:
	input_free_device(ts->input_dev);

exit_err_5:
	himax_gpio_power_deconfig(ts->pdata);
#if !defined(CONFIG_OF)
err_power_failed:
#endif

exit_err_4:
	kfree(ts->hx_touch_data);
	ts->hx_touch_data = NULL;

exit_err_3:
	kfree(ts->ic_data);
	ts->ic_data = NULL;

exit_err_2:
	kfree(ts->pdata);
	ts->pdata = NULL;

exit_err_1:
	devm_kfree(ts->dev, ts->xfer_buff);
	ts->xfer_buff = NULL;

exit_err_0:
	probe_fail_flag = 1;
	return err;
}

void himax_chip_common_deinit(struct himax_ts_data *ts)
{
	himax_ts_unregister_interrupt(ts);
	himax_report_data_deinit(ts);
	input_free_device(ts->input_dev);
	himax_gpio_power_deconfig(ts->pdata);
	himax_mcu_in_cmd_struct_free(ts);

	kfree(ts->hx_touch_data);
	ts->hx_touch_data = NULL;

	kfree(ts->ic_data);
	ts->ic_data = NULL;

	if (ts->pdata->virtual_key) {
		kfree(ts->pdata->virtual_key);
		ts->pdata->virtual_key = NULL;
	}

	devm_kfree(ts->dev, ts->xfer_buff);
	ts->xfer_buff = NULL;

	kfree(ts->pdata);
	ts->pdata = NULL;

	kfree(ts);
	ts = NULL;

	probe_fail_flag = 0;

	D("%s: Common section deinited!\n", __func__);
}

int himax_chip_common_suspend(struct himax_ts_data *ts)
{
	D("%s: ENTER ------\n", __func__);

	if (ts->suspended) {
		D("%s: Already suspended. Skipped.\n", __func__);
		goto END;
	} else {
		ts->suspended = true;
	}

#if defined(HX_SMART_WAKEUP) || defined(HX_HIGH_SENSE) ||                      \
	defined(HX_USB_DETECT_GLOBAL)
#if !defined(HX_RESUME_SEND_CMD)
	himax_mcu_resend_cmd_func(ts);
#endif
#endif

	himax_int_enable(ts, 0);

	if (!ts->use_irq) {
		int32_t cancel_state;

		cancel_state = cancel_work_sync(&ts->work);
		if (cancel_state)
			himax_int_enable(ts, 1);
	}

	/*ts->first_pressed = 0;*/
	atomic_set(&ts->suspend_mode, 1);
	ts->pre_finger_mask = 0;

	if (ts->pdata)
		if (ts->pdata->powerOff3V3 && ts->pdata->power)
			ts->pdata->power(0);

END:
	if (ts->in_self_test == 1)
		ts->suspend_resume_done = 1;

	D("%s: LEAVE ------\n", __func__);
	return 0;
}

int himax_chip_common_resume(struct himax_ts_data *ts)
{
	D("%s: ENTER ------\n", __func__);

	if (ts->suspended == false) {
		D("%s: It had entered resume, skip this step\n", __func__);
		goto END;
	} else {
		ts->suspended = false;
	}

#if defined(HX_ESD_RECOVERY)
	/* continuous N times record, not total N times. */
	ts->g_zero_event_count = 0;
#endif

	atomic_set(&ts->suspend_mode, 0);
	ts->diag_cmd = 0;

	if (ts->pdata)
		if (ts->pdata->powerOff3V3 && ts->pdata->power)
			ts->pdata->power(1);

#if defined(HX_RST_PIN_FUNC) && defined(HX_RESUME_HW_RESET)
	himax_mcu_ic_reset(ts, false, false);
#endif

#if defined(HX_SMART_WAKEUP) || defined(HX_HIGH_SENSE) ||                      \
	defined(HX_USB_DETECT_GLOBAL)
	himax_mcu_resend_cmd_func(ts);
#endif
	himax_report_all_leave_event(ts);
	himax_int_enable(ts, 1);

END:
	if (ts->in_self_test == 1)
		ts->suspend_resume_done = 1;

	D("%s: LEAVE ------\n", __func__);
	return 0;
}
