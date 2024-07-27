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

#ifndef HIMAX_COMMON_H
#define HIMAX_COMMON_H

#include <asm/segment.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/async.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/pm_wakeup.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/kallsyms.h>

#if defined(CONFIG_OF)
#include <linux/of_gpio.h>
#endif

#define HIMAX_DRIVER_VER "2.0.0.51_BOE10.1_01"

/*===========Himax Option function=============*/
#define HX_RST_PIN_FUNC
#define HX_ESD_RECOVERY
#define HX_GESTURE_TRACK
#define HX_RESUME_SEND_CMD /*Need to enable on TDDI chipset*/
#define HX_HIGH_SENSE

#define HX_TP_PROC_GUEST_INFO

#define HX_MAX_WRITE_SZ (64 * 1024 + 4)
#define HX_KEY_MAX_COUNT 4
#define DEFAULT_RETRY_CNT 3
#define HX_TP_BIN_CHECKSUM_SW 1
#define HX_TP_BIN_CHECKSUM_HW 2
#define HX_TP_BIN_CHECKSUM_CRC 3

#define HX_83102A_SERIES_PWON "HX83102A"
#define HX_83102B_SERIES_PWON "HX83102B"
#define HX_83102D_SERIES_PWON "HX83102D"
#define HX_83102E_SERIES_PWON "HX83102E"

#define SHIFTBITS 5

#define FW_SIZE_32k 32768
#define FW_SIZE_60k 61440
#define FW_SIZE_64k 65536
#define FW_SIZE_124k 126976
#define FW_SIZE_128k 131072

#define NO_ERR 0
#define READY_TO_SERVE 1
#define WORK_OUT 2
#define I2C_FAIL -1
#define HX_INIT_FAIL -1
#define MEM_ALLOC_FAIL -2
#define CHECKSUM_FAIL -3
#define GESTURE_DETECT_FAIL -4
#define INPUT_REGISTER_FAIL -5
#define FW_NOT_READY -6
#define LENGTH_FAIL -7
#define OPEN_FILE_FAIL -8
#define PROBE_FAIL -9
#define ERR_WORK_OUT -10
#define ERR_STS_WRONG -11
#define ERR_TEST_FAIL -12
#define HW_CRC_FAIL 1

#define HX_FINGER_ON 1
#define HX_FINGER_LEAVE 2

// TODO: ??? PEN SUPPORT
#define PEN_INFO_SZ 12

enum HX_TS_PATH {
	HX_REPORT_COORD = 1,
	HX_REPORT_SMWP_EVENT,
	HX_REPORT_COORD_RAWDATA,
};

enum HX_TS_STATUS {
	HX_TS_GET_DATA_FAIL = -4,
	HX_ESD_EVENT,
	HX_CHKSUM_FAIL,
	HX_PATH_FAIL,
	HX_TS_NORMAL_END = 0,
	HX_ESD_REC_OK,
	HX_READY_SERVE,
	HX_REPORT_DATA,
	HX_ESD_WARNING,
	HX_IC_RUNNING,
	HX_ZERO_EVENT_COUNT,
	HX_RST_OK,
};

enum cell_type {
	CHIP_IS_IN_CELL = 1,
};

/* CORE_DRIVER */
struct ic_operation {
	uint8_t addr_ahb_addr_byte_0[1];
	uint8_t addr_ahb_rdata_byte_0[1];
	uint8_t addr_ahb_access_direction[1];
	uint8_t addr_conti[1];
	uint8_t addr_incr4[1];
	uint8_t adr_i2c_psw_lb[1];
	uint8_t adr_i2c_psw_ub[1];
	uint8_t data_ahb_access_direction_read[1];
	uint8_t data_conti[1];
	uint8_t data_incr4[1];
	uint8_t data_i2c_psw_lb[1];
	uint8_t data_i2c_psw_ub[1];
	uint8_t addr_tcon_on_rst[4];
	uint8_t addr_adc_on_rst[4];
	uint8_t addr_psl[4];
	uint8_t addr_cs_central_state[4];
	uint8_t data_rst[4];
	uint8_t adr_osc_en[4];
	uint8_t adr_osc_pw[4];
};

struct fw_operation {
	uint8_t addr_system_reset[4];
	uint8_t addr_safe_mode_release_pw[4];
	uint8_t addr_ctrl_fw_isr[4];
	uint8_t addr_flag_reset_event[4];
	uint8_t addr_hsen_enable[4];
	uint8_t addr_smwp_enable[4];
	uint8_t addr_program_reload_from[4];
	uint8_t addr_program_reload_to[4];
	uint8_t addr_program_reload_page_write[4];
	uint8_t addr_raw_out_sel[4];
	uint8_t addr_reload_status[4];
	uint8_t addr_reload_crc32_result[4];
	uint8_t addr_reload_addr_from[4];
	uint8_t addr_reload_addr_cmd_beat[4];
	uint8_t addr_selftest_addr_en[4];
	uint8_t addr_criteria_addr[4];
	uint8_t addr_set_frame_addr[4];
	uint8_t addr_selftest_result_addr[4];
	uint8_t addr_sorting_mode_en[4];
	uint8_t addr_fw_mode_status[4];
	uint8_t addr_icid_addr[4];
	uint8_t addr_fw_ver_addr[4];
	uint8_t addr_fw_cfg_addr[4];
	uint8_t addr_fw_vendor_addr[4];
	uint8_t addr_cus_info[4];
	uint8_t addr_proj_info[4];
	uint8_t addr_fw_state_addr[4];
	uint8_t addr_fw_dbg_msg_addr[4];
	uint8_t addr_chk_fw_status[4];
	uint8_t addr_dd_handshak_addr[4];
	uint8_t addr_dd_data_addr[4];
	uint8_t data_system_reset[4];
	uint8_t data_safe_mode_release_pw_active[4];
	uint8_t data_safe_mode_release_pw_reset[4];
	uint8_t data_clear[4];
	uint8_t data_fw_stop[4];
	uint8_t data_program_reload_start[4];
	uint8_t data_program_reload_compare[4];
	uint8_t data_program_reload_break[4];
	uint8_t data_selftest_request[4];
	uint8_t data_criteria_aa_top[1];
	uint8_t data_criteria_aa_bot[1];
	uint8_t data_criteria_key_top[1];
	uint8_t data_criteria_key_bot[1];
	uint8_t data_criteria_avg_top[1];
	uint8_t data_criteria_avg_bot[1];
	uint8_t data_set_frame[4];
	uint8_t data_selftest_ack_hb[1];
	uint8_t data_selftest_ack_lb[1];
	uint8_t data_selftest_pass[1];
	uint8_t data_normal_cmd[1];
	uint8_t data_normal_status[1];
	uint8_t data_sorting_cmd[1];
	uint8_t data_sorting_status[1];
	uint8_t data_dd_request[1];
	uint8_t data_dd_ack[1];
	uint8_t data_idle_dis_pwd[1];
	uint8_t data_idle_en_pwd[1];
	uint8_t data_rawdata_ready_hb[1];
	uint8_t data_rawdata_ready_lb[1];
	uint8_t addr_ahb_addr[1];
	uint8_t data_ahb_dis[1];
	uint8_t data_ahb_en[1];
	uint8_t addr_event_addr[1];
	uint8_t addr_usb_detect[4];
	uint8_t addr_ulpm_33[1];
	uint8_t addr_ulpm_34[1];
	uint8_t data_ulpm_11[1];
	uint8_t data_ulpm_22[1];
	uint8_t data_ulpm_33[1];
	uint8_t data_ulpm_aa[1];
};

struct flash_operation {
	uint8_t addr_spi200_trans_fmt[4];
	uint8_t addr_spi200_trans_ctrl[4];
	uint8_t addr_spi200_fifo_rst[4];
	uint8_t addr_spi200_rst_status[4];
	uint8_t addr_spi200_flash_speed[4];
	uint8_t addr_spi200_cmd[4];
	uint8_t addr_spi200_addr[4];
	uint8_t addr_spi200_data[4];
	uint8_t addr_spi200_bt_num[4];

	uint8_t data_spi200_txfifo_rst[4];
	uint8_t data_spi200_rxfifo_rst[4];
	uint8_t data_spi200_trans_fmt[4];
	uint8_t data_spi200_trans_ctrl_1[4];
	uint8_t data_spi200_trans_ctrl_2[4];
	uint8_t data_spi200_trans_ctrl_3[4];
	uint8_t data_spi200_trans_ctrl_4[4];
	uint8_t data_spi200_trans_ctrl_5[4];
	uint8_t data_spi200_trans_ctrl_6[4];
	uint8_t data_spi200_trans_ctrl_7[4];
	uint8_t data_spi200_cmd_1[4];
	uint8_t data_spi200_cmd_2[4];
	uint8_t data_spi200_cmd_3[4];
	uint8_t data_spi200_cmd_4[4];
	uint8_t data_spi200_cmd_5[4];
	uint8_t data_spi200_cmd_6[4];
	uint8_t data_spi200_cmd_7[4];
	uint8_t data_spi200_cmd_8[4];
	uint8_t data_spi200_addr[4];
};

struct sram_operation {
	uint8_t addr_mkey[4];
	uint8_t addr_rawdata_addr[4];
	uint8_t addr_rawdata_end[4];
	uint8_t passwrd_start[2];
	uint8_t passwrd_end[2];
};

struct driver_operation {
	uint8_t addr_fw_define_flash_reload[4];
	uint8_t addr_fw_define_2nd_flash_reload[4];
	uint8_t addr_fw_define_int_is_edge[4];
	uint8_t addr_fw_define_rxnum_txnum_maxpt[4];
	uint8_t addr_fw_define_xy_res_enable[4];
	uint8_t addr_fw_define_x_y_res[4];
	uint8_t data_df_rx[1];
	uint8_t data_df_tx[1];
	uint8_t data_df_pt[1];
	uint8_t data_df_x_res[2];
	uint8_t data_df_y_res[2];
	uint8_t data_fw_define_flash_reload_dis[4];
	uint8_t data_fw_define_flash_reload_en[4];
	uint8_t data_fw_define_rxnum_txnum_maxpt_sorting[4];
	uint8_t data_fw_define_rxnum_txnum_maxpt_normal[4];
};

struct zf_operation {
	uint8_t data_dis_flash_reload[4];
	uint8_t addr_system_reset[4];
	uint8_t data_system_reset[4];
	uint8_t data_sram_start_addr[4];
	uint8_t data_sram_clean[4];
	uint8_t data_cfg_info[4];
	uint8_t data_fw_cfg_1[4];
	uint8_t data_fw_cfg_2[4];
	uint8_t data_fw_cfg_3[4];
	uint8_t data_adc_cfg_1[4];
	uint8_t data_adc_cfg_2[4];
	uint8_t data_adc_cfg_3[4];
	uint8_t data_map_table[4];
	/*	uint8_t data_mode_switch[4];*/
	uint8_t addr_sts_chk[4];
	uint8_t data_activ_sts[1];
	uint8_t addr_activ_relod[4];
	uint8_t data_activ_in[1];
};

struct himax_core_command_operation {
	struct ic_operation *ic_op;
	struct fw_operation *fw_op;
	struct flash_operation *flash_op;
	struct sram_operation *sram_op;
	struct driver_operation *driver_op;
	struct zf_operation *zf_op;
};

struct himax_ic_data {
	int vendor_fw_ver;
	int vendor_config_ver;
	int vendor_touch_cfg_ver;
	int vendor_display_cfg_ver;
	int vendor_cid_maj_ver;
	int vendor_cid_min_ver;
	int vendor_panel_ver;
	int vendor_sensor_id;
	int ic_adc_num;
	uint8_t vendor_cus_info[12];
	uint8_t vendor_proj_info[12];
	uint8_t vendor_ic_id[13];
	int HX_RX_NUM;
	int HX_TX_NUM;
	int HX_BT_NUM;
	int HX_X_RES;
	int HX_Y_RES;
	int HX_MAX_PT;
	bool HX_XY_REVERSE;
	bool HX_INT_IS_EDGE;
	bool HX_PEN_FUNC;
};

struct himax_virtual_key {
	int index;
	int keycode;
	int x_range_min;
	int x_range_max;
	int y_range_min;
	int y_range_max;
};

struct himax_target_report_data {
	int *x;
	int *y;
	int *w;
	int *finger_id;
	int finger_on;
	int finger_num;

	int32_t *p_x;
	int32_t *p_y;
	int32_t *p_w;
	int32_t *pen_id;
	uint32_t *p_hover;
	int32_t *p_tilt_x;
	uint32_t *p_btn;
	uint32_t *p_btn2;
	int32_t *p_tilt_y;
	uint32_t *p_on;
	int pre_p_btn;
	int pre_p_btn2;

	int ig_count;
};

struct himax_report_data {
	int touch_all_size;
	int raw_cnt_max;
	int raw_cnt_rmd;
	int touch_info_size;
	uint8_t finger_num;
	uint8_t finger_on;
	uint8_t *hx_coord_buf;
	uint8_t hx_state_info[2];
	int rawdata_size;
	uint8_t diag_cmd;
	uint8_t *hx_rawdata_buf;
	uint8_t rawdata_frame_size;
};

struct himax_ts_data {
	bool initialized;
	bool suspended;
	atomic_t suspend_mode;
	uint8_t x_channel;
	uint8_t y_channel;
	uint8_t useScreenRes;
	uint8_t diag_cmd;
	char chip_name[30];
	uint8_t chip_cell_type;

	uint8_t protocol_type;
	uint8_t first_pressed;
	uint8_t coord_data_size;
	uint8_t area_data_size;
	uint8_t coordInfoSize;
	uint8_t raw_data_frame_size;
	uint8_t raw_data_nframes;
	uint8_t nFinger_support;
	uint8_t irq_enabled;
	uint8_t diag_self[50];

	uint16_t finger_pressed;
	uint16_t last_slot;
	uint16_t pre_finger_mask;
	uint16_t old_finger;
	int hx_point_num;

	uint32_t debug_log_level;
	uint32_t widthFactor;
	uint32_t heightFactor;
	uint32_t tw_x_min;
	uint32_t tw_x_max;
	uint32_t tw_y_min;
	uint32_t tw_y_max;
	uint32_t pl_x_min;
	uint32_t pl_x_max;
	uint32_t pl_y_min;
	uint32_t pl_y_max;

	int rst_gpio;
	int use_irq;
	int (*power)(int on);
	int pre_finger_data[10][2];

	struct device *dev;
	struct workqueue_struct *himax_wq;
	struct work_struct work;
	struct input_dev *input_dev;

	struct input_dev *hx_pen_dev;

	struct hrtimer timer;
	struct i2c_client *client;
	struct himax_i2c_platform_data *pdata;
	struct himax_virtual_key *button;
	struct mutex rw_lock;
	atomic_t irq_state;
	spinlock_t irq_lock;

	/******* SPI-start *******/
	int hx_irq;
	struct spi_device *spi;
	uint8_t *xfer_buff;
	/******* SPI-end *******/

	int in_self_test;
	int suspend_resume_done;
	int bus_speed;

	struct workqueue_struct *flash_wq;
	struct work_struct flash_work;

	struct workqueue_struct *himax_diag_wq;
	struct delayed_work himax_diag_delay_wrok;

#if defined(HX_HIGH_SENSE)
	uint8_t HSEN_enable;
#endif

#if defined(HX_TP_PROC_GUEST_INFO)
	struct workqueue_struct *guest_info_wq;
	struct work_struct guest_info_work;
#endif

	struct himax_ic_data *ic_data;
	struct himax_report_data *hx_touch_data;

	unsigned char IC_CHECKSUM;

	int HX_TOUCH_INFO_POINT_CNT;

#if defined(HX_ESD_RECOVERY)
	u8 HX_ESD_RESET_ACTIVATE;
	int g_zero_event_count;
	int hx_EB_event_flag;
	int hx_EC_event_flag;
	int hx_ED_event_flag;
#endif
	
#if defined(HX_RST_PIN_FUNC)
	u8 HX_HW_RESET_ACTIVATE;
#endif

	uint8_t AA_press;
	uint8_t EN_NoiseFilter;
	uint8_t Last_EN_NoiseFilter;

	/* core command operations addresses */
	struct himax_core_command_operation *g_core_cmd_op;

	/* debug */
	int i2c_error_count;
};

enum input_protocol_type {
	PROTOCOL_TYPE_A = 0x00,
	PROTOCOL_TYPE_B = 0x01,
	PROTOCOL_TYPE_B_3PA = 0x02,
};

#if defined(HX_HIGH_SENSE)
void himax_set_HSEN_func(uint8_t HSEN_enable);
#endif

#if defined(HX_GESTURE_TRACK)
#define GEST_PT_MAX_NUM (128)
#endif

extern int g_mmi_refcnt;
extern int *g_inspt_crtra_flag;

int himax_chip_common_suspend(struct himax_ts_data *ts);
int himax_chip_common_resume(struct himax_ts_data *ts);

void himax_parse_assign_cmd(uint32_t addr, uint8_t *cmd, int len);

int himax_parse_dt(struct himax_ts_data *ts,
		   struct himax_i2c_platform_data *pdata);

int himax_report_data_init(struct himax_ts_data *ts);
int himax_report_data(struct himax_ts_data *ts, int ts_path, int ts_status);

void himax_parse_assign_cmd(uint32_t addr, uint8_t *cmd, int len);

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
char* himax_common_rdtohex(uint8_t *data, uint32_t length);
char* himax_common_wdtohex(uint8_t *data, uint32_t length);
#define rdtohex(a, b) himax_common_rdtohex(a, b)
#define wdtohex(a, b) himax_common_wdtohex(a, b)
#else
#define rdtohex(a, b) ""
#define wdtohex(a, b) ""
#endif

#endif
