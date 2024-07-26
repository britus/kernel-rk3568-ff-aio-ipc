/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for ic core functions
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

#ifndef __HIMAX_IC_CORE_H__
#define __HIMAX_IC_CORE_H__

#include <linux/slab.h>

#include "himax_platform.h"
#include "himax_common.h"

#define DATA_LEN_8 8
#define DATA_LEN_4 4
#define ADDR_LEN_4 4
#define FLASH_RW_MAX_LEN 256
#define FLASH_WRITE_BURST_SZ 8
#define PROGRAM_SZ 48
#define MAX_I2C_TRANS_SZ 128
#define HIMAX_REG_RETRY_TIMES 5
#define FW_BIN_16K_SZ 0x4000
#define HIMAX_TOUCH_DATA_SIZE 128
#define MASK_BIT_0 0x01
#define MASK_BIT_1 0x02
#define MASK_BIT_2 0x04

#define FW_SECTOR_PER_BLOCK 8
#define FW_PAGE_PER_SECTOR 64
#define FW_PAGE_SEZE 128
#define HX256B 0x100
#define HX1K 0x400
#define HX4K 0x1000
#define HX_32K_SZ 0x8000
#define HX_40K_SZ 0xA000
#define HX_48K_SZ 0xC000
#define HX64K 0x10000
#define HX124K 0x1f000
#define HX4000K 0x1000000

#define HX_NORMAL_MODE 1
#define HX_SORTING_MODE 2
#define HX_CHANGE_MODE_FAIL (-1)
#define HX_RW_REG_FAIL (-1)
#define HX_DRIVER_MAX_IC_NUM 12

/* CORE_INIT */
/* CORE_IC */
/* CORE_FW */
/* CORE_FLASH */
/* CORE_SRAM */
/* CORE_DRIVER */

#define HX_0F_DEBUG

#if defined(CONFIG_TOUCHSCREEN_HIMAX_INCELL)
#if defined(HX_TP_PROC_GUEST_INFO)
extern struct hx_guest_info *g_guest_info_data;
#endif
void himax_mcu_in_cmd_struct_free(struct himax_ts_data *ts);
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_ONCELL)
void himax_mcu_on_cmd_struct_free(struct himax_ts_data *ts);
#endif

#if defined(HX_RST_PIN_FUNC)
extern u8 HX_HW_RESET_ACTIVATE;
void himax_rst_gpio_set(int pinnum, uint8_t value);
#endif

int himax_report_data_init(struct himax_ts_data *ts);
extern int i2c_error_count;

/* CORE_INIT */
int himax_mcu_in_cmd_struct_init(struct himax_ts_data *ts);
void himax_mcu_in_cmd_init(struct himax_ts_data *ts);
int himax_mcu_on_cmd_struct_init(void);
void himax_mcu_on_cmd_init(void);
void himax_parse_assign_cmd(uint32_t addr, uint8_t *cmd, int len);
/* CORE_INIT */

#if defined(HX_TP_PROC_GUEST_INFO)
#define HX_GUEST_INFO_FLASH_SADDR 0x20000
#define HX_GUEST_INFO_SIZE 10
#define HX_GUEST_INFO_LEN_SIZE 4
#define HX_GUEST_INFO_ID_SIZE 4

struct hx_guest_info {
	int g_guest_info_ongoing; /* 0 stop, 1 ongoing */
	uint8_t g_guest_str[10][128];
	uint8_t g_guest_str_in_format[10][128];
	uint8_t g_guest_data_type[10];
	int g_guest_data_len[10];
	int g_guest_info_type;
};
#endif

/* CORE_IC */
#define ic_adr_ahb_addr_byte_0 0x00
#define ic_adr_ahb_rdata_byte_0 0x08
#define ic_adr_ahb_access_direction 0x0c
#define ic_adr_conti 0x13
#define ic_adr_incr4 0x0D
#define ic_adr_i2c_psw_lb 0x31
#define ic_adr_i2c_psw_ub 0x32
#define ic_cmd_ahb_access_direction_read 0x00
#define ic_cmd_conti 0x31
#define ic_cmd_incr4 0x10
#define ic_cmd_i2c_psw_lb 0x27
#define ic_cmd_i2c_psw_ub 0x95
#define ic_adr_tcon_on_rst 0x80020020
#define ic_addr_adc_on_rst 0x80020094
#define ic_adr_psl 0x900000A0
#define ic_adr_cs_central_state 0x900000A8
#define ic_cmd_rst 0x00000000
#define ic_adr_osc_en 0x900880A8
#define ic_adr_osc_pw 0x900880E0

#define on_ic_adr_ahb_addr_byte_0 0x00
#define on_ic_adr_ahb_rdata_byte_0 0x08
#define on_ic_adr_ahb_access_direction 0x0c
#define on_ic_adr_conti 0x13
#define on_ic_adr_incr4 0x0D
#define on_ic_cmd_ahb_access_direction_read 0x00
#define on_ic_cmd_conti 0x31
#define on_ic_cmd_incr4 0x10
#define on_ic_adr_mcu_ctrl 0x82
#define on_ic_cmd_mcu_on 0x25
#define on_ic_cmd_mcu_off 0xDA
#define on_ic_adr_sleep_ctrl 0x99
#define on_ic_cmd_sleep_in 0x80
#define on_ic_adr_tcon_ctrl 0x80020000
#define on_ic_cmd_tcon_on 0x00000000
#define on_ic_adr_wdg_ctrl 0x9000800C
#define on_ic_cmd_wdg_psw 0x0000AC53
#define on_ic_adr_wdg_cnt_ctrl 0x90008010
#define on_ic_cmd_wdg_cnt_clr 0x000035CA
/* CORE_IC */

/* CORE_FW */
#define fw_addr_system_reset 0x90000018
#define fw_addr_safe_mode_release_pw 0x90000098
#define fw_addr_ctrl_fw 0x9000005c
#define fw_addr_flag_reset_event 0x900000e4
#define fw_addr_hsen_enable 0x10007F14
#define fw_addr_smwp_enable 0x10007F10
#define fw_usb_detect_addr 0x10007F38
#define fw_addr_program_reload_from 0x00000000
#define fw_addr_program_reload_to 0x08000000
#define fw_addr_program_reload_page_write 0x0000fb00
#define fw_addr_raw_out_sel 0x800204b4
#define fw_addr_reload_status 0x80050000
#define fw_addr_reload_crc32_result 0x80050018
#define fw_addr_reload_addr_from 0x80050020
#define fw_addr_reload_addr_cmd_beat 0x80050028
#define fw_data_system_reset 0x00000055
#define fw_data_safe_mode_release_pw_active 0x00000053
#define fw_data_safe_mode_release_pw_reset 0x00000000
#define fw_data_clear 0x00000000
#define fw_data_fw_stop 0x000000A5
#define fw_data_program_reload_start 0x0A3C3000
#define fw_data_program_reload_compare 0x04663000
#define fw_data_program_reload_break 0x15E75678
#define fw_addr_selftest_addr_en 0x10007F18
#define fw_addr_selftest_result_addr 0x10007f24
#define fw_data_selftest_request 0x00006AA6
#define fw_addr_criteria_addr 0x10007f1c
#define fw_data_criteria_aa_top 0x64
#define fw_data_criteria_aa_bot 0x00
#define fw_data_criteria_key_top 0x64
#define fw_data_criteria_key_bot 0x00
#define fw_data_criteria_avg_top 0x64
#define fw_data_criteria_avg_bot 0x00
#define fw_addr_set_frame_addr 0x10007294
#define fw_data_set_frame 0x0000000A
#define fw_data_selftest_ack_hb 0xa6
#define fw_data_selftest_ack_lb 0x6a
#define fw_data_selftest_pass 0xaa
#define fw_data_normal_cmd 0x00
#define fw_data_normal_status 0x99
#define fw_data_sorting_cmd 0xaa
#define fw_data_sorting_status 0xcc
#define fw_data_idle_dis_pwd 0x17
#define fw_data_idle_en_pwd 0x1f
#define fw_addr_sorting_mode_en 0x10007f04
#define fw_addr_fw_mode_status 0x10007088
#define fw_addr_icid_addr 0x900000d0
#define fw_addr_fw_ver_addr 0x10007004
#define fw_addr_fw_cfg_addr 0x10007084
#define fw_addr_fw_vendor_addr 0x10007000
#define fw_addr_cus_info 0x10007008
#define fw_addr_proj_info 0x10007014
#define fw_addr_fw_state_addr 0x900000f8
#define fw_addr_fw_dbg_msg_addr 0x10007f40
#define fw_addr_chk_fw_status 0x900000a8
#define fw_addr_dd_handshak_addr 0x900000fc
#define fw_addr_dd_data_addr 0x10007f80
#define fw_data_dd_request 0xaa
#define fw_data_dd_ack 0xbb
#define fw_data_rawdata_ready_hb 0xa3
#define fw_data_rawdata_ready_lb 0x3a
#define fw_addr_ahb_addr 0x11
#define fw_data_ahb_dis 0x00
#define fw_data_ahb_en 0x01
#define fw_addr_event_addr 0x30
#define fw_func_handshaking_pwd 0xA55AA55A
#define fw_func_handshaking_end 0x77887788
#define fw_addr_ulpm_33 0x33
#define fw_addr_ulpm_34 0x34
#define fw_data_ulpm_11 0x11
#define fw_data_ulpm_22 0x22
#define fw_data_ulpm_33 0x33
#define fw_data_ulpm_aa 0xAA

#define on_fw_addr_smwp_enable 0xA2
#define on_fw_usb_detect_addr 0xA4
#define on_fw_addr_program_reload_from 0x00000000
#define on_fw_addr_raw_out_sel 0x98
#define on_fw_addr_flash_checksum 0x80000044
#define on_fw_data_flash_checksum 0x00000491
#define on_fw_addr_crc_value 0x80000050
#define on_fw_data_safe_mode_release_pw_active 0x00000053
#define on_fw_data_safe_mode_release_pw_reset 0x00000000
#define on_fw_addr_criteria_addr 0x9A
#define on_fw_data_selftest_pass 0xaa
#define on_fw_addr_reK_crtl 0x8000000C
#define on_fw_data_reK_en 0x02
#define on_fw_data_reK_dis 0xFD
#define on_fw_data_rst_init 0xF0
#define on_fw_data_dc_set 0x02
#define on_fw_data_bank_set 0x03
#define on_fw_addr_selftest_addr_en 0x98
#define on_fw_addr_selftest_result_addr 0x9B
#define on_fw_data_selftest_request 0x06
#define on_fw_data_thx_avg_mul_dc_lsb 0x22
#define on_fw_data_thx_avg_mul_dc_msb 0x0B
#define on_fw_data_thx_mul_dc_up_low_bud 0x64
#define on_fw_data_thx_avg_slf_dc_lsb 0x14
#define on_fw_data_thx_avg_slf_dc_msb 0x05
#define on_fw_data_thx_slf_dc_up_low_bud 0x64
#define on_fw_data_thx_slf_bank_up 0x40
#define on_fw_data_thx_slf_bank_low 0x00
#define on_fw_data_idle_dis_pwd 0x40
#define on_fw_data_idle_en_pwd 0x00
#define on_fw_addr_fw_mode_status 0x99
#define on_fw_addr_icid_addr 0x900000d0
#define on_fw_addr_fw_ver_start 0x90
#define on_fw_data_rawdata_ready_hb 0xa3
#define on_fw_data_rawdata_ready_lb 0x3a
#define on_fw_addr_ahb_addr 0x11
#define on_fw_data_ahb_dis 0x00
#define on_fw_data_ahb_en 0x01
#define on_fw_addr_event_addr 0x30
/* CORE_FW */

/* CORE_FLASH */
#define flash_addr_ctrl_base 0x80000000
#define flash_addr_spi200_trans_fmt (flash_addr_ctrl_base + 0x10)
#define flash_addr_spi200_trans_ctrl (flash_addr_ctrl_base + 0x20)
#define flash_addr_spi200_cmd (flash_addr_ctrl_base + 0x24)
#define flash_addr_spi200_addr (flash_addr_ctrl_base + 0x28)
#define flash_addr_spi200_data (flash_addr_ctrl_base + 0x2c)
#define flash_addr_spi200_fifo_rst (flash_addr_ctrl_base + 0x30)
#define flash_addr_spi200_rst_status (flash_addr_ctrl_base + 0x34)
#define flash_addr_spi200_flash_speed (flash_addr_ctrl_base + 0x40)
#define flash_addr_spi200_bt_num (flash_addr_ctrl_base + 0xe8)
#define flash_data_spi200_txfifo_rst 0x00000004
#define flash_data_spi200_rxfifo_rst 0x00000002
#define flash_data_spi200_trans_fmt 0x00020780
#define flash_data_spi200_trans_ctrl_1 0x42000003
#define flash_data_spi200_trans_ctrl_2 0x47000000
#define flash_data_spi200_trans_ctrl_3 0x67000000
#define flash_data_spi200_trans_ctrl_4 0x610ff000
#define flash_data_spi200_trans_ctrl_5 0x694002ff
#define flash_data_spi200_trans_ctrl_6 0x42000000
#define flash_data_spi200_trans_ctrl_7 0x6940020f
#define flash_data_spi200_cmd_1 0x00000005
#define flash_data_spi200_cmd_2 0x00000006
#define flash_data_spi200_cmd_3 0x000000C7
#define flash_data_spi200_cmd_4 0x000000D8
#define flash_data_spi200_cmd_5 0x00000020
#define flash_data_spi200_cmd_6 0x00000002
#define flash_data_spi200_cmd_7 0x0000003b
#define flash_data_spi200_cmd_8 0x00000003
#define flash_data_spi200_addr 0x00000000

#define on_flash_addr_ctrl_base 0x80000000
#define on_flash_addr_ctrl_auto 0x80000004
#define on_flash_data_main_erase 0x0000A50D
#define on_flash_data_auto 0xA5
#define on_flash_data_main_read 0x03
#define on_flash_data_page_write 0x05
#define on_flash_data_spp_read 0x10
#define on_flash_data_sfr_read 0x14
#define on_flash_addr_ahb_ctrl 0x80000020
#define on_flash_data_ahb_squit 0x00000001
#define on_flash_addr_unlock_0 0x00000000
#define on_flash_addr_unlock_4 0x00000004
#define on_flash_addr_unlock_8 0x00000008
#define on_flash_addr_unlock_c 0x0000000C
#define on_flash_data_cmd0 0x28178EA0
#define on_flash_data_cmd1 0x0A0E03FF
#define on_flash_data_cmd2 0x8C203D0C
#define on_flash_data_cmd3 0x00300263
#define on_flash_data_lock 0x03400000
/* CORE_FLASH */

/* CORE_SRAM */
#define sram_adr_mkey 0x100070E8
#define sram_adr_rawdata_addr 0x10000000
#define sram_adr_rawdata_end 0x00000000
#define sram_passwrd_start 0x5AA5
#define sram_passwrd_end 0xA55A

#define on_sram_adr_rawdata_addr 0x080002E0
#define on_sram_adr_rawdata_end 0x00000000
#define on_sram_cmd_conti 0x44332211
#define on_sram_cmd_fin 0x00000000
#define on_sram_passwrd_start 0x5AA5
#define on_sram_passwrd_end 0xA55A
/* CORE_SRAM */

/* CORE_DRIVER */
#define driver_addr_fw_define_flash_reload 0x10007f00
#define driver_addr_fw_define_2nd_flash_reload 0x100072c0
#define driver_data_fw_define_flash_reload_dis 0x0000a55a
#define driver_data_fw_define_flash_reload_en 0x00000000
#define driver_addr_fw_define_int_is_edge 0x10007088
#define driver_addr_fw_define_rxnum_txnum_maxpt 0x100070f4
#define driver_data_fw_define_rxnum_txnum_maxpt_sorting 0x00000008
#define driver_data_fw_define_rxnum_txnum_maxpt_normal 0x00000014
#define driver_addr_fw_define_xy_res_enable 0x100070f8
#define driver_addr_fw_define_x_y_res 0x100070fc
#define driver_data_df_rx 36
#define driver_data_df_tx 18
#define driver_data_df_pt 10
#define driver_data_df_x_res 1080
#define driver_data_df_y_res 1920
#define on_driver_addr_fw_define_int_is_edge 0x10007088
#define on_driver_data_df_rx 28
#define on_driver_data_df_tx 14
#define on_driver_data_df_pt 10
#define on_driver_data_df_x_res 1080
#define on_driver_data_df_y_res 1920
#if !defined(HX_NEW_EVENT_STACK_FORMAT)
#define on_driver_addr_fw_rx_tx_maxpt_num 0x0800001C
#define on_driver_addr_fw_xy_rev_int_edge 0x0800000C
#define on_driver_addr_fw_define_x_y_res 0x08000030
#else
#define on_driver_addr_fw_rx_tx_maxpt_num 0x08000004
#define on_driver_addr_fw_maxpt_bt_num 0x0800000C
#define on_driver_addr_fw_xy_rev_int_edge 0x08000110
#define on_driver_addr_fw_define_x_y_res 0x08000010
#endif

struct on_ic_operation {
	uint8_t addr_ahb_addr_byte_0[1];
	uint8_t addr_ahb_rdata_byte_0[1];
	uint8_t addr_ahb_access_direction[1];
	uint8_t addr_conti[1];
	uint8_t addr_incr4[1];
	uint8_t adr_mcu_ctrl[1];
	uint8_t data_ahb_access_direction_read[1];
	uint8_t data_conti[1];
	uint8_t data_incr4[1];
	uint8_t cmd_mcu_on[1];
	uint8_t cmd_mcu_off[1];
	uint8_t adr_sleep_ctrl[1];
	uint8_t cmd_sleep_in[1];
	uint8_t adr_tcon_ctrl[4];
	uint8_t cmd_tcon_on[4];
	uint8_t adr_wdg_ctrl[4];
	uint8_t cmd_wdg_psw[4];
	uint8_t adr_wdg_cnt_ctrl[4];
	uint8_t cmd_wdg_cnt_clr[4];
};

struct on_fw_operation {
	uint8_t addr_smwp_enable[1];
	uint8_t addr_program_reload_from[4];
	uint8_t addr_raw_out_sel[1];
	uint8_t addr_flash_checksum[4];
	uint8_t data_flash_checksum[4];
	uint8_t addr_crc_value[4];
	uint8_t addr_reload_status[4];
	uint8_t addr_reload_crc32_result[4];
	uint8_t addr_reload_addr_from[4];
	uint8_t addr_reload_addr_cmd_beat[4];
	uint8_t addr_set_frame_addr[4];
	uint8_t addr_fw_mode_status[1];
	uint8_t addr_icid_addr[4];
	uint8_t addr_fw_ver_start[1];
	uint8_t data_safe_mode_release_pw_active[4];
	uint8_t data_safe_mode_release_pw_reset[4];
	uint8_t data_clear[4];
	uint8_t addr_criteria_addr[1];
	uint8_t data_selftest_pass[1];
	uint8_t addr_reK_crtl[4];
	uint8_t data_reK_en[1];
	uint8_t data_reK_dis[1];
	uint8_t data_rst_init[1];
	uint8_t data_dc_set[1];
	uint8_t data_bank_set[1];
	uint8_t addr_selftest_addr_en[1];
	uint8_t addr_selftest_result_addr[1];
	uint8_t data_selftest_request[1];
	uint8_t data_thx_avg_mul_dc_lsb[1];
	uint8_t data_thx_avg_mul_dc_msb[1];
	uint8_t data_thx_mul_dc_up_low_bud[1];
	uint8_t data_thx_avg_slf_dc_lsb[1];
	uint8_t data_thx_avg_slf_dc_msb[1];
	uint8_t data_thx_slf_dc_up_low_bud[1];
	uint8_t data_thx_slf_bank_up[1];
	uint8_t data_thx_slf_bank_low[1];
	uint8_t data_idle_dis_pwd[1];
	uint8_t data_idle_en_pwd[1];
	uint8_t data_rawdata_ready_hb[1];
	uint8_t data_rawdata_ready_lb[1];
	uint8_t addr_ahb_addr[1];
	uint8_t data_ahb_dis[1];
	uint8_t data_ahb_en[1];
	uint8_t addr_event_addr[1];
	uint8_t addr_usb_detect[1];
};

struct on_flash_operation {
	uint8_t addr_ctrl_base[4];
	uint8_t addr_ctrl_auto[4];
	uint8_t data_main_erase[4];
	uint8_t data_auto[1];
	uint8_t data_main_read[1];
	uint8_t data_page_write[1];
	uint8_t data_sfr_read[1];
	uint8_t data_spp_read[1];
	uint8_t addr_ahb_ctrl[4];
	uint8_t data_ahb_squit[4];

	uint8_t addr_unlock_0[4];
	uint8_t addr_unlock_4[4];
	uint8_t addr_unlock_8[4];
	uint8_t addr_unlock_c[4];
	uint8_t data_cmd0[4];
	uint8_t data_cmd1[4];
	uint8_t data_cmd2[4];
	uint8_t data_cmd3[4];
	uint8_t data_lock[4];
};

struct on_sram_operation {
	uint8_t addr_rawdata_addr[4];
	uint8_t addr_rawdata_end[4];
	uint8_t data_conti[4];
	uint8_t data_fin[4];
	uint8_t passwrd_start[2];
	uint8_t passwrd_end[2];
};

struct on_driver_operation {
	uint8_t addr_fw_define_int_is_edge[4];
	uint8_t addr_fw_rx_tx_maxpt_num[4];
#if defined(HX_NEW_EVENT_STACK_FORMAT)
	uint8_t addr_fw_maxpt_bt_num[4];
#endif
	uint8_t addr_fw_xy_rev_int_edge[4];
	uint8_t addr_fw_define_x_y_res[4];
	uint8_t data_fw_define_rxnum_txnum_maxpt_sorting[4];
	uint8_t data_fw_define_rxnum_txnum_maxpt_normal[4];
	uint8_t data_df_rx[1];
	uint8_t data_df_tx[1];
	uint8_t data_df_pt[1];
	uint8_t data_df_x_res[2];
	uint8_t data_df_y_res[2];
};

struct himax_on_core_command_operation {
	struct on_ic_operation *ic_op;
	struct on_fw_operation *fw_op;
	struct on_flash_operation *flash_op;
	struct on_sram_operation *sram_op;
	struct on_driver_operation *driver_op;
};

#endif
