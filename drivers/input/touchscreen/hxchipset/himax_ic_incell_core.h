#ifndef _HIMAX_IC_INCELL_CORE_H_
#define _HIMAX_IC_INCELL_CORE_H_

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>

#include "himax_common.h"

int himax_write_read_reg(struct himax_ts_data *ts, uint8_t *tmp_addr,
				uint8_t *tmp_data, uint8_t hb, uint8_t lb);

void himax_mcu_in_cmd_init(struct himax_ts_data *ts);
int himax_mcu_in_cmd_struct_init(struct himax_ts_data *ts);
/* int himax_mcu_on_cmd_struct_init(void); */
/* void himax_mcu_on_cmd_init(void); */
	
/* used in himax_ic_HX83102 */
int himax_mcu_register_read(struct himax_ts_data *ts, uint8_t *read_addr,
			    uint32_t read_length, uint8_t *read_data,
			    uint8_t cfg_flag);
int himax_mcu_register_write(struct himax_ts_data *ts, uint8_t *write_addr,
			     uint32_t write_length, uint8_t *write_data,
			     uint8_t cfg_flag);
void himax_mcu_interface_on(struct himax_ts_data *ts);

/* used in himax_common / himax_ic_HX83102 */
void himax_mcu_ic_reset(struct himax_ts_data *ts, uint8_t loadconfig,
			uint8_t int_off);

/* used in himax_common */
void himax_mcu_resend_cmd_func(struct himax_ts_data *ts);

#if defined(HX_ESD_RECOVERY)
/* used in himax_common */
void himax_mcu_esd_ic_reset(struct himax_ts_data *ts);
#endif

/* used in himax_common */
int himax_mcu_get_touch_data_size(void);
int himax_mcu_cal_data_len(int raw_cnt_rmd, int HX_MAX_PT, int raw_cnt_max);
int himax_mcu_ic_esd_recovery(struct himax_ts_data *ts, int hx_esd_event,
			      int hx_zero_event, int length);
int himax_mcu_power_on_init(struct himax_ts_data *ts);

/* used in himax_ic_incell */
void himax_mcu_touch_information(struct himax_ts_data *ts);
int himax_mcu_assign_sorting_mode(struct himax_ts_data *ts, uint8_t *tmp_data);
void himax_mcu_dd_reg_en(struct himax_ts_data *ts, bool enable);
bool himax_mcu_dd_reg_write(struct himax_ts_data *ts, uint8_t addr,
			    uint8_t pa_num, int len, uint8_t *data,
			    uint8_t bank);
bool himax_mcu_dd_reg_read(struct himax_ts_data *ts, uint8_t addr,
			   uint8_t pa_num, int len, uint8_t *data,
			   uint8_t bank);

#if defined(HX_TP_PROC_GUEST_INFO)
int himax_guest_info_read(struct himax_ts_data *ts, uint32_t start_addr,
				 uint8_t *flash_tmp_buffer);
void himax_guest_info_set_status(int setting);
int himax_guest_info_get_status(void);
#endif /*HX_TP_PROC_GUEST_INFO*/

bool himax_mcu_diag_check_sum(struct himax_report_data *hx_touch_data);
int himax_mcu_determin_diag_storage(int diag_command);
int himax_mcu_determin_diag_rawdata(int diag_command);
int himax_mcu_hand_shaking(void);
int himax_mcu_check_sorting_mode(struct himax_ts_data *ts,
					uint8_t *tmp_data);
bool himax_mcu_read_event_stack(struct himax_ts_data *ts, uint8_t *buf,
				       uint8_t length);
void himax_mcu_system_reset(struct himax_ts_data *ts);
bool himax_mcu_wait_wip(struct himax_ts_data *ts, int Timing);
void himax_mcu_init_psl(struct himax_ts_data *ts);
bool himax_mcu_sense_off(struct himax_ts_data *ts, bool check_en);

#endif /*_HIMAX_IC_INCELL_CORE_H_*/