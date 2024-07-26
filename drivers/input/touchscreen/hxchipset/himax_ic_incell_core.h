#ifndef _HIMAX_IC_INCELL_CORE_H_
#define _HIMAX_IC_INCELL_CORE_H_

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>

#include "himax_common.h"
#include "himax_platform.h"

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

#endif /*_HIMAX_IC_INCELL_CORE_H_*/