/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for QCT platform
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

#ifndef HIMAX_PLATFORM_H
#define HIMAX_PLATFORM_H

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>

#include "himax_common.h"

#define HIMAX_I2C_RETRY_TIMES 3

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
#define D(x...) pr_debug("[himax] " x)
#define I(x...) pr_info("[himax] " x)
#define W(x...) pr_warn("[himax] " x)
#define E(x...) pr_err("[himax] " x)
#define DIF(x...)                                                              \
	do {                                                                   \
		if (debug_flag)                                                \
			pr_debug("[himax] " x)                                 \
	} while (0)
#else
#define D(x...)
#define I(x...) pr_info("[himax] " x)
#define W(x...) pr_warn("[himax] " x)
#define E(x...) pr_err("[himax] " x)
#define DIF(x...)
#endif

#define HIMAX_I2C_ADDR 0x48
#define HIMAX_common_NAME "himax_tp"
#define INPUT_DEV_NAME "himax-touchscreen"

struct himax_i2c_platform_data {
	int abs_x_min;
	int abs_x_max;
	int abs_x_fuzz;
	int abs_y_min;
	int abs_y_max;
	int abs_y_fuzz;
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_pressure_fuzz;
	int abs_width_min;
	int abs_width_max;
	int screenWidth;
	int screenHeight;
	uint8_t fw_version;
	uint8_t tw_id;
	uint8_t powerOff3V3;
	uint8_t cable_config[2];
	uint8_t protocol_type;
	int gpio_irq;
	int gpio_reset;
	int gpio_3v3_en;
	int gpio_pon;
	int (*power)(int on);
	void (*reset)(void);
	struct himax_virtual_key *virtual_key;
	struct kobject *vk_obj;
	struct kobj_attribute *vk2Use;
	int hx_config_size;
};

/* forward decl from himax_common.h */
enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer);
int himax_bus_read(struct i2c_client *, uint8_t command, uint8_t *data,
	  uint32_t length, uint8_t toRetry);
int himax_bus_write(struct i2c_client *, uint8_t command, uint8_t *data,
	   uint32_t length, uint8_t toRetry);
int himax_bus_write_command(struct i2c_client *, uint8_t command,
		   uint8_t toRetry);
int himax_ts_register_interrupt(struct himax_ts_data *);
void himax_ts_work(struct himax_ts_data *ts);
int himax_ts_unregister_interrupt(struct himax_ts_data *ts);
int himax_chip_common_init(struct himax_ts_data *);
void himax_chip_common_deinit(struct himax_ts_data *);
void himax_int_enable(struct himax_ts_data *ts, int enable);
uint8_t himax_int_gpio_read(int pinnum);
int himax_gpio_power_config(struct himax_ts_data *);
void himax_gpio_power_deconfig(struct himax_i2c_platform_data *pdata);
int himax_dev_set(struct himax_ts_data *ts);
int himax_input_register_device(struct input_dev *input_dev);

#endif
