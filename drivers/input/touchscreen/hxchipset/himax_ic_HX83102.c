/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for HX83102 chipset
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

static void hx83102_burst_enable(struct i2c_client *client,
				 uint8_t auto_add_4_byte)
{
	uint8_t tmp_data[4];
	int ret = 0;

	tmp_data[0] = 0x31;

	ret = himax_bus_write(client, 0x13, tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}

	tmp_data[0] = (0x10 | auto_add_4_byte);

	ret = himax_bus_write(client, 0x0D, tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}
}

static int hx83102_flash_write_burst(struct i2c_client *client,
				     uint8_t *reg_byte, uint8_t *write_data)
{
	uint8_t data_byte[8];
	int i = 0, j = 0, ret = 0;

	D("%s: ENTER ******", __func__);

	for (i = 0; i < 4; i++)
		data_byte[i] = reg_byte[i];

	for (j = 4; j < 8; j++)
		data_byte[j] = write_data[j - 4];

	ret = himax_bus_write(client, 0x00, data_byte, 8,
			      HIMAX_I2C_RETRY_TIMES);

	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}

	D("%s: LEAVE ******", __func__);
	return 0;
}

static int hx83102_register_read(struct himax_ts_data *ts, uint8_t *read_addr,
				 int read_length, uint8_t *read_data)
{
	struct ic_operation *pic_op = ts->g_core_cmd_op->ic_op;
	struct fw_operation *pfw_op = ts->g_core_cmd_op->fw_op;
	uint8_t tmp_data[4];
	int i = 0;
	int address = 0;
	int ret = 0;

	D("%s: ENTER ******", __func__);

	if (read_length > 256) {
		E("%s: read len over 256!\n", __func__);
		return LENGTH_FAIL;
	}
	if (read_length > 4)
		hx83102_burst_enable(ts->client, 1);
	else
		hx83102_burst_enable(ts->client, 0);

	address = (read_addr[3] << 24) + (read_addr[2] << 16) +
		  (read_addr[1] << 8) + read_addr[0];

	i = address;
	tmp_data[0] = (uint8_t)i;
	tmp_data[1] = (uint8_t)(i >> 8);
	tmp_data[2] = (uint8_t)(i >> 16);
	tmp_data[3] = (uint8_t)(i >> 24);

	D("%s: bus_write 0x00 address=%04x data=[%02x %02x %02x %02x]",
	  __func__, address, tmp_data[0], tmp_data[1], tmp_data[2],
	  tmp_data[3]);

	ret = himax_bus_write(ts->client, 0x00, tmp_data, 4,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}
	tmp_data[0] = 0x00;

	D("%s: bus_write 0x0C data=[%02x]", __func__, tmp_data[0]);

	ret = himax_bus_write(ts->client, 0x0C, tmp_data, 1,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}

	D("%s: bus_read 0x08 read_length=%d", __func__, read_length);

	ret = himax_bus_read(ts->client, 0x08, read_data, read_length,
			     HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return I2C_FAIL;
	}

	if (read_length > 4)
		hx83102_burst_enable(ts->client, 0);

	D("%s: LEAVE ******", __func__);
	return 0;
}

#if defined(HX_RST_PIN_FUNC)
void hx83102_pin_reset(struct himax_ts_data *ts)
{
	D("%s: ENTER ***** Now reset the touch chip. gpio=%d\n", __func__,
	  ts->rst_gpio);

	himax_rst_gpio_set(ts->rst_gpio, 0);
	msleep(20);

	himax_rst_gpio_set(ts->rst_gpio, 1);
	msleep(50);

	D("%s: LEAVE *****\n", __func__);
}
EXPORT_SYMBOL(hx83102_pin_reset);
#endif

static bool hx83102_sense_off(struct himax_ts_data *ts, bool check_en)
{
	uint8_t cnt = 0;
	uint8_t tmp_addr[DATA_LEN_4];
	uint8_t tmp_writ[DATA_LEN_4];
	uint8_t tmp_data[DATA_LEN_4];
	int ret = 0;

	D("%s: ENTER ****** check_en=%d", __func__, check_en);

	do {
		if (cnt == 0 || (tmp_data[0] != 0xA5 && tmp_data[0] != 0x00 &&
				 tmp_data[0] != 0x87)) {
			tmp_addr[3] = 0x90;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x5C;

			tmp_writ[3] = 0x00;
			tmp_writ[2] = 0x00;
			tmp_writ[1] = 0x00;
			tmp_writ[0] = 0xA5;
			hx83102_flash_write_burst(ts->client, tmp_addr,
						  tmp_writ);
		}
		msleep(20);

		/* check fw status */
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		hx83102_register_read(ts, tmp_addr, DATA_LEN_4, tmp_data);

		if (tmp_data[0] != 0x05) {
			D("%s: Do not need wait FW, Status = 0x%02X!\n",
			  __func__, tmp_data[0]);
			break;
		}

		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x5C;
		hx83102_register_read(ts, tmp_addr, DATA_LEN_4, tmp_data);
		D("%s: cnt = %d, data[0] = 0x%02X!\n", __func__, cnt,
		  tmp_data[0]);
	} while (tmp_data[0] != 0x87 && (++cnt < 50) && check_en == true);

	cnt = 0;

	do {
		/**
		 *I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x27
		 */
		D("%s: I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x27",
		  __func__);
		tmp_data[0] = 0x27;
		ret = himax_bus_write(ts->client, 0x31, tmp_data, 1,
				      HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}
		/**
		 *I2C_password[15:8] set Enter safe mode :0x32 ==> 0x95
		 */
		D("%s: I2C_password[15:8] set Enter safe mode :0x32 ==> 0x95",
		  __func__);
		tmp_data[0] = 0x95;
		ret = himax_bus_write(ts->client, 0x32, tmp_data, 1,
				      HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/**
		 *Check enter_save_mode
		 */
		D("%s: Check enter_save_mode", __func__);
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		hx83102_register_read(ts, tmp_addr, ADDR_LEN_4, tmp_data);
		D("%s: Check enter_save_mode data[0]=%X\n", __func__,
		  tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			/**
			 *Reset TCON
			 */
			D("%s: Reset TCON", __func__);
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x02;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x20;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			hx83102_flash_write_burst(ts->client, tmp_addr,
						  tmp_data);
			usleep_range(1000, 1001);
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x01;
			hx83102_flash_write_burst(ts->client, tmp_addr,
						  tmp_data);
			/**
			 *Reset ADC
			 */
			D("%s: Reset ADC", __func__);
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x02;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x94;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			hx83102_flash_write_burst(ts->client, tmp_addr,
						  tmp_data);
			usleep_range(1000, 1001);
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x01;
			hx83102_flash_write_burst(ts->client, tmp_addr,
						  tmp_data);

			D("%s: LEAVE true", __func__);
			return true;
		}
		usleep_range(10000, 10001);

#if defined(HX_RST_PIN_FUNC)
		hx83102_pin_reset(ts);
#endif

	} while (cnt++ < 15);

	D("%s: LEAVE FALSE", __func__);
	return false;
}

static void hx83102e_sense_on(struct himax_ts_data *ts, uint8_t FlashMode)
{
	struct fw_operation *pfw_op = ts->g_core_cmd_op->fw_op;

	D("%s: ENTER ***** FlashMode=%d\n", __func__, FlashMode);

	himax_mcu_interface_on(ts);
	himax_mcu_register_write(ts, pfw_op->addr_ctrl_fw_isr,
				 sizeof(pfw_op->data_clear), pfw_op->data_clear,
				 0);

	/*msleep(20);*/
	usleep_range(10000, 11000);

#ifdef HX_RST_PIN_FUNC
	himax_mcu_ic_reset(ts, false, false);
#else
	himax_mcu_system_reset(ts);
#endif

	D("%s: LEAVE *****\n", __func__);
}

bool hx83102e_sense_off(struct himax_ts_data *ts, bool check_en)
{
	struct ic_operation *pic_op = ts->g_core_cmd_op->ic_op;
	struct fw_operation *pfw_op = ts->g_core_cmd_op->fw_op;
	uint8_t cnt = 0;
	uint8_t tmp_addr[DATA_LEN_4];
	uint8_t tmp_data[DATA_LEN_4];
	int ret = 0;

	D("%s: ENTER *****\n", __func__);

	do {
		if (cnt == 0 || (tmp_data[0] != 0xA5 && tmp_data[0] != 0x00 &&
				 tmp_data[0] != 0x87))
			himax_mcu_register_write(ts, pfw_op->addr_ctrl_fw_isr,
						 DATA_LEN_4,
						 pfw_op->data_fw_stop, 0);
		/*msleep(20);*/
		usleep_range(10000, 10001);

		/* check fw status */
		himax_mcu_register_read(ts, pic_op->addr_cs_central_state,
					ADDR_LEN_4, tmp_data, 0);

		if (tmp_data[0] != 0x05) {
			D("%s: Do not need wait FW, Status = 0x%02X!\n",
			  __func__, tmp_data[0]);
			break;
		}

		himax_mcu_register_read(ts, pfw_op->addr_ctrl_fw_isr, 4,
					tmp_data, false);
		D("%s: cnt = %d, data[0] = 0x%02X!\n", __func__, cnt,
		  tmp_data[0]);
	} while (tmp_data[0] != 0x87 && (++cnt < 10) && check_en == true);

	cnt = 0;

	do {
		/**
		 *I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x27
		 */
		D("%s: I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x27\n",
		  __func__);
		tmp_data[0] = 0x27;
		ret = himax_bus_write(ts->client, 0x31, tmp_data, 1,
				      HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/**
		 *I2C_password[15:8] set Enter safe mode :0x32 ==> 0x95
		 */
		D("%s: I2C_password[15:8] set Enter safe mode :0x32 ==> 0x95\n",
		  __func__);
		tmp_data[0] = 0x95;
		ret = himax_bus_write(ts->client, 0x32, tmp_data, 1,
				      HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		/**
		 *Check enter_save_mode
		 */
		D("%s: Check enter_save_mode\n", __func__);
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xA8;
		hx83102_register_read(ts, tmp_addr, ADDR_LEN_4, tmp_data);
		D("%s: Check enter_save_mode data[0]=%X\n", __func__,
		  tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			/**
			 *Reset TCON
			 */
			D("%s: Reset TCON\n", __func__);
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x02;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x20;
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			hx83102_flash_write_burst(ts->client, tmp_addr,
						  tmp_data);
			usleep_range(1000, 1001);

			D("%s: LEAVE ***** TRUE\n", __func__);
			return true;
		}

		/*msleep(10);*/
		usleep_range(5000, 5001);

#if defined(HX_RST_PIN_FUNC)
		himax_rst_gpio_set(ts->rst_gpio, 0);
		msleep(20);
		himax_rst_gpio_set(ts->rst_gpio, 1);
		msleep(50);
#endif

	} while (cnt++ < 5);

	D("%s: LEAVE ***** FALSE\n", __func__);
	return false;
}
EXPORT_SYMBOL(hx83102e_sense_off);

bool hx83102e_read_event_stack(struct himax_ts_data *ts, uint8_t *buf,
			       uint8_t length)
{
	struct fw_operation *pfw_op = ts->g_core_cmd_op->fw_op;
	struct timespec t_start, t_end, t_delta;
	int len = length;
	int i2c_speed = 0;

	D("%s: ENTER *****\n", __func__);

	if (ts->debug_log_level & BIT(2))
		getnstimeofday(&t_start);

	himax_bus_read(ts->client, pfw_op->addr_event_addr[0], buf, length,
		       HIMAX_I2C_RETRY_TIMES);

	if (ts->debug_log_level & BIT(2)) {
		getnstimeofday(&t_end);
		t_delta.tv_nsec =
			(t_end.tv_sec * 1000000000 + t_end.tv_nsec) -
			(t_start.tv_sec * 1000000000 + t_start.tv_nsec); /*ns*/

		i2c_speed =
			(len * 9 * 1000000 / (int)t_delta.tv_nsec) * 13 / 10;
		ts->bus_speed = (int)i2c_speed;
	}

	D("%s: LEAVE *****\n", __func__);
	return 1;
}
EXPORT_SYMBOL(hx83102e_read_event_stack);

static void himax_hx83102e_reg_re_init(struct himax_ts_data *ts)
{
	struct fw_operation *fw_op = ts->g_core_cmd_op->fw_op;
	struct ic_operation *ic_op = ts->g_core_cmd_op->ic_op;
	struct driver_operation *dr_op = ts->g_core_cmd_op->driver_op;
	D("%s: ENTER *****!\n", __func__);

	himax_parse_assign_cmd(hx83102e_fw_addr_raw_out_sel,
			       fw_op->addr_raw_out_sel,
			       sizeof(fw_op->addr_raw_out_sel));
	himax_parse_assign_cmd(hx83102e_data_df_rx, dr_op->data_df_rx,
			       sizeof(dr_op->data_df_rx));
	himax_parse_assign_cmd(hx83102e_data_df_tx, dr_op->data_df_tx,
			       sizeof(dr_op->data_df_tx));
	himax_parse_assign_cmd(hx83102e_data_df_x_res, dr_op->data_df_x_res,
			       sizeof(dr_op->data_df_x_res));
	himax_parse_assign_cmd(hx83102e_data_df_y_res, dr_op->data_df_y_res,
			       sizeof(dr_op->data_df_y_res));
	himax_parse_assign_cmd(hx83102e_ic_adr_tcon_rst,
			       ic_op->addr_tcon_on_rst,
			       sizeof(ic_op->addr_tcon_on_rst));

	D("%s: LEAVE *****!\n", __func__);
}

bool hx83102_chip_detect(struct himax_ts_data *ts)
{
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t tmp_addr[DATA_LEN_4];
	int ret = 0;
	int i = 0;

	D("%s: ENTER *****\n", __func__);

#if defined(HX_RST_PIN_FUNC)
	hx83102_pin_reset(ts);
#endif

	ret = himax_bus_read(ts->client, 0x13, tmp_data, 1,
			     HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return false;
	}

	if (hx83102_sense_off(ts, false) == false) {
		E("%s: hx83102_sense_off failed.\n", __func__);
		return false;
	}

	for (i = 0; i < 5; i++) {
		tmp_addr[3] = 0x90;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0xD0;
		ret = hx83102_register_read(ts, tmp_addr, DATA_LEN_4, tmp_data);
		if (ret != 0) {
			E("%s: hx83102_register_read failed.\n", __func__);
			return false;
		}
		/* is major chip id? */
		else if ((tmp_data[3] == 0x83) && (tmp_data[2] == 0x10)) {
			break;
		}
	}

	I("%s: Read driver IC ID = %X,%X,%X\n", __func__, tmp_data[3],
	  tmp_data[2], tmp_data[1]); /*83,10,2X*/

	if ((tmp_data[3] != 0x83) || (tmp_data[2] != 0x10)) {
		E("%s: Could not find major chip ID 0x8310 failed.\n",
		  __func__);
		goto exit_chip_not_found;
	}

	switch (tmp_data[1]) {
	case 0x2a: {
		ts->ic_data->ic_adc_num = hx83102a_data_adc_num;
		strlcpy(ts->chip_name, HX_83102A_SERIES_PWON, 30);
		break;
	}
	case 0x2b: {
		ts->ic_data->ic_adc_num = hx83102b_data_adc_num;
		strlcpy(ts->chip_name, HX_83102B_SERIES_PWON, 30);
		break;
	}
	case 0x2d: {
		ts->ic_data->ic_adc_num = hx83102d_data_adc_num;
		strlcpy(ts->chip_name, HX_83102D_SERIES_PWON, 30);
		break;
	}
	case 0x2e: {
		strlcpy(ts->chip_name, HX_83102E_SERIES_PWON, 30);
		ts->ic_data->ic_adc_num = hx83102e_data_adc_num;
		ts->chip_cell_type = CHIP_IS_IN_CELL;
		ts->IC_CHECKSUM = HX_TP_BIN_CHECKSUM_CRC;
		break;
	}
	}

	I("%s: detect IC %s successfully\n", __func__, ts->chip_name);

	/* Initialize function pointers */
	if (himax_mcu_in_cmd_struct_init(ts) < 0) {
		E("%s: himax_mcu_in_cmd_struct_init() failed.\n", __func__);
		return false;
	}

	himax_mcu_in_cmd_init(ts);

	switch (tmp_data[1]) {
	case 0x2a:
	case 0x2b:
	case 0x2d: {
		goto exit_unsupported_chip;
	}
	case 0x2e: {
		himax_hx83102e_reg_re_init(ts);
		break;
	}
	}

	return true;

exit_unsupported_chip:
	E("%s: Unsupported chipd ID. HX83102E only.\n", __func__);
	return false;

exit_chip_not_found:
	E("%s: Read driver ID register failed.\n"
	  "Could NOT find Himax Chipset\n"
	  "Please check 1.VCCD,VCCA,VSP,VSN\n"
	  "2. LCM_RST,TP_RST\n"
	  "3. Power On Sequence\n",
	  __func__);
	return false;
}
EXPORT_SYMBOL(hx83102_chip_detect);

static int himax_hx83102_remove(void)
{
	return 0;
}

static int __init himax_hx83102_init(void)
{
	return 0;
}

static void __exit himax_hx83102_exit(void)
{
	himax_hx83102_remove();
}

module_init(himax_hx83102_init);
module_exit(himax_hx83102_exit);

MODULE_DESCRIPTION("HIMAX HX83102E touch driver");
MODULE_LICENSE("GPL");
