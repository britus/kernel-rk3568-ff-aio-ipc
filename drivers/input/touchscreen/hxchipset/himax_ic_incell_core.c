/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for incell ic core functions
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

#include "himax_common.h"
#include "himax_ic_core.h"
#include "himax_ic_incell_core.h"

extern struct hx_guest_info *g_guest_info_data;

#if defined(HX_ZERO_FLASH)
struct zf_operation *pzf_op;
EXPORT_SYMBOL(pzf_op);
#if defined(HX_CODE_OVERLAY)
uint8_t *ovl_idx;
EXPORT_SYMBOL(ovl_idx);
#endif
#endif

static uint8_t *g_internal_buffer;

void himax_mcu_in_cmd_struct_free(struct himax_ts_data *ts);

/* CORE_IC */
/* IC side start*/
static void himax_mcu_burst_enable(struct himax_ts_data *ts,
				   uint8_t auto_add_4_byte)
{
	struct ic_operation *pic_op = ts->g_core_cmd_op->ic_op;
	uint8_t tmp_data[DATA_LEN_4];
	int ret;

	D("%s: Entering auto_add_4_byte=%d\n", __func__, auto_add_4_byte);

	tmp_data[0] = pic_op->data_conti[0];

	ret = himax_bus_write(ts->client, pic_op->addr_conti[0], tmp_data, 1,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}

	tmp_data[0] = (pic_op->data_incr4[0] | auto_add_4_byte);

	ret = himax_bus_write(ts->client, pic_op->addr_incr4[0], tmp_data, 1,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return;
	}

	D("%s: Leave\n", __func__);
}

int himax_mcu_register_read(struct himax_ts_data *ts, uint8_t *read_addr,
			    uint32_t read_length, uint8_t *read_data,
			    uint8_t cfg_flag)
{
	struct ic_operation *pic_op = ts->g_core_cmd_op->ic_op;
	uint8_t tmp_data[DATA_LEN_4];
	int i = 0;
	int address = 0;
	int ret = 0;

	D("%s: ENTER+++++ cfg_flag=%d\n", __func__, cfg_flag);

	if (cfg_flag == false) {
		if (read_length > FLASH_RW_MAX_LEN) {
			E("%s: read len over %d!\n", __func__,
			  FLASH_RW_MAX_LEN);
			return LENGTH_FAIL;
		}

		if (read_length > DATA_LEN_4)
			himax_mcu_burst_enable(ts, 1);
		else
			himax_mcu_burst_enable(ts, 0);

		address = (read_addr[3] << 24) + (read_addr[2] << 16) +
			  (read_addr[1] << 8) + read_addr[0];
		i = address;
		tmp_data[0] = (uint8_t)i;
		tmp_data[1] = (uint8_t)(i >> 8);
		tmp_data[2] = (uint8_t)(i >> 16);
		tmp_data[3] = (uint8_t)(i >> 24);

		ret = himax_bus_write(ts->client,
				      pic_op->addr_ahb_addr_byte_0[0], tmp_data,
				      DATA_LEN_4, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return I2C_FAIL;
		}

		tmp_data[0] = pic_op->data_ahb_access_direction_read[0];

		ret = himax_bus_write(ts->client,
				      pic_op->addr_ahb_access_direction[0],
				      tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return I2C_FAIL;
		}

		ret = himax_bus_read(ts->client,
				     pic_op->addr_ahb_rdata_byte_0[0],
				     read_data, read_length,
				     HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return I2C_FAIL;
		}

		if (read_length > DATA_LEN_4)
			himax_mcu_burst_enable(ts, 0);

	} else {
		ret = himax_bus_read(ts->client, read_addr[0], read_data,
				     read_length, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return I2C_FAIL;
		}
	}

	D("%s: LEAVE++++++\n", __func__);
	return NO_ERR;
}
EXPORT_SYMBOL(himax_mcu_register_read);

static int himax_mcu_flash_write_burst_lenth(struct himax_ts_data *ts,
					     uint8_t *reg_byte,
					     uint8_t *write_data,
					     uint32_t length)
{
	struct ic_operation *pic_op = ts->g_core_cmd_op->ic_op;
	uint8_t *data_byte;
	int ret = 0;

	if (!g_internal_buffer) {
		E("%s: internal buffer not initialized!\n", __func__);
		return MEM_ALLOC_FAIL;
	}

	data_byte = g_internal_buffer;

	/* assign addr 4bytes */
	memcpy(data_byte, reg_byte, ADDR_LEN_4);

	/* assign data n bytes */
	memcpy(data_byte + ADDR_LEN_4, write_data, length);

	ret = himax_bus_write(ts->client, pic_op->addr_ahb_addr_byte_0[0],
			      data_byte, length + ADDR_LEN_4,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: xfer fail!\n", __func__);
		return I2C_FAIL;
	}

	return NO_ERR;
}

int himax_mcu_register_write(struct himax_ts_data *ts, uint8_t *write_addr,
			     uint32_t write_length, uint8_t *write_data,
			     uint8_t cfg_flag)
{
	int address;
	uint8_t tmp_addr[4];
	uint8_t *tmp_data;
	int total_read_times = 0;
	uint32_t max_bus_size = MAX_I2C_TRANS_SZ;
	uint32_t total_size_temp = 0;
	int i = 0;
	int ret = 0;

	D("%s: ENTER++++++ cfg_flag=%d write_length=%d\n", __func__, cfg_flag,
	  write_length);

	if (cfg_flag == 0) {
		total_size_temp = write_length;
		tmp_addr[3] = write_addr[3];
		tmp_addr[2] = write_addr[2];
		tmp_addr[1] = write_addr[1];
		tmp_addr[0] = write_addr[0];

		if (total_size_temp % max_bus_size == 0)
			total_read_times = total_size_temp / max_bus_size;
		else
			total_read_times = total_size_temp / max_bus_size + 1;

		if (write_length > DATA_LEN_4)
			himax_mcu_burst_enable(ts, 1);
		else
			himax_mcu_burst_enable(ts, 0);

		for (i = 0; i < (total_read_times); i++) {
			if (total_size_temp >= max_bus_size) {
				tmp_data = write_data + (i * max_bus_size);

				ret = himax_mcu_flash_write_burst_lenth(
					ts, tmp_addr, tmp_data, max_bus_size);
				if (ret < 0) {
					D("%s: i2c access fail!\n", __func__);
					return I2C_FAIL;
				}
				total_size_temp =
					total_size_temp - max_bus_size;
			} else {
				tmp_data = write_data + (i * max_bus_size);
				ret = himax_mcu_flash_write_burst_lenth(
					ts, tmp_addr, tmp_data,
					total_size_temp);
				if (ret < 0) {
					D("%s: i2c access fail!\n", __func__);
					return I2C_FAIL;
				}
			}

			address = ((i + 1) * max_bus_size);
			tmp_addr[0] =
				write_addr[0] + (uint8_t)((address)&0x00FF);

			if (tmp_addr[0] < write_addr[0])
				tmp_addr[1] =
					write_addr[1] +
					(uint8_t)((address >> 8) & 0x00FF) + 1;
			else
				tmp_addr[1] =
					write_addr[1] +
					(uint8_t)((address >> 8) & 0x00FF);

			udelay(100);
		}
	} else if (cfg_flag == 1) {
		ret = himax_bus_write(ts->client, write_addr[0], write_data,
				      write_length, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return I2C_FAIL;
		}
	} else
		E("%s: cfg_flag = %d, value is wrong!\n", __func__, cfg_flag);

	D("%s: LEAVE++++++\n", __func__);
	return 0;
}
EXPORT_SYMBOL(himax_mcu_register_write);

static int himax_write_read_reg(struct himax_ts_data *ts, uint8_t *tmp_addr,
				uint8_t *tmp_data, uint8_t hb, uint8_t lb)
{
	int cnt = 0;

	D("%s: Entering hb=%d lb=%d\n", __func__, hb, lb);

	do {
		himax_mcu_register_write(ts, tmp_addr, DATA_LEN_4, tmp_data, 0);
		usleep_range(10000, 11000);
		himax_mcu_register_read(ts, tmp_addr, DATA_LEN_4, tmp_data, 0);
		/* D("%s: Now tmp_data[0]=0x%02X,[1]=0x%02X,
		 *	[2]=0x%02X,[3]=0x%02X\n",
		 *	__func__, tmp_data[0],
		 *	tmp_data[1], tmp_data[2], tmp_data[3]);
		 */
	} while ((tmp_data[1] != hb && tmp_data[0] != lb) && cnt++ < 100);

	if (cnt >= 100)
		return HX_RW_REG_FAIL;

	D("%s: Now register 0x%08X : high byte=0x%02X,low byte=0x%02X\n",
	  __func__, tmp_addr[3], tmp_data[1], tmp_data[0]);

	D("%s: Leave\n", __func__);
	return NO_ERR;
}

void himax_mcu_interface_on(struct himax_ts_data *ts)
{
	struct ic_operation *pic_op = ts->g_core_cmd_op->ic_op;
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t tmp_data2[DATA_LEN_4];
	int cnt = 0;
	int ret = 0;

	D("%s: Entering\n", __func__);

	/* Read a dummy register to wake up I2C.*/
	ret = himax_bus_read(ts->client, pic_op->addr_ahb_rdata_byte_0[0],
			     tmp_data, DATA_LEN_4, HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) { /* to knock I2C*/
		E("%s: i2c access fail!\n", __func__);
		return;
	}

	do {
		tmp_data[0] = pic_op->data_conti[0];

		ret = himax_bus_write(ts->client, pic_op->addr_conti[0],
				      tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return;
		}

		tmp_data[0] = pic_op->data_incr4[0];

		ret = himax_bus_write(ts->client, pic_op->addr_incr4[0],
				      tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return;
		}

		/*Check cmd*/
		ret = himax_bus_read(ts->client, pic_op->addr_conti[0],
				     tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		ret = himax_bus_read(ts->client, pic_op->addr_incr4[0],
				     tmp_data2, 1, HIMAX_I2C_RETRY_TIMES);

		if (tmp_data[0] == pic_op->data_conti[0] &&
		    tmp_data2[0] == pic_op->data_incr4[0])
			break;

		usleep_range(1000, 1100);
	} while (++cnt < 10);

	if (cnt > 0)
		D("%s: Polling burst mode: %d times\n", __func__, cnt);

	D("%s: Leave\n", __func__);
}
EXPORT_SYMBOL(himax_mcu_interface_on);

#define WIP_PRT_LOG "%s: retry:%d, bf[0]=%d, bf[1]=%d,bf[2]=%d, bf[3]=%d\n"
static bool himax_mcu_wait_wip(struct himax_ts_data *ts, int Timing)
{
	struct flash_operation *pflash_op = ts->g_core_cmd_op->flash_op;
	uint8_t tmp_data[DATA_LEN_4];
	int retry_cnt = 0;

	D("%s: Entering\n", __func__);

	himax_mcu_register_write(ts, pflash_op->addr_spi200_trans_fmt,
				 DATA_LEN_4, pflash_op->data_spi200_trans_fmt,
				 0);
	tmp_data[0] = 0x01;

	do {
		himax_mcu_register_write(ts, pflash_op->addr_spi200_trans_ctrl,
					 DATA_LEN_4,
					 pflash_op->data_spi200_trans_ctrl_1,
					 0);
		himax_mcu_register_write(ts, pflash_op->addr_spi200_cmd,
					 DATA_LEN_4,
					 pflash_op->data_spi200_cmd_1, 0);
		tmp_data[0] = tmp_data[1] = tmp_data[2] = tmp_data[3] = 0xFF;

		himax_mcu_register_read(ts, pflash_op->addr_spi200_data, 4,
					tmp_data, 0);
		if ((tmp_data[0] & 0x01) == 0x00)
			return true;

		retry_cnt++;

		if (tmp_data[0] != 0x00 || tmp_data[1] != 0x00 ||
		    tmp_data[2] != 0x00 || tmp_data[3] != 0x00)
			I(WIP_PRT_LOG, __func__, retry_cnt, tmp_data[0],
			  tmp_data[1], tmp_data[2], tmp_data[3]);

		if (retry_cnt > 100) {
			E("%s: Wait wip error!\n", __func__);
			return false;
		}

		msleep(Timing);
	} while ((tmp_data[0] & 0x01) == 0x01);

	D("%s: Leave\n", __func__);
	return true;
}

static void himax_mcu_sense_on(struct himax_ts_data *ts, uint8_t FlashMode)
{
	struct fw_operation *pfw_op = ts->g_core_cmd_op->fw_op;
	struct ic_operation *pic_op = ts->g_core_cmd_op->ic_op;

	uint8_t tmp_data[DATA_LEN_4];
	int retry = 0;
	int ret = 0;

	D("%s: Entering FlashMode=%d\n", __func__, FlashMode);

	himax_mcu_interface_on(ts);
	himax_mcu_register_write(ts, pfw_op->addr_ctrl_fw_isr,
				 sizeof(pfw_op->data_clear), pfw_op->data_clear,
				 0);

	/*msleep(20);*/
	usleep_range(10000, 11000);
	if (!FlashMode) {
#if defined(HX_RST_PIN_FUNC)
		himax_mcu_ic_reset(ts, false, false);
#else
		himax_mcu_system_reset(ts);
#endif
	} else {
		do {
			himax_mcu_register_write(
				ts, pfw_op->addr_safe_mode_release_pw,
				sizeof(pfw_op->data_safe_mode_release_pw_active),
				pfw_op->data_safe_mode_release_pw_active, 0);
			himax_mcu_register_read(ts,
						pfw_op->addr_flag_reset_event,
						DATA_LEN_4, tmp_data, 0);
			D("%s: Read status from IC = %X,%X\n", __func__,
			  tmp_data[0], tmp_data[1]);
		} while ((tmp_data[1] != 0x01 || tmp_data[0] != 0x00) &&
			 retry++ < 5);

		if (retry >= 5) {
			E("%s: Fail:\n", __func__);
#if defined(HX_RST_PIN_FUNC)
			himax_mcu_ic_reset(ts, false, false);
#else
			himax_mcu_system_reset(ts);
#endif
		} else {
			D("%s: OK and Read status from IC = %X,%X\n", __func__,
			  tmp_data[0], tmp_data[1]);

			/* reset code*/
			tmp_data[0] = 0x00;

			ret = himax_bus_write(ts->client,
					      pic_op->adr_i2c_psw_lb[0],
					      tmp_data, 1,
					      HIMAX_I2C_RETRY_TIMES);
			if (ret < 0) {
				E("%s: i2c access fail!\n", __func__);
				ret = himax_bus_write(ts->client,
						      pic_op->adr_i2c_psw_ub[0],
						      tmp_data, 1,
						      HIMAX_I2C_RETRY_TIMES);
			}
			if (ret < 0) {
				E("%s: i2c access fail!\n", __func__);
			}
			himax_mcu_register_write(
				ts, pfw_op->addr_safe_mode_release_pw,
				sizeof(pfw_op->data_safe_mode_release_pw_reset),
				pfw_op->data_safe_mode_release_pw_reset, 0);
		}
	}

	D("%s: Leave\n", __func__);
}

static bool himax_mcu_sense_off(struct himax_ts_data *ts, bool check_en)
{
	struct ic_operation *pic_op = ts->g_core_cmd_op->ic_op;
	uint8_t cnt = 0;
	uint8_t tmp_data[DATA_LEN_4];
	int ret = 0;

	D("%s: Entering check_en=%d\n", __func__, check_en);

	do {
		tmp_data[0] = pic_op->data_i2c_psw_lb[0];

		ret = himax_bus_write(ts->client, pic_op->adr_i2c_psw_lb[0],
				      tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		tmp_data[0] = pic_op->data_i2c_psw_ub[0];

		ret = himax_bus_write(ts->client, pic_op->adr_i2c_psw_ub[0],
				      tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0) {
			E("%s: i2c access fail!\n", __func__);
			return false;
		}

		himax_mcu_register_read(ts, pic_op->addr_cs_central_state,
					ADDR_LEN_4, tmp_data, 0);
		D("%s: Check enter_save_mode data[0]=%X\n", __func__,
		  tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			himax_mcu_register_write(ts, pic_op->addr_tcon_on_rst,
						 DATA_LEN_4, pic_op->data_rst,
						 0);
			usleep_range(1000, 1100);
			tmp_data[3] = pic_op->data_rst[3];
			tmp_data[2] = pic_op->data_rst[2];
			tmp_data[1] = pic_op->data_rst[1];
			tmp_data[0] = pic_op->data_rst[0] | 0x01;
			himax_mcu_register_write(ts, pic_op->addr_tcon_on_rst,
						 DATA_LEN_4, tmp_data, 0);

			himax_mcu_register_write(ts, pic_op->addr_adc_on_rst,
						 DATA_LEN_4, pic_op->data_rst,
						 0);
			usleep_range(1000, 1100);
			tmp_data[3] = pic_op->data_rst[3];
			tmp_data[2] = pic_op->data_rst[2];
			tmp_data[1] = pic_op->data_rst[1];
			tmp_data[0] = pic_op->data_rst[0] | 0x01;
			himax_mcu_register_write(ts, pic_op->addr_adc_on_rst,
						 DATA_LEN_4, tmp_data, 0);
			goto TRUE_END;
		} else {
			/*msleep(10);*/
#if defined(HX_RST_PIN_FUNC)
			himax_mcu_ic_reset(ts, false, false);
#else
			himax_mcu_system_reset(ts);
#endif
		}
	} while (cnt++ < 15);

	D("%s: Leave false\n", __func__);
	return false;

TRUE_END:
	D("%s: Leave true\n", __func__);
	return true;
}

/*power saving level*/
static void himax_mcu_init_psl(struct himax_ts_data *ts)
{
	struct ic_operation *pic_op = ts->g_core_cmd_op->ic_op;
	D("%s: ENTER ::::::: power saving level reset OK!\n", __func__);
	himax_mcu_register_write(ts, pic_op->addr_psl, sizeof(pic_op->data_rst),
				 pic_op->data_rst, 0);
	D("%s: LEAVE ::::::: power saving level reset OK!\n", __func__);
}

static void himax_mcu_suspend_ic_action(void)
{
	/* Nothing to do */
}

int himax_mcu_power_on_init(struct himax_ts_data *ts)
{
	struct fw_operation *pfw_op = ts->g_core_cmd_op->fw_op;
	int ret;

	D("%s: Entering\n", __func__);

	himax_mcu_touch_information(ts);

	/*RawOut select initial*/
	ret = himax_mcu_register_write(ts, pfw_op->addr_raw_out_sel,
				       sizeof(pfw_op->data_clear),
				       pfw_op->data_clear, 0);
	if (ret) {
		E("%s: RawOut select initialization failed.", __func__);
		return ret;
	}

	/*DSRAM func initial*/
	ret = himax_mcu_assign_sorting_mode(ts, pfw_op->data_clear);
	if (ret) {
		E("%s: DSRAM func initialization failed.", __func__);
		return ret;
	}

	himax_mcu_sense_on(ts, 0x00);

	D("%s: Leave\n", __func__);
	return 0;
}
EXPORT_SYMBOL(himax_mcu_power_on_init);

static bool himax_mcu_dd_clk_set(struct himax_ts_data *ts, bool enable)
{
	struct ic_operation *pic_op = ts->g_core_cmd_op->ic_op;
	uint8_t data[4] = { 0 };
	data[0] = (enable ? 1 : 0);
	return (himax_mcu_register_write(ts, pic_op->adr_osc_en,
					 sizeof(pic_op->adr_osc_en), data,
					 0) == NO_ERR);
}

void himax_mcu_dd_reg_en(struct himax_ts_data *ts, bool enable)
{
	struct ic_operation *pic_op = ts->g_core_cmd_op->ic_op;
	uint8_t data[4] = { 0 };

	D("%s: Entering\n", __func__);

	himax_mcu_dd_reg_read(ts, 0xCB, 8, 1, data, 0);

	if (data[0] != 0x44) { /*need DD Touch PW*/
		data[0] = 0xA5;
		data[1] = 0x00;
		data[2] = 0x00;
		data[3] = 0x00;
		himax_mcu_register_write(ts, pic_op->adr_osc_pw, DATA_LEN_4,
					 data, 0);
		data[0] = 0x00;
		data[1] = 0x55;
		data[2] = 0xAA;
		data[3] = 0x00;
		himax_mcu_dd_reg_write(ts, 0xEB, 0, 4, data, 0);
	}

	data[0] = 0x00;
	data[1] = 0x83;
	data[2] = 0x11;
	data[3] = 0x2A;
	himax_mcu_dd_reg_write(ts, 0xB9, 0, 4, data, 0);

	D("%s: Leave\n", __func__);
}
EXPORT_SYMBOL(himax_mcu_dd_reg_en);

bool himax_mcu_dd_reg_write(struct himax_ts_data *ts, uint8_t addr,
			    uint8_t pa_num, int len, uint8_t *data,
			    uint8_t bank)
{
	/*Calculate total write length*/
	uint32_t data_len = (((len + pa_num - 1) / 4 - pa_num / 4) + 1) * 4;
	uint8_t w_data[data_len];
	uint8_t tmp_addr[4] = { 0 };
	uint8_t tmp_data[4] = { 0 };
	bool chk_data[data_len];
	uint32_t chk_idx = 0;
	int ret, i = 0;

	D("%s: Entering addr=%x pa_num=%d len=%d bank=%d\n", __func__, addr,
	  pa_num, len, bank);

	memset(w_data, 0, data_len * sizeof(uint8_t));
	memset(chk_data, 0, data_len * sizeof(bool));

	/*put input data*/
	chk_idx = pa_num % 4;
	for (i = 0; i < len; i++) {
		w_data[chk_idx] = data[i];
		chk_data[chk_idx++] = true;
	}

	/*get original data*/
	chk_idx = (pa_num / 4) * 4;
	for (i = 0; i < data_len; i++) {
		if (!chk_data[i]) {
			himax_mcu_dd_reg_read(ts, addr, (uint8_t)(chk_idx + i),
					      1, tmp_data, bank);
			w_data[i] = tmp_data[0];
			chk_data[i] = true;
		}
		D("%s w_data[%d] = %2X\n", __func__, i, w_data[i]);
	}

	tmp_addr[3] = 0x30;
	tmp_addr[2] = addr >> 4;
	tmp_addr[1] = (addr << 4) | (bank * 4);
	tmp_addr[0] = chk_idx;
	D("%s Address = %02X%02X%02X%02X\n", __func__, tmp_addr[3], tmp_addr[2],
	  tmp_addr[1], tmp_addr[0]);
	ret = himax_mcu_register_write(ts, tmp_addr, data_len, w_data, 0);

	D("%s: Leave ret=%d\n", __func__, ret);
	return (ret == NO_ERR);
}
EXPORT_SYMBOL(himax_mcu_dd_reg_write);

bool himax_mcu_dd_reg_read(struct himax_ts_data *ts, uint8_t addr,
			   uint8_t pa_num, int len, uint8_t *data, uint8_t bank)
{
	uint8_t tmp_addr[4] = { 0 };
	uint8_t tmp_data[4] = { 0 };
	int i = 0;

	D("%s: Entering addr=%x pa_num=%d len=%d bank=%d\n", __func__, addr,
	  pa_num, len, bank);

	for (i = 0; i < len; i++) {
		tmp_addr[3] = 0x30;
		tmp_addr[2] = addr >> 4;
		tmp_addr[1] = (addr << 4) | (bank * 4);
		tmp_addr[0] = pa_num + i;

		if (himax_mcu_register_read(ts, tmp_addr, DATA_LEN_4, tmp_data,
					    0))
			goto READ_FAIL;

		data[i] = tmp_data[0];

		D("%s Address = %02X%02X%02X%02X.result = %2X\n", __func__,
		  tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0], data[i]);
	}

	D("%s: Leave\n", __func__);
	return true;

READ_FAIL:
	E("%s Read DD reg Failed.\n", __func__);
	return false;
}
EXPORT_SYMBOL(himax_mcu_dd_reg_read);

static bool himax_mcu_ic_id_read(struct himax_ts_data *ts)
{
	int i = 0;
	uint8_t data[4] = { 0 };

	himax_mcu_dd_clk_set(ts, true);
	himax_mcu_dd_reg_en(ts, true);

	for (i = 0; i < 13; i++) {
		data[0] = 0x28 + i;
		himax_mcu_dd_reg_write(ts, 0xBB, 2, 1, data, 0);
		data[0] = 0x80;
		himax_mcu_dd_reg_write(ts, 0xBB, 4, 1, data, 0);
		data[0] = 0x00;
		himax_mcu_dd_reg_write(ts, 0xBB, 4, 1, data, 0);
		himax_mcu_dd_reg_read(ts, 0xBB, 5, 1, data, 0);
		ts->ic_data->vendor_ic_id[i] = data[0];
		D("%s: ic_data->vendor_ic_id[%d] = %02X\n", __func__, i,
		  ts->ic_data->vendor_ic_id[i]);
	}

	himax_mcu_dd_clk_set(ts, false);
	return true;
}

/* IC side end*/
/* CORE_IC */

/* CORE_FW */
#define PRT_DATA "%s:[3]=0x%2X, [2]=0x%2X, [1]=0x%2X, [0]=0x%2X\n"
#define PRT_TMP_DATA "%s:[0]=0x%2X,[1]=0x%2X,	[2]=0x%2X,[3]=0x%2X\n"

/* FW side start*/
static void himax_mcu_system_reset(struct himax_ts_data *ts)
{
	struct fw_operation *pfw_op = ts->g_core_cmd_op->fw_op;
	int ret = 0;

	struct ic_operation *pic_op = ts->g_core_cmd_op->ic_op;
	uint8_t tmp_data[DATA_LEN_4];
	int retry = 0;

	himax_mcu_interface_on(ts);
	himax_mcu_register_write(ts, pfw_op->addr_ctrl_fw_isr,
				 sizeof(pfw_op->data_clear), pfw_op->data_clear,
				 0);
	do {
		/* reset code*/
		/**
		 * I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x27
		 */
		tmp_data[0] = pic_op->data_i2c_psw_lb[0];

		ret = himax_bus_write(ts->client, pic_op->adr_i2c_psw_lb[0],
				      tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0)
			E("%s: i2c access fail!\n", __func__);

		/**
		 * I2C_password[15:8] set Enter safe mode :0x32 ==> 0x95
		 */
		tmp_data[0] = pic_op->data_i2c_psw_ub[0];

		ret = himax_bus_write(ts->client, pic_op->adr_i2c_psw_ub[0],
				      tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0)
			E("%s: i2c access fail!\n", __func__);

		/**
		 * I2C_password[7:0] set Enter safe mode : 0x31 ==> 0x00
		 */
		tmp_data[0] = 0x00;

		ret = himax_bus_write(ts->client, pic_op->adr_i2c_psw_lb[0],
				      tmp_data, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0)
			E("%s: i2c access fail!\n", __func__);

		usleep_range(10000, 11000);

		himax_mcu_register_read(ts, pfw_op->addr_flag_reset_event,
					DATA_LEN_4, tmp_data, 0);
		D("%s: Read status from IC = %X,%X\n", __func__, tmp_data[0],
		  tmp_data[1]);
	} while ((tmp_data[1] != 0x02 || tmp_data[0] != 0x00) && retry++ < 5);
}

static uint32_t himax_mcu_check_CRC(struct himax_ts_data *ts,
				    uint8_t *start_addr, int reload_length)
{
	struct fw_operation *pfw_op = ts->g_core_cmd_op->fw_op;
	uint32_t result = 0;
	uint8_t tmp_data[DATA_LEN_4];
	int cnt = 0, ret = 0;
	int length = reload_length / DATA_LEN_4;

	ret = himax_mcu_register_write(ts, pfw_op->addr_reload_addr_from,
				       DATA_LEN_4, start_addr, 0);
	if (ret < NO_ERR) {
		E("%s: i2c access fail!\n", __func__);
		return HW_CRC_FAIL;
	}

	tmp_data[3] = 0x00;
	tmp_data[2] = 0x99;
	tmp_data[1] = (length >> 8);
	tmp_data[0] = length;
	ret = himax_mcu_register_write(ts, pfw_op->addr_reload_addr_cmd_beat,
				       DATA_LEN_4, tmp_data, 0);
	if (ret < NO_ERR) {
		E("%s: i2c access fail!\n", __func__);
		return HW_CRC_FAIL;
	}
	cnt = 0;

	do {
		ret = himax_mcu_register_read(ts, pfw_op->addr_reload_status,
					      DATA_LEN_4, tmp_data, 0);
		if (ret < NO_ERR) {
			E("%s: i2c access fail!\n", __func__);
			return HW_CRC_FAIL;
		}

		if ((tmp_data[0] & 0x01) != 0x01) {
			ret = himax_mcu_register_read(
				ts, pfw_op->addr_reload_crc32_result,
				DATA_LEN_4, tmp_data, 0);
			if (ret < NO_ERR) {
				E("%s: i2c access fail!\n", __func__);
				return HW_CRC_FAIL;
			}
			D("%s: data[3]=%X,data[2]=%X,data[1]=%X,data[0]=%X\n",
			  __func__, tmp_data[3], tmp_data[2], tmp_data[1],
			  tmp_data[0]);
			result = ((tmp_data[3] << 24) + (tmp_data[2] << 16) +
				  (tmp_data[1] << 8) + tmp_data[0]);
			goto END;
		} else {
			usleep_range(1000, 1100);
		}

	} while (cnt++ < 100);
END:
	return result;
}

static void himax_mcu_set_HSEN_enable(struct himax_ts_data *ts)
{
	struct fw_operation *pfw_op = ts->g_core_cmd_op->fw_op;
	uint8_t tmp_data[DATA_LEN_4];
	uint8_t back_data[DATA_LEN_4];
	uint8_t retry_cnt = 0;

	do {
		if (ts->HSEN_enable) {
			himax_parse_assign_cmd(fw_func_handshaking_pwd,
					       tmp_data, 4);
			himax_mcu_register_write(ts, pfw_op->addr_hsen_enable,
						 DATA_LEN_4, tmp_data, 0);
			himax_parse_assign_cmd(fw_func_handshaking_pwd,
					       back_data, 4);
		} else {
			himax_parse_assign_cmd(
				fw_data_safe_mode_release_pw_reset, tmp_data,
				4);
			himax_mcu_register_write(ts, pfw_op->addr_hsen_enable,
						 DATA_LEN_4, tmp_data, 0);
			himax_parse_assign_cmd(
				fw_data_safe_mode_release_pw_reset, back_data,
				4);
		}

		himax_mcu_register_read(ts, pfw_op->addr_hsen_enable,
					DATA_LEN_4, tmp_data, 0);
		D("%s: tmp_data[0]=%d, HSEN_enable=%d, retry_cnt=%d\n",
		  __func__, tmp_data[0], ts->HSEN_enable, retry_cnt);

		retry_cnt++;
	} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] ||
		  tmp_data[1] != back_data[1] || tmp_data[0] != back_data[0]) &&
		 retry_cnt < HIMAX_REG_RETRY_TIMES);
}

static bool himax_mcu_read_event_stack(struct himax_ts_data *ts, uint8_t *buf,
				       uint8_t length)
{
	struct fw_operation *pfw_op = ts->g_core_cmd_op->fw_op;
	uint8_t cmd[DATA_LEN_4];
	struct timespec t_start, t_end, t_delta;
	int len = length;
	int i2c_speed = 0;
	int ret = 0;

	D("%s: ENTER ++++", __func__);

	/*  AHB_I2C Burst Read Off */
	cmd[0] = pfw_op->data_ahb_dis[0];

	ret = himax_bus_write(ts->client, pfw_op->addr_ahb_addr[0], cmd, 1,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	if (ts->debug_log_level & BIT(2))
		getnstimeofday(&t_start);

	ret = himax_bus_read(ts->client, pfw_op->addr_event_addr[0], buf,
			     length, HIMAX_I2C_RETRY_TIMES);

	if (ts->debug_log_level & BIT(2)) {
		getnstimeofday(&t_end);
		t_delta.tv_nsec =
			(t_end.tv_sec * 1000000000 + t_end.tv_nsec) -
			(t_start.tv_sec * 1000000000 + t_start.tv_nsec);

		i2c_speed =
			(len * 9 * 1000000 / (int)t_delta.tv_nsec) * 13 / 10;
		ts->bus_speed = (int)i2c_speed;
	}

	/*  AHB_I2C Burst Read On */
	cmd[0] = pfw_op->data_ahb_en[0];

	ret = himax_bus_write(ts->client, pfw_op->addr_ahb_addr[0], cmd, 1,
			      HIMAX_I2C_RETRY_TIMES);
	if (ret < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	D("%s: LEAVE ++++", __func__);
	return 1;
}

int himax_mcu_assign_sorting_mode(struct himax_ts_data *ts, uint8_t *tmp_data)
{
	struct fw_operation *pfw_op = ts->g_core_cmd_op->fw_op;
	int ret;

	D("%s: Now data[3]=0x%02X,data[2]=0x%02X,data[1]=0x%02X,data[0]=0x%02X\n",
	  __func__, tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);

	ret = himax_mcu_register_write(ts, pfw_op->addr_sorting_mode_en,
				       DATA_LEN_4, tmp_data, 0);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL(himax_mcu_assign_sorting_mode);

static int himax_mcu_check_sorting_mode(struct himax_ts_data *ts,
					uint8_t *tmp_data)
{
	struct fw_operation *pfw_op = ts->g_core_cmd_op->fw_op;

	himax_mcu_register_read(ts, pfw_op->addr_sorting_mode_en, DATA_LEN_4,
				tmp_data, 0);
	D("%s: tmp_data[0]=%x,tmp_data[1]=%x\n", __func__, tmp_data[0],
	  tmp_data[1]);

	return NO_ERR;
}
/* FW side end*/
/* CORE_FW */

/* CORE_DRIVER */
#if defined(HX_RST_PIN_FUNC)
static void himax_mcu_pin_reset(struct himax_ts_data *ts)
{
	D("%s: Now reset the Touch chip.\n", __func__);
	himax_rst_gpio_set(ts->rst_gpio, 0);
	msleep(20);
	himax_rst_gpio_set(ts->rst_gpio, 1);
	msleep(50);
}

static void himax_mcu_irq_switch(struct himax_ts_data *ts, int switch_on)
{
	if (switch_on) {
		if (ts->use_irq)
			himax_int_enable(ts, switch_on);
		else
			hrtimer_start(&ts->timer, ktime_set(1, 0),
				      HRTIMER_MODE_REL);

	} else {
		if (ts->use_irq)
			himax_int_enable(ts, switch_on);
		else {
			hrtimer_cancel(&ts->timer);
			cancel_work_sync(&ts->work);
		}
	}
}

static void himax_mcu_reload_config(struct himax_ts_data *ts)
{
	if (himax_report_data_init(ts))
		E("%s: allocate data fail\n", __func__);
	himax_mcu_sense_on(ts, 0x00);
}

void himax_mcu_ic_reset(struct himax_ts_data *ts, uint8_t loadconfig,
			uint8_t int_off)
{
	HX_HW_RESET_ACTIVATE = 0;

	D("%s: status: loadconfig=%d,int_off=%d\n", __func__, loadconfig,
	  int_off);

	if (ts->rst_gpio >= 0) {
		if (int_off)
			himax_mcu_irq_switch(ts, 0);

		himax_mcu_pin_reset(ts);

		if (loadconfig)
			himax_mcu_reload_config(ts);

		if (int_off)
			himax_mcu_irq_switch(ts, 1);
	}
}
EXPORT_SYMBOL(himax_mcu_ic_reset);
#endif

static uint8_t himax_mcu_tp_info_check(struct himax_ts_data *ts)
{
	struct driver_operation *pdriver_op = ts->g_core_cmd_op->driver_op;
	struct himax_ic_data *ic_data = ts->ic_data;

	uint8_t rx = pdriver_op->data_df_rx[0];
	uint8_t tx = pdriver_op->data_df_tx[0];
	uint8_t pt = pdriver_op->data_df_pt[0];
	uint16_t x_res = pdriver_op->data_df_x_res[1] << 8 |
			 pdriver_op->data_df_x_res[0];
	uint16_t y_res = pdriver_op->data_df_y_res[1] << 8 |
			 pdriver_op->data_df_y_res[0];
	uint8_t err_cnt = 0;

	if (ic_data->HX_RX_NUM < (rx / 2) ||
	    ic_data->HX_RX_NUM > (rx * 3 / 2)) {
		ic_data->HX_RX_NUM = rx;
		err_cnt |= 0x01;
	}
	if (ic_data->HX_TX_NUM < (tx / 2) ||
	    ic_data->HX_TX_NUM > (tx * 3 / 2)) {
		ic_data->HX_TX_NUM = tx;
		err_cnt |= 0x02;
	}
	if (ic_data->HX_MAX_PT < (pt / 2) ||
	    ic_data->HX_MAX_PT > (pt * 3 / 2)) {
		ic_data->HX_MAX_PT = pt;
		err_cnt |= 0x04;
	}
	if (ic_data->HX_Y_RES < (y_res / 2) ||
	    ic_data->HX_Y_RES > (y_res * 3 / 2)) {
		ic_data->HX_Y_RES = y_res;
		err_cnt |= 0x08;
	}
	if (ic_data->HX_X_RES < (x_res / 2) ||
	    ic_data->HX_X_RES > (x_res * 3 / 2)) {
		ic_data->HX_X_RES = x_res;
		err_cnt |= 0x10;
	}
	return err_cnt;
}

void himax_mcu_touch_information(struct himax_ts_data *ts)
{
	struct himax_core_command_operation *gcop = ts->g_core_cmd_op;
	struct driver_operation *pdriver_op = gcop->driver_op;
	struct sram_operation *psram_op = gcop->sram_op;
	struct himax_ic_data *ic_data = ts->ic_data;

	uint8_t tmp_addr[4] = { 0 };
	uint8_t tmp_data[4] = { 0 };

	char data[DATA_LEN_8] = { 0 };
	uint8_t err_cnt = 0;

	D("%s: Entering ------", __func__);

	himax_mcu_register_read(ts,
				pdriver_op->addr_fw_define_rxnum_txnum_maxpt,
				DATA_LEN_8, data, 0);
	ic_data->HX_RX_NUM = data[2];
	ic_data->HX_TX_NUM = data[3];
	ic_data->HX_MAX_PT = data[4];

	himax_mcu_register_read(ts, pdriver_op->addr_fw_define_xy_res_enable,
				DATA_LEN_4, data, 0);

	if ((data[1] & 0x04) == 0x04)
		ic_data->HX_XY_REVERSE = true;
	else
		ic_data->HX_XY_REVERSE = false;

	himax_mcu_register_read(ts, pdriver_op->addr_fw_define_x_y_res,
				DATA_LEN_4, data, 0);
	ic_data->HX_Y_RES = data[0] * 256 + data[1];
	ic_data->HX_X_RES = data[2] * 256 + data[3];

	himax_mcu_register_read(ts, pdriver_op->addr_fw_define_int_is_edge,
				DATA_LEN_4, data, 0);

	if ((data[1] & 0x01) == 1)
		ic_data->HX_INT_IS_EDGE = true;
	else
		ic_data->HX_INT_IS_EDGE = false;

	/*1. Read number of MKey R100070E8H to determin data size*/
	himax_mcu_register_read(ts, psram_op->addr_mkey, DATA_LEN_4, data, 0);

	ic_data->HX_BT_NUM = data[0] & 0x03;

	err_cnt = himax_mcu_tp_info_check(ts);
	if (err_cnt > 0)
		E("TP Info from IC is wrong, err_cnt = 0x%X", err_cnt);

	himax_mcu_ic_id_read(ts);
	D("%s: HX_RX_NUM=%d HX_TX_NUM=%d HX_MAX_PT=%d\n", __func__,
	  ic_data->HX_RX_NUM, ic_data->HX_TX_NUM, ic_data->HX_MAX_PT);
	D("%s: HX_XY_REVERSE=%d HX_Y_RES=%d HX_X_RES=%d\n", __func__,
	  ic_data->HX_XY_REVERSE, ic_data->HX_Y_RES, ic_data->HX_X_RES);
	D("%s: HX_INT_IS_EDGE=%d\n", __func__, ic_data->HX_INT_IS_EDGE);

	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x71;
	tmp_addr[0] = 0x9C;
	himax_mcu_register_read(ts, tmp_addr, DATA_LEN_4, tmp_data, 0);
	ic_data->HX_PEN_FUNC = tmp_data[3];
	D("%s: HX_PEN_FUNC = %d\n", __func__, ic_data->HX_PEN_FUNC);

	D("%s: Leave ------", __func__);
}
EXPORT_SYMBOL(himax_mcu_touch_information);

int himax_mcu_get_touch_data_size(void)
{
	return HIMAX_TOUCH_DATA_SIZE;
}
EXPORT_SYMBOL(himax_mcu_get_touch_data_size);

static int himax_mcu_hand_shaking(void)
{
	/* 0:Running, 1:Stop, 2:I2C Fail */
	int result = 0;
	return result;
}

static int himax_mcu_determin_diag_rawdata(int diag_command)
{
	return diag_command % 10;
}

static int himax_mcu_determin_diag_storage(int diag_command)
{
	return diag_command / 10;
}

int himax_mcu_cal_data_len(int raw_cnt_rmd, int HX_MAX_PT, int raw_cnt_max)
{
	int RawDataLen;

	if (raw_cnt_rmd != 0x00)
		RawDataLen = MAX_I2C_TRANS_SZ -
			     ((HX_MAX_PT + raw_cnt_max + 3) * 4) - 1;
	else
		RawDataLen = MAX_I2C_TRANS_SZ -
			     ((HX_MAX_PT + raw_cnt_max + 2) * 4) - 1;

	return RawDataLen;
}
EXPORT_SYMBOL(himax_mcu_cal_data_len);

static bool himax_mcu_diag_check_sum(struct himax_report_data *hx_touch_data)
{
	uint16_t check_sum_cal = 0;
	int i;

	/* Check 128th byte CRC */
	for (i = 0, check_sum_cal = 0; i < (hx_touch_data->touch_all_size -
					    hx_touch_data->touch_info_size);
	     i += 2) {
		check_sum_cal += (hx_touch_data->hx_rawdata_buf[i + 1] *
					  FLASH_RW_MAX_LEN +
				  hx_touch_data->hx_rawdata_buf[i]);
	}

	if (check_sum_cal % HX64K != 0) {
		W("%s: checksum fail=%2X\n", __func__, check_sum_cal);
		return 0;
	}

	return 1;
}

#if defined(HX_ESD_RECOVERY)
int himax_mcu_ic_esd_recovery(struct himax_ts_data *ts, int hx_esd_event,
			      int hx_zero_event, int length)
{
	int ret_val = NO_ERR;

	if (hx_esd_event == length) {
		ts->g_zero_event_count = 0;
		ret_val = HX_ESD_EVENT;
	} else if (hx_zero_event == length) {
		if (ts->g_zero_event_count > 5) {
			ts->g_zero_event_count = 0;
			D("%s: [HIMAX TP MSG]: ESD event checked - ALL Zero.\n",
			  __func__);
			ret_val = HX_ESD_EVENT;
		} else {
			ts->g_zero_event_count++;
			D("%s: [HIMAX TP MSG]: ALL Zero event is %d times.\n",
			  __func__, ts->g_zero_event_count);
			ret_val = HX_ZERO_EVENT_COUNT;
		}
	}

	return ret_val;
}
EXPORT_SYMBOL(himax_mcu_ic_esd_recovery);

void himax_mcu_esd_ic_reset(struct himax_ts_data *ts)
{
	ts->HX_ESD_RESET_ACTIVATE = 0;
#if defined(HX_RST_PIN_FUNC)
	himax_mcu_pin_reset(ts);
#else
	himax_mcu_system_reset(ts);
#endif
	D("%s: \n", __func__);
}
EXPORT_SYMBOL(himax_mcu_esd_ic_reset);
#endif

#if defined(HX_TP_PROC_GUEST_INFO)
static int himax_guest_info_get_status(void)
{
	return g_guest_info_data->g_guest_info_ongoing;
}

static void himax_guest_info_set_status(int setting)
{
	g_guest_info_data->g_guest_info_ongoing = setting;
}

static int himax_guest_info_read(struct himax_ts_data *ts, uint32_t start_addr,
				 uint8_t *flash_tmp_buffer)
{
	uint32_t temp_addr = 0;
	uint8_t tmp_addr[4];
	uint32_t flash_page_len = 0x1000;
	/* uint32_t checksum = 0x00; */
	int result = 0;

	D("%s: Reading guest info in start_addr = 0x%08X !\n", __func__,
	  start_addr);

	tmp_addr[0] = start_addr % 0x100;
	tmp_addr[1] = (start_addr >> 8) % 0x100;
	tmp_addr[2] = (start_addr >> 16) % 0x100;
	tmp_addr[3] = start_addr / 0x1000000;

	D("%s: addr[0]=0x%2X,addr[1]=0x%2X,addr[2]=0x%2X,addr[3]=0x%2X\n",
	  __func__, tmp_addr[0], tmp_addr[1], tmp_addr[2], tmp_addr[3]);

	result = himax_mcu_check_CRC(ts, tmp_addr, flash_page_len);
	if (result != 0) {
		W("%s: CRC checksum failed.\n", __func__);
		goto END_FUNC;
	}

	for (temp_addr = start_addr; temp_addr < (start_addr + flash_page_len);
	     temp_addr = temp_addr + 128) {
		tmp_addr[0] = temp_addr % 0x100;
		tmp_addr[1] = (temp_addr >> 8) % 0x100;
		tmp_addr[2] = (temp_addr >> 16) % 0x100;
		tmp_addr[3] = temp_addr / 0x1000000;
		himax_mcu_register_read(
			ts, tmp_addr, 128,
			&flash_tmp_buffer[temp_addr - start_addr], false);
	}

END_FUNC:
	return result;
}
#endif
/* CORE_DRIVER */

#if defined(HX_SMART_WAKEUP) || defined(HX_HIGH_SENSE) ||                      \
	defined(HX_USB_DETECT_GLOBAL)
void himax_mcu_resend_cmd_func(struct himax_ts_data *ts)
{
#if defined(HX_HIGH_SENSE)
	himax_mcu_set_HSEN_enable(ts);
#endif
}
EXPORT_SYMBOL(himax_mcu_resend_cmd_func);
#endif

int himax_mcu_in_cmd_struct_init(struct himax_ts_data *ts)
{
	int err = 0;

	D("%s: ENTER ****!\n", __func__);

	ts->g_core_cmd_op = kzalloc(sizeof(struct himax_core_command_operation),
				    GFP_KERNEL);
	if (ts->g_core_cmd_op == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_fail;
	}

	ts->g_core_cmd_op->ic_op =
		kzalloc(sizeof(struct ic_operation), GFP_KERNEL);
	if (ts->g_core_cmd_op->ic_op == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_ic_op_fail;
	}

	ts->g_core_cmd_op->fw_op =
		kzalloc(sizeof(struct fw_operation), GFP_KERNEL);
	if (ts->g_core_cmd_op->fw_op == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_fw_op_fail;
	}

	ts->g_core_cmd_op->flash_op =
		kzalloc(sizeof(struct flash_operation), GFP_KERNEL);
	if (ts->g_core_cmd_op->flash_op == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_flash_op_fail;
	}

	ts->g_core_cmd_op->sram_op =
		kzalloc(sizeof(struct sram_operation), GFP_KERNEL);
	if (ts->g_core_cmd_op->sram_op == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_sram_op_fail;
	}

	ts->g_core_cmd_op->driver_op =
		kzalloc(sizeof(struct driver_operation), GFP_KERNEL);
	if (ts->g_core_cmd_op->driver_op == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_driver_op_fail;
	}

	g_internal_buffer =
		kzalloc(sizeof(uint8_t) * HX_MAX_WRITE_SZ, GFP_KERNEL);
	if (g_internal_buffer == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_g_internal_buffer_fail;
	}

	D("%s: LEAVE ****\n", __func__);
	return NO_ERR;

err_g_core_cmd_op_g_internal_buffer_fail:
	kfree(ts->g_core_cmd_op->driver_op);
	ts->g_core_cmd_op->driver_op = NULL;

err_g_core_cmd_op_driver_op_fail:
	kfree(ts->g_core_cmd_op->sram_op);
	ts->g_core_cmd_op->sram_op = NULL;

err_g_core_cmd_op_sram_op_fail:
	kfree(ts->g_core_cmd_op->flash_op);
	ts->g_core_cmd_op->flash_op = NULL;

err_g_core_cmd_op_flash_op_fail:
	kfree(ts->g_core_cmd_op->fw_op);
	ts->g_core_cmd_op->fw_op = NULL;

err_g_core_cmd_op_fw_op_fail:
	kfree(ts->g_core_cmd_op->ic_op);
	ts->g_core_cmd_op->ic_op = NULL;

err_g_core_cmd_op_ic_op_fail:
	kfree(ts->g_core_cmd_op);
	ts->g_core_cmd_op = NULL;

err_g_core_cmd_op_fail:
	D("%s: LEAVE **** error=%d\n", __func__, err);
	return err;
}
EXPORT_SYMBOL(himax_mcu_in_cmd_struct_init);

void himax_mcu_in_cmd_struct_free(struct himax_ts_data *ts)
{
	kfree(g_internal_buffer);
	g_internal_buffer = NULL;

	kfree(ts->g_core_cmd_op->driver_op);
	ts->g_core_cmd_op->driver_op = NULL;

	kfree(ts->g_core_cmd_op->sram_op);
	ts->g_core_cmd_op->sram_op = NULL;

	kfree(ts->g_core_cmd_op->flash_op);
	ts->g_core_cmd_op->flash_op = NULL;

	kfree(ts->g_core_cmd_op->fw_op);
	ts->g_core_cmd_op->fw_op = NULL;

	kfree(ts->g_core_cmd_op->ic_op);
	ts->g_core_cmd_op->ic_op = NULL;

	kfree(ts->g_core_cmd_op);
	ts->g_core_cmd_op = NULL;

	D("%s: release completed\n", __func__);
}
EXPORT_SYMBOL(himax_mcu_in_cmd_struct_free);

void himax_mcu_in_cmd_init(struct himax_ts_data *ts)
{
	struct ic_operation *ic_op = ts->g_core_cmd_op->ic_op;
	struct fw_operation *fw_op = ts->g_core_cmd_op->fw_op;
	struct flash_operation *fl_op = ts->g_core_cmd_op->flash_op;
	struct sram_operation *sr_op = ts->g_core_cmd_op->sram_op;
	struct driver_operation *drv_op = ts->g_core_cmd_op->driver_op;

	D("%s: Entering!\n", __func__);
	/* CORE_IC */
	himax_parse_assign_cmd(ic_adr_ahb_addr_byte_0,
			       ic_op->addr_ahb_addr_byte_0,
			       sizeof(ic_op->addr_ahb_addr_byte_0));
	himax_parse_assign_cmd(ic_adr_ahb_rdata_byte_0,
			       ic_op->addr_ahb_rdata_byte_0,
			       sizeof(ic_op->addr_ahb_rdata_byte_0));
	himax_parse_assign_cmd(ic_adr_ahb_access_direction,
			       ic_op->addr_ahb_access_direction,
			       sizeof(ic_op->addr_ahb_access_direction));
	himax_parse_assign_cmd(ic_adr_conti, ic_op->addr_conti,
			       sizeof(ic_op->addr_conti));
	himax_parse_assign_cmd(ic_adr_incr4, ic_op->addr_incr4,
			       sizeof(ic_op->addr_incr4));
	himax_parse_assign_cmd(ic_adr_i2c_psw_lb, ic_op->adr_i2c_psw_lb,
			       sizeof(ic_op->adr_i2c_psw_lb));
	himax_parse_assign_cmd(ic_adr_i2c_psw_ub, ic_op->adr_i2c_psw_ub,
			       sizeof(ic_op->adr_i2c_psw_ub));
	himax_parse_assign_cmd(ic_cmd_ahb_access_direction_read,
			       ic_op->data_ahb_access_direction_read,
			       sizeof(ic_op->data_ahb_access_direction_read));
	himax_parse_assign_cmd(ic_cmd_conti, ic_op->data_conti,
			       sizeof(ic_op->data_conti));
	himax_parse_assign_cmd(ic_cmd_incr4, ic_op->data_incr4,
			       sizeof(ic_op->data_incr4));
	himax_parse_assign_cmd(ic_cmd_i2c_psw_lb, ic_op->data_i2c_psw_lb,
			       sizeof(ic_op->data_i2c_psw_lb));
	himax_parse_assign_cmd(ic_cmd_i2c_psw_ub, ic_op->data_i2c_psw_ub,
			       sizeof(ic_op->data_i2c_psw_ub));
	himax_parse_assign_cmd(ic_adr_tcon_on_rst, ic_op->addr_tcon_on_rst,
			       sizeof(ic_op->addr_tcon_on_rst));
	himax_parse_assign_cmd(ic_addr_adc_on_rst, ic_op->addr_adc_on_rst,
			       sizeof(ic_op->addr_adc_on_rst));
	himax_parse_assign_cmd(ic_adr_psl, ic_op->addr_psl,
			       sizeof(ic_op->addr_psl));
	himax_parse_assign_cmd(ic_adr_cs_central_state,
			       ic_op->addr_cs_central_state,
			       sizeof(ic_op->addr_cs_central_state));
	himax_parse_assign_cmd(ic_cmd_rst, ic_op->data_rst,
			       sizeof(ic_op->data_rst));
	himax_parse_assign_cmd(ic_adr_osc_en, ic_op->adr_osc_en,
			       sizeof(ic_op->adr_osc_en));
	himax_parse_assign_cmd(ic_adr_osc_pw, ic_op->adr_osc_pw,
			       sizeof(ic_op->adr_osc_pw));
	/* CORE_IC */
	/* CORE_FW */
	himax_parse_assign_cmd(fw_addr_system_reset, fw_op->addr_system_reset,
			       sizeof(fw_op->addr_system_reset));
	himax_parse_assign_cmd(fw_addr_safe_mode_release_pw,
			       fw_op->addr_safe_mode_release_pw,
			       sizeof(fw_op->addr_safe_mode_release_pw));
	himax_parse_assign_cmd(fw_addr_ctrl_fw, fw_op->addr_ctrl_fw_isr,
			       sizeof(fw_op->addr_ctrl_fw_isr));
	himax_parse_assign_cmd(fw_addr_flag_reset_event,
			       fw_op->addr_flag_reset_event,
			       sizeof(fw_op->addr_flag_reset_event));
	himax_parse_assign_cmd(fw_addr_hsen_enable, fw_op->addr_hsen_enable,
			       sizeof(fw_op->addr_hsen_enable));
	himax_parse_assign_cmd(fw_addr_smwp_enable, fw_op->addr_smwp_enable,
			       sizeof(fw_op->addr_smwp_enable));
	himax_parse_assign_cmd(fw_addr_program_reload_from,
			       fw_op->addr_program_reload_from,
			       sizeof(fw_op->addr_program_reload_from));
	himax_parse_assign_cmd(fw_addr_program_reload_to,
			       fw_op->addr_program_reload_to,
			       sizeof(fw_op->addr_program_reload_to));
	himax_parse_assign_cmd(fw_addr_program_reload_page_write,
			       fw_op->addr_program_reload_page_write,
			       sizeof(fw_op->addr_program_reload_page_write));
	himax_parse_assign_cmd(fw_addr_raw_out_sel, fw_op->addr_raw_out_sel,
			       sizeof(fw_op->addr_raw_out_sel));
	himax_parse_assign_cmd(fw_addr_reload_status, fw_op->addr_reload_status,
			       sizeof(fw_op->addr_reload_status));
	himax_parse_assign_cmd(fw_addr_reload_crc32_result,
			       fw_op->addr_reload_crc32_result,
			       sizeof(fw_op->addr_reload_crc32_result));
	himax_parse_assign_cmd(fw_addr_reload_addr_from,
			       fw_op->addr_reload_addr_from,
			       sizeof(fw_op->addr_reload_addr_from));
	himax_parse_assign_cmd(fw_addr_reload_addr_cmd_beat,
			       fw_op->addr_reload_addr_cmd_beat,
			       sizeof(fw_op->addr_reload_addr_cmd_beat));
	himax_parse_assign_cmd(fw_addr_selftest_addr_en,
			       fw_op->addr_selftest_addr_en,
			       sizeof(fw_op->addr_selftest_addr_en));
	himax_parse_assign_cmd(fw_addr_criteria_addr, fw_op->addr_criteria_addr,
			       sizeof(fw_op->addr_criteria_addr));
	himax_parse_assign_cmd(fw_addr_set_frame_addr,
			       fw_op->addr_set_frame_addr,
			       sizeof(fw_op->addr_set_frame_addr));
	himax_parse_assign_cmd(fw_addr_selftest_result_addr,
			       fw_op->addr_selftest_result_addr,
			       sizeof(fw_op->addr_selftest_result_addr));
	himax_parse_assign_cmd(fw_addr_sorting_mode_en,
			       fw_op->addr_sorting_mode_en,
			       sizeof(fw_op->addr_sorting_mode_en));
	himax_parse_assign_cmd(fw_addr_fw_mode_status,
			       fw_op->addr_fw_mode_status,
			       sizeof(fw_op->addr_fw_mode_status));
	himax_parse_assign_cmd(fw_addr_icid_addr, fw_op->addr_icid_addr,
			       sizeof(fw_op->addr_icid_addr));
	himax_parse_assign_cmd(fw_addr_fw_ver_addr, fw_op->addr_fw_ver_addr,
			       sizeof(fw_op->addr_fw_ver_addr));
	himax_parse_assign_cmd(fw_addr_fw_cfg_addr, fw_op->addr_fw_cfg_addr,
			       sizeof(fw_op->addr_fw_cfg_addr));
	himax_parse_assign_cmd(fw_addr_fw_vendor_addr,
			       fw_op->addr_fw_vendor_addr,
			       sizeof(fw_op->addr_fw_vendor_addr));
	himax_parse_assign_cmd(fw_addr_cus_info, fw_op->addr_cus_info,
			       sizeof(fw_op->addr_cus_info));
	himax_parse_assign_cmd(fw_addr_proj_info, fw_op->addr_proj_info,
			       sizeof(fw_op->addr_proj_info));
	himax_parse_assign_cmd(fw_addr_fw_state_addr, fw_op->addr_fw_state_addr,
			       sizeof(fw_op->addr_fw_state_addr));
	himax_parse_assign_cmd(fw_addr_fw_dbg_msg_addr,
			       fw_op->addr_fw_dbg_msg_addr,
			       sizeof(fw_op->addr_fw_dbg_msg_addr));
	himax_parse_assign_cmd(fw_addr_chk_fw_status, fw_op->addr_chk_fw_status,
			       sizeof(fw_op->addr_chk_fw_status));
	himax_parse_assign_cmd(fw_addr_dd_handshak_addr,
			       fw_op->addr_dd_handshak_addr,
			       sizeof(fw_op->addr_dd_handshak_addr));
	himax_parse_assign_cmd(fw_addr_dd_data_addr, fw_op->addr_dd_data_addr,
			       sizeof(fw_op->addr_dd_data_addr));
	himax_parse_assign_cmd(fw_data_system_reset, fw_op->data_system_reset,
			       sizeof(fw_op->data_system_reset));
	himax_parse_assign_cmd(fw_data_safe_mode_release_pw_active,
			       fw_op->data_safe_mode_release_pw_active,
			       sizeof(fw_op->data_safe_mode_release_pw_active));
	himax_parse_assign_cmd(fw_data_clear, fw_op->data_clear,
			       sizeof(fw_op->data_clear));
	himax_parse_assign_cmd(fw_data_clear, fw_op->data_clear,
			       sizeof(fw_op->data_clear));
	himax_parse_assign_cmd(fw_data_fw_stop, fw_op->data_fw_stop,
			       sizeof(fw_op->data_fw_stop));
	himax_parse_assign_cmd(fw_data_safe_mode_release_pw_reset,
			       fw_op->data_safe_mode_release_pw_reset,
			       sizeof(fw_op->data_safe_mode_release_pw_reset));
	himax_parse_assign_cmd(fw_data_program_reload_start,
			       fw_op->data_program_reload_start,
			       sizeof(fw_op->data_program_reload_start));
	himax_parse_assign_cmd(fw_data_program_reload_compare,
			       fw_op->data_program_reload_compare,
			       sizeof(fw_op->data_program_reload_compare));
	himax_parse_assign_cmd(fw_data_program_reload_break,
			       fw_op->data_program_reload_break,
			       sizeof(fw_op->data_program_reload_break));
	himax_parse_assign_cmd(fw_data_selftest_request,
			       fw_op->data_selftest_request,
			       sizeof(fw_op->data_selftest_request));
	himax_parse_assign_cmd(fw_data_criteria_aa_top,
			       fw_op->data_criteria_aa_top,
			       sizeof(fw_op->data_criteria_aa_top));
	himax_parse_assign_cmd(fw_data_criteria_aa_bot,
			       fw_op->data_criteria_aa_bot,
			       sizeof(fw_op->data_criteria_aa_bot));
	himax_parse_assign_cmd(fw_data_criteria_key_top,
			       fw_op->data_criteria_key_top,
			       sizeof(fw_op->data_criteria_key_top));
	himax_parse_assign_cmd(fw_data_criteria_key_bot,
			       fw_op->data_criteria_key_bot,
			       sizeof(fw_op->data_criteria_key_bot));
	himax_parse_assign_cmd(fw_data_criteria_avg_top,
			       fw_op->data_criteria_avg_top,
			       sizeof(fw_op->data_criteria_avg_top));
	himax_parse_assign_cmd(fw_data_criteria_avg_bot,
			       fw_op->data_criteria_avg_bot,
			       sizeof(fw_op->data_criteria_avg_bot));
	himax_parse_assign_cmd(fw_data_set_frame, fw_op->data_set_frame,
			       sizeof(fw_op->data_set_frame));
	himax_parse_assign_cmd(fw_data_selftest_ack_hb,
			       fw_op->data_selftest_ack_hb,
			       sizeof(fw_op->data_selftest_ack_hb));
	himax_parse_assign_cmd(fw_data_selftest_ack_lb,
			       fw_op->data_selftest_ack_lb,
			       sizeof(fw_op->data_selftest_ack_lb));
	himax_parse_assign_cmd(fw_data_selftest_pass, fw_op->data_selftest_pass,
			       sizeof(fw_op->data_selftest_pass));
	himax_parse_assign_cmd(fw_data_normal_cmd, fw_op->data_normal_cmd,
			       sizeof(fw_op->data_normal_cmd));
	himax_parse_assign_cmd(fw_data_normal_status, fw_op->data_normal_status,
			       sizeof(fw_op->data_normal_status));
	himax_parse_assign_cmd(fw_data_sorting_cmd, fw_op->data_sorting_cmd,
			       sizeof(fw_op->data_sorting_cmd));
	himax_parse_assign_cmd(fw_data_sorting_status,
			       fw_op->data_sorting_status,
			       sizeof(fw_op->data_sorting_status));
	himax_parse_assign_cmd(fw_data_dd_request, fw_op->data_dd_request,
			       sizeof(fw_op->data_dd_request));
	himax_parse_assign_cmd(fw_data_dd_ack, fw_op->data_dd_ack,
			       sizeof(fw_op->data_dd_ack));
	himax_parse_assign_cmd(fw_data_idle_dis_pwd, fw_op->data_idle_dis_pwd,
			       sizeof(fw_op->data_idle_dis_pwd));
	himax_parse_assign_cmd(fw_data_idle_en_pwd, fw_op->data_idle_en_pwd,
			       sizeof(fw_op->data_idle_en_pwd));
	himax_parse_assign_cmd(fw_data_rawdata_ready_hb,
			       fw_op->data_rawdata_ready_hb,
			       sizeof(fw_op->data_rawdata_ready_hb));
	himax_parse_assign_cmd(fw_data_rawdata_ready_lb,
			       fw_op->data_rawdata_ready_lb,
			       sizeof(fw_op->data_rawdata_ready_lb));
	himax_parse_assign_cmd(fw_addr_ahb_addr, fw_op->addr_ahb_addr,
			       sizeof(fw_op->addr_ahb_addr));
	himax_parse_assign_cmd(fw_data_ahb_dis, fw_op->data_ahb_dis,
			       sizeof(fw_op->data_ahb_dis));
	himax_parse_assign_cmd(fw_data_ahb_en, fw_op->data_ahb_en,
			       sizeof(fw_op->data_ahb_en));
	himax_parse_assign_cmd(fw_addr_event_addr, fw_op->addr_event_addr,
			       sizeof(fw_op->addr_event_addr));
	himax_parse_assign_cmd(fw_usb_detect_addr, fw_op->addr_usb_detect,
			       sizeof(fw_op->addr_usb_detect));
	/* CORE_FW */
	/* CORE_FLASH */
	himax_parse_assign_cmd(flash_addr_spi200_trans_fmt,
			       fl_op->addr_spi200_trans_fmt,
			       sizeof(fl_op->addr_spi200_trans_fmt));
	himax_parse_assign_cmd(flash_addr_spi200_trans_ctrl,
			       fl_op->addr_spi200_trans_ctrl,
			       sizeof(fl_op->addr_spi200_trans_ctrl));
	himax_parse_assign_cmd(flash_addr_spi200_fifo_rst,
			       fl_op->addr_spi200_fifo_rst,
			       sizeof(fl_op->addr_spi200_fifo_rst));
	himax_parse_assign_cmd(flash_addr_spi200_flash_speed,
			       fl_op->addr_spi200_flash_speed,
			       sizeof(fl_op->addr_spi200_flash_speed));
	himax_parse_assign_cmd(flash_addr_spi200_rst_status,
			       fl_op->addr_spi200_rst_status,
			       sizeof(fl_op->addr_spi200_rst_status));
	himax_parse_assign_cmd(flash_addr_spi200_cmd, fl_op->addr_spi200_cmd,
			       sizeof(fl_op->addr_spi200_cmd));
	himax_parse_assign_cmd(flash_addr_spi200_addr, fl_op->addr_spi200_addr,
			       sizeof(fl_op->addr_spi200_addr));
	himax_parse_assign_cmd(flash_addr_spi200_data, fl_op->addr_spi200_data,
			       sizeof(fl_op->addr_spi200_data));
	himax_parse_assign_cmd(flash_addr_spi200_bt_num,
			       fl_op->addr_spi200_bt_num,
			       sizeof(fl_op->addr_spi200_bt_num));
	himax_parse_assign_cmd(flash_data_spi200_trans_fmt,
			       fl_op->data_spi200_trans_fmt,
			       sizeof(fl_op->data_spi200_trans_fmt));
	himax_parse_assign_cmd(flash_data_spi200_txfifo_rst,
			       fl_op->data_spi200_txfifo_rst,
			       sizeof(fl_op->data_spi200_txfifo_rst));
	himax_parse_assign_cmd(flash_data_spi200_rxfifo_rst,
			       fl_op->data_spi200_rxfifo_rst,
			       sizeof(fl_op->data_spi200_rxfifo_rst));
	himax_parse_assign_cmd(flash_data_spi200_trans_ctrl_1,
			       fl_op->data_spi200_trans_ctrl_1,
			       sizeof(fl_op->data_spi200_trans_ctrl_1));
	himax_parse_assign_cmd(flash_data_spi200_trans_ctrl_2,
			       fl_op->data_spi200_trans_ctrl_2,
			       sizeof(fl_op->data_spi200_trans_ctrl_2));
	himax_parse_assign_cmd(flash_data_spi200_trans_ctrl_3,
			       fl_op->data_spi200_trans_ctrl_3,
			       sizeof(fl_op->data_spi200_trans_ctrl_3));
	himax_parse_assign_cmd(flash_data_spi200_trans_ctrl_4,
			       fl_op->data_spi200_trans_ctrl_4,
			       sizeof(fl_op->data_spi200_trans_ctrl_4));
	himax_parse_assign_cmd(flash_data_spi200_trans_ctrl_5,
			       fl_op->data_spi200_trans_ctrl_5,
			       sizeof(fl_op->data_spi200_trans_ctrl_5));
	himax_parse_assign_cmd(flash_data_spi200_trans_ctrl_6,
			       fl_op->data_spi200_trans_ctrl_6,
			       sizeof(fl_op->data_spi200_trans_ctrl_6));
	himax_parse_assign_cmd(flash_data_spi200_trans_ctrl_7,
			       fl_op->data_spi200_trans_ctrl_7,
			       sizeof(fl_op->data_spi200_trans_ctrl_7));
	himax_parse_assign_cmd(flash_data_spi200_cmd_1,
			       fl_op->data_spi200_cmd_1,
			       sizeof(fl_op->data_spi200_cmd_1));
	himax_parse_assign_cmd(flash_data_spi200_cmd_2,
			       fl_op->data_spi200_cmd_2,
			       sizeof(fl_op->data_spi200_cmd_2));
	himax_parse_assign_cmd(flash_data_spi200_cmd_3,
			       fl_op->data_spi200_cmd_3,
			       sizeof(fl_op->data_spi200_cmd_3));
	himax_parse_assign_cmd(flash_data_spi200_cmd_4,
			       fl_op->data_spi200_cmd_4,
			       sizeof(fl_op->data_spi200_cmd_4));
	himax_parse_assign_cmd(flash_data_spi200_cmd_5,
			       fl_op->data_spi200_cmd_5,
			       sizeof(fl_op->data_spi200_cmd_5));
	himax_parse_assign_cmd(flash_data_spi200_cmd_6,
			       fl_op->data_spi200_cmd_6,
			       sizeof(fl_op->data_spi200_cmd_6));
	himax_parse_assign_cmd(flash_data_spi200_cmd_7,
			       fl_op->data_spi200_cmd_7,
			       sizeof(fl_op->data_spi200_cmd_7));
	himax_parse_assign_cmd(flash_data_spi200_cmd_8,
			       fl_op->data_spi200_cmd_8,
			       sizeof(fl_op->data_spi200_cmd_8));
	himax_parse_assign_cmd(flash_data_spi200_addr, fl_op->data_spi200_addr,
			       sizeof(fl_op->data_spi200_addr));
	/* CORE_FLASH */
	/* CORE_SRAM */
	/* sram start*/
	himax_parse_assign_cmd(sram_adr_mkey, sr_op->addr_mkey,
			       sizeof(sr_op->addr_mkey));
	himax_parse_assign_cmd(sram_adr_rawdata_addr, sr_op->addr_rawdata_addr,
			       sizeof(sr_op->addr_rawdata_addr));
	himax_parse_assign_cmd(sram_adr_rawdata_end, sr_op->addr_rawdata_end,
			       sizeof(sr_op->addr_rawdata_end));
	himax_parse_assign_cmd(sram_passwrd_start, sr_op->passwrd_start,
			       sizeof(sr_op->passwrd_start));
	himax_parse_assign_cmd(sram_passwrd_end, sr_op->passwrd_end,
			       sizeof(sr_op->passwrd_end));
	/* sram end*/
	/* CORE_SRAM */
	/* CORE_DRIVER */
	himax_parse_assign_cmd(driver_addr_fw_define_flash_reload,
			       drv_op->addr_fw_define_flash_reload,
			       sizeof(drv_op->addr_fw_define_flash_reload));
	himax_parse_assign_cmd(driver_addr_fw_define_2nd_flash_reload,
			       drv_op->addr_fw_define_2nd_flash_reload,
			       sizeof(drv_op->addr_fw_define_2nd_flash_reload));
	himax_parse_assign_cmd(driver_addr_fw_define_int_is_edge,
			       drv_op->addr_fw_define_int_is_edge,
			       sizeof(drv_op->addr_fw_define_int_is_edge));
	himax_parse_assign_cmd(
		driver_addr_fw_define_rxnum_txnum_maxpt,
		drv_op->addr_fw_define_rxnum_txnum_maxpt,
		sizeof(drv_op->addr_fw_define_rxnum_txnum_maxpt));
	himax_parse_assign_cmd(driver_addr_fw_define_xy_res_enable,
			       drv_op->addr_fw_define_xy_res_enable,
			       sizeof(drv_op->addr_fw_define_xy_res_enable));
	himax_parse_assign_cmd(driver_addr_fw_define_x_y_res,
			       drv_op->addr_fw_define_x_y_res,
			       sizeof(drv_op->addr_fw_define_x_y_res));
	himax_parse_assign_cmd(driver_data_df_rx, drv_op->data_df_rx,
			       sizeof(drv_op->data_df_rx));
	himax_parse_assign_cmd(driver_data_df_tx, drv_op->data_df_tx,
			       sizeof(drv_op->data_df_tx));
	himax_parse_assign_cmd(driver_data_df_pt, drv_op->data_df_pt,
			       sizeof(drv_op->data_df_pt));
	himax_parse_assign_cmd(driver_data_df_x_res, drv_op->data_df_x_res,
			       sizeof(drv_op->data_df_x_res));
	himax_parse_assign_cmd(driver_data_df_y_res, drv_op->data_df_y_res,
			       sizeof(drv_op->data_df_y_res));
	himax_parse_assign_cmd(driver_data_fw_define_flash_reload_dis,
			       drv_op->data_fw_define_flash_reload_dis,
			       sizeof(drv_op->data_fw_define_flash_reload_dis));
	himax_parse_assign_cmd(driver_data_fw_define_flash_reload_en,
			       drv_op->data_fw_define_flash_reload_en,
			       sizeof(drv_op->data_fw_define_flash_reload_en));
	himax_parse_assign_cmd(
		driver_data_fw_define_rxnum_txnum_maxpt_sorting,
		drv_op->data_fw_define_rxnum_txnum_maxpt_sorting,
		sizeof(drv_op->data_fw_define_rxnum_txnum_maxpt_sorting));
	himax_parse_assign_cmd(
		driver_data_fw_define_rxnum_txnum_maxpt_normal,
		drv_op->data_fw_define_rxnum_txnum_maxpt_normal,
		sizeof(drv_op->data_fw_define_rxnum_txnum_maxpt_normal));
	/* CORE_DRIVER */
}
EXPORT_SYMBOL(himax_mcu_in_cmd_init);
/* CORE_INIT init end */
