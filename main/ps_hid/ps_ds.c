// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#define pr_fmt "DS"

#include "ps.h"
#include "ps_ds.h"
#include <string.h>

static struct ds_input_report *report = NULL;
static ps_calib_data_t calib_data = { 0 };
static bool use_vibration_v2 = true;

void ds_firmware_info(esp_bd_addr_t mac)
{
	ps_hid_get_report(mac, DS_FIRMWARE_INFO, DS_FIRMWARE_INFO_SIZE);
}

void ds_calibration_info(esp_bd_addr_t mac)
{
	ps_hid_get_report(mac, DS_CALIBRATION, DS_CALIBRATION_SIZE);
}

void ds_get_report(struct hidh_get_rpt_evt_param *buf)
{
	switch (buf->len) {
	case DS_FIRMWARE_INFO_SIZE:
		uint16_t update_version = 0;
		/*
		 * Firmware version and hw version can be used to determine fix or patches
		 * but, is not usefull in any other way so just print it.
		 */
		pr_info("DualSense controller: hw_version=0x%lx fw_version=0x%lx",
						(uint32_t)buf->data[24], (uint32_t)buf->data[28]);

		update_version = (uint16_t)buf->data[44];
		if (update_version < DS_FEATURE_VERSION(2, 21))
			use_vibration_v2 = false;

		break;
	case DS_CALIBRATION_SIZE:
		int speed_2x = 0;
		int range_2g = 0;

		/* GYRO CALIB DATA */
		speed_2x = ((short)buf->data[GYRO_SPEED_PLUS] + (short)buf->data[GYRO_SPEED_MINUS]);
		calib_data.gyro[0].bias = 0;
		calib_data.gyro[0].sens_numer = speed_2x * DS_GYRO_RES_PER_DEG_S;
		calib_data.gyro[0].sens_denom =
			abs(buf->data[GYRO_PITCH_PLUS] - buf->data[GYRO_PITCH_BIAS]) +
			abs(buf->data[GYRO_PITCH_MINUS] - buf->data[GYRO_PITCH_BIAS]);

		calib_data.gyro[1].bias = 0;
		calib_data.gyro[1].sens_numer = speed_2x * DS_GYRO_RES_PER_DEG_S;
		calib_data.gyro[1].sens_denom =
			abs(buf->data[GYRO_YAW_PLUS] - buf->data[GYRO_YAW_BIAS]) +
			abs(buf->data[GYRO_YAW_MINUS] - buf->data[GYRO_YAW_BIAS]);

		calib_data.gyro[2].bias = 0;
		calib_data.gyro[2].sens_numer = speed_2x * DS_GYRO_RES_PER_DEG_S;
		calib_data.gyro[2].sens_denom =
			abs(buf->data[GYRO_ROLL_PLUS] - buf->data[GYRO_ROLL_BIAS]) +
			abs(buf->data[GYRO_ROLL_MINUS] - buf->data[GYRO_ROLL_BIAS]);

		/* ACCEL CALIB DATA */
		range_2g = buf->data[ACCEL_X_PLUS] - buf->data[ACCEL_X_MINUS];
		calib_data.accel[0].bias = buf->data[ACCEL_X_PLUS] - range_2g / 2;
		calib_data.accel[0].sens_numer = 2 * DS_ACC_RES_PER_G;
		calib_data.accel[0].sens_denom = range_2g;

		range_2g = buf->data[ACCEL_Y_PLUS] - buf->data[ACCEL_Y_MINUS];
		calib_data.accel[1].bias = buf->data[ACCEL_Y_PLUS] - range_2g / 2;
		calib_data.accel[1].sens_numer = 2 * DS_ACC_RES_PER_G;
		calib_data.accel[1].sens_denom = range_2g;

		range_2g = buf->data[ACCEL_Z_PLUS] - buf->data[ACCEL_Z_MINUS];
		calib_data.accel[2].bias = buf->data[ACCEL_Z_PLUS] - range_2g / 2;
		calib_data.accel[2].sens_numer = 2 * DS_ACC_RES_PER_G;
		calib_data.accel[2].sens_denom = range_2g;
		break;
	default:
		break;
	}
}

void ds_output_handler(esp_bd_addr_t mac, ps_hid_output_t *buf)
{
	struct ds_output_report report = { 0 };
	static bool first_output = true;
	static uint8_t output_seq = 0;

	report.id = DS_OUTPUT_REPORT_BT;
	report.tag = DS_OUTPUT_TAG;
	report.seq_tag = (output_seq << 4) | 0x0;
	if (++output_seq == 16)
		output_seq = 0;

	// TODO:  mic status (hw)
	if (first_output) {
		report.valid_flag2 = DS_OUTPUT_VALID_FLAG2_LIGHTBAR_SETUP_CONTROL_ENABLE;
		report.lightbar_setup = DS_OUTPUT_LIGHTBAR_SETUP_LIGHT_OUT;

		esp_bt_hid_host_set_report(mac, ESP_HIDH_REPORT_TYPE_OUTPUT,
							(uint8_t *)&report, DS_OUTPUT_REPORT_BT_SIZE);

		memset(&report, 0, sizeof(struct ds_output_report));

		report.player_leds = BIT(2);
		esp_bt_hid_host_set_report(mac, ESP_HIDH_REPORT_TYPE_OUTPUT,
							(uint8_t *)&report, DS_OUTPUT_REPORT_BT_SIZE);

		memset(&report, 0, sizeof(struct ds_output_report));
	}

	if (buf->update_rumble) {
		report.valid_flag0 |= DS_OUTPUT_VALID_FLAG0_HAPTICS_SELECT;
		if (use_vibration_v2)
			report.valid_flag2 |= DS_OUTPUT_VALID_FLAG2_COMPATIBLE_VIBRATION2;
		else
			report.valid_flag0 |= DS_OUTPUT_VALID_FLAG0_COMPATIBLE_VIBRATION;

		report.motor_right = buf->motor_right;
		report.motor_left = buf->motor_left;
		buf->update_rumble = false;
	}

	if (buf->update_lightbar) {
		report.valid_flag1 |= DS_OUTPUT_VALID_FLAG1_LIGHTBAR_CONTROL_ENABLE;
		report.lightbar_red = buf->lightbar_red;
		report.lightbar_blue = buf->lightbar_blue;
		report.lightbar_green = buf->lightbar_green;
		buf->update_lightbar = false;
	}

	esp_bt_hid_host_set_report(mac, ESP_HIDH_REPORT_TYPE_OUTPUT,
					(uint8_t *)&report, DS_OUTPUT_REPORT_BT_SIZE);
}

void ds_data_handler(struct hidh_data_ind_evt_param *dbuf)
{
	if (dbuf->data[0] == DS_INPUT_REPORT_BT
			&& dbuf->len == DS_INPUT_REPORT_BT_SIZE)
		report = (struct ds_input_report *)&dbuf->data[2];
}

void ds_get_data(ps_hid_input_t *dbuf)
{
	ps_hid_input_t ldata = { 0 };
	uint8_t battery_data = 0;
	uint8_t hat_val = 0;

	if (report == NULL) {
		dbuf = NULL;
		return;
	}

	/* Analog Sticks */
	ldata.lx = report->x;
	ldata.ly = report->y;
	ldata.rx = report->rx;
	ldata.ry = report->ry;

	/* Ananlog shoulder buttons */
	ldata.l2_ana = report->z;
	ldata.r2_ana = report->rz;

	/* Digital Buttons */
	ldata.square = (report->buttons[0] & DS_BUTTONS0_SQUARE);
	ldata.cross = (report->buttons[0] & DS_BUTTONS0_CROSS);
	ldata.circle = (report->buttons[0] & DS_BUTTONS0_CIRCLE);
	ldata.triangle = (report->buttons[0] & DS_BUTTONS0_TRIANGLE);

	hat_val = report->buttons[0] & DS_BUTTONS0_HAT_SWITCH;
	if (hat_val >= HAT_ARRAY_SIZE - 1)
		hat_val = 8;

	ldata.hat_x = ps_gamepad_hat_mapping[hat_val].x;
	ldata.hat_y = ps_gamepad_hat_mapping[hat_val].y;

	ldata.l1 = (report->buttons[1] & DS_BUTTONS1_L1);
	ldata.r1 = (report->buttons[1] & DS_BUTTONS1_R1);
	ldata.l2 = (report->buttons[1] & DS_BUTTONS1_L2);
	ldata.r2 = (report->buttons[1] & DS_BUTTONS1_R2);
	ldata.l3 = (report->buttons[1] & DS_BUTTONS1_L3);
	ldata.r3 = (report->buttons[1] & DS_BUTTONS1_R3);

	/* secondary buttons */
	ldata.create = (report->buttons[1] & DS_BUTTONS1_CREATE);
	ldata.options = (report->buttons[1] & DS_BUTTONS1_OPTIONS);
	ldata.ps_home = (report->buttons[2] & DS_BUTTONS2_PS_HOME);
	ldata.touch_pad = (report->buttons[2] & DS_BUTTONS2_TOUCHPAD);

	/* Battery Capacity */
	battery_data = report->status & DS_BATTERY_MASK;
	if (battery_data < 10)
		ldata.battery_capacity = battery_data * 10 + 5;
	else
		ldata.battery_capacity = 100;

	if (calib_data.gyro[0].sens_denom == 0) {
		calib_data.gyro[0].sens_numer = DS_GYRO_RANGE;
		calib_data.gyro[0].sens_denom = S16_MAX;
	}

	if (calib_data.gyro[1].sens_denom == 0) {
		calib_data.gyro[1].sens_numer = DS_GYRO_RANGE;
		calib_data.gyro[1].sens_denom = S16_MAX;
	}

	if (calib_data.gyro[2].sens_denom == 0) {
		calib_data.gyro[2].sens_numer = DS_GYRO_RANGE;
		calib_data.gyro[2].sens_denom = S16_MAX;
	}

	ldata.gyro.x = mult_frac(calib_data.gyro[0].sens_numer,
					(int)(short)report->gyro[0], calib_data.gyro[0].sens_denom);
	ldata.gyro.y = mult_frac(calib_data.gyro[1].sens_numer,
					(int)(short)report->gyro[1], calib_data.gyro[1].sens_denom);
	ldata.gyro.z = mult_frac(calib_data.gyro[2].sens_numer,
					(int)(short)report->gyro[2], calib_data.gyro[2].sens_denom);

	if (calib_data.accel[0].sens_denom == 0) {
		calib_data.accel[0].bias = 0;
		calib_data.accel[0].sens_numer = DS_ACC_RANGE;
		calib_data.accel[0].sens_denom = S16_MAX;
	}

	if (calib_data.accel[1].sens_denom == 0) {
		calib_data.accel[1].bias = 0;
		calib_data.accel[1].sens_numer = DS_ACC_RANGE;
		calib_data.accel[1].sens_denom = S16_MAX;
	}

	if (calib_data.accel[2].sens_denom == 0) {
		calib_data.accel[2].bias = 0;
		calib_data.accel[2].sens_numer = DS_ACC_RANGE;
		calib_data.accel[2].sens_denom = S16_MAX;
	}

	ldata.accel.x = mult_frac(calib_data.accel[0].sens_numer,
					(int)(short)report->accel[0] - calib_data.accel[0].bias,
					calib_data.accel[0].sens_denom);
	ldata.accel.y = mult_frac(calib_data.accel[1].sens_numer,
					(int)(short)report->accel[1] - calib_data.accel[1].bias,
					calib_data.accel[1].sens_denom);
	ldata.accel.z = mult_frac(calib_data.accel[2].sens_numer,
					(int)(short)report->accel[2] - calib_data.accel[2].bias,
					calib_data.accel[2].sens_denom);

	*dbuf = ldata;
}

ps_ops_t ds_ops = {
	.firmware = ds_firmware_info,
	.calibration = ds_calibration_info,
	.get_report = ds_get_report,
	.output_handler = ds_output_handler,
	.input_handler = ds_data_handler,
	.get_data = ds_get_data,
};
