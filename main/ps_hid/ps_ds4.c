// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#define pr_fmt "DS4"

#include "ps.h"
#include "ps_ds4.h"

static struct ds4_input_report *report = NULL;
static ps_calib_data_t calib_data = { 0 };

void ds4_firmware_info(esp_bd_addr_t mac)
{
	ps_hid_get_report(mac, DS4_FIRMWARE_INFO, DS4_FIRMWARE_INFO_SIZE);
}

void ds4_calibration_info(esp_bd_addr_t mac)
{
	ps_hid_get_report(mac, DS4_CALIBRATION, DS4_CALIBRATION_SIZE);
}

void ds4_get_report(struct hidh_get_rpt_evt_param *buf)
{
	switch (buf->len) {
	case DS4_FIRMWARE_INFO_SIZE:
		/*
		 * Firmware version and hw version can be used to determine fix or patches
		 * but, is not usefull in any other way so just print it.
		 */
		pr_info("DualShock 4 controller: hw_version=0x%08lx fw_version=0x%08lx",
						(uint32_t)buf->data[35], (uint32_t)buf->data[41]);
		break;
	case DS4_CALIBRATION_SIZE:
		int speed_2x = 0;
		int range_2g = 0;

		/* GYRO CALIB DATA */
		speed_2x = ((short)buf->data[GYRO_SPEED_PLUS] + (short)buf->data[GYRO_SPEED_MINUS]);
		calib_data.gyro[0].bias = 0;
		calib_data.gyro[0].sens_numer = speed_2x * DS4_GYRO_RES_PER_DEG_S;
		calib_data.gyro[0].sens_denom =
			abs(buf->data[GYRO_PITCH_PLUS] - buf->data[GYRO_PITCH_BIAS]) +
			abs(buf->data[GYRO_PITCH_MINUS] - buf->data[GYRO_PITCH_BIAS]);

		calib_data.gyro[1].bias = 0;
		calib_data.gyro[1].sens_numer = speed_2x * DS4_GYRO_RES_PER_DEG_S;
		calib_data.gyro[1].sens_denom =
			abs(buf->data[GYRO_YAW_PLUS] - buf->data[GYRO_YAW_BIAS]) +
			abs(buf->data[GYRO_YAW_MINUS] - buf->data[GYRO_YAW_BIAS]);

		calib_data.gyro[2].bias = 0;
		calib_data.gyro[2].sens_numer = speed_2x * DS4_GYRO_RES_PER_DEG_S;
		calib_data.gyro[2].sens_denom =
			abs(buf->data[GYRO_ROLL_PLUS] - buf->data[GYRO_ROLL_BIAS]) +
			abs(buf->data[GYRO_ROLL_MINUS] - buf->data[GYRO_ROLL_BIAS]);

		/* ACCEL CALIB DATA */
		range_2g = buf->data[ACCEL_X_PLUS] - buf->data[ACCEL_X_MINUS];
		calib_data.accel[0].bias = buf->data[ACCEL_X_PLUS] - range_2g / 2;
		calib_data.accel[0].sens_numer = 2 * DS4_ACC_RES_PER_G;
		calib_data.accel[0].sens_denom = range_2g;

		range_2g = buf->data[ACCEL_Y_PLUS] - buf->data[ACCEL_Y_MINUS];
		calib_data.accel[1].bias = buf->data[ACCEL_Y_PLUS] - range_2g / 2;
		calib_data.accel[1].sens_numer = 2 * DS4_ACC_RES_PER_G;
		calib_data.accel[1].sens_denom = range_2g;

		range_2g = buf->data[ACCEL_Z_PLUS] - buf->data[ACCEL_Z_MINUS];
		calib_data.accel[2].bias = buf->data[ACCEL_Z_PLUS] - range_2g / 2;
		calib_data.accel[2].sens_numer = 2 * DS4_ACC_RES_PER_G;
		calib_data.accel[2].sens_denom = range_2g;
		break;
	default:
		break;
	}
}

void ds4_output_handler(esp_bd_addr_t mac, ps_hid_output_t *buf)
{
	struct ds4_output_report report = { 0 };
	static bool interval_set = false;

	report.id = DS4_OUTPUT_REPORT_BT;
	report.hw_ctrl = DS4_OUTPUT_HWCTL_HID;

	if (!interval_set) {
		report.hw_ctrl |= DS4_BT_POLL_INTERVAL_MS;
		interval_set = true;
	}

	if (buf->update_rumble) {
		report.valid_flag0 |= DS4_OUTPUT_VALID_FLAG0_MOTOR;
		report.motor_right = buf->motor_right;
		report.motor_left = buf->motor_left;
		buf->update_rumble = false;
	}

	if (buf->update_lightbar) {
		report.valid_flag0 |= DS4_OUTPUT_VALID_FLAG0_LED;
		report.lightbar_red = buf->lightbar_red;
		report.lightbar_blue = buf->lightbar_blue;
		report.lightbar_green = buf->lightbar_green;
		buf->update_lightbar = false;
	}

	if (buf->update_lightbar_blink) {
		report.valid_flag0 |= DS4_OUTPUT_VALID_FLAG0_LED_BLINK;
		report.lightbar_blink_on = buf->lightbar_blink_on;
		report.lightbar_blink_off = buf->lightbar_blink_off;
		buf->update_lightbar_blink = false;
	}

	esp_bt_hid_host_set_report(mac, ESP_HIDH_REPORT_TYPE_OUTPUT,
					(uint8_t *)&report, DS4_OUTPUT_REPORT_BT_SIZE);
}

void ds4_data_handler(struct hidh_data_ind_evt_param *dbuf)
{
	// TODO: add support for minimal report
	if (dbuf->data[0] == DS4_INPUT_REPORT_BT
			&& dbuf->len == DS4_INPUT_REPORT_BT_SIZE)
		report = (struct ds4_input_report *)dbuf->data;
}

void ds4_get_data(ps_hid_input_t *dbuf)
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
	battery_data = report->status[0] & DS_BATTERY_MASK;
	if (battery_data < 10)
		ldata.battery_capacity = battery_data * 10 + 5;
	else
		ldata.battery_capacity = 100;

	if (calib_data.gyro[0].sens_denom == 0) {
		calib_data.gyro[0].sens_numer = DS4_GYRO_RANGE;
		calib_data.gyro[0].sens_denom = S16_MAX;
	}

	if (calib_data.gyro[1].sens_denom == 0) {
		calib_data.gyro[1].sens_numer = DS4_GYRO_RANGE;
		calib_data.gyro[1].sens_denom = S16_MAX;
	}

	if (calib_data.gyro[2].sens_denom == 0) {
		calib_data.gyro[2].sens_numer = DS4_GYRO_RANGE;
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
		calib_data.accel[0].sens_numer = DS4_ACC_RANGE;
		calib_data.accel[0].sens_denom = S16_MAX;
	}

	if (calib_data.accel[1].sens_denom == 0) {
		calib_data.accel[1].bias = 0;
		calib_data.accel[1].sens_numer = DS4_ACC_RANGE;
		calib_data.accel[1].sens_denom = S16_MAX;
	}

	if (calib_data.accel[2].sens_denom == 0) {
		calib_data.accel[2].bias = 0;
		calib_data.accel[2].sens_numer = DS4_ACC_RANGE;
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

ps_ops_t ds4_ops = {
	.firmware = ds4_firmware_info,
	.calibration = ds4_calibration_info,
	.get_report = ds4_get_report,
	.output_handler = ds4_output_handler,
	.input_handler = ds4_data_handler,
	.get_data = ds4_get_data,
};
