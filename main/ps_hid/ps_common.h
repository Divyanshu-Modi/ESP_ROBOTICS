// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#include "esp_bit_defs.h"
#include "esp_hidh_api.h"

#pragma once

/*
 * Button masks for input report
 */
#define DS_BUTTONS0_SQUARE		BIT(4)
#define DS_BUTTONS0_CROSS		BIT(5)
#define DS_BUTTONS0_CIRCLE		BIT(6)
#define DS_BUTTONS0_TRIANGLE	BIT(7)

#define DS_BUTTONS1_L1			BIT(0)
#define DS_BUTTONS1_R1			BIT(1)
#define DS_BUTTONS1_L2			BIT(2)
#define DS_BUTTONS1_R2			BIT(3)
#define DS_BUTTONS1_CREATE		BIT(4)
#define DS_BUTTONS1_OPTIONS		BIT(5)
#define DS_BUTTONS1_L3			BIT(6)
#define DS_BUTTONS1_R3			BIT(7)

#define DS_BUTTONS2_PS_HOME		BIT(0)
#define DS_BUTTONS2_TOUCHPAD	BIT(1)
#define DS_BUTTONS2_MIC_MUTE	BIT(2)

/* BATTERY MASKS */
#define DS_BATTERY_MASK			(0x0F)

/* HAT SWITCH */
#define DS_BUTTONS0_HAT_SWITCH (0x0F)
#define HAT_ARRAY_SIZE 			9

static const struct { int x; int y; } ps_gamepad_hat_mapping[HAT_ARRAY_SIZE] = {
	{ 0, -1 }, { 1, -1 }, { 1, 0 },
	{ 1, 1 }, { 0, 1 }, { -1, 1 },
	{ -1, 0 }, { -1, -1 }, { 0, 0 },
};

/* SENSOR CALIBRATION */
#define S16_MAX 32767

/* Calculate "x * n / d" without unnecessary overflow or loss of precision. */
#define mult_frac(x, n, d)	\
({					\
	typeof(x) x_ = (x);		\
	typeof(n) n_ = (n);		\
	typeof(d) d_ = (d);		\
					\
	typeof(x_) q = x_ / d_;	\
	typeof(x_) r = x_ % d_;	\
	q * n_ + r * n_ / d_;	\
})

struct calib_data {
	short bias;
	int sens_numer;
	int sens_denom;
};

typedef struct {
	struct calib_data gyro[3];
	struct calib_data accel[3];
} ps_calib_data_t;

/*
 * function: ps_hid_get_report
 * - type: esp_err_t (int)
 * - inputs:
 * 		- mac: bluetooth mac address of the controller
 *		- report_id: HID report id
 *		- report_size: HID report size
 *
 * - return: ESP_OK on success, others on failure
 */
static inline esp_err_t ps_hid_get_report(esp_bd_addr_t mac, uint8_t report_id, int report_size)
{
	return esp_bt_hid_host_get_report(mac, ESP_HIDH_REPORT_TYPE_FEATURE, report_id, report_size);
}
