// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_hidh_api.h"
#include "esp_err.h"
#include "pr_log.h"

#pragma once

/*
 * Vendor ID for sony controller
 */
#define VENDOR_SONY				0x054c

/*
 * DualShock 4 device IDs
 */
#define DEV_ID_SONY_PS4			0x05c4
#define DEV_ID_SONY_PS4_V2 		0x09cc

/*
 * DualSense device IDs
 */
#define DEV_ID_SONY_PS5			0x0ce6
#define DEV_ID_SONY_PS5_V2 		0x0df2

struct sensor_data {
	int x;
	int y;
	int z;
};

/* HID controller Input */
typedef struct {
	/* Analog Stick */
	uint8_t lx;
	uint8_t ly;
	uint8_t rx;
	uint8_t ry;

	/* Analog Shoulder Buttons */
	uint8_t l2_ana;
	uint8_t r2_ana;

	/* Battery Capacity */
	uint8_t battery_capacity;

	/* Digital buttons */
	bool square;
	bool cross;
	bool circle;
	bool triangle;
	bool l1;
	bool l2;
	bool l3;
	bool r1;
	bool r2;
	bool r3;

	/* HAT switch */
	int8_t hat_x;
	int8_t hat_y;

	/* secondary buttons */
	bool create;
	bool options;
	bool ps_home;
	bool touch_pad;

	/* sensor data */
	struct sensor_data gyro;
	struct sensor_data accel;
} ps_hid_input_t;

/* HID controller Output */
typedef struct {
	/* Rumble */
	bool update_rumble;

	/* RGB lightbar */
	bool update_lightbar;
	bool update_lightbar_blink;

	/* Rumble */
	uint8_t motor_left;
	uint8_t motor_right;

	/* RGB lightbar */
	uint8_t lightbar_red;
	uint8_t lightbar_green;
	uint8_t lightbar_blue;
	uint8_t lightbar_blink_on; // DualShock 4 specific
	uint8_t lightbar_blink_off; // DualShock 4 specific
} ps_hid_output_t;

/*
 * struct ps_ops_t
 * 	- function ops for Playstation controllers
 *		- firmware: HID report for retrieving firmware info
 * 		- calibration: HID report for retrieving calibration data
 *		- get_rpt: Handler the HID report data via callback
 * 		- output_handler: Output handler for the HID reports
 * 		- input_handler: input data handler for the controller
 * 		- get_calib_data: retrieve calibration data from the controller
 * 		- get_data: retrieve input data from the controller
 */
typedef struct {
	void (* firmware)(esp_bd_addr_t mac);
	void (* calibration)(esp_bd_addr_t mac);
	void (* get_report)(struct hidh_get_rpt_evt_param *buf);
	void (* output_handler)(esp_bd_addr_t mac, ps_hid_output_t *buf);
	void (* input_handler)(struct hidh_data_ind_evt_param *dbuf);
	void (* get_data)(ps_hid_input_t *dbuf);
} ps_ops_t;

/*
 * DualShock 4 function ops
 */
extern ps_ops_t ds4_ops;

/*
 * DualSense function ops
 */
extern ps_ops_t ds_ops;

/*
 * function: ps_hid_set_rumble
 * - type: void
 * - inputs:
 * 		- r: right motor rumble strenght
 *		- l: left motor rumble strenght
 */
void ps_hid_set_rumble(uint8_t r, uint8_t l);

/*
 * function: ps_hid_set_led
 * - type: void
 * - inputs:
 * 		- r: red light strenght
 *		- g: green light strenght
 *		- b: blue light strenght
 */
void ps_hid_set_led(uint8_t r, uint8_t g, uint8_t b);

/*
 * function: ps_hid_set_led
 * - type: void
 * - inputs:
 * 		- on_delay: red light strenght
 *		- g: green light strenght
 *		- b: blue light strenght
 */
void ps_hid_set_led_blink(uint8_t on_delay, uint8_t off_delay);

/*
 * function: ps_hid_is_dev_connected
 * - type: bool
 *
 * - return: true if dev connected, else false
 */
bool ps_hid_is_dev_connected(void);

/*
 * function: ps_hid_get_data
 * - type: ps_hid_input_t
 *
 * - return: data received from controller
 */
ps_hid_input_t ps_hid_get_data(void);

/*
 * function: ps_hid_init
 * - type: esp_err_t (int)
 * - inputs:
 * 		- mac: bluetooth mac address of the controller
 *
 * - return: ESP_OK on success, others on failure
 */
esp_err_t ps_hid_init(esp_bd_addr_t mac);

/*
 * function: ps_hid_exit
 * - type: void
 * - inputs:
 * 		- mac: bluetooth mac address of the controller
 */
void ps_hid_exit(esp_bd_addr_t mac);
