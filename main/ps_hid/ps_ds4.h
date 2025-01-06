// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#include "ps_common.h"

#pragma once

#define DS4_OUTPUT_REPORT_BT				0x11
#define DS4_OUTPUT_REPORT_BT_SIZE			78
#define DS4_INPUT_REPORT_BT					0x11
#define DS4_INPUT_REPORT_BT_SIZE			78
#define DS4_INPUT_REPORT_BT_MINIMAL			0x01
#define DS4_INPUT_REPORT_BT_MINIMAL_SIZE	10

#define DS4_FIRMWARE_INFO				0xa3
#define DS4_FIRMWARE_INFO_SIZE			49
#define DS4_CALIBRATION					0x02
#define DS4_CALIBRATION_SIZE			37

/* Default to 4ms poll interval, which is same as USB (not adjustable). */
#define DS4_BT_POLL_INTERVAL_MS			4

// OUTPUT REPORT
#define DS4_OUTPUT_HWCTL_CRC32		0x40
#define DS4_OUTPUT_HWCTL_HID		0x80

#define DS4_OUTPUT_VALID_FLAG0_MOTOR		0x01
#define DS4_OUTPUT_VALID_FLAG0_LED			0x02
#define DS4_OUTPUT_VALID_FLAG0_LED_BLINK	0x04

/* DualShock4 hardware limits */
#define DS4_ACC_RES_PER_G			8192
#define DS4_ACC_RANGE				(4 * DS4_ACC_RES_PER_G)
#define DS4_GYRO_RES_PER_DEG_S		1024
#define DS4_GYRO_RANGE				(2048 * DS4_GYRO_RES_PER_DEG_S)
#define DS4_TOUCHPAD_WIDTH			1920
#define DS4_TOUCHPAD_HEIGHT			942

/* SENSOR INDEX */
/* GYRO INDEX */
#define GYRO_PITCH_BIAS 		(1)
#define GYRO_YAW_BIAS 			(3)
#define GYRO_ROLL_BIAS 			(5)
#define GYRO_PITCH_PLUS 		(7)
#define GYRO_YAW_PLUS 			(9)
#define GYRO_ROLL_PLUS			(11)
#define GYRO_PITCH_MINUS 		(13)
#define GYRO_YAW_MINUS 			(15)
#define GYRO_ROLL_MINUS			(17)
#define GYRO_SPEED_PLUS 		(19)
#define GYRO_SPEED_MINUS 		(21)

/* ACCEL INDEX */
#define ACCEL_X_PLUS 			(23)
#define ACCEL_X_MINUS 			(25)
#define ACCEL_Y_PLUS 			(27)
#define ACCEL_Y_MINUS 			(29)
#define ACCEL_Z_PLUS 			(31)
#define ACCEL_Z_MINUS 			(33)

/* TOUCHPAD */
#define DS4_TOUCH_POINT_INACTIVE 0x80

struct ds4_touch_point {
	uint8_t contact;
	uint8_t x_lo;
	uint8_t x_hi : 4, y_lo : 4;
	uint8_t y_hi;
} __packed;

struct ds4_touch_report {
	uint8_t timestamp;
	struct ds4_touch_point points[2];
} __packed;

struct ds4_input_report {
	uint8_t report_id; /* 0x11 */
	uint8_t reserved[2];

	uint8_t x, y;
	uint8_t rx, ry;
	uint8_t buttons[3];
	uint8_t z, rz;

	/* Motion sensors */
	uint16_t sensor_timestamp;
	uint8_t sensor_temperature;
	uint16_t gyro[3]; /* x, y, z */
	uint16_t accel[3]; /* x, y, z */
	uint8_t reserved3[5];

	uint8_t status[2];
	uint8_t reserved4;

	/* touch pad */
	uint8_t num_touch;
	struct ds4_touch_report touch[4];

	uint8_t reserved5[2];
	uint32_t crc32;
} __packed;

struct ds4_output_report {
	uint8_t id; /* 0x11 */
	uint8_t hw_ctrl;
	uint8_t audio_ctrl;

	uint8_t valid_flag0;
	uint8_t valid_flag1;

	uint8_t reserved;

	uint8_t motor_right;
	uint8_t motor_left;

	uint8_t lightbar_red;
	uint8_t lightbar_green;
	uint8_t lightbar_blue;
	uint8_t lightbar_blink_on;
	uint8_t lightbar_blink_off;

	uint8_t reserved2[61];
	uint32_t crc32;
} __packed;
