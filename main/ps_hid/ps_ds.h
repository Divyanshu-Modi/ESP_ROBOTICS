// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#include "ps_common.h"

#pragma once

/* Feature version from DualSense Firmware Info report. */
#define DS_FEATURE_VERSION(major, minor) ((major & 0xff) << 8 | (minor & 0xff))

#define DS_OUTPUT_REPORT_BT				0x31
#define DS_OUTPUT_REPORT_BT_SIZE		78
#define DS_INPUT_REPORT_BT				0x31
#define DS_INPUT_REPORT_BT_SIZE			78

#define DS_FIRMWARE_INFO				0x20
#define DS_FIRMWARE_INFO_SIZE			64
#define DS_CALIBRATION					0x05
#define DS_CALIBRATION_SIZE				41

#define DS_OUTPUT_TAG 					0x10

#define DS_OUTPUT_VALID_FLAG0_COMPATIBLE_VIBRATION	 BIT(0)
#define DS_OUTPUT_VALID_FLAG0_HAPTICS_SELECT		 BIT(1)

#define DS_OUTPUT_VALID_FLAG1_MIC_MUTE_LED_CONTROL_ENABLE 		BIT(0)
#define DS_OUTPUT_VALID_FLAG1_POWER_SAVE_CONTROL_ENABLE			BIT(1)
#define DS_OUTPUT_VALID_FLAG1_LIGHTBAR_CONTROL_ENABLE 			BIT(2)
#define DS_OUTPUT_VALID_FLAG1_RELEASE_LEDS 						BIT(3)
#define DS_OUTPUT_VALID_FLAG1_PLAYER_INDICATOR_CONTROL_ENABLE 	BIT(4)

#define DS_OUTPUT_VALID_FLAG2_LIGHTBAR_SETUP_CONTROL_ENABLE 	BIT(1)
#define DS_OUTPUT_VALID_FLAG2_COMPATIBLE_VIBRATION2 			BIT(2)
#define DS_OUTPUT_POWER_SAVE_CONTROL_MIC_MUTE 					BIT(4)
#define DS_OUTPUT_LIGHTBAR_SETUP_LIGHT_OUT 						BIT(1)

/* DualShock4 hardware limits */
#define DS_ACC_RES_PER_G			8192
#define DS_ACC_RANGE				(4 * DS_ACC_RES_PER_G)
#define DS_GYRO_RES_PER_DEG_S		1024
#define DS_GYRO_RANGE				(2048 * DS_GYRO_RES_PER_DEG_S)
#define DS_TOUCHPAD_WIDTH			1920
#define DS_TOUCHPAD_HEIGHT			1080

/* SENSOR INDEX */
/* GYRO INDEX */
#define GYRO_PITCH_BIAS 		(1)
#define GYRO_YAW_BIAS 			(3)
#define GYRO_ROLL_BIAS 			(5)
#define GYRO_PITCH_PLUS 		(7)
#define GYRO_PITCH_MINUS 		(9)
#define GYRO_YAW_PLUS 			(11)
#define GYRO_YAW_MINUS 			(13)
#define GYRO_ROLL_PLUS			(15)
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

struct ds_input_report {
	uint8_t x, y;
	uint8_t rx, ry;
	uint8_t z, rz;

	uint8_t seq_number;
	uint8_t buttons[4];
	uint8_t reserved[4];

	/* Motion sensors */
	uint16_t gyro[3]; /* x, y, z */
	uint16_t accel[3]; /* x, y, z */
	uint32_t sensor_timestamp;
	uint8_t reserved2;

	uint8_t touch_reserve[8];

	uint8_t reserved3[12];
	uint8_t status;
	uint8_t reserved4[10];
} __packed;

struct ds_output_report {
	uint8_t id; /* 0x11 */
	uint8_t seq_tag;
	uint8_t tag;

	uint8_t valid_flag0;
	uint8_t valid_flag1;

	/* For DualShock 4 compatibility mode. */
	uint8_t motor_right;
	uint8_t motor_left;

	/* Audio controls */
	uint8_t reserved[4];
	uint8_t mute_button_led;

	uint8_t power_save_control;
	uint8_t reserved2[28];

	/* LEDs and lightbar */
	uint8_t valid_flag2;
	uint8_t reserved3[2];
	uint8_t lightbar_setup;
	uint8_t led_brightness;
	uint8_t player_leds;
	uint8_t lightbar_red;
	uint8_t lightbar_green;
	uint8_t lightbar_blue;

	uint8_t reserved4[24];
	uint32_t crc32;
} __packed;
