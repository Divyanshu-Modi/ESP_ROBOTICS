// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>
/*
 * This driver is for the lsa module manufacturer CYTRON
 * Model: LSA-08
 */

#include "driver/uart.h"
#include "esp_err.h"

#pragma once

// Driver Version
#define LSA_VERSION 	"1.2.2"

// Uart configuration
#define LSA_BUFFER 		(1024)
#define LSA_BAUD_RATE 	115200

// LSA-08 COMMAND
#define LSA_BACKLIGHT 		0x42
#define LSA_CALIBRATE 		0x43
#define LSA_DATA_SET 		0x44
#define LSA_JUNC_WIDTH 		0x4A
#define LSA_LINE_MODE 		0x4C
#define LSA_LINE_POS 		0x4F
#define LSA_JUNC_INFO 		0x50
#define LSA_LCD_CONTRAST 	0x53
#define LSA_LINE_THRES 		0x54
#define LSA_READ_SUCCESS 	0x9A

// NOTE: Refer Datasheet while configuration
/*
 * struct lsa_cfg_t:
 *		rx: RX gpio
 *		tx: TX gpio
 *		en: EN gpio
 *		baud: Baud rate
 *		port: UART port
 */
typedef struct {
	uint8_t rx;
	uint8_t tx;
	int8_t en;
	uint16_t baud;
	uart_port_t port;
} lsa_cfg_t;

/*
 * struct lsa_dev_t:
 *    cfg: lsa configuration struct
 *    is_drv_installed: Variable to inidicate a succesfull driver installation
 */
typedef struct {
	uint8_t en;
	bool disabled_en;
	uart_port_t port;
} lsa_dev_t;

/*
 * Handle for lsa driver
 */
typedef lsa_dev_t *lsa_handle_t;

/*
 * fucntion: lsa_write
 * type: esp_err_t (int)
 * input:
 *     handle: handle to lsa data
 *     addr: Address of the lsa module uart bus
 *     cmd: The command to be sent to the module
 *     data: The data bit to be sent to the module
 * return: ESP_OK on success
 */
esp_err_t lsa_write(lsa_handle_t lsa, uint8_t addr, uint8_t cmd, uint8_t data);

/*
 * fucntion: lsa_read_locked
 * type: void
 * desc: Pulls UEN gpio low inorder to get data from lsa module
 * input:
 *     handle: handle to lsa data
 *     lsa_buf: ptr to variable to store data in
 * return: ESP_OK on success
 */
void lsa_read_byte(lsa_handle_t lsa, uint8_t *lbuf);

/*
 * fucntion: lsa_read
 * type: void
 * input:
 *     handle: handle to lsa data
 *     lsa_buf: ptr to the buf to read data in
 *     size: number of bytes to read
 */
void lsa_read_bytes(lsa_handle_t lsa, uint8_t *lbuf, size_t size);

/*
 * fucntion: lsa_init
 * type: esp_err_t (int)
 * input:
 *     lsa_cfg: ptr to lsa configuration data
 *     handle: ptr to handle of lsa
 * return: ESP_OK on success
 */
esp_err_t lsa_init(const lsa_cfg_t *lsa_cfg, lsa_handle_t *handle);

/*
 * fucntion: lsa_deinit
 * type: esp_err_t (int)
 * input:
 *     handle: handle to lsa data
 * return: ESP_OK on success
 */
esp_err_t lsa_deinit(lsa_handle_t lsa);
