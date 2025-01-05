// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#include "driver/uart.h"
#include "esp_err.h"

#pragma once

#define MDDS_DRV_VERSION "1.2.1"

/*
 * Default baud rate for mdds driver
 */
#define MDDS_UART_BAUD 115200

/*
 * Type: mdds_drv_t
 * Members:
 *   MOT_DRV_CW: Clockwise rotation
 *   MOT_DRV_CCW: Counter-clockwise rotation
 *   MOT_DRV_MAX: Max
 */
typedef enum {
  DRV_CW = 0,
  DRV_CCW = 1,
  DRV_MAX,
} mdds_drv_t;

/*
 * Type: mdds_chan_t
 * Members:
 *   MOT_CHAN_LEFT: Left channel
 *   MOT_CHAN_RIGHT: Right channel
 *   MOT_CHAN_MAX: Max
 */
typedef enum {
  CHAN_LEFT = 0,
  CHAN_RIGHT = 1,
  CHAN_MAX,
} mdds_chan_t;

// NOTE: Refer Datasheet while configuring
/*
 * struct mdds_cfg_t:
 *    port: UART port
 *    tx: TX gpio
 *    baud: Baud rate
 */
typedef struct {
  uart_port_t port;
  uint8_t tx_pin;
  uint16_t baud;
} mdds_cfg_t;

/*
 * struct mdds_dev_t:
 *    cfg: Motor configuration struct
 *    is_drv_installed: Variable to inidicate a succesfull driver installation
 */
typedef struct {
  uart_port_t port;
  bool is_drv_installed;
} mdds_dev_t;

/*
 * Handle for mdds driver
 */
typedef mdds_dev_t *mdds_handle_t;

/*
 * Function: mdds_send_sig
 * Type: esp_err_t (int)
 * Input:
 *     pmdds: Motor handle
 *     chan: Motor channel
 *     dir: Motor direction
 *     speed: Motor speed
 * Return: ESP_OK on success
 */
esp_err_t mdds_write(mdds_handle_t pmdds, mdds_chan_t chan, mdds_drv_t dir, uint8_t speed);

/*
 * Function: mdds_init
 * Type: esp_err_t (int)
 * Input:
 *     mdds_cfg: Motor configuration
 *     handle: Motor handle
 * Return: ESP_OK on success
 */
esp_err_t mdds_init(const mdds_cfg_t *mdds_cfg, mdds_handle_t *handle);

/*
 * Function: mdds_deinit
 * Type: esp_err_t (int)
 * Input:
 *     pmdds: Motor handle
 * Return: ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t mdds_deinit(mdds_handle_t pmdds);
