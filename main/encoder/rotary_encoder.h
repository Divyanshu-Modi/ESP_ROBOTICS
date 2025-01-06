// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#include "driver/pulse_cnt.h"
#include "esp_err.h"

#pragma once

#define DRV_VERSION "2.0"

/*
 * struct enc_cfg_t:
 *    h_lim: high (+ve) limit for the encoder
 *    l_lim: low (-ve) limit for the encoder
 *    chan_a: Channel A gpio for the encoder
 *    chan_b: Channel B gpio for the encoder
 */
typedef struct {
  int h_lim;
  int l_lim;
  uint8_t chan_a;
  uint8_t chan_b;
} enc_cfg_t;

/*
 * struct mot_dev_t:
 *    h_lim: high (+ve) limit for the encoder
 *    l_lim: low (-ve) limit for the encoder
 *    unit: Encoder pulse counter unit ptr
 *    chan: Encoder pulse counter channel ptr
 */
typedef struct {
  int h_lim, l_lim;
  pcnt_unit_handle_t unit;
  pcnt_channel_handle_t chan[2];
} enc_dev_t;

/*
 * Handle for pulse counter
 */
typedef enc_dev_t *enc_handle_t;

/*
 * Function: rotary_enc_init
 * Type: esp_err_t (int)
 * Input:
 *     enc_cfg: ptr to encoder configuration
 *     handle: ptr to encoder handle
 * Return: ESP_OK on success
 */
esp_err_t rotary_enc_init(const enc_cfg_t *enc_cfg, enc_handle_t *handle);

/*
 * Function: rotary_enc_deinit
 * Type: esp_err_t (int)
 * Input:
 *     handle: encoder handle
 * Return: ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t rotary_enc_deinit(enc_handle_t handle);

/*
 * Function: rotary_enc_get_count
 * Type: esp_err_t (int)
 * Input:
 *     handle: encoder handle
 *     val: ptr to the value returned by the pulse counter unit
 * Return: ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t rotary_enc_get_count(enc_handle_t handle, int *val);

/*
 * Function: rotary_enc_clear_count
 * Type: esp_err_t (int)
 * Input:
 *     handle: encoder handle
 * Return: ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t rotary_enc_clear_count(enc_handle_t handle);
