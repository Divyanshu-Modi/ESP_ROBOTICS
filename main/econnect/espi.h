// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#pragma once

#define ESPI_VERSION "2.0"

#define ESPI_MAX_CHILDS 3

/*
 * Enum: espi_dev_t
 * Values:
 *		ESPI_DEV_PARENT,
 *		ESPI_DEV_CHILD,
 */
typedef enum {
	ESPI_DEV_PARENT = 0,
	ESPI_DEV_CHILD  = 1,
	ESPI_DEV_MAX,
} espi_dev_t;

/*
 * Struct: espi_cfg_t
 * Members:
 *     poci: GPIO pin for the Parent Out, Child In (POCI) signal
 *     pico: GPIO pin for the Parent In, Child Out (PICO) signal
 *     clk: GPIO pin for the clock signal
 *     sel: Array of CS pins only applicable for parent, only sel[0] be used for child device
 *     num: Number of child device to be made available on this parent
 *     dev: SPI host type
 *     host: SPI HOST for the spi device
 */
typedef struct {
	uint8_t poci;
	uint8_t pico;
	uint8_t clk;
	uint8_t sel[ESPI_MAX_CHILDS];
	uint8_t num;
	espi_dev_t dev;
	spi_host_device_t host;
} espi_cfg_t;

/*
 * Struct: espi_spi_t
 * Members:
 *     cfg: Configuration for ESPI GPIO pins (poci, pico, clk, sel)
 *     spi: SPI device handle
 *     mutex: mutex lock inorder to maintain thread safety
 */
typedef struct {
	uint8_t num;
	uint8_t sel[ESPI_MAX_CHILDS];
	spi_host_device_t host;
	SemaphoreHandle_t mutex;
	spi_device_handle_t spi[ESPI_MAX_CHILDS];
} espi_spi_t;

// ESPI handle
typedef espi_spi_t *espi_handle_t;

/*
 * function: espi_init
 * type: esp_err_t (int)
 * input:
 *     espi_cfg: ptr to espi_cfg
 *     handle: ptr to ESPI handle
 * return: ESP_OK on success, ESP_FAIL on failure
 * description: Initializes the ESPI SPI communication.
 */
esp_err_t espi_init(const espi_cfg_t *espi_cfg, espi_handle_t *handle);

/*
 * function: espi_deinit
 * type: esp_err_t (int)
 * input:
 *     espi: ESPI handle
 * return: ESP_OK on success, ESP_FAIL on failure
 * description: Deinitializes the ESPI SPI communication.
 */
esp_err_t espi_deinit(espi_handle_t espi);

/*
 * function: espi_transact
 * type: esp_err_t (int)
 * input:
 *     espi: ESPI handle
 *     size: Number of bytes to copy
 *     cmd: Pointer to the command data to be transmitted
 *     recv: Pointer to the received data buffer
 * return: ESP_OK on success, ESP_FAIL on failure
 * description: transact the data over spi bus.
 */
esp_err_t espi_transact(espi_handle_t espi, uint8_t id, uint8_t size, const void *cmd, void *recv);
