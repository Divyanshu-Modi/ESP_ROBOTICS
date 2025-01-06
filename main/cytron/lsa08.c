// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>
/*
 * This driver is for the lsa module manufacturer CYTRON
 * Model: LSA-08
 */

#define pr_fmt "LSA"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "gpio.h"
#include "lsa08.h"
#include <string.h>

esp_err_t lsa_write(lsa_handle_t lsa, uint8_t addr, uint8_t cmd, uint8_t data)
{
	esp_err_t ret = ESP_OK;
	uint8_t __cmd[4] = { 0 };
	uint8_t buf[2] = { 0 };
	uint8_t retries = 0;

	if (!(lsa && uart_is_driver_installed(lsa->port)))
		return ESP_ERR_INVALID_ARG;

	__cmd[0] = addr;
	__cmd[1] = cmd;
	__cmd[2] = data;
	__cmd[3] = addr | cmd | data;

	while (retries < 3) {
		ret = uart_write_bytes(lsa->port, (const uint8_t *)&__cmd, 4);
		if (ret < ESP_OK) {
			pr_err("lsa_write: failed to write %d", cmd);
			return ret;
		}

		lsa_read_bytes(lsa, buf, sizeof(buf));
		if ((buf[0] | buf[1]) == LSA_READ_SUCCESS)
			break;

		retries++;
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

	return ret;
}

void lsa_read_bytes(lsa_handle_t lsa, uint8_t *lbuf, size_t size)
{
	uint8_t *buf = NULL;
	esp_err_t ret = ESP_OK;

	if (!(lsa && uart_is_driver_installed(lsa->port)))
		return;

	buf = (uint8_t *)heap_caps_calloc(size, sizeof(uint8_t), MALLOC_CAP_DEFAULT);
	if (buf == NULL) {
		pr_err("memory allocation failed!");
		lbuf = NULL;
		return;
	}

	ret = uart_read_bytes(lsa->port, buf, size, 20 / portTICK_PERIOD_MS);
	if (ret > 0)
		memcpy(lbuf, buf, size);
	else if (ret < 0)
		pr_err("read failed!");

	uart_flush(lsa->port);
	heap_caps_free(buf);

	return;
}

void lsa_read_byte(lsa_handle_t lsa, uint8_t *lbuf)
{
	esp_err_t ret = ESP_OK;

	if (lsa->disabled_en) {
		lsa_read_bytes(lsa, lbuf, 1);
		if (lbuf == NULL)
			pr_err("read failed!");

		return;
	}

	ret = gpio_set_level(lsa->en, 0);
	if (ret) {
		pr_err("Enable gpio state change failed!");
		return;
	}

	lsa_read_bytes(lsa, lbuf, 1);
	if (lbuf == NULL)
		pr_err("read failed!");

	ret = gpio_set_level(lsa->en, 1);
	if (ret)
		pr_err("Enable gpio state change failed!");

	return;
}

esp_err_t lsa_init(const lsa_cfg_t *lsa_cfg, lsa_handle_t *handle)
{
	bool probe = false;
	esp_err_t ret = ESP_OK;
	lsa_dev_t *lsa_dev = NULL;
	uart_config_t lsa_ctrl = { 0 };
	bool disabled_en = true;

	if (lsa_cfg == NULL) {
		pr_err("Configuration empty");
		return ESP_ERR_INVALID_ARG;
	}

	lsa_ctrl.parity = UART_PARITY_DISABLE;
	lsa_ctrl.data_bits = UART_DATA_8_BITS;
	lsa_ctrl.stop_bits = UART_STOP_BITS_1;
	lsa_ctrl.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	// Set the specified baud if not present then fall back to default
	lsa_ctrl.baud_rate = lsa_cfg->baud ? lsa_cfg->baud : LSA_BAUD_RATE;

	if (lsa_cfg->en > 0) {
		ret = gpio_init(lsa_cfg->en, GPIO_MODE_OUTPUT, 1);
		if (ret) {
			pr_err("en_gpio init failed");
			return ESP_FAIL;
		}
		disabled_en = false;
	}

	ret = uart_driver_install(lsa_cfg->port, LSA_BUFFER, 0, 0, NULL, 0);
	if (ret) {
		pr_err("lsa uart driver install failed!");
		goto err;
	}

	ret = uart_param_config(lsa_cfg->port, &lsa_ctrl);
	if (ret) {
		pr_err("lsa uart setup failed!");
		goto err1;
	}

	ret = uart_set_pin(lsa_cfg->port, lsa_cfg->tx, lsa_cfg->rx,
						UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (ret) {
		pr_err("lsa uart gpio setup failed!");
		goto err1;
	}

	probe = uart_is_driver_installed(lsa_cfg->port);
	if (!probe)
		goto err1;

	lsa_dev = (lsa_dev_t *)
		heap_caps_malloc(sizeof(lsa_dev_t), MALLOC_CAP_DEFAULT);
	if (lsa_dev == NULL) {
		pr_err("Memory allocation failed!");
		ret = ESP_ERR_NO_MEM;
		goto err1;
	}

	pr_info("Probe succeeded: %s!", LSA_VERSION);
	lsa_dev->port = lsa_cfg->port;
	lsa_dev->en = lsa_cfg->en;
	lsa_dev->disabled_en = disabled_en;
	*handle = lsa_dev;
	return ret;

err1:
	uart_driver_delete(lsa_cfg->port);
err:
	gpio_reset_pin(lsa_cfg->en);
	return ret;
}

esp_err_t lsa_deinit(lsa_handle_t lsa)
{
	if (!(lsa && uart_is_driver_installed(lsa->port)))
		return ESP_ERR_INVALID_ARG;

	uart_driver_delete(lsa->port);
	gpio_reset_pin(lsa->en);
	heap_caps_free(lsa);

	return ESP_OK;
}

