// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#define pr_fmt "mdds"

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mdds.h"
#include "pr_log.h"

esp_err_t mdds_write(mdds_handle_t pmot, mdds_chan_t chan, mdds_drv_t dir, uint8_t speed)
{
	esp_err_t ret = ESP_OK;
	uint16_t cmd = 0;

	if (!(pmot && pmot->is_drv_installed))
		return ESP_ERR_INVALID_ARG;

	if (!(chan >= MOT_CHAN_LEFT && chan < MOT_CHAN_MAX))
		return ESP_FAIL;

	if (!(dir >= MOT_DRV_CW && dir < MOT_DRV_MAX))
		return ESP_FAIL;

	speed &= 0x40; // Speed above 63 isn't allowed

	cmd |= (chan ? 128 : 0) | (dir ? 64 : 0) | speed;

	ret = uart_write_bytes(pmot->port, (const uint8_t*)&cmd, 1);
	if (ret < 0)
		pr_err("%d write failed", cmd);

	return ret;
}

esp_err_t mdds_init(const mdds_cfg_t *mdds_cfg, mdds_handle_t *handle)
{
	bool probe = false;
	esp_err_t ret = ESP_OK;
	uart_config_t mdds_ctrl = { 0 };
	mdds_dev_t *mdds_dev = NULL;

	if (mdds_cfg == NULL) {
		pr_err("Invalid parameters");
		return ESP_ERR_INVALID_ARG;
	}

	// If baud rate is specified use that or fall back to default
	mdds_ctrl.baud_rate = mdds_cfg->baud ? mdds_cfg->baud : MDDS_UART_BAUD;
	mdds_ctrl.data_bits = UART_DATA_8_BITS;
	mdds_ctrl.parity = UART_PARITY_DISABLE;
	mdds_ctrl.stop_bits = UART_STOP_BITS_1;
	mdds_ctrl.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

	ret = uart_driver_install(mdds_cfg->port, SOC_UART_FIFO_LEN + 1, 0, 0, NULL, 0);
	if (ret) {
		pr_err("driver install failed!");
		goto drv_err;
	}

	ret = uart_param_config(mdds_cfg->port, &mdds_ctrl);
	if (ret) {
		pr_err("param config failed!");
		goto param_err;
	}

	ret = uart_set_pin(mdds_cfg->port, mdds_cfg->tx_pin,
		UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (ret) {
		pr_err("failed to set GPIO!");
		goto param_err;
	}

	probe = uart_is_driver_installed(mdds_cfg->port);
	if (!probe)
		goto param_err;

	mdds_dev = (mdds_dev_t *)
		heap_caps_malloc(sizeof(mdds_dev_t), MALLOC_CAP_DEFAULT);
	if (mdds_dev == NULL) {
		pr_err("Memory allocation failed!");
		ret = ESP_ERR_NO_MEM;
		goto param_err;
	}

	pr_info("Probe succeeded: %s!", MDDS_DRV_VERSION);
	mdds_dev->is_drv_installed = true;
	mdds_dev->port = mdds_cfg->port;
	*handle = mdds_dev;
	return ret;

param_err:
	uart_driver_delete(mdds_cfg->port);
drv_err:
	return ret;
}

esp_err_t mdds_deinit(mdds_handle_t pmot)
{
	if (!(pmot && pmot->is_drv_installed))
		return ESP_FAIL;

	uart_driver_delete(pmot->port);
	heap_caps_free(pmot);

	return ESP_OK;
}
