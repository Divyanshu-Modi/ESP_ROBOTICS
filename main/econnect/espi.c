// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#define pr_fmt "ESPI"

#include "esp_heap_caps.h"
#include "gpio.h"
#include "espi.h"
#include <string.h>
#include <stdint.h>
#include "sdkconfig.h"

/*
 * Max buffer size for the espi packets sent over spi
 */
#define ESPI_BUF_SIZE 128

/*
 * Mode must be set to either 1 or 3 for DMA mode operation
 */
#define ESPI_PARENT_MODE 1
#define ESPI_CHILD_MODE 3

/*
 * Internal struct
 */
typedef struct {
	esp_err_t (*probe)(const espi_cfg_t *espi_cfg, espi_spi_t *espi);
	esp_err_t (*exit)(espi_handle_t handle);
	esp_err_t (*transact)(espi_handle_t espi, uint8_t id, uint8_t size, const void *cmd, void *recv);
} espi_ops_t;

typedef struct {
	espi_spi_t espi;
	espi_ops_t ops;
} espi_context_t;

static inline void  espi_pre_transfer_cb(spi_transaction_t *t)
{
	uint8_t gpio = (int)t->user;

	gpio_set_level(gpio, 0);
}

static inline void espi_post_transfer_cb(spi_transaction_t *t)
{
	uint8_t gpio = (int)t->user;

	gpio_set_level(gpio, 1);
}

static inline esp_err_t espi_parent_probe(const espi_cfg_t *espi_cfg, espi_spi_t *espi)
{
	uint8_t i = 0, dev = espi_cfg->num;
	esp_err_t ret = ESP_OK;
	spi_bus_config_t espi_bus = { 0 };
	spi_device_interface_config_t espi_dcfg = { 0 };

	espi_bus.mosi_io_num = espi_cfg->poci;
	espi_bus.miso_io_num = espi_cfg->pico;
	espi_bus.sclk_io_num = espi_cfg->clk;

	/*
	 * As per the esp idf documentation the spi slave driver is
	 * not capable of quad channel operation and ignores any
	 * pin configure for them so, disable the quad channel pins
	 */
	espi_bus.quadwp_io_num = -1;
	espi_bus.quadhd_io_num = -1;

	espi_bus.isr_cpu_id = ESP_INTR_CPU_AFFINITY_0;
	espi_bus.flags = SPICOMMON_BUSFLAG_MASTER
					| SPICOMMON_BUSFLAG_SCLK
					| SPICOMMON_BUSFLAG_DUAL
					| SPICOMMON_BUSFLAG_IOMUX_PINS;

#ifdef CONFIG_IDF_TARGET_ESP32S3
	/*
	 * SPI3 on ESP32 S3 have dedicated (IOMUX) pins
	 */
	if (espi_cfg->host == SPI3_HOST)
		espi_bus.flags &= ~SPICOMMON_BUSFLAG_IOMUX_PINS;
#endif

	/*
	 * As per:
	 * https://docs.espressif.com/projects/esp-idf/en/release-v5.2/esp32/api-reference/peripherals/spi_master.html#timing-considerations
	 */
	espi_dcfg.input_delay_ns = 50;
	espi_dcfg.mode = ESPI_PARENT_MODE; // Mandate for DMA mode
	// TODO: Do we need queues ?
	espi_dcfg.queue_size = 1;
	espi_dcfg.clock_speed_hz = 8000000; // 8MHz is most stable for espi operation
	espi_dcfg.flags = SPI_DEVICE_NO_DUMMY; // We operate in full duplex mode

	espi_dcfg.pre_cb = espi_pre_transfer_cb;
	espi_dcfg.post_cb = espi_post_transfer_cb;

	if (dev > ESPI_MAX_CHILDS) {
		pr_warn("Maximum number childs per device are 3! limiting to 3");
		dev = ESPI_MAX_CHILDS;
	}

	for (i = 0; i < dev; i++) {
		ret |= gpio_init(espi_cfg->sel[dev], GPIO_MODE_INPUT_OUTPUT, 1);
		if (ret) {
			pr_err("failed to set sel pin!");
			goto gpio_err;
		}
		espi->sel[i] = espi_cfg->sel[i];
	}

	ret = spi_bus_initialize(espi_cfg->host, &espi_bus, SPI_DMA_CH_AUTO);
	if (ret) {
		pr_err("bus init failed!, ret: %d", ret);
		goto gpio_err;
	}

	for (i = 0; i < dev; i++) {
		espi_dcfg.spics_io_num = espi_cfg->sel[i]; // Set proper cs pin
		ret = spi_bus_add_device(espi_cfg->host, &espi_dcfg, &espi->spi[i]);
		if (ret) {
			pr_err("failed to register device ID: %d, err: %d! ", i, ret);
			goto bus_err;
		} else
			pr_info("Device registered ID: %d", i);
	}

	return ESP_OK;

bus_err:
	for (i = 0; i < dev; i++) {
		if (espi->spi[i])
			spi_bus_remove_device(espi->spi[i]);
	}

	spi_bus_free(espi_cfg->host);
gpio_err:
	for (i = 0; i < dev; i++) {
		if (espi->sel[i])
			gpio_reset_pin(espi_cfg->sel[i]);
	}

	return ret;
}

static inline esp_err_t espi_parent_exit(espi_handle_t handle)
{
	esp_err_t ret = ESP_OK;
	uint8_t i = 0;

	for (i = 0; i < handle->num; i--) {
		ret = spi_bus_remove_device(handle->spi[i]);
		if (ret) {
			pr_err("failed to remove device with ID[%d]", i);
			return ESP_FAIL;
		}
	}

	ret = spi_bus_free(handle->host);
	if (ret) {
		pr_err("Failed to release bus resources!");
		return ESP_FAIL;
	}

	for (i = 0; i < handle->num; i--) {
		ret = gpio_reset_pin(handle->sel[i]);
		if (ret)
			return ESP_FAIL;
	}

	return ESP_OK;
}

static inline esp_err_t espi_parent_transact(espi_handle_t handle, uint8_t id, uint8_t size, const void *cmd, void *recv)
{
	esp_err_t ret = ESP_OK;
	void *tbuf = heap_caps_malloc(ESPI_BUF_SIZE, MALLOC_CAP_DEFAULT);
	spi_transaction_t *buf = (spi_transaction_t *)
		heap_caps_calloc(1, sizeof(spi_transaction_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_32BIT);

	if (buf == NULL || tbuf == NULL) {
		pr_err("memory allocation failed!");
		ret = ESP_FAIL;
		goto err;
	}

	buf->length = ESPI_BUF_SIZE * 8;
	buf->tx_buffer = cmd;
	buf->rx_buffer = (void *)tbuf;
	buf->user = (void *)(long)handle->sel[id];

	ret = spi_device_acquire_bus(handle->spi[id], portMAX_DELAY);
	if (ret)
		pr_info("Failed to acquire bus!");

	xSemaphoreTake(handle->mutex, portMAX_DELAY);

	ret = spi_device_transmit(handle->spi[id], buf);
	if (ret)
		pr_info("transaction failed! err: %d", ret);
	else if (recv != NULL)
		memcpy(recv, (void *)tbuf, size);

	xSemaphoreGive(handle->mutex);

	spi_device_release_bus(handle->spi[id]);

err:
	if (buf)
		heap_caps_free(buf);
	if (tbuf)
		heap_caps_free(tbuf);

	return ret;
}

espi_ops_t espi_parent_ops = {
	.probe = espi_parent_probe,
	.exit = espi_parent_exit,
	.transact = espi_parent_transact,
};

static inline esp_err_t espi_child_probe(const espi_cfg_t *espi_cfg, espi_spi_t *espi)
{
	esp_err_t ret = ESP_OK;
	spi_bus_config_t espi_bus = { 0 };
	spi_slave_interface_config_t espi_dcfg = { 0 };

	espi_bus.mosi_io_num = espi_cfg->poci;
	espi_bus.miso_io_num = espi_cfg->pico;
	espi_bus.sclk_io_num = espi_cfg->clk;

	/*
	 * As per the esp idf documentation the spi slave driver is
	 * not capable of quad channel operation and ignores any
	 * pin configure for them so, disable the quad channel pins
	 */
	espi_bus.quadwp_io_num = -1;
	espi_bus.quadhd_io_num = -1;

	espi_bus.isr_cpu_id = ESP_INTR_CPU_AFFINITY_0;
	espi_bus.flags = SPICOMMON_BUSFLAG_SLAVE
					| SPICOMMON_BUSFLAG_SCLK
					| SPICOMMON_BUSFLAG_DUAL
					| SPICOMMON_BUSFLAG_IOMUX_PINS;

#ifdef CONFIG_IDF_TARGET_ESP32S3
	/*
	 * SPI3 on ESP32 S3 have dedicated (IOMUX) pins
	 */
	if (espi_cfg->host == SPI3_HOST)
		espi_bus.flags &= ~SPICOMMON_BUSFLAG_IOMUX_PINS;
#endif

	espi_dcfg.mode = ESPI_CHILD_MODE;
	// TODO: Do we need queues ?
	espi_dcfg.queue_size = 1;
	espi_dcfg.spics_io_num = espi_cfg->sel[0];
	espi->sel[0] = espi_cfg->sel[0];

	ret = gpio_init(espi_cfg->sel[0], GPIO_MODE_INPUT_OUTPUT, 1);
	if (ret) {
		pr_err("failed to set sel pin!");
		goto gpio_err;
	}

	ret = spi_slave_initialize(espi_cfg->host, &espi_bus, &espi_dcfg, SPI_DMA_CH_AUTO);
	if (ret) {
		pr_err("bus init failed!, ret: %d", ret);
		goto gpio_err;
	}

	return ESP_OK;

gpio_err:
	gpio_reset_pin(espi_cfg->sel[0]);

	return ret;
}

static inline esp_err_t espi_child_exit(espi_handle_t handle)
{
	esp_err_t ret = ESP_OK;

	ret = spi_slave_free(handle->host);
	if (ret) {
		pr_err("Failed to release bus resources!");
		return ESP_FAIL;
	}

	ret = gpio_reset_pin(handle->sel[0]);
	if (ret)
		return ESP_FAIL;

	return ESP_OK;
}

static inline esp_err_t espi_child_transact(espi_handle_t handle, uint8_t id, uint8_t size, const void *cmd, void *recv)
{
	esp_err_t ret = ESP_OK;
	void *tbuf = NULL;
	spi_slave_transaction_t *buf = NULL;

	if (!gpio_get_level(handle->sel[0])) {
		memset(recv, 0, size);
		pr_debug("skipping transaction, child not selected!");
		return ret;
	}

	tbuf = heap_caps_malloc(ESPI_BUF_SIZE, MALLOC_CAP_DEFAULT);
	buf = (spi_slave_transaction_t *)heap_caps_calloc(1, sizeof(spi_transaction_t),
			MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_32BIT);

	if (buf == NULL || tbuf == NULL) {
		pr_err("memory allocation failed!");
		ret = ESP_FAIL;
		goto err;
	}

	buf->length = ESPI_BUF_SIZE * 8;
	buf->tx_buffer = cmd;
	buf->rx_buffer = tbuf;

	xSemaphoreTake(handle->mutex, portMAX_DELAY);

	ret = spi_slave_transmit(handle->host, buf, portMAX_DELAY);
	if (ret)
		pr_info("transaction failed! err: %d", ret);
	else if (recv != NULL)
		memcpy(recv, tbuf, size);

	xSemaphoreGive(handle->mutex);

err:
	if (buf)
		heap_caps_free(buf);
	if (tbuf)
		heap_caps_free(tbuf);

	return ret;
}

espi_ops_t espi_child_ops = {
	.probe = espi_child_probe,
	.exit = espi_child_exit,
	.transact = espi_child_transact,
};

esp_err_t espi_transact(espi_handle_t handle, uint8_t id, uint8_t size, const void *cmd, void *recv)
{
	esp_err_t ret = ESP_OK;
	espi_context_t *ctx = __containerof(handle, espi_context_t, espi);
	espi_ops_t ops = ctx->ops;

	ret = ops.transact(handle, id, size, cmd, recv);
	if (ret)
		pr_err("transaction failure!");

	return ret;
}

static inline bool is_espi_dev_valid(espi_dev_t espi_dev)
{
	return ((espi_dev >= ESPI_DEV_PARENT) && (espi_dev < ESPI_DEV_MAX));
}

esp_err_t espi_init(const espi_cfg_t *espi_cfg, espi_handle_t *handle)
{
	esp_err_t ret = ESP_OK;
	espi_context_t *ctx = NULL;
	espi_ops_t *ops = NULL;

	if (espi_cfg == NULL)
		return ESP_ERR_INVALID_ARG;

	if (unlikely(!is_espi_dev_valid(espi_cfg->dev))) {
		pr_err("Invalid device!");
		return ESP_ERR_INVALID_ARG;
	}

	ctx = (espi_context_t *)heap_caps_malloc(sizeof(espi_context_t), MALLOC_CAP_DEFAULT);
	if (ctx == NULL) {
		pr_err("Memory allocation failed for internal buf");
		return ESP_ERR_NO_MEM;
	}

	ops = (espi_ops_t *)heap_caps_malloc(sizeof(espi_ops_t), MALLOC_CAP_DEFAULT);
	if (ops == NULL) {
		pr_err("Memory allocation failed for internal buf");
		heap_caps_free(ctx);
		return ESP_ERR_NO_MEM;
	}

	if (espi_cfg->dev == ESPI_DEV_PARENT)
		ops = &espi_parent_ops;
	else
		ops = &espi_child_ops;

	ctx->espi.mutex = xSemaphoreCreateMutex();
	ctx->ops = *ops;
	ctx->espi.num = espi_cfg->num;
	ctx->espi.host = espi_cfg->host;

	ret = ops->probe(espi_cfg, &ctx->espi);
	if (ret) {
		pr_err("Init failed!, ret: %d", ret);
		goto init_err;
	}

	pr_info("ESPI version: %s!", ESPI_VERSION);

	*handle = &ctx->espi;

	return ESP_OK;

init_err:
	vSemaphoreDelete(ctx->espi.mutex);
	heap_caps_free(ops);
	heap_caps_free(ctx);
	return ret;
}

esp_err_t espi_deinit(espi_handle_t handle)
{
	espi_context_t *ctx = __containerof(handle, espi_context_t, espi);
	espi_ops_t ops = ctx->ops;

	ops.exit(handle);

	vSemaphoreDelete(ctx->espi.mutex);
	heap_caps_free(&ops);
	heap_caps_free(ctx);

	return ESP_OK;
}
