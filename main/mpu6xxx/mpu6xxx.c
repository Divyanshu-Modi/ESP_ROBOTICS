// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#define pr_fmt "MPU6xxx"

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "mpu6xxx.h"
#include "mpu6886_reg.h"
#include "pr_log.h"

#define KHZ(x) 			(x * 1000)
#define DEFAULT_DPS 	2000.00
#define DEFAULT_AVAL 	8.00

esp_err_t mpu6xxx_reg_write(mpu_dev_t dev, uint8_t cmd, uint8_t data)
{
	#define WR_DATA_SIZE 2
	esp_err_t ret = ESP_OK;
	uint8_t buf[WR_DATA_SIZE] = { 0 };

	buf[0] = cmd;
	buf[1] = data;

	ret = i2c_master_transmit(dev, (const uint8_t *)&buf, WR_DATA_SIZE, 100);
	if (ret)
		pr_err(pr_fmt, "write error, ret: %d", ret);

	return ret;
}

esp_err_t mpu6xxx_reg_read(mpu_dev_t dev, uint8_t cmd, uint8_t *recv)
{
	#define RD_DATA_SIZE 1
	esp_err_t ret = ESP_OK;

	ret = i2c_master_transmit_receive(dev, (const uint8_t *)&cmd, RD_DATA_SIZE, (uint8_t *)recv, RD_DATA_SIZE, 10);
	if (ret)
		pr_err(pr_fmt, "read error, ret: %d", ret);

	return ret;
}

void mpu6xxx_get_accel_data(mpu_dev_t dev, float aval, mpu_accel_data_t *data)
{
	esp_err_t ret = ESP_OK;
	float aRes = aval ? aval : DEFAULT_AVAL;
	uint8_t regval = 0;
	int16_t lbuf = 0;

	ret = mpu6xxx_reg_read(dev, MPU6886_ACCEL_XOUT_H, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf = regval << 8;

	ret = mpu6xxx_reg_read(dev, MPU6886_ACCEL_XOUT_L, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf |= regval;

	data->x = (float)(lbuf * aRes) / 32768.0;

	ret = mpu6xxx_reg_read(dev, MPU6886_ACCEL_YOUT_H, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf = regval << 8;

	ret = mpu6xxx_reg_read(dev, MPU6886_ACCEL_YOUT_L, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf |= regval;

	data->y = (float)(lbuf * aRes) / 32768.0;

	ret = mpu6xxx_reg_read(dev, MPU6886_ACCEL_ZOUT_H, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf = regval << 8;

	ret = mpu6xxx_reg_read(dev, MPU6886_ACCEL_ZOUT_L, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf |= regval;

	data->z = (float)(lbuf * aRes) / 32768.0;
}

void mpu6xxx_get_gyro_data(mpu_dev_t dev, float dps, mpu_gyro_data_t *data)
{
	esp_err_t ret = ESP_OK;
	float gRes = dps ? dps : DEFAULT_DPS;
	uint8_t regval = 0;
	int16_t lbuf = 0;

	ret = mpu6xxx_reg_read(dev, MPU6886_GYRO_XOUT_H, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf = regval << 8;

	ret = mpu6xxx_reg_read(dev, MPU6886_GYRO_XOUT_L, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf |= regval;

	data->x = (float)(lbuf * gRes) / 32768.0;

	ret = mpu6xxx_reg_read(dev, MPU6886_GYRO_YOUT_H, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf = regval << 8;

	ret = mpu6xxx_reg_read(dev, MPU6886_GYRO_YOUT_L, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf |= regval;

	data->y = (float)(lbuf * gRes) / 32768.0;

	ret = mpu6xxx_reg_read(dev, MPU6886_GYRO_ZOUT_H, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf = regval << 8;

	ret = mpu6xxx_reg_read(dev, MPU6886_GYRO_ZOUT_L, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf |= regval;

	data->z = (float)(lbuf * gRes) / 32768.0;
}

void mpu6xxx_get_temp_data(mpu_dev_t dev, float *temp)
{
	esp_err_t ret = ESP_OK;
	uint8_t regval = 0;
	int16_t lbuf = 0;

	ret = mpu6xxx_reg_read(dev, MPU6886_TEMP_OUT_H, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf = regval << 8;

	ret = mpu6xxx_reg_read(dev, MPU6886_TEMP_OUT_L, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return;
	}
	lbuf |= regval;

	*temp = (lbuf + 25) / 326.8;
}

static inline esp_err_t mpu6xxx_probe(mpu_dev_t dev)
{
	esp_err_t ret = ESP_OK;
	uint8_t regval = 0;

	if (dev == NULL) {
		pr_err(pr_fmt, "dev handle is NULL");
		return ESP_ERR_INVALID_ARG;
	}

	ret = mpu6xxx_reg_read(dev, MPU6886_CHIP_ID, &regval);
	if (ret) {
		pr_err(pr_fmt, "Error reading chip id! ret: %d", ret);
		return ret;
	}

	switch (regval) {
	case MPU6886_ID:
		pr_info(pr_fmt, "MPU6886 Detected!");
		break;
	default:
		pr_warn(pr_fmt, "Unknow chip probe abort! regval: %d", regval);
		return ESP_OK;
	}

	// Perform chip reset
	ret = mpu6xxx_reg_write(dev, MPU6886_PM_1, MPU6886_PM_RESET);
	if (ret) {
		pr_err(pr_fmt, "pm chip reset failed!, ret: %d", ret);
		return ret;
	}

	vTaskDelay(10 / portTICK_PERIOD_MS);

	// Select clocksource which ever is preferred by the chip
	ret = mpu6xxx_reg_write(dev, MPU6886_PM_1, MPU6886_PM_CLK_SEL);
	if (ret) {
		pr_err(pr_fmt, "chip clk set failed!, ret: %d", ret);
		return ret;
	}

	// Clear the internal sensors before configurations
	ret = mpu6xxx_reg_write(dev, MPU6886_USER_CTRL, MPU6886_RESET_COND);
	if (ret) {
		pr_err(pr_fmt, "chip user ctrl reset failed!, ret: %d", ret);
		return ret;
	}

	ret = mpu6xxx_reg_write(dev, MPU6886_CONFIG, 0x01);
	if (ret) {
		pr_err(pr_fmt, "chip config failed!, ret: %d", ret);
		return ret;
	}

	ret = mpu6xxx_reg_write(dev, MPU6886_SMPLRT_DIV, 0x05);
	if (ret) {
		pr_err(pr_fmt, "chip sample rate config failed!, ret: %d", ret);
		return ret;
	}

	/*
	 * Set to +-2000 dps for gyroscope
	 * ref: https://github.com/tanakamasayuki/I2C_MPU6886/blob/master/src/I2C_MPU6886.cpp#L62
	 */
	ret = mpu6xxx_reg_write(dev, MPU6886_GYRO_CONFIG, MPU6886_GYRO_FS_SEL_2000);
	if (ret) {
		pr_err(pr_fmt, "chip gyro config failed!, ret: %d", ret);
		return ret;
	}

	/*
	 * Set to +-8 G for accelerometer
	 * ref: https://github.com/tanakamasayuki/I2C_MPU6886/blob/master/src/I2C_MPU6886.cpp#L58
	 */
	ret = mpu6xxx_reg_write(dev, MPU6886_ACCEL_CONFIG, MPU6886_ACCEL_FS_SEL_8G);
	if (ret) {
		pr_err(pr_fmt, "chip accel config failed!, ret: %d", ret);
		return ret;
	}

	ret = mpu6xxx_reg_write(dev, MPU6886_ACCEL_CONFIG2, 0x00);
	if (ret) {
		pr_err(pr_fmt, "chip accel config 2 failed!, ret: %d", ret);
		return ret;
	}

	ret = mpu6xxx_reg_write(dev, MPU6886_FIFO_EN, 0x00);
	if (ret) {
		pr_err(pr_fmt, "chip fifo disabling failed!, ret: %d", ret);
		return ret;
	}

	// TODO: test FSYNC_INT_LEVEL with high and low
	ret = mpu6xxx_reg_write(dev, MPU6886_INT_PIN_CFG, MPU6886_LATCH_INT_EN);
	if (ret) {
		pr_err(pr_fmt, "chip int pin config failed!, ret: %d", ret);
		return ret;
	}

	ret = mpu6xxx_reg_write(dev, MPU6886_INT_ENABLE, 0x01);
	if (ret) {
		pr_err(pr_fmt, "chip int enabling failed!, ret: %d", ret);
		return ret;
	}

	return ESP_OK;
}

esp_err_t mpu6xxx_add_device(uint8_t addr, mpu_handle_t handle, mpu_dev_t *dev)
{
	i2c_device_config_t mpu6xx_dev_cfg = { 0 };
	esp_err_t ret = ESP_OK;

	if (handle == NULL) {
		pr_err(pr_fmt, "Invalid handle");
		return ESP_ERR_INVALID_ARG;
	}

	mpu6xx_dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	mpu6xx_dev_cfg.device_address = addr;
	mpu6xx_dev_cfg.scl_speed_hz = KHZ(400);

	ret = i2c_master_probe(handle->bus, addr, 10);
	if (ret) {
		pr_err(pr_fmt, "device addition failed! probe failure: %d", ret);
		return ret;
	}

	ret = i2c_master_bus_add_device(handle->bus, &mpu6xx_dev_cfg, dev);
	if (ret) {
		pr_err(pr_fmt, "device addition failed! ret: %d", ret);
		return ret;
	}

	return mpu6xxx_probe(*dev);
}

esp_err_t mpu6xxx_remove_device(mpu_dev_t dev)
{
	esp_err_t ret = ESP_OK;

	if (dev == NULL) {
		pr_err(pr_fmt, "Invalid handle");
		return ESP_ERR_INVALID_ARG;
	}

	// Perform chip reset
	ret = mpu6xxx_reg_write(dev, MPU6886_PM_1, MPU6886_PM_RESET);
	if (ret) {
		pr_err(pr_fmt, "chip reset failed!, ret: %d", ret);
		return ret;
	}

	vTaskDelay(10 / portTICK_PERIOD_MS);

	// Select clocksource which ever is preferred by the chip
	ret = mpu6xxx_reg_write(dev, MPU6886_PM_1, MPU6886_PM_CLK_SEL);
	if (ret) {
		pr_err(pr_fmt, "chip clk set failed!, ret: %d", ret);
		return ret;
	}

	// Clear the internal sensors before configurations
	ret = mpu6xxx_reg_write(dev, MPU6886_USER_CTRL, MPU6886_RESET_COND);
	if (ret) {
		pr_err(pr_fmt, "chip user ctrl reset failed!, ret: %d", ret);
		return ret;
	}

	ret = i2c_master_bus_rm_device(dev);
	if (ret) {
		pr_err(pr_fmt, "device removal failed! ret: %d", ret);
		return ret;
	}

	return ESP_OK;
}

esp_err_t mpu6xxx_init(const mpu_cfg_t *cfg, mpu_handle_t *handle)
{
	i2c_master_bus_config_t mpu6xxx_cfg = { 0 };
	mpu_idev_t *mpu6xx_dev = NULL;
	esp_err_t ret = ESP_OK;

	if (cfg == NULL) {
		pr_err(pr_fmt, "Invalid configuration");
		return ESP_ERR_INVALID_ARG;
	}

	mpu6xx_dev = (mpu_idev_t *)heap_caps_malloc(sizeof(mpu_idev_t), MALLOC_CAP_DEFAULT);
	if (mpu6xx_dev == NULL) {
		pr_err(pr_fmt, "failed to alloc memory!");
		return ESP_ERR_NO_MEM;
	}

	mpu6xxx_cfg.i2c_port = cfg->port;
	mpu6xxx_cfg.sda_io_num = cfg->sda;
	mpu6xxx_cfg.scl_io_num = cfg->scl;
	mpu6xxx_cfg.intr_priority = 0;
	mpu6xxx_cfg.clk_source = I2C_CLK_SRC_APB;
	mpu6xxx_cfg.flags.enable_internal_pullup = true;

	ret = i2c_new_master_bus(&mpu6xxx_cfg, &mpu6xx_dev->bus);
	if (ret) {
		pr_err(pr_fmt, "i2c bus initialisation failed! ret: %d", ret);
		heap_caps_free(mpu6xx_dev);
		return ret;
	}

	pr_info(pr_fmt, "driver version: %s", MPU6XXX_VERSION);

	*handle = mpu6xx_dev;

	return ret;
}

esp_err_t mpu6xxx_exit(mpu_handle_t handle)
{
	esp_err_t ret = ESP_OK;

	if (handle == NULL) {
		pr_err(pr_fmt, "Re check handle!");
		return ESP_ERR_INVALID_ARG;
	}

	ret = i2c_del_master_bus(handle->bus);
	if (ret)
		pr_err(pr_fmt, "Error occured during bus removal!");

	heap_caps_free(handle);

	return ret;
}
