// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#include "driver/i2c_master.h"
#include "esp_err.h"

#pragma once

#define MPU6XXX_VERSION "v1.2.0"

/*
 * Struct: mpu_cfg_t
 * Members:
 *     port: I2C port for the mpu device
 *     sda: GPIO pin for the serial data line
 *     scl: GPIO pin for the serial clock line
 */
typedef struct {
	uint8_t port;
	uint8_t sda;
	uint8_t scl;
} mpu_cfg_t;

/*
 * Struct: mpu_idev_t
 * Members:
 *     bus: I2C master bus handle
 */
typedef struct {
	i2c_master_bus_handle_t bus;	
} mpu_idev_t;

// mpu main device handle
typedef mpu_idev_t *mpu_handle_t;

// mpu per device handle
typedef i2c_master_dev_handle_t mpu_dev_t;

/*
 * Struct: mpu_raw_t
 * Members:
 *     x: x axis data for the given sensor
 *     y: y axis data for the given sensor
 *     z: z axis data for the given sensor
 */
typedef struct {
	float x;
	float y;
	float z;
} mpu_raw_t;

typedef mpu_raw_t mpu_accel_data_t;
typedef mpu_raw_t mpu_gyro_data_t;

/*
 * Struct: mpu_data_t (deprecated)
 * Members:
 *     accel: accelerometer data from the imu
 *     gyro: gyroscope data from the imu
 *     temp: thermal data from the imu
 */
typedef struct {
	mpu_raw_t accel;
	mpu_raw_t gyro;
	float temp;
} mpu_data_t;

/*
 * function: mpu6xxx_reg_write
 * type: esp_err_t (int)
 * input:
 *     dev: mpu per device handle
 *     cmd: cmd to send to the imu
 *     data: value for the cmd
 * return: ESP_OK on success, error on failure
 * description: write to the register of the mpu6xxx series
 */
esp_err_t mpu6xxx_reg_write(mpu_dev_t dev, uint8_t cmd, uint8_t data);

/*
 * function: mpu6xxx_reg_read
 * type: esp_err_t (int)
 * input:
 *     dev: mpu per device handle
 *     cmd: cmd to send to the imu
 *     recv: pointer to the buffer for storing the read value.
 * return: ESP_OK on success, error on failure
 * description: read the register of the mpu6xxx series
 */
esp_err_t mpu6xxx_reg_read(mpu_dev_t dev, uint8_t cmd, uint8_t *recv);

/*
 * function: mpu6xxx_get_accel_data
 * type: void
 * input:
 *     dev: mpu per device handle
 *     aval: The G value for the accelerometer, Default is 8G
 *     data: Pointer to the struct to store the received data
 * description: get accel the data from the mpu device
 */
void mpu6xxx_get_accel_data(mpu_dev_t dev, float aval, mpu_accel_data_t *data);

/*
 * function: mpu6xxx_get_gyro_data
 * type: void
 * input:
 *     dev: mpu per device handle
 *     dps: The dps value to be used, Default is 2000
 *     data: Pointer to the struct to store the received data
 * description: get gyro the data from the mpu device
 */
void mpu6xxx_get_gyro_data(mpu_dev_t dev, float dps, mpu_gyro_data_t *data);

/*
 * function: mpu6xxx_get_temp_data
 * type: void
 * input:
 *     dev: mpu per device handle
 *     temp: Pointer to the float for storing the received temp data
 * description: get temp the data from the mpu device
 */
void mpu6xxx_get_temp_data(mpu_dev_t dev, float *temp);

/*
 * function: mpu6xxx_add_device
 * type: esp_err_t (int)
 * input:
 *     addr: address for the i2c bus to connect to the mpu
 *     handle: handle to the master i2c bus initialised by the init.
 *     dev: pointer to the mpu per device handle
 * return: ESP_OK on success, error on failure
 * description: Add device to the initialised bus
 */
esp_err_t mpu6xxx_add_device(uint8_t addr, mpu_handle_t handle, mpu_dev_t *dev);

/*
 * function: mpu6xxx_remove_device
 * type: esp_err_t (int)
 * input:
 *     dev: mpu per device handle
 * return: ESP_OK on success, error on failure
 * description: remove device from the initialised bus
 */
esp_err_t mpu6xxx_remove_device(mpu_dev_t dev);

/*
 * function: mpu6xxx_init
 * type: esp_err_t (int)
 * input:
 *     mpu_cfg_t: ptr to mpu configuration
 *     handle: ptr to mpu bus handle
 * return: ESP_OK on success, error on failure
 * description: initializes the mpu device bus.
 */
esp_err_t mpu6xxx_init(const mpu_cfg_t *cfg, mpu_handle_t *handle);

/*
 * function: mpu6xxx_exit
 * type: esp_err_t (int)
 * input:
 *     handle: mpu bus handle
 * return: ESP_OK on success, error on failure
 * description: deinitializes the mpu device bus.
 */
esp_err_t mpu6xxx_exit(mpu_handle_t handle);
