// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#include "driver/gpio.h"
#include "esp_err.h"
#include "pr_log.h"

#pragma once

static inline esp_err_t gpio_init(uint8_t gpio_num, gpio_mode_t mode, uint8_t level)
{
	esp_err_t ret = ESP_OK;
	gpio_config_t io_conf = { 0 };

	ret = gpio_reset_pin(gpio_num);
	if (ret) {
		pr_err("%d gpio reset failed, ret: %d", gpio_num, ret);
		return ret;
	}

	io_conf.pin_bit_mask = BIT64(gpio_num);
	io_conf.mode = mode;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.intr_type = GPIO_INTR_DISABLE;

	ret = gpio_config(&io_conf);
	if (ret) {
		pr_err("%d gpio config failed, ret: %d", gpio_num, ret);
		return ret;
	}

	// If not set for output don't mess with it's level
	if (mode & (GPIO_MODE_DISABLE | GPIO_MODE_INPUT))
		return ret;

	ret = gpio_set_level(gpio_num, level);
	if (ret) {
		pr_err("%d gpio %s set failed, reseting gpio", gpio_num, level ? "HIGH" : "LOW");
		gpio_reset_pin(gpio_num);
		return ret;
	}

	return ret;
}
