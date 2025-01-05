// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#include "esp_log.h"

#pragma once

#ifndef pr_fmt
#define pr_fmt "log"
#endif

#define pr_info(fmt, args...) ESP_LOGI(pr_fmt, fmt, ##args)
#define pr_err(fmt, args...) ESP_LOGE(pr_fmt, fmt, ##args)
#define pr_warn(fmt, args...) ESP_LOGW(pr_fmt, fmt, ##args)
#define pr_debug(fmt, args...) ESP_LOGD(pr_fmt, fmt, ##args)
