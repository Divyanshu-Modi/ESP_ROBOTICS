// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "ps_spp.h"

#define pr_fmt "SPP"

static inline void spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t* param)
{
	if (event == ESP_SPP_INIT_EVT)
		esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
}

void spp_init(void)
{
	esp_err_t ret = ESP_OK;
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_bluedroid_config_t bluedroid_cfg = { .ssp_en = true };
	esp_spp_cfg_t ssp_cfg = { .mode = ESP_SPP_MODE_CB, .enable_l2cap_ertm = true, };

	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		pr_err(pr_fmt, "controller init failed: %d", ret);
		return;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
	if (ret) {
		pr_err(pr_fmt, "failed to enable controller: %d", ret);
		goto init_err;
	}

	ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
	if (ret) {
		pr_err(pr_fmt, "bluedroid init failed: %d", ret);
		goto en_err;
	}

	ret = esp_bluedroid_enable();
	if (ret) {
		pr_err(pr_fmt, "failed to enable bluedroid: %d", ret);
		goto bld_init_err;
	}

	ret = esp_spp_register_callback(spp_cb);
	if (ret) {
		pr_err(pr_fmt, "spp register failed: %d", ret);
		goto bld_en_err;
	}

	ret = esp_spp_enhanced_init(&ssp_cfg);
	if (ret) {
		pr_err(pr_fmt, "spp init failed: %d", ret);
		goto bld_en_err;
	}

	return;

bld_en_err:
	esp_bluedroid_disable();
bld_init_err:
	esp_bluedroid_deinit();
en_err:
	esp_bt_controller_disable();
init_err:
	esp_bt_controller_deinit();
	return;
}

void spp_exit(void)
{
	esp_spp_deinit();
	esp_bluedroid_disable();
	esp_bluedroid_deinit();
	esp_bt_controller_disable();
	esp_bt_controller_deinit();
}
