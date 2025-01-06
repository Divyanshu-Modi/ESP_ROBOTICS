// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#define pr_fmt "PS_HID"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "esp_spp_api.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "string.h"

#include "ps.h"

typedef struct {
	ps_ops_t *ops;
	TaskHandle_t worker;
	QueueHandle_t work;
	esp_hidh_connection_state_t status;
} ps_hid_ctx_t;

ps_hid_ctx_t *ps_hid_dev = NULL;

void __ps_hid_cb(esp_hidh_cb_event_t event, esp_hidh_cb_param_t *param)
{
	static const char *ps_desc[] = {
		[DEV_ID_SONY_PS4] = "DualShock 4",
		[DEV_ID_SONY_PS4_V2] = "DualShock 4 V2",
		[DEV_ID_SONY_PS5] = "DualSense",
		[DEV_ID_SONY_PS5_V2] = "DualSense V2",
	};

	switch (event) {
	case ESP_HIDH_OPEN_EVT:
		struct hidh_open_evt_param odev = param->open;

		/*
		 * Device was disconnected and in device list already.
		 * We call firmware info, calib and led_set to restore controller state
		 *
		 * NOTE: Once mac is added to hidh add dev won't be called again
		 */
		if (ps_hid_dev->status == ESP_HIDH_CONN_STATE_DISCONNECTED) {
			ps_hid_dev->status = odev.conn_status;

			pr_info("reconnected! mac: %x:%x:%x:%x:%x:%x",
						odev.bd_addr[0], odev.bd_addr[1],
						odev.bd_addr[2], odev.bd_addr[3],
						odev.bd_addr[4], odev.bd_addr[5]);

			if (ps_hid_dev->ops->firmware)
				ps_hid_dev->ops->firmware(odev.bd_addr);

			if (ps_hid_dev->ops->calibration)
				ps_hid_dev->ops->calibration(odev.bd_addr);

			ps_hid_set_led(0, 0, 128);
		}
		break;
	case ESP_HIDH_CLOSE_EVT:
		ps_hid_dev->status = param->close.conn_status;
		break;
	case ESP_HIDH_ADD_DEV_EVT:
		struct hidh_add_dev_evt_param dev = param->add_dev;
		pr_info("connected! mac: %x:%x:%x:%x:%x:%x",
					dev.bd_addr[0], dev.bd_addr[1],
					dev.bd_addr[2], dev.bd_addr[3],
					dev.bd_addr[4], dev.bd_addr[5]);

		ps_hid_dev->status = ESP_HIDH_CONN_STATE_CONNECTED;
		break;
	case ESP_HIDH_GET_DSCP_EVT:
		struct hidh_get_dscp_evt_param desc = param->dscp;

		if (desc.vendor_id != VENDOR_SONY) {
			pr_info("Unknown vendor Unsupported!");
			break;
		}

		pr_info("vendor: Sony, product: %s", ps_desc[desc.product_id]);

		switch (desc.product_id) {
		case DEV_ID_SONY_PS4:
		case DEV_ID_SONY_PS4_V2:
			ps_hid_dev->ops = &ds4_ops;
			break;
		case DEV_ID_SONY_PS5:
		case DEV_ID_SONY_PS5_V2:
			ps_hid_dev->ops = &ds_ops;
			break;
		default:
			break;
		}
		break;
	case ESP_HIDH_GET_RPT_EVT:
		if (ps_hid_dev->ops->get_report)
			ps_hid_dev->ops->get_report(&param->get_rpt);

		break;
	case ESP_HIDH_DATA_IND_EVT:
		if (ps_hid_dev->ops->input_handler)
			ps_hid_dev->ops->input_handler(&param->data_ind);

		break;
	default:
		break;
	}
}

void ps_hid_set_rumble(uint8_t r, uint8_t l)
{
	ps_hid_output_t rpt = { 0 };

	rpt.update_rumble = true;
	rpt.motor_right = r;
	rpt.motor_left = l;

	xQueueSend(ps_hid_dev->work, &rpt, 0);
}

void ps_hid_set_led(uint8_t r, uint8_t g, uint8_t b)
{
	ps_hid_output_t rpt = { 0 };

	rpt.update_lightbar = true;
	rpt.lightbar_red = r;
	rpt.lightbar_green = g;
	rpt.lightbar_blue = b;

	xQueueSend(ps_hid_dev->work, &rpt, 0);
}

void ps_hid_set_led_blink(uint8_t on_delay, uint8_t off_delay)
{
	ps_hid_output_t rpt = { 0 };

	rpt.update_lightbar_blink = true;
	rpt.lightbar_blink_on = on_delay;
	rpt.lightbar_blink_off = off_delay;

	xQueueSend(ps_hid_dev->work, &rpt, 0);
}

bool ps_hid_is_dev_connected(void)
{
	return (ps_hid_dev->status == ESP_HIDH_CONN_STATE_CONNECTED);
}

ps_hid_input_t ps_hid_get_data(void)
{
	ps_hid_input_t rpt = { 0 };

	if (ps_hid_is_dev_connected() && ps_hid_dev->ops->get_data)
		ps_hid_dev->ops->get_data(&rpt);

	return rpt;
}

void ps_ouput_handler(void *pdata)
{
	ps_hid_output_t buf = { 0 };
	esp_bd_addr_t dev_mac = { 0 };
	uint8_t count = 0;

	memcpy(dev_mac, pdata, sizeof(dev_mac));

	while (!ps_hid_is_dev_connected())
		vTaskDelay(100 / portTICK_PERIOD_MS);

	if (ps_hid_dev->ops->firmware)
		ps_hid_dev->ops->firmware(dev_mac);

	if (ps_hid_dev->ops->calibration)
		ps_hid_dev->ops->calibration(dev_mac);

	while (1) {
		/*
		 * The device may be disconnected even
		 * after a succesful initial pairing.
		 * So, make sure it's handled properly.
		 */
		if (!ps_hid_is_dev_connected()) {
		    vTaskDelay(10 / portTICK_PERIOD_MS);
		    continue;
		}

		count = uxQueueMessagesWaiting(ps_hid_dev->work);
		while (count) {
			count--;
			xQueueReceive(ps_hid_dev->work, &buf, 0);

			if (ps_hid_dev->ops->output_handler)
				ps_hid_dev->ops->output_handler(dev_mac, &buf);
		}

	    vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}


static inline void spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t* param)
{
	if (event == ESP_SPP_INIT_EVT)
		esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
}

static inline void spp_init(void)
{
	esp_err_t ret = ESP_OK;
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_bluedroid_config_t bluedroid_cfg = { .ssp_en = true };
	esp_spp_cfg_t ssp_cfg = { .mode = ESP_SPP_MODE_CB, .enable_l2cap_ertm = true, };

	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		pr_err("controller init failed: %d", ret);
		return;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
	if (ret) {
		pr_err("failed to enable controller: %d", ret);
		goto init_err;
	}

	ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
	if (ret) {
		pr_err("bluedroid init failed: %d", ret);
		goto en_err;
	}

	ret = esp_bluedroid_enable();
	if (ret) {
		pr_err("failed to enable bluedroid: %d", ret);
		goto bld_init_err;
	}

	ret = esp_spp_register_callback(spp_cb);
	if (ret) {
		pr_err("spp register failed: %d", ret);
		goto bld_en_err;
	}

	ret = esp_spp_enhanced_init(&ssp_cfg);
	if (ret) {
		pr_err("spp init failed: %d", ret);
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

static inline void spp_exit(void)
{
	esp_spp_deinit();
	esp_bluedroid_disable();
	esp_bluedroid_deinit();
	esp_bt_controller_disable();
	esp_bt_controller_deinit();
}

esp_err_t ps_hid_init(esp_bd_addr_t mac)
{
	esp_err_t ret = ESP_OK;

	nvs_flash_init();
	spp_init();

	ret = esp_bt_hid_host_register_callback(&__ps_hid_cb);
	if (ret) {
		pr_err("unable to register hid callback!, ret: %d", ret);
		return ret;
	}

	ret = esp_bt_hid_host_init();
	if (ret) {
		pr_err("init failed!, ret: %d", ret);
		return ret;
	}

	ps_hid_dev = (ps_hid_ctx_t *)heap_caps_calloc(1, sizeof(ps_hid_ctx_t), MALLOC_CAP_DEFAULT);
	if (ps_hid_dev == NULL) {
		pr_err("failed to alloc ops!");
		goto host_exit;
	}

	ps_hid_dev->ops = (ps_ops_t *)heap_caps_calloc(1, sizeof(ps_ops_t), MALLOC_CAP_DEFAULT);
	if (ps_hid_dev->ops == NULL) {
		pr_err("failed to alloc ops!");
		goto mem_exit;
	}

	ps_hid_dev->work = xQueueCreate(10, sizeof(ps_hid_output_t));
	if (!ps_hid_dev->work) {
		pr_err("failed to initialise work!");
		goto ops_exit;
	}

	xTaskCreate(ps_ouput_handler, "ps_hid_worker", 4096, (uint8_t *)mac, 1, &ps_hid_dev->worker);
	if (!ps_hid_dev->worker) {
		pr_err("failed to initialise worker!");
		goto work_exit;
	}

	ps_hid_dev->status = ESP_HIDH_CONN_STATE_UNKNOWN;
	ret = esp_bt_hid_host_connect(mac);
	if (ret) {
		pr_err("unable to connect device!, mac: %x:%x:%x:%x:%x:%x",
					mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		goto worker_exit;
	}

	ps_hid_set_led(0, 0, 128);

	return ret;

worker_exit:
	vTaskDelete(ps_hid_dev->worker);
work_exit:
	vQueueDelete(ps_hid_dev->work);
ops_exit:
	heap_caps_free(ps_hid_dev->ops);
mem_exit:
	heap_caps_free(ps_hid_dev);
host_exit:
	esp_bt_hid_host_deinit();
	return ret;
}

void ps_hid_exit(esp_bd_addr_t mac)
{
	esp_bt_hid_host_disconnect(mac);

	if (ps_hid_dev) {
		vTaskDelete(ps_hid_dev->worker);
		vQueueDelete(ps_hid_dev->work);
		heap_caps_free(ps_hid_dev);
	}

	esp_bt_hid_host_deinit();
	spp_exit();
	nvs_flash_deinit();
}
