// SPDX-License-Identifier: MIT
// Copyright(c) Divyanshu-Modi <divyan.m05@gmail.com>

#define pr_fmt "ROT_ENC"

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rotary_encoder.h"
#include "pr_log.h"

esp_err_t rotary_enc_get_count(enc_handle_t handle, int *val)
{
	return pcnt_unit_get_count(handle->unit, val);
}

esp_err_t rotary_enc_clear_count(enc_handle_t handle)
{
	return pcnt_unit_clear_count(handle->unit);
}

esp_err_t rotary_enc_init(const enc_cfg_t *enc_cfg, enc_handle_t *handle)
{
	esp_err_t ret = ESP_OK;
	enc_dev_t *enc_dev = NULL;
	pcnt_unit_config_t lenc_cfg = { 0 };
	pcnt_chan_config_t chan_cfg = { 0 };

	if (enc_cfg == NULL) {
		pr_err("configuration cannot be empty");
		return ESP_ERR_INVALID_ARG;
	}

	enc_dev = (enc_dev_t *)heap_caps_malloc(sizeof(enc_dev_t), MALLOC_CAP_DEFAULT);
	if (enc_dev == NULL) {
		pr_err("Memory allocation failed!");
		return ESP_ERR_NO_MEM;
	}

	lenc_cfg.high_limit = enc_cfg->h_lim;;
	lenc_cfg.low_limit = enc_cfg->l_lim;
	lenc_cfg.flags.accum_count = 1;

	ret = pcnt_new_unit(&lenc_cfg, &enc_dev->unit);
	if (ret) {
		pr_err("unit initialisation failed!");
		goto free_mem;
	}

	chan_cfg.edge_gpio_num = enc_cfg->chan_a;
	chan_cfg.level_gpio_num = enc_cfg->chan_b;

	ret = pcnt_new_channel(enc_dev->unit, &chan_cfg, &enc_dev->chan[0]);
	if (ret) {
		pr_err("unit initialisation failed!");
		goto del_unit;
	}

	ret = pcnt_channel_set_edge_action(enc_dev->chan[0],
					PCNT_CHANNEL_EDGE_ACTION_DECREASE,
					PCNT_CHANNEL_EDGE_ACTION_INCREASE);
	if (ret) {
		pr_err("channel edge setting failed!");
		goto del_chan;
	}

	ret = pcnt_channel_set_level_action(enc_dev->chan[0],
					PCNT_CHANNEL_LEVEL_ACTION_KEEP,
					PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
	if (ret) {
		pr_err("level edge setting failed!");
		goto del_chan;
	}

	chan_cfg.edge_gpio_num = enc_cfg->chan_b;
	chan_cfg.level_gpio_num = enc_cfg->chan_a;

	ret = pcnt_new_channel(enc_dev->unit, &chan_cfg, &enc_dev->chan[1]);
	if (ret) {
		pr_err("unit initialisation failed!");
		goto del_chan;
	}

	ret = pcnt_channel_set_edge_action(enc_dev->chan[1],
					PCNT_CHANNEL_EDGE_ACTION_INCREASE,
					PCNT_CHANNEL_EDGE_ACTION_DECREASE);
	if (ret) {
		pr_err("channel edge setting failed!");
		goto del_chan_2;
	}

	ret = pcnt_channel_set_level_action(enc_dev->chan[1],
					PCNT_CHANNEL_LEVEL_ACTION_KEEP,
					PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
	if (ret) {
		pr_err("level edge setting failed!");
		goto del_chan_2;
	}

	pcnt_unit_add_watch_point(enc_dev->unit, enc_cfg->h_lim);
	pcnt_unit_add_watch_point(enc_dev->unit, enc_cfg->l_lim);

	ret = pcnt_unit_enable(enc_dev->unit);
	if (!ret)
		pr_debug("Unit successfully enabled!");

	ret = pcnt_unit_clear_count(enc_dev->unit);
	if (ret)
		goto del_chan_2;

	pr_debug("Unit cleared successfully!");

	ret = pcnt_unit_start(enc_dev->unit);
	if (ret)
		goto unit_err;

	pr_debug("Unit started successfully!");

	*handle = enc_dev;

	pr_info("Encoder init succeeded: %s!", DRV_VERSION);

	return ret;

unit_err:
	pcnt_unit_disable(enc_dev->unit);
	pcnt_unit_remove_watch_point(enc_dev->unit, enc_cfg->h_lim);
	pcnt_unit_remove_watch_point(enc_dev->unit, enc_cfg->l_lim);
del_chan_2:
	pcnt_del_channel(enc_dev->chan[1]);
del_chan:
	pcnt_del_channel(enc_dev->chan[0]);
del_unit:
	pcnt_del_unit(enc_dev->unit);
free_mem:
	heap_caps_free(enc_dev);
	return ret;
}

esp_err_t rotary_enc_deinit(enc_handle_t handle)
{
	if (handle == NULL)
		return ESP_ERR_INVALID_ARG;

	pcnt_unit_stop(handle->unit);
	pcnt_unit_disable(handle->unit);
	pcnt_unit_remove_watch_point(handle->unit, handle->h_lim);
	pcnt_unit_remove_watch_point(handle->unit, handle->l_lim);
	pcnt_del_channel(handle->chan[1]);
	pcnt_del_channel(handle->chan[0]);
	pcnt_del_unit(handle->unit);
	heap_caps_free(handle);

	return ESP_OK;
}
