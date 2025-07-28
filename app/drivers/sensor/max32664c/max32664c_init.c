/*
 * Initialization code for the MAX32664C biometric sensor hub.
 *
 * Copyright (c) 2025, Daniel Kampert
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>

#include "max32664c.h"

LOG_MODULE_REGISTER(maxim_max32664c_init, CONFIG_SENSOR_LOG_LEVEL);

/** @brief      Run a basic initialization on the sensor hub.
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
int max32664c_init_hub(const struct device *dev)
{
	uint8_t rx[2];
	uint8_t tx[5];
	const struct max32664c_config *config = dev->config;
	struct max32664c_data *data = dev->data;

	LOG_DBG("Initialize sensor hub");

	/* Write the default settings */
	tx[0] = 0x50;
	tx[1] = 0x07;
	tx[2] = 0x13;
	tx[3] = config->min_integration_time_idx;
	if (max32664c_i2c_transmit(dev, tx, 4, rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not write minimum integration time!");
		return -EINVAL;
	}

	tx[0] = 0x50;
	tx[1] = 0x07;
	tx[2] = 0x14;
	tx[3] = config->min_sampling_rate_idx;
	if (max32664c_i2c_transmit(dev, tx, 4, rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not write minimum sampling rate!");
		return -EINVAL;
	}

	tx[0] = 0x50;
	tx[1] = 0x07;
	tx[2] = 0x15;
	tx[3] = config->max_integration_time_idx;
	if (max32664c_i2c_transmit(dev, tx, 4, rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not write maximum integration time!");
		return -EINVAL;
	}

	tx[0] = 0x50;
	tx[1] = 0x07;
	tx[2] = 0x16;
	tx[3] = config->max_sampling_rate_idx;
	if (max32664c_i2c_transmit(dev, tx, 4, rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not write maximum sampling rate!");
		return -EINVAL;
	}

	/* Read the default settings and update the local device configuration */
	tx[0] = 0x51;
	tx[1] = 0x07;
	tx[2] = 0x13;
	if (max32664c_i2c_transmit(dev, tx, 3, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not read minimum integration time!");
		return -EINVAL;
	}
	data->min_integration_time_idx = rx[1];

	tx[0] = 0x51;
	tx[1] = 0x07;
	tx[2] = 0x14;
	if (max32664c_i2c_transmit(dev, tx, 3, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not read minimum sampling rate!");
		return -EINVAL;
	}
	data->min_sampling_rate_idx = rx[1];

	tx[0] = 0x51;
	tx[1] = 0x07;
	tx[2] = 0x15;
	if (max32664c_i2c_transmit(dev, tx, 3, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not read maximum integration time!");
		return -EINVAL;
	}
	data->max_integration_time_idx = rx[1];

	tx[0] = 0x51;
	tx[1] = 0x07;
	tx[2] = 0x16;
	if (max32664c_i2c_transmit(dev, tx, 3, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not read maximum sampling rate!");
		return -EINVAL;
	}
	data->max_sampling_rate_idx = rx[1];

	/* Set the report rate to one report per sensor sample */
	tx[0] = 0x10;
	tx[1] = 0x02;
	tx[2] = 0x01;
	if (max32664c_i2c_transmit(dev, tx, 3, rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not set report rate!");
		return -EINVAL;
	}

	/* Set interrupt threshold */
	tx[0] = 0x10;
	tx[1] = 0x01;
	tx[2] = 0x01;
	if (max32664c_i2c_transmit(dev, tx, 3, rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not set interrupt threshold!");
		return -EINVAL;
	}

	data->motion_time = config->motion_time;
	data->motion_threshold = config->motion_threshold;
	data->use_scd = config->use_scd;
	memcpy(data->led_current, config->led_current, sizeof(data->led_current));

	LOG_DBG("Initial configuration:");

#ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
	LOG_DBG("\tUsing dynamic memory for queues and buffers");
#else
	LOG_DBG("\tUsing static memory for queues and buffers");
#endif

	LOG_DBG("\tMinimum integration time: %u", data->min_integration_time_idx);
	LOG_DBG("\tMinimum sampling rate: %u", data->min_sampling_rate_idx);
	LOG_DBG("\tMaximum integration time: %u", data->max_integration_time_idx);
	LOG_DBG("\tMaximum sampling rate: %u", data->max_sampling_rate_idx);
	LOG_DBG("\tUse SCD: %u", data->use_scd);

	return 0;
}
