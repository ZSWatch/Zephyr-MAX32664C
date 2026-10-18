/*
 * External accelerometer driver for the MAX32664C biometric sensor hub.
 *
 * Copyright (c) 2025, Daniel Kampert
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "max32664c.h"

LOG_MODULE_REGISTER(maxim_max32664c_acc, CONFIG_SENSOR_LOG_LEVEL);

int max32664c_acc_enable(const struct device *dev, bool enable)
{
	uint8_t tx[4];
	uint8_t rx;

	tx[0] = 0x44;
	tx[1] = 0x04;
	tx[2] = enable;

#if CONFIG_MAX32664C_USE_EXTERNAL_ACC
	tx[3] = 1;
#else
	tx[3] = 0;
#endif /* CONFIG_MAX32664C_USE_EXTERNAL_ACC */

	if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, 20)) {
		return -EINVAL;
	}

#ifndef CONFIG_MAX32664C_USE_EXTERNAL_ACC
	const struct max32664c_data *hub = dev->data;

	/* Disabling hub reports leaves LIS2DS12 sampling. Clear ODR and HF_ODR
	 * explicitly, preserving the full-scale and block-data-update settings.
	 */
	if (!enable && hub->accel_id == LIS2DS12_WHO_AM_I_VALUE) {
		uint8_t ctrl1[2];
		uint8_t read_ctrl1[] = {MAX32664C_FAMILY_READ_REGISTER, MAX32664C_SENSOR_ACCEL,
					LIS2DS12_REG_CTRL1};

		if (max32664c_i2c_transmit(dev, read_ctrl1, sizeof(read_ctrl1),
					  ctrl1, sizeof(ctrl1), 10)) {
			return -EINVAL;
		}
		uint8_t power_down[] = {MAX32664C_FAMILY_WRITE_REGISTER, MAX32664C_SENSOR_ACCEL,
					LIS2DS12_REG_CTRL1,
					ctrl1[1] & (LIS2DS12_CTRL1_FS_MASK | LIS2DS12_CTRL1_BDU)};

		if (max32664c_i2c_transmit(dev, power_down, sizeof(power_down), &rx, 1, 10)) {
			return -EINVAL;
		}
	}
#endif

	return 0;
}

#ifdef CONFIG_MAX32664C_USE_EXTERNAL_ACC
int max32664c_acc_fill_fifo(const struct device *dev, struct max32664c_acc_data_t *data,
			    uint8_t length)
{
	uint8_t tx[2 + 16 * sizeof(struct max32664c_acc_data_t)];
	uint8_t rx;

	if (length > 16) {
		LOG_ERR("Length exceeds maximum of 16 samples!");
		return -EINVAL;
	}

	tx[0] = 0x14;
	tx[1] = 0x00;
	memcpy(&tx[2], data, length * sizeof(struct max32664c_acc_data_t));

	if (max32664c_i2c_transmit(dev, tx, 2 + (length * sizeof(struct max32664c_acc_data_t)), &rx,
				   1, 20)) {
		return -EINVAL;
	}

	return 0;
}
#endif /* CONFIG_MAX32664C_USE_EXTERNAL_ACC */
