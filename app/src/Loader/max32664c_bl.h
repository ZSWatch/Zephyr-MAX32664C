/*
 * I2C firmware loader for the MAX32664C sensor hub.
 *
 * Copyright (c) 2025, Daniel Kampert
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>

/** @brief          Enter the bootloader mode and run a firmware update.
 *  @param dev      Pointer to device
 *  @param firmware Pointer to firmware data
 *  @param size     Size of the firmware
 *  @return         0 when successful
*/
int max32664c_bl_enter(const struct device *dev, const uint8_t *firmware, uint32_t size);

/** @brief      Leave the bootloader and enter the application mode.
 *  @param dev  Pointer to device
 *  @return     0 when successful
*/
int max32664c_bl_leave(const struct device *dev);