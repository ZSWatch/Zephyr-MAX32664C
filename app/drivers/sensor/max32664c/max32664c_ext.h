/*
 * Copyright (c) 2025 Daniel Kampert
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX32664C_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX32664C_H_

#include <zephyr/device.h>

/** @brief Converts a motion time in milli-seconds to the corresponding value for the MAX32664C
 * sensor. This macro should be used when configuring the motion based wake up settings for the
 * sensor.
 */
#define MAX32664C_MOTION_TIME(ms) ((uint8_t)((ms * 25UL) / 1000))

/** @brief Converts a motion threshold in milli-g (Acceleration) to the corresponding value for the
 * MAX32664C sensor. This macro should be used when configuring the motion based wake up settings
 * for the sensor.
 */
#define MAX32664C_MOTION_THRESHOLD(mg) ((uint8_t)((mg * 16UL) / 1000))

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Device operating modes for the MAX32664C sensor.
 *
 *  This enum defines the various operating modes that the MAX32664C sensor
 *  can be configured to. These modes control the sensor's behavior and
 *  functionality, such as calibration, idle state, raw data output, and
 *  algorithm-based operations.
 */
enum max32664c_device_mode {
	MAX32664C_OP_MODE_CAL,
	MAX32664C_OP_MODE_IDLE,
	MAX32664C_OP_MODE_RAW,
	MAX32664C_OP_MODE_ALGO_AEC,
	MAX32664C_OP_MODE_ALGO_AEC_EXT,
	MAX32664C_OP_MODE_ALGO_AGC,
	MAX32664C_OP_MODE_ALGO_AGC_EXT,
	MAX32664C_OP_MODE_SCD,
	MAX32664C_OP_MODE_WAKE_ON_MOTION,
	MAX32664C_OP_MODE_EXIT_WAKE_ON_MOTION,
	MAX32664C_OP_MODE_STOP_ALGO,
};

/** @brief Algorithm modes for the MAX32664C sensor.
 *
 *  This enum defines the various algorithm modes supported by the MAX32664C sensor.
 *  These modes determine the type of data processing performed by the sensor,
 *  such as continuous heart rate monitoring, SpO2 calculation, or activity tracking.
 */
enum max32664c_algo_mode {
	MAX32664C_ALGO_MODE_CONT_HR_CONT_SPO2,
	MAX32664C_ALGO_MODE_CONT_HR_SHOT_SPO2,
	MAX32664C_ALGO_MODE_CONT_HRM,
	MAX32664C_ALGO_MODE_SAMPLED_HRM,
	MAX32664C_ALGO_MODE_SAMPLED_HRM_SHOT_SPO2,
	MAX32664C_ALGO_MODE_ACT_TRACK,
	MAX32664C_ALGO_MODE_SPO2_CAL,
};

/** @brief Gender settings for the MAX32664C sensor.
 *
 *  This enum defines the supported gender settings for the MAX32664C sensor.
 */
enum max32664c_algo_gender {
	MAX32664_ALGO_GENDER_MALE,
	MAX32664_ALGO_GENDER_FEMALE,
};

#ifdef CONFIG_MAX32664C_USE_FIRMWARE_LOADER
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
#endif /* CONFIG_MAX32664C_USE_FIRMWARE_LOADER */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_MAX32664C_H_ */
