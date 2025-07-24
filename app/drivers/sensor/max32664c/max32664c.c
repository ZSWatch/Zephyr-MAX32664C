/*
 * Copyright (c) 2025, Daniel Kampert
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "max32664c.h"

#ifdef CONFIG_MAX32664C_USE_MAX86141
    #define CONFIG_MAX32664C_AFE_ID     0x25
#elif CONFIG_MAX32664C_USE_MAX86161
    #define CONFIG_MAX32664C_AFE_ID     0x36
#else
    #error "No heart rate sensor defined!"
#endif

#define DT_DRV_COMPAT maxim_max32664c

#if(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0)
    #warning "max62664c driver enabled without any devices"
#endif

LOG_MODULE_REGISTER(maxim_max32664c, CONFIG_MAXIM_MAX32664C_LOG_LEVEL);

#if CONFIG_MAX32664C_USE_STATIC_MEMORY
    static uint8_t max32664c_raw_queue_buffer[CONFIG_MAX32664C_QUEUE_SIZE * sizeof(struct max32664c_raw_t)];
    static uint8_t max32664c_report_queue_buffer[CONFIG_MAX32664C_QUEUE_SIZE * (sizeof(struct max32664c_raw_t) + sizeof(struct max32664c_report_t))];

    #if CONFIG_MAX32664C_USE_EXTENDED_REPORTS
        static uint8_t max32664c_ext_report_queue_buffer[CONFIG_MAX32664C_QUEUE_SIZE * (sizeof(struct max32664c_raw_t) + sizeof(struct max32664c_ext_report_t))];
    #endif
#endif

static K_THREAD_STACK_DEFINE(max32664c_thread_stack, CONFIG_MAX32664C_THREAD_STACK_SIZE);

/** @brief          Basic I2C transmission to and from the sensor hub.
 *  @param dev      Pointer to device
 *  @param tx_buf   Pointer to Tx buffer
 *  @param tx_len   Length of Tx buffer
 *  @param rx_buf   Pointer to Rx buffer
 *  @param rx_len   Length of Rx buffer
 *  @param delay    Command delay in milliseconds. See the documentation to get the delay for a specific command.
 *                  Use MAX32664C_DEFAULT_CMD_DELAY for the default delay.
 *  @return         0 when successful
 */
int max32664c_i2c_transmit(const struct device *dev, uint8_t* tx_buf, uint8_t tx_len, uint8_t* rx_buf, uint32_t rx_len, uint16_t delay_ms)
{
    const struct max32664c_config *config = dev->config;

    /* Wake up the sensor hub before the transmission starts */
    gpio_pin_set_dt(&config->mfio_gpio, false);
    k_msleep(500);

    if (i2c_write_dt(&config->i2c, tx_buf, tx_len)) {
        LOG_ERR("I2C transmission error!");
        return -EBUSY;
    }

    k_msleep(delay_ms);

    if (i2c_read_dt(&config->i2c, rx_buf, rx_len)) {
        LOG_ERR("I2C read error!");
        return -EBUSY;
    }

    k_sleep(K_MSEC(MAX32664C_DEFAULT_CMD_DELAY));

    /* The sensor hub can enter sleep mode again now */
    gpio_pin_set_dt(&config->mfio_gpio, true);
    k_usleep(300);

    /* Check the status byte for a valid transaction */
    if (rx_buf[0] != 0) {
        return -EINVAL;
    }

    return 0;
}

/** @brief          Enable / Disable the accelerometer.
 *  @param dev      Pointer to device
 *  @param enable   Enable / Disable 
 *  @return         0 when successful
 */
static int max32664c_acc_enable(const struct device *dev, bool enable)
{
    uint8_t tx[4];
    uint8_t rx;

    tx[0] = 0x44;
    tx[1] = 0x04;
    tx[2] = enable;

#if CONFIG_MAX32664C_USE_EXTERNAL_ACC
    tx[3] = 1;
    #error "Not implemented!"
#else
    tx[3] = 0;
#endif

    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, 20)) {
        return -EINVAL;
    }

    return 0;
}

/** @brief      Set the SpO2 calibration coefficients.
 *              NOTE: See page 10 of the SpO2 and Heart Rate User Guide for additional information.
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
static int max32664c_set_spo2_coeffs(const struct device *dev)
{
    const struct max32664c_config *config = dev->config;

    uint8_t tx[15];
    uint8_t rx;

    /* Write the calibration coefficients */
    tx[0] = 0x50;
    tx[1] = 0x07;
    tx[2] = 0x00;

    /* Copy the A value (index 0) into the transmission buffer */
    memcpy(&tx[3], &config->spo2_calib[0], sizeof(int32_t));

    /* Copy the B value (index 1) into the transmission buffer */
    memcpy(&tx[7], &config->spo2_calib[1], sizeof(int32_t));

    /* Copy the C value (index 2) into the transmission buffer */
    memcpy(&tx[11], &config->spo2_calib[2], sizeof(int32_t));

    return max32664c_i2c_transmit(dev, tx, 15, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY);
}

/** @brief      
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
static int max32664c_check_sensors(const struct device *dev)
{
    uint8_t tx[3];
    uint8_t rx[2];
    struct max32664c_data *data = dev->data;

    LOG_DBG("Checking sensors...");

    /* Read MAX86141 WHOAMI */
    tx[0] = 0x41;
    tx[1] = 0x00;
    tx[2] = 0xFF;
    if (max32664c_i2c_transmit(dev, tx, 3, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    data->afe_id = rx[1];
    if (data->afe_id != CONFIG_MAX32664C_AFE_ID) {
        LOG_ERR("\tAFE WHOAMI failed: 0x%X", data->afe_id);
    } else {
        LOG_DBG("\tAFE WHOAMI OK: 0x%X", data->afe_id);
    }

    /* Read Accelerometer WHOAMI */
    tx[0] = 0x41;
    tx[1] = 0x04;
    tx[2] = 0x0F;
    if (max32664c_i2c_transmit(dev, tx, 3, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    data->accel_id = rx[1];
    if (data->accel_id != CONFIG_MAX32664C_ACC_ID) {
        LOG_ERR("\tAccelerometer WHOAMI failed: 0x%X", data->accel_id);
    } else {
        LOG_DBG("\tAccelerometer WHOAMI OK: 0x%X", data->accel_id);
    }

    return 0;
}

static int max32664c_set_mode_raw(const struct device *dev)
{
    uint8_t rx;
    uint8_t tx[4];
    struct max32664c_data *data = dev->data;

    LOG_INF("Entering RAW mode...");

    /* Output mode Raw */
    tx[0] = 0x10;
    tx[1] = 0x00;
    tx[2] = MAX32664C_OUT_SENSOR_ONLY;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Enable AFE */
    tx[0] = 0x44;
    tx[1] = 0x00;
    tx[2] = 0x01;
    tx[3] = 0x00;
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, 250)) {
        return -EINVAL;
    }

    /* Enable the accelerometer */
    if (max32664c_acc_enable(dev, true)) {
        return -EINVAL;
    }

    /* Set AFE sample rate to 100 Hz */
    tx[0] = 0x40;
    tx[1] = 0x00;
    tx[2] = 0x12;
    tx[3] = 0x18;
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set LED1 (Green) current */
    tx[0] = 0x40;
    tx[1] = 0x00;
    tx[2] = 0x23;
    tx[3] = data->led_current[0];
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set LED2 (IR) current */
    tx[0] = 0x40;
    tx[1] = 0x00;
    tx[2] = 0x24;
    tx[3] = data->led_current[1];
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set LED3 (Red) current */
    tx[0] = 0x40;
    tx[1] = 0x00;
    tx[2] = 0x25;
    tx[3] = data->led_current[2];
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    #ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
        k_msgq_alloc_init(&data->raw_queue, sizeof(struct max32664c_raw_t), CONFIG_MAX32664C_QUEUE_SIZE);
    #endif

    data->op_mode = MAX32664C_OP_MODE_RAW;
    data->threadRunning = true;
    data->threadID = k_thread_create(&data->thread,
        max32664c_thread_stack,
        sizeof(max32664c_thread_stack),
        (k_thread_entry_t)max32664c_worker,
        (void*)dev, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);

    return 0;
}

static int max32664c_disable_sensors(const struct device *dev)
{
    uint8_t rx;
    uint8_t tx[4];
    struct max32664c_data *data = dev->data;

    LOG_DBG("Disable the sensors...");

    /* Disable AFE */
    tx[0] = 0x44;
    tx[1] = 0x00;
    tx[2] = 0x00;
    tx[2] = 0x00;
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, 250)) {
        return -EINVAL;
    }

    /* Disable the accelerometer */
    if (max32664c_acc_enable(dev, false)) {
        return -EINVAL;
    }

    data->op_mode = MAX32664C_OP_MODE_IDLE;

    return 0;
}

// TODO: Enable needed
static int max32664c_stop_algo(const struct device *dev)
{
    uint8_t rx;
    uint8_t tx[3];
    struct max32664c_data *data = dev->data;

    LOG_DBG("Stopping the algorithm...");

    /* Stop the algorithm */
    tx[0] = 0x52;
    tx[1] = 0x07;
    tx[2] = 0x00;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, 120)) {
        return -EINVAL;
    }

    data->op_mode = MAX32664C_OP_MODE_STOP_ALGO;

    if (data->op_mode == MAX32664C_OP_MODE_RAW) {
        #ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
            k_msgq_cleanup(&data->raw_queue);
        #endif
    } else if ((data->op_mode == MAX32664C_OP_MODE_ALGO_AEC) || (data->op_mode == MAX32664C_OP_MODE_ALGO_AGC)) {
        #ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
            k_msgq_cleanup(&data->report_queue);
        #endif
    }
#if CONFIG_MAX32664C_USE_EXTENDED_REPORTS
    else if ((data->op_mode == MAX32664C_OP_MODE_ALGO_AEC_EXT) || (data->op_mode == MAX32664C_OP_MODE_ALGO_AGC_EXT)) {
        #ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
            k_msgq_cleanup(&data->report_queue);
        #endif
    }
#endif

    return 0;
}

static int max32664c_set_mode_scd(const struct device *dev)
{
    uint8_t rx;
    uint8_t tx[4];
    struct max32664c_data *data = dev->data;

    LOG_DBG("MAX32664C entering SCD mode...");

    if (max32664c_set_spo2_coeffs(dev)) {
        return -EINVAL;
    }

    /* Set LED for SCD */
    tx[0] = 0xE5;
    tx[1] = 0x02;
    if (max32664c_i2c_transmit(dev, tx, 2, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set the output mode to algorithm data */
    tx[0] = 0x10;
    tx[1] = 0x00;
    tx[2] = MAX32664C_OUT_ALGORITHM_ONLY;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Enable SCD only algorithm */
    tx[0] = 0x52;
    tx[1] = 0x07;
    tx[2] = 0x03;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, 500)) {
        return -EINVAL;
    }
    
    data->op_mode = MAX32664C_OP_MODE_SCD;

    return 0;
}

static int max32664c_set_mode_wake_on_motion(const struct device *dev)
{
    uint8_t rx;
    uint8_t tx[6];
    struct max32664c_data *data = dev->data;

    LOG_DBG("MAX32664C entering wake on motion mode...");

    /* Stop the current algorithm */
    tx[0] = 0x52;
    tx[1] = 0x07;
    tx[2] = 0x00;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set the motion detection threshold (see Table 12 in the SpO2 and Heart Rate Using Guide) */
    /*  Duration: 0.2 s */
    /*  Threshold: 0.5 g */
    tx[0] = 0x46;
    tx[1] = 0x04;
    tx[2] = 0x00;
    tx[3] = 0x01;
    tx[4] = MAX32664C_MOTION_TIME(data->motion_time);
    tx[5] = MAX32664C_MOTION_THRESHOLD(data->motion_threshold);
    if (max32664c_i2c_transmit(dev, tx, 6, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set the output mode to sensor data */
    tx[0] = 0x10;
    tx[1] = 0x00;
    tx[2] = MAX32664C_OUT_SENSOR_ONLY;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, MAX32664C_OUT_SENSOR_ONLY, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Enable the accelerometer */
    if (max32664c_acc_enable(dev, true)) {
        return -EINVAL;
    }

    data->op_mode = MAX32664C_OP_MODE_WAKE_ON_MOTION;

    return 0;
}

static int max32664c_exit_mode_wake_on_motion(const struct device *dev)
{
    uint8_t rx;
    uint8_t tx[6];
    struct max32664c_data *data = dev->data;

    LOG_DBG("MAX32664C exiting wake on motion mode...");

    /* Disable the accelerometer */
    if (max32664c_acc_enable(dev, false)) {
        return -EINVAL;
    }

    /* Exit wake on motion mode */
    tx[0] = 0x46;
    tx[1] = 0x04;
    tx[2] = 0x00;
    tx[3] = 0x00;
    tx[4] = 0xFF;
    tx[5] = 0xFF;
    if (max32664c_i2c_transmit(dev, tx, 6, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    data->op_mode = MAX32664C_OP_MODE_IDLE;

    return 0;
}

/** @brief              Put the sensor hub into algorithm mode.
 *  @param dev          Pointer to device
 *  @param device_mode  Target device mode
 *  @param algo_mode    Target algorithm mode
 *  @param extended     Set to #True when the extended mode should be used
 *  @return             0 when successful
 */
static int max32664c_set_mode_algo(const struct device *dev, enum max32664c_device_mode device_mode, enum max32664c_algo_mode algo_mode, bool extended)
{
    uint8_t rx;
    uint8_t tx[5];
    struct max32664c_data *data = dev->data;
    const struct max32664c_config *config = dev->config;

    LOG_DBG("Entering algorithm mode...");

#ifndef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
    if (extended) {
        LOG_ERR("No support for extended reports enabled!");
        return -EINVAL;
    }
#endif

    /* Set the output mode to sensor and algorithm data */
    tx[0] = 0x10;
    tx[1] = 0x00;
    tx[2] = MAX32664C_OUT_ALGO_AND_SENSOR;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }
    
    /* Set the algorithm mode */
    tx[0] = 0x50;
    tx[1] = 0x07;
    tx[2] = 0x0A;
    tx[3] = algo_mode;
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    if (device_mode == MAX32664C_OP_MODE_ALGO_AEC) {
        LOG_DBG("Entering AEC mode...");

        /* Enable AEC */
        tx[0] = 0x50;
        tx[1] = 0x07;
        tx[2] = 0x0B;
        tx[3] = 0x01;
        if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
            return -EINVAL;
        }

        /* Enable Auto PD */
        tx[0] = 0x50;
        tx[1] = 0x07;
        tx[2] = 0x12;
        tx[3] = 0x01;
        if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
            return -EINVAL;
        }

        /* Enable SCD */
        tx[0] = 0x50;
        tx[1] = 0x07;
        tx[2] = 0x0C;
        tx[3] = 0x01;
        if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
            return -EINVAL;
        }

        data->op_mode = MAX32664C_OP_MODE_ALGO_AEC;

        if (extended) {
            data->op_mode = MAX32664C_OP_MODE_ALGO_AEC_EXT;
        }
    } else if (device_mode == MAX32664C_OP_MODE_ALGO_AGC) {
        LOG_DBG("Entering AGC mode...");

        /* Disable PD auto current calculation */
        tx[0] = 0x50;
        tx[1] = 0x07;
        tx[2] = 0x12;
        tx[3] = 0x00;
        if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
            return -EINVAL;
        }

        /* Disable SCD */
        tx[0] = 0x50;
        tx[1] = 0x07;
        tx[2] = 0x0C;
        tx[3] = 0x00;
        if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
            return -EINVAL;
        }

        /* Set AGC target PD current */
        tx[0] = 0x50;
        tx[1] = 0x07;
        tx[2] = 0x11;
        tx[3] = 0x00;
        tx[4] = 0x64;
        if (max32664c_i2c_transmit(dev, tx, 5, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
            return -EINVAL;
        }

        data->op_mode = MAX32664C_OP_MODE_ALGO_AGC;

        if (extended) {
            data->op_mode = MAX32664C_OP_MODE_ALGO_AGC_EXT;
        }
    } else {
        LOG_ERR("Invalid mode!");
        return -EINVAL;
    }

    /* Configure WHRM */
    tx[0] = 0x50;
    tx[1] = 0x07;
    tx[2] = 0x17;
    tx[3] = config->hr_config[0];
    tx[4] = config->hr_config[1];
    if (max32664c_i2c_transmit(dev, tx, 5, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        LOG_ERR("Can not configure WHRM!");
        return -EINVAL;
    }

    /* Configure SpO2 */
    tx[0] = 0x50;
    tx[1] = 0x07;
    tx[2] = 0x18;
    tx[3] = config->spo2_config[0];
    tx[4] = config->spo2_config[1];
    if (max32664c_i2c_transmit(dev, tx, 5, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        LOG_ERR("Can not configure SpO2!");
        return -EINVAL;
    }

    /* Enable HR and SpO2 algorithm */
    tx[2] = 0x01;
    if (extended) {
        tx[2] = 0x02;
    }

    tx[0] = 0x52;
    tx[1] = 0x07;

    /* Use the maximum time to cover all modes (see Table 6 in the User Guide) */
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, 320)) {
        return -EINVAL;
    }

#ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
        if (!extended) {
            k_msgq_alloc_init(&data->report_queue, sizeof(max32664c_report_t), CONFIG_MAX32664C_QUEUE_SIZE);
        }
    #if CONFIG_MAX32664C_USE_EXTENDED_REPORTS
        else {
            k_msgq_alloc_init(&data->ext_report_queue, sizeof(max32664c_ext_report_t), CONFIG_MAX32664C_QUEUE_SIZE);
        }
    #endif
#endif

    data->threadRunning = true;
    data->threadID = k_thread_create(&data->thread,
        max32664c_thread_stack,
        sizeof(max32664c_thread_stack),
        (k_thread_entry_t)max32664c_worker,
        (void*)dev, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);

    return 0;
}

/** @brief      Run a basic initialization on the sensor hub.
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
static int max32664c_init_hub(const struct device *dev)
{
    uint8_t rx;
    uint8_t tx[5];
    const struct max32664c_config *config = dev->config;

    LOG_DBG("Initialize sensor hub");

    /* Set the report rate to one report per sensor sample */
    tx[0] = 0x10;
    tx[1] = 0x02;
    tx[2] = 0x01;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        LOG_ERR("Can not set report rate!");
        return -EINVAL;
    }

    /* Set interrupt threshold */
    tx[0] = 0x10;
    tx[1] = 0x01;
    tx[2] = 0x01;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        LOG_ERR("Can not set interrupt threshold!");
        return -EINVAL;
    }

    return 0;
}

static int max32664c_sample_fetch(const struct device *dev,
                                  enum sensor_channel chan)
{
    struct max32664c_data *data = dev->data;

    k_msgq_get(&data->raw_queue, &data->raw, K_NO_WAIT);

    if ((data->op_mode == MAX32664C_OP_MODE_ALGO_AEC) || (data->op_mode == MAX32664C_OP_MODE_ALGO_AGC)) {
        k_msgq_get(&data->report_queue, &data->report, K_NO_WAIT);
    }
#if CONFIG_MAX32664C_USE_EXTENDED_REPORTS
    else if ((data->op_mode == MAX32664C_OP_MODE_ALGO_AEC_EXT) || (data->op_mode == MAX32664C_OP_MODE_ALGO_AGC_EXT)) {
        k_msgq_get(&data->ext_report_queue, &data->ext_report, K_NO_WAIT);
    }
#endif

    return 0;
}

static int max32664c_channel_get(const struct device *dev,
                                 enum sensor_channel chan,
                                 struct sensor_value *val)
{
    struct max32664c_data *data = dev->data;

    switch (chan)
    {
        case SENSOR_CHAN_ACCEL_X:
        {
            val->val1 = data->raw.acc.x;
            break;
        }
        case SENSOR_CHAN_ACCEL_Y:
        {
            val->val1 = data->raw.acc.y;
            break;
        }
        case SENSOR_CHAN_ACCEL_Z:
        {
            val->val1 = data->raw.acc.z;
            break;
        }
        case SENSOR_CHAN_IR:
        {
            val->val1 = data->raw.PPG5;
            break;
        }
        case SENSOR_CHAN_RED:
        {
            val->val1 = data->raw.PPG6;
            break;
        }
        case SENSOR_CHAN_GREEN:
        {
            val->val1 = data->raw.PPG1;
            val->val2 = data->raw.PPG4;
            break;
        }
        case SENSOR_CHAN_HEARTRATE:
        {
            val->val1 = data->report.hr;
            val->val2 = data->report.hr_confidence;
            break;
        }
        case SENSOR_CHAN_RR:
        {
            val->val1 = data->report.rr;
            val->val2 = data->report.rr_confidence;
            break;
        }
        case SENSOR_CHAN_BLOOD_OXYGEN_SATURATION:
        {
            val->val1 = data->report.spo2;
            val->val2 = data->report.spo2_confidence;
            break;
        }
        default:
        {
            LOG_ERR("Channel %u not supported!", chan);
            break;
        }
    }

    return 0;
}

static int max32664c_attr_set(const struct device *dev,
                              enum sensor_channel chan,
                              enum sensor_attribute attr,
                              const struct sensor_value *val)
{
    struct max32664c_data *data = dev->data;

    if ((chan != SENSOR_CHAN_HEARTRATE) && (chan != SENSOR_CHAN_BLOOD_OXYGEN_SATURATION) &&
        (chan != SENSOR_CHAN_RR) && (chan != SENSOR_CHAN_ACCEL_X) &&
        (chan != SENSOR_CHAN_ACCEL_Z) && (chan != SENSOR_CHAN_ACCEL_Y) &&
        (chan != SENSOR_CHAN_RED) && (chan != SENSOR_CHAN_GREEN) &&
        (chan != SENSOR_CHAN_IR) && (chan != SENSOR_CHAN_ALL)) {
        return -EINVAL;
    }

    switch (attr)
    {
        case SENSOR_ATTR_SAMPLING_FREQUENCY:
        {
            break;
        }
        case SENSOR_ATTR_HEIGHT:
        {
            data->algo_conf.height = val->val1;
            break;
        }
        case SENSOR_ATTR_WEIGHT:
        {
            data->algo_conf.weight = val->val1;
            break;
        }
        case SENSOR_ATTR_AGE:
        {
            data->algo_conf.age = val->val1;
            break;
        }
        case SENSOR_ATTR_GENDER:
        {
            data->algo_conf.gender = val->val1;
            break;
        }
        case SENSOR_ATTR_SLOPE_DUR:
        {
            data->motion_time = val->val1;
            break;
        }
        case SENSOR_ATTR_SLOPE_TH:
        {
            data->motion_threshold = val->val1;
            break;
        }
        case SENSOR_ATTR_CONFIGURATION:
        {
            switch (chan)
            {
                case SENSOR_CHAN_GREEN:
                {
                    data->led_current[0] = val->val1 & 0xFF;
                    break;
                }
                case SENSOR_CHAN_RED:
                {
                    data->led_current[1] = val->val1 & 0xFF;
                    break;
                }
                case SENSOR_CHAN_IR:
                {
                    data->led_current[2] = val->val1 & 0xFF;
                    break;
                }
                default:
                {
                    return -ENOTSUP;
                }
            }
            break;
        }
        case SENSOR_ATTR_OP_MODE:
        {
            switch (val->val1)
            {
                case MAX32664C_OP_MODE_ALGO_AEC:
                {
                    max32664c_set_mode_algo(dev, MAX32664C_OP_MODE_ALGO_AEC, val->val2, false);
                    break;
                }
                case MAX32664C_OP_MODE_ALGO_AEC_EXT:
                {
                    //max32664c_set_mode_algo(dev, MAX32664C_OP_MODE_ALGO_AEC, val->val2, true);
                    return -ENOTSUP;
                }
                case MAX32664C_OP_MODE_ALGO_AGC:
                {
                    max32664c_set_mode_algo(dev, MAX32664C_OP_MODE_ALGO_AGC, val->val2, false);
                    break;
                }
                case MAX32664C_OP_MODE_ALGO_AGC_EXT:
                {
                    //max32664c_set_mode_algo(dev, MAX32664C_OP_MODE_ALGO_AGC, val->val2, true);
                    return -ENOTSUP;
                }
                case MAX32664C_OP_MODE_RAW:
                {
                    max32664c_set_mode_raw(dev);
                    break;
                }
                case MAX32664C_OP_MODE_SCD:
                {
                    max32664c_set_mode_scd(dev);
                    break;
                }
                case MAX32664C_OP_MODE_WAKE_ON_MOTION:
                {
                    max32664c_set_mode_wake_on_motion(dev);
                    break;
                }
                case MAX32664C_OP_MODE_EXIT_WAKE_ON_MOTION:
                {
                    max32664c_exit_mode_wake_on_motion(dev);
                    break;
                }
                case MAX32664C_OP_MODE_STOP_ALGO:
                {
                    max32664c_stop_algo(dev);
                    break;
                }
                default:
                {
                    LOG_ERR("Unsupported sensor operation mode");
                    return -ENOTSUP;
                }
            }

            break;
        }
        default:
        {
            LOG_ERR("Unsupported sensor attribute!");
            return -ENOTSUP;
        }
    }

    return 0;
}

static int max32664c_attr_get(const struct device *dev,
                              enum sensor_channel chan,
                              enum sensor_attribute attr,
                              struct sensor_value *val)
{
    struct max32664c_data *data = dev->data;

    if ((chan != SENSOR_CHAN_HEARTRATE) && (chan != SENSOR_CHAN_BLOOD_OXYGEN_SATURATION) &&
        (chan != SENSOR_CHAN_RR) && (chan != SENSOR_CHAN_ACCEL_X) &&
        (chan != SENSOR_CHAN_ACCEL_Z) && (chan != SENSOR_CHAN_ACCEL_Y) &&
        (chan != SENSOR_CHAN_RED) && (chan != SENSOR_CHAN_GREEN) &&
        (chan != SENSOR_CHAN_IR) && (chan != SENSOR_CHAN_ALL)) {
        return -EINVAL;
    }

    switch (attr)
    {
        case SENSOR_ATTR_OP_MODE:
        {
            val->val1 = data->op_mode;
            val->val2 = 0;
            break;
        }
        case SENSOR_ATTR_CONFIGURATION:
        {
            switch (chan)
            {
                case SENSOR_CHAN_GREEN:
                {
                    val->val1 = data->led_current[0];
                    break;
                }
                case SENSOR_CHAN_RED:
                {
                    val->val1 = data->led_current[1];
                    break;
                }
                case SENSOR_CHAN_IR:
                {
                    val->val1 = data->led_current[2];
                    break;
                }
                default:
                {
                    return -ENOTSUP;
                }
            }
            break;
        }
        default:
        {
            LOG_ERR("Unsupported sensor attribute");
            return -ENOTSUP;
        }
    }

    return 0;
}

static const struct sensor_driver_api max32664c_driver_api = {
    .attr_set = max32664c_attr_set,
    .attr_get = max32664c_attr_get,
    .sample_fetch = max32664c_sample_fetch,
    .channel_get = max32664c_channel_get,

#ifdef CONFIG_SENSOR_ASYNC_API
    .submit = max32664c_submit,
    .get_decoder = max32664c_get_decoder,
#endif
};

static int max32664c_init(const struct device *dev)
{
    uint8_t tx[2];
    uint8_t rx[4];
    const struct max32664c_config *config = dev->config;
    struct max32664c_data *data = dev->data;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C not ready");
        return -ENODEV;
    }

    data->motion_time = config->motion_time;
    data->motion_threshold = config->motion_threshold;
    memcpy(data->led_current, config->led_current, sizeof(data->led_current));

    gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT);
    gpio_pin_configure_dt(&config->mfio_gpio, GPIO_OUTPUT);

    /* Put the hub into application mode */
    LOG_DBG("Set app mode");
    gpio_pin_set_dt(&config->reset_gpio, false);
    k_sleep(K_MSEC(20));

    gpio_pin_set_dt(&config->mfio_gpio, true);
    k_sleep(K_MSEC(20));

    /* Wait for 50 ms (switch into app mode) + 1500 ms (initialization) (see page 17 of the User Guide) */
    gpio_pin_set_dt(&config->reset_gpio, true);
    k_sleep(K_MSEC(1600));

    /* Read the device mode */
    tx[0] = 0x02;
    tx[1] = 0x00;
    if (max32664c_i2c_transmit(dev, tx, 2, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    data->op_mode = rx[1];
    LOG_DBG("Mode: %x ", data->op_mode);
    if (data->op_mode != 0) {
        return -EINVAL;
    }

    /* Read the firmware version */
    tx[0] = 0xFF;
    tx[1] = 0x03;
    if (max32664c_i2c_transmit(dev, tx, 2, rx, 4, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    memcpy(data->hub_ver, &rx[1], 3);

    LOG_DBG("Version: %d.%d.%d", data->hub_ver[0], data->hub_ver[1], data->hub_ver[2]);

    if (max32664c_check_sensors(dev)) {
        return -EINVAL;
    }

    if (max32664c_init_hub(dev)) {
        return -EINVAL;
    }

#if CONFIG_MAX32664C_USE_STATIC_MEMORY
        k_msgq_init(&data->raw_queue, max32664c_raw_queue_buffer, sizeof(struct max32664c_raw_t), sizeof(max32664c_raw_queue_buffer) / sizeof(struct max32664c_raw_t));
        k_msgq_init(&data->report_queue, max32664c_report_queue_buffer, sizeof(struct max32664c_report_t), sizeof(max32664c_report_queue_buffer) / sizeof(struct max32664c_report_t));
        
    #if CONFIG_MAX32664C_USE_EXTENDED_REPORTS
        k_msgq_init(&data->ext_report_queue, max32664c_ext_report_queue_buffer, sizeof(struct max32664c_ext_report_t), sizeof(max32664c_ext_report_queue_buffer) / sizeof(struct max32664c_ext_report_t));
    #endif
#endif

    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int max32664c_pm_action(const struct device *dev,
                               enum pm_device_action action)
{
    switch (action)
    {
        case PM_DEVICE_ACTION_RESUME:
        {
            break;
        }
        case PM_DEVICE_ACTION_SUSPEND:
        {
            const struct max32664c_config *config = dev->config;

            /* Pulling MFIO high will cause the hub to enter sleep mode */
            gpio_pin_set_dt(&config->mfio_gpio, true);
            k_sleep(K_MSEC(20));
            break;
        }
        case PM_DEVICE_ACTION_TURN_OFF:
        {
            uint8_t rx;
            uint8_t tx[3];

            /* Send a shut down command */
            /* NOTE: Toggling RSTN is needed to wake the device */
            tx[0] = 0x01;
            tx[1] = 0x00;
            tx[2] = 0x01;
            if (max32664c_i2c_transmit(dev, tx, 3, rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
                return -EINVAL;
            }
            break;
        }
        case PM_DEVICE_ACTION_TURN_ON:
        {
            /* Toggling RSTN is needed to turn the device on */
            max32664c_init(dev);
            break;
        }
        default:
        {
            return -ENOTSUP;
        }
    }

    return 0;
}
#endif /* CONFIG_PM_DEVICE */

#define MAX32664C_INIT(inst)                                                    \
    static struct max32664c_data max32664c_data_##inst;                         \
                                                                                \
    static const struct max32664c_config max32664c_config_##inst =              \
    {                                                                           \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                      \
        .reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),                 \
        .mfio_gpio = GPIO_DT_SPEC_INST_GET(inst, mfio_gpios),                   \
        .spo2_calib = DT_INST_PROP(inst, spo2_calib),                           \
        .hr_config = DT_INST_PROP(inst, hr_config),                             \
        .spo2_config = DT_INST_PROP(inst, spo2_config),                         \
        .motion_time = DT_INST_PROP(inst, motion_time),                         \
        .motion_threshold = DT_INST_PROP(inst, motion_threshold),               \
        .led_current = DT_INST_PROP(inst, led_current),                         \
    };                                                                          \
                                                                                \
    PM_DEVICE_DT_INST_DEFINE(inst, max32664c_pm_action);                        \
                                                                                \
    SENSOR_DEVICE_DT_INST_DEFINE(inst,                                          \
                                 max32664c_init,                                \
                                 PM_DEVICE_DT_INST_GET(inst),                   \
                                 &max32664c_data_##inst,                        \
                                 &max32664c_config_##inst,                      \
                                 POST_KERNEL,                                   \
                                 CONFIG_SENSOR_INIT_PRIORITY,                   \
                                 &max32664c_driver_api)

DT_INST_FOREACH_STATUS_OKAY(MAX32664C_INIT)