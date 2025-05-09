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

#ifdef CONFIG_MAX32664C_USE_STATIC_MEMORY
    static uint8_t max32664c_algo_config_queue_buffer[CONFIG_MAX32664C_QUEUE_SIZE * sizeof(max32664c_algo_config_t)];
    static uint8_t max32664c_raw_queue_buffer[CONFIG_MAX32664C_QUEUE_SIZE * sizeof(max32664c_raw_t)];
    static uint8_t max32664c_report_queue_buffer[CONFIG_MAX32664C_QUEUE_SIZE * (sizeof(max32664c_raw_t) + sizeof(max32664c_report_t))];
#endif

static K_THREAD_STACK_DEFINE(max32664c_thread_stack, CONFIG_MAX32664C_THREAD_STACK_SIZE);

int max32664c_i2c_transmit(const struct device *dev, uint8_t* tx_buf, uint8_t tx_len, uint8_t* rx_buf, uint32_t rx_len, uint16_t delay)
{
    const struct max32664c_config *config = dev->config;

    gpio_pin_set_dt(&config->mfio_gpio, false);
    k_sleep(K_USEC(500));

    if (i2c_write_dt(&config->i2c, tx_buf, tx_len)) {
        LOG_ERR("I2C transmission error!");
        return -EBUSY;
    }

    k_sleep(K_MSEC(delay));

    if (i2c_read_dt(&config->i2c, rx_buf, rx_len)) {
        LOG_ERR("I2C read error!");
        return -EBUSY;
    }

    k_sleep(K_MSEC(MAX32664C_DEFAULT_CMD_DELAY));

    gpio_pin_set_dt(&config->mfio_gpio, true);
    k_sleep(K_USEC(300));

    /* Check the status byte for a valid transaction */
    //LOG_DBG("Status: %u", rx_buf[0]);
    if (rx_buf[0] != 0) {
        return -EINVAL;
    }

    return 0;
}

static int max32664c_set_spo2_coeffs(const struct device *dev, int32_t a, int32_t b, int32_t c)
{
    uint8_t tx[15] = {0x50, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xD7, 0xFB, 0xDD, 0x00, 0xAB, 0x61, 0xFE};
    uint8_t rx[3];

    // int32_t a_int = (int32_t)(a * 100000);

    /*wr_buf[3] = (a_int & 0xff000000) >> 24;
    wr_buf[4] = (a_int & 0x00ff0000) >> 16;
    wr_buf[5] = (a_int & 0x0000ff00) >> 8;
    wr_buf[6] = (a_int & 0x000000ff);

    int32_t b_int = (int32_t)(b * 100000);

    wr_buf[7] = (b_int & 0xff000000) >> 24;
    wr_buf[8] = (b_int & 0x00ff0000) >> 16;
    wr_buf[9] = (b_int & 0x0000ff00) >> 8;
    wr_buf[10] = (b_int & 0x000000ff);

    int32_t c_int = (int32_t)(c * 100000);

    wr_buf[11] = (c_int & 0xff000000) >> 24;
    wr_buf[12] = (c_int & 0x00ff0000) >> 16;
    wr_buf[13] = (c_int & 0x0000ff00) >> 8;
    wr_buf[14] = (c_int & 0x000000ff);
    */

    //m_i2c_write(dev, wr_buf, sizeof(wr_buf));

    return 0;
}

static int max32664c_check_sensors(const struct device *dev)
{
    uint8_t tx[3];
    uint8_t rx[3];
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
    tx[2] = 0x01;
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
    tx[0] = 0x44;
    tx[1] = 0x04;
    tx[2] = 0x01;
    tx[3] = 0x00;
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, 20)) {
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

    /* Set LED1 current */
    tx[0] = 0x40;
    tx[1] = 0x00;
    tx[2] = 0x23;
    tx[3] = 0x7F;
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set LED2 current */
    tx[0] = 0x40;
    tx[1] = 0x00;
    tx[2] = 0x24;
    tx[3] = 0x7F;
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set LED3 current */
    tx[0] = 0x40;
    tx[1] = 0x00;
    tx[2] = 0x25;
    tx[3] = 0x7F;
    if( max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    #ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
        k_msgq_alloc_init(&data->raw_queue, sizeof(max32664c_acc_data_t), CONFIG_MAX32664C_QUEUE_SIZE);
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

    /* Disable the accelerometers */
    tx[0] = 0x44;
    tx[1] = 0x04;
    tx[2] = 0x00;
    tx[2] = 0x00;
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, 20)) {
        return -EINVAL;
    }

    data->op_mode = MAX32664C_OP_MODE_IDLE;

    return 0;
}

// TODO: ENable needed
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

    //data->threadRunning = false;
    //k_thread_join(&data->thread, K_FOREVER);
    if (data->op_mode == MAX32664C_OP_MODE_RAW) {
        #ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
            k_msgq_cleanup(&data->raw_queue);
        #endif
    } else if ((data->op_mode == MAX32664C_OP_MODE_ALGO_AEC) || (data->op_mode == MAX32664C_OP_MODE_ALGO_AGC)) {
        #ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
            k_msgq_cleanup(&data->report_queue);
        #endif
    }

    return 0;
}

static int max32664c_set_mode_scd(const struct device *dev)
{
    uint8_t rx;
    uint8_t tx[4];
    struct max32664c_data *data = dev->data;

    LOG_DBG("MAX32664C entering SCD mode...");

    // max32664c_set_spo2_coeffs(dev, DEFAULT_SPO2_A, DEFAULT_SPO2_B, DEFAULT_SPO2_C);

    // Set LED for SCD
    tx[0] = 0xE5;
    tx[1] = 0x02;
    if (max32664c_i2c_transmit(dev, tx, 2, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set the output mode to algorithm data */
    tx[0] = 0x10;
    tx[1] = 0x00;
    tx[2] = MAX32664C_MODE_ALGORITHM_ONLY;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set the report rate to one report per sensor sample */
    tx[0] = 0x10;
    tx[1] = 0x02;
    tx[2] = 0x01;
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

    /* Set the motion detection threshold (see table 12 in the SPO2 and heart rate using guide) */
    /*  Duration: 0.2 s */
    /*  Threshold: 0.5 g */
    tx[0] = 0x46;
    tx[1] = 0x04;
    tx[2] = 0x00;
    tx[3] = MAX32664C_MOTION_ENABLE;
    tx[4] = MAX32664C_MOTION_TIME(data->motion_time);
    tx[5] = MAX32664C_MOTION_THRESHOLD(data->motion_threshold);
    if (max32664c_i2c_transmit(dev, tx, 6, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set output mode accelerometer only */
    tx[0] = 0x10;
    tx[1] = 0x00;
    tx[2] = 0x01;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, MAX32664C_MODE_SENSOR_ONLY, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set the report rate to one report per sensor sample */
    tx[0] = 0x10;
    tx[1] = 0x02;
    tx[2] = 0x01;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Enable the accelerometer */
    tx[0] = 0x44;
    tx[1] = 0x04;
    tx[2] = 0x01;
    tx[3] = 0x00;
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, 20)) {
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
    tx[0] = 0x44;
    tx[1] = 0x04;
    tx[2] = 0x00;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, 30)) {
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

/** @brief      Read the status from the sensor hub.
 *              NOTE: Table 7. Host Commands—AGC Mode
 *  @param dev  Pointer to device
 */
static int max32664c_set_mode_algo(const struct device *dev, enum max32664c_mode device_mode, max32664c_algo_mode_t algo_mode, bool extended)
{
    uint8_t tx[5];
    uint8_t rx;
    struct max32664c_data *data = dev->data;

    LOG_DBG("Entering algorithm mode...");

    /* Output mode sensor + algo data */
    tx[0] = 0x10;
    tx[1] = 0x00;
    tx[2] = 0x03;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set report period */
    tx[0] = 0x10;
    tx[1] = 0x02;
    tx[2] = 0x01;
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

        /* Disable Auto PD */
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

    /* Enable HR and SpO2 algo */
    tx[2] = 0x01;
    if (extended) {
        tx[2] = 0x02;
    }

    tx[0] = 0x52;
    tx[1] = 0x07;
    if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, 320)) {
        return -EINVAL;
    }

    #ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
        k_msgq_alloc_init(&data->report_queue, sizeof(max32664c_report_t), CONFIG_MAX32664C_QUEUE_SIZE);
    #endif

    data->threadRunning = true;
    data->threadID = k_thread_create(&data->thread,
        max32664c_thread_stack,
        sizeof(max32664c_thread_stack),
        (k_thread_entry_t)max32664c_worker,
        (void*)dev, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);

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
        case SENSOR_CHAN_SPO2:
        {
            val->val1 = data->report.spo2;
            val->val2 = data->report.sp02_confidence;
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

    if (chan != SENSOR_CHAN_HEARTRATE) {
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
            while (k_msgq_put(&data->config_queue, &data->algo_conf, K_NO_WAIT) != 0) {
                k_msgq_purge(&data->config_queue);
            }
            break;
        }
        case SENSOR_ATTR_WEIGHT:
        {
            data->algo_conf.weight = val->val1;
            while (k_msgq_put(&data->config_queue, &data->algo_conf, K_NO_WAIT) != 0) {
                k_msgq_purge(&data->config_queue);
            }
            break;
        }
        case SENSOR_ATTR_AGE:
        {
            data->algo_conf.age = val->val1;
            while (k_msgq_put(&data->config_queue, &data->algo_conf, K_NO_WAIT) != 0) {
                k_msgq_purge(&data->config_queue);
            }
            break;
        }
        case SENSOR_ATTR_GENDER:
        {
            data->algo_conf.gender = val->val1;
            while (k_msgq_put(&data->config_queue, &data->algo_conf, K_NO_WAIT) != 0) {
                k_msgq_purge(&data->config_queue);
            }
            break;
        }
        case SENSOR_ATTR_CALIB_VECTOR:
        {
            break;
        }
        case SENSOR_ATTR_SPO2_COEFFS:
        {
            int32_t* coeffs = (int32_t*)val->val1;

            if (max32664c_set_spo2_coeffs(dev, coeffs[0], coeffs[1], coeffs[2])) {
                return -EINVAL;
            }

            break;
        }
        case SENSOR_ATTR_MOTION_TIME:
        {
            data->motion_time = val->val1;
            break;
        }
        case SENSOR_ATTR_MOTION_THRES:
        {
            data->motion_threshold = val->val1;
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

    if (chan != SENSOR_CHAN_HEARTRATE) {
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
        case SENSOR_ATTR_SENSOR_IDS:
        {
            val->val1 = data->afe_id;
            val->val2 = data->accel_id;
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
    uint8_t tx[4];
    uint8_t rx[4];
    const struct max32664c_config *config = dev->config;
    struct max32664c_data *data = dev->data;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C not ready");
        return -ENODEV;
    }

    data->motion_time = config->motion_time;
    data->motion_threshold = config->motion_threshold;

    gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT);
    gpio_pin_configure_dt(&config->mfio_gpio, GPIO_OUTPUT);

    /* Put the hub into application mode */
    LOG_DBG("Set app mode");
    gpio_pin_set_dt(&config->reset_gpio, false);
    k_sleep(K_MSEC(20));

    gpio_pin_set_dt(&config->mfio_gpio, true);
    k_sleep(K_MSEC(20));

    /* Wait for 50 ms (switch into app mode) + 1500 ms (initialization) (see page 17 of the user guide) */
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

    /* Set interrupt threshold */
    tx[0] = 0x10;
    tx[1] = 0x01;
    tx[2] = 0x01;
    if (max32664c_i2c_transmit(dev, tx, 3, rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    if (max32664c_check_sensors(dev)) {
        return -EINVAL;
    }

    #ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
        k_msgq_alloc_init(&data->config_queue, sizeof(max32664c_algo_config_t), CONFIG_MAX32664C_QUEUE_SIZE);
    #else
        k_msgq_init(&data->config_queue, max32664c_algo_config_queue_buffer, sizeof(max32664c_algo_config_t), sizeof(max32664c_algo_config_queue_buffer) / sizeof(max32664c_algo_config_t));
    #endif

    #ifdef CONFIG_MAX32664C_USE_STATIC_MEMORY
        k_msgq_init(&data->raw_queue, max32664c_raw_queue_buffer, sizeof(max32664c_raw_t), sizeof(max32664c_raw_queue_buffer) / sizeof(max32664c_raw_t));
        k_msgq_init(&data->report_queue, max32664c_report_queue_buffer, sizeof(max32664c_report_t), sizeof(max32664c_report_queue_buffer) / sizeof(max32664c_report_t));
    #endif

    /* Run the initial config of the algorithm based on the Device Tree */
    //while (k_msgq_put(&data->config_queue, &config->algo_config, K_NO_WAIT) != 0) {
    //   k_msgq_purge(&data->config_queue);
    //}

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
            break;
        }
        case PM_DEVICE_ACTION_TURN_OFF:
        {
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
        .motion_time = DT_INST_PROP(inst, motion_time),                         \
        .motion_threshold = DT_INST_PROP(inst, motion_threshold),               \
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