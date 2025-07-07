/*
* Copyright (c) 2025, Daniel Kampert
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "max32664c.h"

static uint8_t max32664_buffer[(32 * (sizeof(struct max32664c_raw_t) + sizeof(struct max32664c_report_t))) + 1];

LOG_MODULE_REGISTER(maxim_max32664c_worker, CONFIG_MAXIM_MAX32664C_LOG_LEVEL);

/** @brief          Read the status from the sensor hub.
 *                  NOTE: Table 7. Sensor Hub Status Byte
 *  @param dev      Pointer to device
 *  @param status   Pointer to status byte
 */
static uint8_t max32664c_get_hub_status(const struct device *dev, uint8_t *status)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = 0x00;
    tx[1] = 0x00;
    if (max32664c_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx), MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    *status = rx[1];

    return rx[0];
}

/** @brief      Read the FIFO sample count.
 *  @param dev  Pointer to device
 *  @param fifo Pointer to FIFO count
 */
static int max32664c_get_fifo_count(const struct device *dev, uint8_t *fifo)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = 0x12;
    tx[1] = 0x00;
    if (max32664c_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx), MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    *fifo = rx[1];
    LOG_DBG("FIFO count: %u", *fifo);

    return rx[0];
}

/** @brief              
 *  @param dev          Pointer to device
 *  @param algo_config  Pointer to algorithm configuration
 */
static int max32664c_set_algo_config(const struct device *dev, struct max32664c_algo_config_t *algo_config)
{
    uint8_t tx[5];
    uint8_t rx;

    /* Set the height */
    /* NOTE: Test it if correct */
    tx[0] = 0x50;
    tx[1] = 0x07;
    tx[2] = 0x06;
    tx[3] = (((uint16_t)0xAF) << 8) & 0xFF;
    tx[4] = ((uint16_t)0xAF) & 0xFF;
    if (max32664c_i2c_transmit(dev, tx, 5, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set the weight */
    /* NOTE: Test it if correct */
    tx[0] = 0x50;
    tx[1] = 0x07;
    tx[2] = 0x06;
    tx[3] = (((uint16_t)0x4E) << 8)  & 0xFF;
    tx[4] = ((uint16_t)0x4E) & 0xFF;
    if (max32664c_i2c_transmit(dev, tx, 5, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set the age */
    tx[0] = 0x50;
    tx[1] = 0x07;
    tx[2] = 0x06;
    tx[3] = algo_config->age;
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    /* Set the gender */
    tx[0] = 0x50;
    tx[1] = 0x07;
    tx[2] = 0x06;
    tx[3] = algo_config->gender;
    if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
        return -EINVAL;
    }

    return 0;
}

/** @brief      Worker thread to read the sensor hub.
 *              This thread does the following:
 *                  - It polls the sensor hub periodically for new results
 *                  - If new messages are available it reads the number of samples
 *                  - Then it reads all the samples to clear the FIFO.
 *                    It's neccessary to clear the complete FIFO because the sensor hub 
 *                    doesn´t support the reading of a single message and not clearing 
 *                    the FIFO can cause a FIFO overrun.
 *                  - Extract the message data from the FIRST item from the FIFO and 
 *                    copy them into the right message structure
 *                  - Put the message into a message queue
 *  @param dev  Pointer to device
 */
void max32664c_worker(const struct device *dev)
{
    uint8_t fifo;
    uint8_t status;
    struct max32664c_data *data = dev->data;

    while (data->threadRunning)
    {
        if (max32664c_get_hub_status(dev, &status)) {
            // TODO
        }

        if (status & (1 << MAX32664C_BIT_STATUS_DATA_RDY)) {
            uint8_t tx[2];

            max32664c_get_fifo_count(dev, &fifo);

            if (data->op_mode == MAX32664C_OP_MODE_RAW) {
                struct max32664c_raw_t raw_data;

                /* Get all samples to clear the FIFO */
                tx[0] = 0x12;
                tx[1] = 0x01;
                max32664c_i2c_transmit(dev, tx, 2, max32664_buffer, fifo * sizeof(struct max32664c_raw_t) + 1, MAX32664C_DEFAULT_CMD_DELAY);

                if (max32664_buffer[0] == 0) {
                    raw_data.acc.x = ((int16_t)(max32664_buffer[19]) << 8) | max32664_buffer[20];
                    raw_data.acc.y = ((int16_t)(max32664_buffer[21]) << 8) | max32664_buffer[22];
                    raw_data.acc.z = ((int16_t)(max32664_buffer[23]) << 8) | max32664_buffer[24];

                    while (k_msgq_put(&data->raw_queue, &raw_data, K_NO_WAIT) != 0) {
                        k_msgq_purge(&data->raw_queue);
                    }
                } else {
                    LOG_ERR("Can not read raw data! Status: 0x%X", max32664_buffer[0]);
                }
            } else if ((data->op_mode == MAX32664C_OP_MODE_ALGO_AEC) || (data->op_mode == MAX32664C_OP_MODE_ALGO_AGC)) {
                struct max32664c_raw_t raw_data;
                struct max32664c_report_t report_data;

                /* Get all samples to clear the FIFO */
                tx[0] = 0x12;
                tx[1] = 0x01;
                max32664c_i2c_transmit(dev, tx, 2, max32664_buffer, (fifo * (sizeof(struct max32664c_raw_t) + sizeof(struct max32664c_report_t))) + 1, MAX32664C_DEFAULT_CMD_DELAY);

                if (max32664_buffer[0] == 0) {
                    raw_data.acc.x = ((int16_t)(max32664_buffer[19]) << 8) | max32664_buffer[20];
                    raw_data.acc.y = ((int16_t)(max32664_buffer[21]) << 8) | max32664_buffer[22];
                    raw_data.acc.z = ((int16_t)(max32664_buffer[23]) << 8) | max32664_buffer[24];

                    report_data.op_mode = max32664_buffer[25];
                    report_data.hr = (((uint16_t)(max32664_buffer[26]) << 8) | max32664_buffer[27]) / 10;
                    report_data.hr_confidence = max32664_buffer[28];
                    report_data.rr = (((uint16_t)(max32664_buffer[29]) << 8) | max32664_buffer[30]) / 10;
                    report_data.rr_confidence = max32664_buffer[31];
                    report_data.activity_class = max32664_buffer[32];
                    report_data.r = (((uint16_t)(max32664_buffer[33]) << 8) | max32664_buffer[34]) / 1000;
                    report_data.spo2_confidence = max32664_buffer[35];
                    report_data.spo2 = (((uint16_t)(max32664_buffer[36]) << 8) | max32664_buffer[37]) / 10;
                    report_data.spo2_complete = max32664_buffer[38];
                    report_data.spo2_low_signal_quality = max32664_buffer[39];
                    report_data.spo2_motion = max32664_buffer[40];
                    report_data.spo2_low_pi = max32664_buffer[41];
                    report_data.spo2_unreliable_r = max32664_buffer[42];
                    report_data.spo2_state = max32664_buffer[43];
                    report_data.scd = max32664_buffer[44];

                    while (k_msgq_put(&data->raw_queue, &raw_data, K_NO_WAIT) != 0) {
                        k_msgq_purge(&data->raw_queue);
                    }

                    while (k_msgq_put(&data->report_queue, &report_data, K_NO_WAIT) != 0) {
                        k_msgq_purge(&data->report_queue);
                    }
                } else {
                    LOG_ERR("Can not read report! Status: 0x%X", max32664_buffer[0]);
                }
            }
#if CONFIG_MAX32664C_USE_EXTENDED_REPORTS
            else if ((data->op_mode == MAX32664C_OP_MODE_ALGO_AEC_EXT) || (data->op_mode == MAX32664C_OP_MODE_ALGO_AGC_EXT)) {
                struct max32664c_raw_t raw_data;
                struct max32664c_ext_report_t report_data;

                /* Get all samples to clear the FIFO */
                tx[0] = 0x12;
                tx[1] = 0x01;
                max32664c_i2c_transmit(dev, tx, 2, max32664_buffer, (fifo * (sizeof(struct max32664c_raw_t) + sizeof(struct max32664c_ext_report_t))) + 1, MAX32664C_DEFAULT_CMD_DELAY);

                if (max32664_buffer[0] == 0) {
                } else {
                    LOG_ERR("Can not read report! Status: 0x%X", max32664_buffer[0]);
                }
            }
#endif
        }
        else {
            LOG_WRN("No data ready! Status: 0x%X", max32664_buffer[0]);
        }

        k_msleep(100);
    }
}