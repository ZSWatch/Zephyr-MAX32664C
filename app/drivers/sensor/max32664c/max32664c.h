/*
* Copyright (c) 2025, Daniel Kampert
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#include "max32664c_ext.h"

#define SENSOR_ATTR_GENDER                  (SENSOR_ATTR_PRIV_START + 2)
#define SENSOR_ATTR_AGE                     (SENSOR_ATTR_PRIV_START + 3)
#define SENSOR_ATTR_WEIGHT                  (SENSOR_ATTR_PRIV_START + 4)
#define SENSOR_ATTR_HEIGHT                  (SENSOR_ATTR_PRIV_START + 5)
#define SENSOR_ATTR_OP_MODE                 (SENSOR_ATTR_PRIV_START + 9)

#define SENSOR_CHAN_HEARTRATE               (SENSOR_CHAN_PRIV_START + 1)
#define SENSOR_CHAN_BLOOD_OXYGEN_SATURATION (SENSOR_CHAN_PRIV_START + 2)
#define SENSOR_CHAN_RR                      (SENSOR_CHAN_PRIV_START + 3)

#define MAX32664C_BIT_STATUS_COMM_ERR   0
#define MAX32664C_BIT_STATUS_DATA_RDY   3
#define MAX32664C_BIT_STATUS_OUT_OVFL   4
#define MAX32664C_BIT_STATUS_IN_OVFL    5
#define MAX32664C_BIT_STATUS_BUSY       6

#define MAX32664C_DEFAULT_CMD_DELAY     10

/** @brief Output formats of the sensor hub.
 */
enum max32664c_output_format {
    MAX32664C_OUT_PAUSE,
    MAX32664C_OUT_SENSOR_ONLY,
    MAX32664C_OUT_ALGORITHM_ONLY,
    MAX32664C_OUT_ALGO_AND_SENSOR,
};

/** @brief 
 */
enum max32664c_attribute {
    MAX32664C_ATTR_DATE_TIME,
    MAX32664C_ATTR_BP_CAL_SYS,
    MAX32664C_ATTR_BP_CAL,
    MAX32664C_ATTR_START_EST,
    MAX32664C_ATTR_STOP_EST,
    MAX32664C_ATTR_LOAD_CALIB,
};

/** @brief 
 */
enum max32664c_scd_states {
    MAX32664C_SCD_STATE_UNKNOWN,
    MAX32664C_SCD_STATE_OFF_SKIN,
    MAX32664C_SCD_STATE_ON_OBJECT,
    MAX32664C_SCD_STATE_ON_SKIN,
};

/** @brief 
 */
struct max32664c_acc_data_t {
    int16_t x;
    int16_t y;
    int16_t z;
} __attribute__((packed));

/** @brief Raw data structure, reported by the sensor hub.
 */
struct max32664c_raw_t {
    uint32_t PPG1:24;
    uint32_t PPG2:24;
    uint32_t PPG3:24;
    uint32_t PPG4:24;
    uint32_t PPG5:24;
    uint32_t PPG6:24;
    struct max32664c_acc_data_t acc;
} __attribute__((packed));

/** @brief Algorithm data structure, reported by the sensor hub.
 */
struct max32664c_report_t {
    uint8_t op_mode;
    uint16_t hr;
    uint8_t hr_confidence;
    uint16_t rr;
    uint8_t rr_confidence;
    uint8_t activity_class;
    uint16_t r;
    uint8_t spo2_confidence;
    uint16_t spo2;
    uint8_t spo2_complete;
    uint8_t spo2_low_signal_quality;
    uint8_t spo2_motion;
    uint8_t spo2_low_pi;
    uint8_t spo2_unreliable_r;
    uint8_t spo2_state;
    uint8_t scd;
} __attribute__((packed));

/** @brief Extended algorithm data structure, reported by the sensor hub.
 */
struct max32664c_ext_report_t {
} __attribute__((packed));


/** @brief 
 */
struct max32664c_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec reset_gpio;
    struct gpio_dt_spec mfio_gpio;
    int32_t spo2_calib[3];
    uint16_t motion_time;
    uint16_t motion_threshold;
    uint8_t hr_config[2];
    uint8_t spo2_config[2];
    uint8_t init_samp_avg;
    uint8_t min_samp_avg;
    uint8_t max_samp_avg;
    uint8_t init_int_time;
    uint8_t min_int_time;
    uint8_t max_int_time;

    uint8_t led_current[3];                     /**< Initial LED current in mA */
};

/** @brief 
 */
struct max32664c_data {
    struct max32664c_raw_t raw;                 /**<  */
    struct max32664c_report_t report;           /**<  */
    struct max32664c_ext_report_t ext_report;   /**<  */

    enum max32664c_device_mode op_mode;         /**< Current device mode */

    uint8_t motion_time;                        /**< Motion time in milliseconds */
    uint8_t motion_threshold;                   /**< Motion threshold in milli-g */

    uint8_t led_current[3];                     /**< LED current in mA */

    uint8_t afe_id;
    uint8_t accel_id;
    uint8_t hub_ver[3];

    /* Internal */
    struct k_thread thread;
    k_tid_t threadID;
    bool threadRunning;
    bool configChanged;

    struct k_msgq raw_queue;
    struct k_msgq report_queue;
    struct k_msgq ext_report_queue;
};

/** @brief      
 *  @param dev  Pointer to device
 */
void max32664c_worker(const struct device *dev);

/** @brief          Read / write bootloader data from / to the sensor hub.
 *  @param dev      Pointer to device
 *  @param tx_buf   Pointer to transmit buffer
 *  @param tx_len   Length of transmit buffer
 *  @param rx_buf   Pointer to receive buffer
 *                  NOTE: The buffer must be large enough to store the response and the status byte!
 *  @param rx_len   Length of the receive buffer
 *  @param delay    Command delay (milliseconds)
 *  @return         0 when successful
 */
int max32664c_i2c_transmit(const struct device *dev, uint8_t* tx_buf, uint8_t tx_len, uint8_t* rx_buf, uint32_t rx_len, uint16_t delay);