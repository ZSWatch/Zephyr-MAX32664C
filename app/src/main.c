/*
* Copyright (c) 2025, Daniel Kampert
*
* This example demonstrates the use of the MAX32664C sensor hub with Zephyr.
* It includes Bluetooth Low Energy (BLE) services for heart rate and battery
* monitoring.
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

#ifdef CONFIG_APPLICATION_RUN_FW_UPDATE
#include "../drivers/sensor/max32664c/max32664c_bl.h"

#include "firmware/max32664c_kx122_z_32_9_23.h"
#include "firmware/MAX32664C_HSP2_WHRM_AEC_SCD_WSPO2_C_30_13_31.h"
#endif

#include "../drivers/sensor/max32664c/max32664c.h"

#define BLE_CONNECTED               1U
#define BLE_DISCONNECTED            2U

static void auth_cancel(struct bt_conn *conn);
static void hrs_ntf_changed(bool enabled);
static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);

static ATOMIC_DEFINE(state, 2U);
static bool hrf_ntf_enabled;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
                BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
                BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
};

static struct bt_conn_auth_cb auth_cb_display = {
    .cancel = auth_cancel,
};

static struct bt_hrs_cb hrs_cb = {
    .ntf_changed = hrs_ntf_changed,
};

struct bt_le_adv_param adv_param = {
    .options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
    .interval_min = BT_GAP_ADV_SLOW_INT_MIN,
    .interval_max = BT_GAP_ADV_SLOW_INT_MAX,
};

const struct device *const sensor_hub = DEVICE_DT_GET(DT_ALIAS(sensor));
static const struct gpio_dt_spec led_en = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing cancelled: %s", addr);
}

void hrs_ntf_changed(bool enabled)
{
    hrf_ntf_enabled = enabled;

    LOG_INF("HRS notification status changed: %s\n", enabled ? "enabled" : "disabled");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
    } else {
        LOG_INF("Connected");

        atomic_set_bit(state, BLE_CONNECTED);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected, reason 0x%02x %s", reason, bt_hci_err_to_str(reason));

    atomic_set_bit(state, BLE_DISCONNECTED);
}

static void hrs_notify(void)
{
    struct sensor_value value;

    sensor_sample_fetch(sensor_hub);
    sensor_attr_get(sensor_hub, SENSOR_CHAN_HEARTRATE, SENSOR_ATTR_OP_MODE, &value);
    if (value.val1 == MAX32664C_OP_MODE_RAW) {
        struct sensor_value x;
        struct sensor_value y;
        struct sensor_value z;

        LOG_INF("Raw mode");

        sensor_channel_get(sensor_hub, SENSOR_CHAN_ACCEL_X, &x);
        sensor_channel_get(sensor_hub, SENSOR_CHAN_ACCEL_Y, &y);
        sensor_channel_get(sensor_hub, SENSOR_CHAN_ACCEL_Z, &z);

        LOG_INF("\tx: %i", x.val1);
        LOG_INF("\ty: %i", y.val1);
        LOG_INF("\tz: %i", z.val1);
    } else if (value.val1 == MAX32664C_OP_MODE_ALGO_AEC) {
        struct sensor_value hr;

        sensor_channel_get(sensor_hub, SENSOR_CHAN_HEARTRATE, &hr);
        LOG_INF("HR: %u", hr.val1);
        LOG_INF("Confidence: %u", hr.val2);

        if (hrf_ntf_enabled) {
            bt_hrs_notify(66);
        }
    }
}

int main(void)
{
    struct sensor_value value;

    if (!device_is_ready(sensor_hub)) {
        LOG_ERR("Sensor hub not ready!");
    } else {
        LOG_INF("Sensor hub ready");
    }

    if (!gpio_is_ready_dt(&led_en)) {
        return 0;
    }

    if (gpio_pin_configure_dt(&led_en, GPIO_OUTPUT_ACTIVE)) {
        return 0;
    }

    gpio_pin_set_dt(&led_en, 1);

#ifdef CONFIG_APPLICATION_RUN_FW_UPDATE
    max32664c_bl_enter(sensor_hub, MAX32664C_HSP2_WHRM_AEC_SCD_WSPO2_C_30_13_31, sizeof(MAX32664C_HSP2_WHRM_AEC_SCD_WSPO2_C_30_13_31));
    max32664c_bl_leave(sensor_hub);
#endif

    if (bt_enable(NULL)) {
        LOG_ERR("Bluetooth init failed!");
        return 0;
    }

    bt_conn_auth_cb_register(&auth_cb_display);
    bt_hrs_cb_register(&hrs_cb);

    LOG_INF("Starting Legacy Advertising (connectable and scannable)");
    int32_t err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start! %i", err);
        return 0;
    }
    LOG_INF("Advertising successfully started");

    //value.val1 = MAX32664C_OP_MODE_RAW;
    value.val1 = MAX32664C_OP_MODE_ALGO_AEC;
    value.val2 = MAX32664C_ALGO_MODE_CONT_HRM;
    sensor_attr_set(sensor_hub, SENSOR_CHAN_HEARTRATE, SENSOR_ATTR_OP_MODE, &value);

    while (1)
    {
        hrs_notify();

        if (atomic_test_bit(state, BLE_CONNECTED)) {
            LOG_INF("Connected!");
        }

        if (atomic_test_and_clear_bit(state, BLE_CONNECTED)) {
        } else if (atomic_test_and_clear_bit(state, BLE_DISCONNECTED)) {
        }

        k_msleep(1000);
    }

    return 0;
}
