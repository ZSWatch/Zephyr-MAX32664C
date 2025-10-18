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
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

#include <zephyr/settings/settings.h>

#include "sensor/max32664c.h"

#define LOG_DATA_FOR_PLOTTING 0

#ifdef CONFIG_MAX32664C_USE_FIRMWARE_LOADER
#define FW_VERSION_MAJOR 30
#define FW_VERSION_MINOR 13
#define FW_VERSION_PATCH 31
#include "firmware/MAX32664C_HSP2_WHRM_AEC_SCD_WSPO2_C_30_13_31.h"
#endif

#define BLE_CONNECTED               1U
#define BLE_DISCONNECTED            2U

static void auth_cancel(struct bt_conn *conn);
static void hrs_ntf_changed(bool enabled);
static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static void recycled(void);
static int mtu_exchange(struct bt_conn *conn);

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
    .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_NAME,
    .interval_min = BT_GAP_ADV_SLOW_INT_MIN,
    .interval_max = BT_GAP_ADV_SLOW_INT_MAX,
};

static const struct device *const sensor_hub = DEVICE_DT_GET_OR_NULL(DT_ALIAS(sensor));
static const struct gpio_dt_spec led_en = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static bool button_last_state = false;

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .recycled = recycled
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
        mtu_exchange(conn);
        atomic_set_bit(state, BLE_CONNECTED);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected, reason 0x%02x %s", reason, bt_hci_err_to_str(reason));

    atomic_set_bit(state, BLE_DISCONNECTED);
}

static void recycled(void)
{
    LOG_INF("Connection recycled");
    LOG_INF("Starting Legacy Advertising (connectable and scannable)");
    int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start! %i", err);
        return;
    }
    LOG_INF("Advertising successfully started");
}

static void hrs_notify(void)
{
    struct sensor_value value;

    sensor_sample_fetch(sensor_hub);
    sensor_attr_get(sensor_hub, SENSOR_CHAN_MAX32664C_HEARTRATE, SENSOR_ATTR_MAX32664C_OP_MODE, &value);
    if ((value.val1 == MAX32664C_OP_MODE_RAW) || (value.val1 == MAX32664C_OP_MODE_ALGO_AEC_EXT) || (value.val1 == MAX32664C_OP_MODE_ALGO_AGC_EXT)) {
        struct sensor_value x;
        struct sensor_value y;
        struct sensor_value z;
        struct sensor_value green;
        struct sensor_value ir;
        struct sensor_value red;

        sensor_channel_get(sensor_hub, SENSOR_CHAN_GREEN, &green);
        sensor_channel_get(sensor_hub, SENSOR_CHAN_IR, &ir);
        sensor_channel_get(sensor_hub, SENSOR_CHAN_RED, &red);
        sensor_channel_get(sensor_hub, SENSOR_CHAN_ACCEL_X, &x);
        sensor_channel_get(sensor_hub, SENSOR_CHAN_ACCEL_Y, &y);
        sensor_channel_get(sensor_hub, SENSOR_CHAN_ACCEL_Z, &z);

#if LOG_DATA_FOR_PLOTTING
    // Output format:
    // GREEN1,<value>,;GREEN2,<value>,;IR1,<value>,;IR2,<value>,;RED1,<value>,;RED2,<value>,;X,<value>,;Y,<value>,;Z,<value>,
    LOG_PRINTK(
        "GREEN1,%i,;GREEN2,%i,;IR1,%i,;IR2,%i,;RED1,%i,;RED2,%i,;X,%i,;Y,%i,;Z,%i;",
        green.val1, green.val2,
        ir.val1, ir.val2,
        red.val1, red.val2,
        x.val1, y.val1, z.val1
    );
#else
    LOG_DBG("\tGreen: %i", green.val1);
    LOG_DBG("\tIR: %i", ir.val1);
    LOG_DBG("\tRed: %i", red.val1);
    LOG_INF("\tx: %i", x.val1);
    LOG_INF("\ty: %i", y.val1);
    LOG_INF("\tz: %i", z.val1);
#endif
    }

    if ((value.val1 == MAX32664C_OP_MODE_ALGO_AEC) || (value.val1 == MAX32664C_OP_MODE_ALGO_AGC)) {
        struct sensor_value hr;
        struct sensor_value blood_oxygen;

        sensor_channel_get(sensor_hub, SENSOR_CHAN_MAX32664C_HEARTRATE, &hr);
        sensor_channel_get(sensor_hub, SENSOR_CHAN_MAX32664C_BLOOD_OXYGEN_SATURATION, &blood_oxygen);

        LOG_INF("HR: %u bpm (Confidence: %u)", hr.val1, hr.val2);
        LOG_INF("SpO2: %u bpm (Confidence: %u)", blood_oxygen.val1, blood_oxygen.val2);

        if (hrf_ntf_enabled) {
            bt_hrs_notify(hr.val1);
        }
    }

    if ((value.val1 == MAX32664C_OP_MODE_ALGO_AEC_EXT) || (value.val1 == MAX32664C_OP_MODE_ALGO_AGC_EXT)) {
        struct sensor_value hr;
        struct sensor_value rr;
        struct sensor_value skin_contact;
        struct sensor_value activity;
        struct sensor_value blood_oxygen;

        sensor_channel_get(sensor_hub, SENSOR_CHAN_MAX32664C_HEARTRATE, &hr);
        sensor_channel_get(sensor_hub, SENSOR_CHAN_MAX32664C_RESPIRATION_RATE, &rr);
        sensor_channel_get(sensor_hub, SENSOR_CHAN_MAX32664C_SKIN_CONTACT, &skin_contact);
        sensor_channel_get(sensor_hub, SENSOR_CHAN_MAX32664C_ACTIVITY, &activity);
        sensor_channel_get(sensor_hub, SENSOR_CHAN_MAX32664C_BLOOD_OXYGEN_SATURATION, &blood_oxygen);
#ifdef LOG_DATA_FOR_PLOTTING
        //   Output format
        //  HR,<value>,bpm;Conf,<value>;,RR,<value>;ms,SC,<value>;,Activity,<value>;,SpO2,<value>;%,Conf,<value>;,
        LOG_PRINTK("HR,%u,bpm;"
                "HR_Conf,%u,;"
                "RR,%u,ms;"
                "RR_Conf,%u,;"
                "SC,%u,;"
                "Activity,%u,;"
                "SpO2,%u,%%;"
                "SpO2_Conf,%u;",
            hr.val1, hr.val2, rr.val1, rr.val2, skin_contact.val1, activity.val1, blood_oxygen.val1, blood_oxygen.val2);    
#else
        LOG_INF("HR: %u bpm (Confidence: %u)", hr.val1, hr.val2);
        LOG_INF("SpO2: %u bpm (Confidence: %u)", blood_oxygen.val1, blood_oxygen.val2);
#endif

        if (hrf_ntf_enabled) {
            bt_hrs_notify(hr.val1);
        }
    }

#ifdef LOG_DATA_FOR_PLOTTING
    LOG_PRINTK("\n");
#endif
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	printk("%s: MTU exchange %s (%u)\n", __func__,
	       err == 0U ? "successful" : "failed",
	       bt_gatt_get_mtu(conn));
}

static struct bt_gatt_exchange_params mtu_exchange_params = {
	.func = mtu_exchange_cb
};

static int mtu_exchange(struct bt_conn *conn)
{
	int err;

	printk("%s: Current MTU = %u\n", __func__, bt_gatt_get_mtu(conn));

	printk("%s: Exchange MTU...\n", __func__);
	err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
	if (err) {
		printk("%s: MTU exchange failed (err %d)", __func__, err);
	}

	return err;
}

static void check_button(void)
{
    bool current_state = gpio_pin_get_dt(&button) == 0;

    if (current_state && !button_last_state) {
        LOG_INF("Button pressed!");
    }

    button_last_state = current_state;
}

int main(void)
{
    int32_t err;
    struct sensor_value value;

    if (!device_is_ready(sensor_hub)) {
        LOG_ERR("Sensor hub not ready!");
    } else {
        LOG_INF("Sensor hub ready");
    }

    if (!gpio_is_ready_dt(&led_en)) {
        return 0;
    }

    if (!gpio_is_ready_dt(&led_green)) {
        return 0;
    }

    if (gpio_pin_configure_dt(&led_en, GPIO_OUTPUT_ACTIVE)) {
        return 0;
    }

    if (gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_ACTIVE)) {
        return 0;
    }

    gpio_pin_set_dt(&led_en, 1);
    gpio_pin_set_dt(&led_green, 0);

    // Configure button
    if (!gpio_is_ready_dt(&button)) {
        LOG_ERR("Button device not ready!");
        return 0;
    }

    err = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (err) {
        LOG_ERR("Failed to configure button pin: %d", err);
        return 0;
    }

    LOG_INF("Button configured on P2.03 (polling mode)");

#ifdef CONFIG_MAX32664C_USE_FIRMWARE_LOADER
    uint8_t major, minor, patch;
    err = max32664c_read_fw_version(sensor_hub, &major, &minor, &patch);
    if (err || major != FW_VERSION_MAJOR || minor != FW_VERSION_MINOR || patch != FW_VERSION_PATCH) {
        if (err) {
            LOG_ERR("Failed to read firmware version");
        }
        LOG_DBG("Updating firmware to version %u.%u.%u", FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
        max32664c_bl_enter(sensor_hub, MAX32664C_HSP2_WHRM_AEC_SCD_WSPO2_C_30_13_31, sizeof(MAX32664C_HSP2_WHRM_AEC_SCD_WSPO2_C_30_13_31));
        max32664c_bl_leave(sensor_hub);

        return 0;
    } else {
        LOG_INF("Firmware up to date: %u.%u.%u", major, minor, patch);
    }
#endif /* CONFIG_MAX32664C_USE_FIRMWARE_LOADER */

    if (bt_enable(NULL)) {
        LOG_ERR("Bluetooth init failed!");
        return 0;
    }

    settings_load();

    if (gpio_pin_get_dt(&button) == 1) {
        LOG_WRN("Button pressed during startup - clearing all bonded devices!");

        err = bt_unpair(BT_ID_DEFAULT, NULL);
        if (err) {
            LOG_ERR("Failed to clear bonded devices: %d", err);
        } else {
            LOG_INF("All bonded devices cleared successfully!");
        }
    }

    bt_conn_auth_cb_register(&auth_cb_display);
    bt_hrs_cb_register(&hrs_cb);

    LOG_INF("Starting Legacy Advertising (connectable and scannable)");
    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start! %i", err);
        return 0;
    }
    LOG_INF("Advertising successfully started");

    //value.val1 = MAX32664C_OP_MODE_RAW;
#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
    value.val1 = MAX32664C_OP_MODE_ALGO_AEC_EXT;
#else
    value.val1 = MAX32664C_OP_MODE_ALGO_AEC;
#endif
    value.val2 = MAX32664C_ALGO_MODE_CONT_HR_CONT_SPO2;
    sensor_attr_set(sensor_hub, SENSOR_CHAN_MAX32664C_HEARTRATE, SENSOR_ATTR_MAX32664C_OP_MODE, &value);

    int i = 0;
    while (1)
    {
        // Poll button every 100ms
        check_button();

        // Send heart rate notification every second (every 10 iterations)
        if (i % 10 == 0) {
            hrs_notify();
        }

        if (atomic_test_bit(state, BLE_CONNECTED)) {
            LOG_INF("Connected!");
        }

        if (atomic_test_and_clear_bit(state, BLE_CONNECTED)) {
        } else if (atomic_test_and_clear_bit(state, BLE_DISCONNECTED)) {
        }

        // Blink LED every 5 seconds (every 50 iterations)
        if (i % 50 == 0) {
            gpio_pin_toggle_dt(&led_green);
            k_msleep(100);
            gpio_pin_toggle_dt(&led_green);
        }
        i++;

        k_msleep(100);
    }

    return 0;
}
