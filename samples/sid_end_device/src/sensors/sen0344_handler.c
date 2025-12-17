/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * SEN0344 Heart Rate and SpO2 Handler Implementation
 * DFRobot MAX30102-based pulse oximeter with custom I2C protocol
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "sen0344_handler.h"
#include "sensor_manager.h"

LOG_MODULE_REGISTER(sen0344_handler, LOG_LEVEL_INF);

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/* I2C device from devicetree */
#if DT_NODE_EXISTS(DT_NODELABEL(sen0344))
#define SEN0344_NODE DT_NODELABEL(sen0344)
#define SEN0344_AVAILABLE 1
#else
#define SEN0344_AVAILABLE 0
#endif

/* SEN0344 I2C Register Addresses (DFRobot protocol) */
#define SEN0344_REG_HR_SPO2     0x0C    /* Heart rate and SpO2 data */
#define SEN0344_REG_TEMP        0x14    /* Temperature data */
#define SEN0344_REG_COLLECT     0x20    /* Start collection command */

/* Default interval */
#define DEFAULT_INTERVAL_MS     5000    /* 5 seconds */

/* ==========================================================================
 * Private Data
 * ========================================================================== */

#if SEN0344_AVAILABLE
/* I2C device specification */
static const struct i2c_dt_spec sen0344_i2c = I2C_DT_SPEC_GET(SEN0344_NODE);
#endif

/* Delayed work item for periodic reads */
static struct k_work_delayable heartrate_work;

/* Synchronization */
static struct k_mutex data_mutex;
static volatile bool running = false;

/* Latest sensor data */
static heartrate_data_t latest_data;

/* Callback */
static heartrate_callback_t data_callback = NULL;

/* Configuration */
static uint32_t read_interval_ms = DEFAULT_INTERVAL_MS;

/* ==========================================================================
 * I2C Communication
 * ========================================================================== */

#if SEN0344_AVAILABLE

static int sen0344_read_register(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_write_read_dt(&sen0344_i2c, &reg, 1, buf, len);
}

static int sen0344_write_register(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[16];

    if (len + 1 > sizeof(buf)) {
        return -ENOMEM;
    }

    buf[0] = reg;
    memcpy(&buf[1], data, len);

    return i2c_write_dt(&sen0344_i2c, buf, len + 1);
}

static int sen0344_start_collection(void)
{
    uint8_t cmd[] = {0x00, 0x01};
    return sen0344_write_register(SEN0344_REG_COLLECT, cmd, sizeof(cmd));
}

static int sen0344_probe(void)
{
    uint8_t buf[8];
    return sen0344_read_register(SEN0344_REG_HR_SPO2, buf, sizeof(buf));
}

#endif /* SEN0344_AVAILABLE */

/* ==========================================================================
 * Work Handler
 * ========================================================================== */

static void heartrate_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!running) {
        return;
    }

    heartrate_data_t data = {0};

#if SEN0344_AVAILABLE
    if (device_is_ready(sen0344_i2c.bus)) {
        uint8_t rbuf[8];
        int ret;

        sensor_manager_i2c_lock();

        /* Read heart rate and SpO2 */
        ret = sen0344_read_register(SEN0344_REG_HR_SPO2, rbuf, sizeof(rbuf));
        if (ret == 0) {
            data.spo2_percent = rbuf[0];
            /* Heart rate is big-endian 32-bit at offset 2 */
            data.heart_rate_bpm = ((uint32_t)rbuf[2] << 24) |
                                  ((uint32_t)rbuf[3] << 16) |
                                  ((uint32_t)rbuf[4] << 8) |
                                  ((uint32_t)rbuf[5]);
            data.valid = (data.spo2_percent > 0 && data.heart_rate_bpm > 0);
        } else {
            LOG_WRN("Failed to read HR/SpO2: %d", ret);
            data.valid = false;
        }

        /* Read temperature */
        uint8_t tbuf[2];
        ret = sen0344_read_register(SEN0344_REG_TEMP, tbuf, sizeof(tbuf));
        if (ret == 0) {
            /* Temperature in centidegrees: whole + fractional */
            data.temp_centidegrees = (tbuf[0] * 100) + tbuf[1];
        } else {
            LOG_WRN("Failed to read temperature: %d", ret);
        }

        sensor_manager_i2c_unlock();
    } else {
        /* Device not ready - return zeros */
        data.spo2_percent = 0;
        data.heart_rate_bpm = 0;
        data.temp_centidegrees = 0;
        data.valid = false;
    }
#else
    /* No hardware - return zeros */
    data.spo2_percent = 0;
    data.heart_rate_bpm = 0;
    data.temp_centidegrees = 0;
    data.valid = false;
#endif

    data.timestamp_ms = k_uptime_get();

    /* Update shared data */
    k_mutex_lock(&data_mutex, K_FOREVER);
    latest_data = data;
    k_mutex_unlock(&data_mutex);

    /* Invoke callback if registered */
    if (data_callback) {
        data_callback(&data);
    }

    /* Log the reading */
    if (data.valid) {
        LOG_INF("Heart Rate: %u BPM, SpO2: %u%%, Temp: %d.%02d C",
                data.heart_rate_bpm, data.spo2_percent,
                data.temp_centidegrees / 100,
                data.temp_centidegrees % 100);
    } else {
        LOG_DBG("Invalid heart rate reading (sensor may need finger contact)");
    }

    /* Reschedule next read */
    if (running) {
        k_work_schedule(&heartrate_work, K_MSEC(read_interval_ms));
    }
}

/* ==========================================================================
 * Public API
 * ========================================================================== */

int sen0344_handler_init(void)
{
    /* Initialize mutex */
    k_mutex_init(&data_mutex);

    /* Initialize work item */
    k_work_init_delayable(&heartrate_work, heartrate_work_handler);

    memset(&latest_data, 0, sizeof(latest_data));

#if SEN0344_AVAILABLE
    /* Check I2C bus readiness */
    if (!device_is_ready(sen0344_i2c.bus)) {
        LOG_WRN("I2C bus not ready for SEN0344 - will use zeros");
        goto init_done;
    }

    /* Probe the sensor */
    sensor_manager_i2c_lock();
    int ret = sen0344_probe();
    sensor_manager_i2c_unlock();

    if (ret != 0) {
        LOG_WRN("Failed to probe SEN0344: %d - will use zeros", ret);
        goto init_done;
    }

    LOG_INF("SEN0344 sensor detected");
#else
    LOG_WRN("SEN0344 not available in devicetree - will use zeros");
#endif

init_done:
    LOG_INF("SEN0344 handler initialized");
    return 0;
}

int sen0344_handler_start(void)
{
    if (running) {
        LOG_WRN("SEN0344 handler already running");
        return -EALREADY;
    }

#if SEN0344_AVAILABLE
    if (device_is_ready(sen0344_i2c.bus)) {
        /* Start data collection mode on sensor */
        sensor_manager_i2c_lock();
        int ret = sen0344_start_collection();
        sensor_manager_i2c_unlock();

        if (ret != 0) {
            LOG_WRN("Failed to start SEN0344 collection: %d", ret);
        }
    }
#endif

    running = true;

    /* Schedule first read after short delay for sensor to stabilize */
    k_work_schedule(&heartrate_work, K_MSEC(500));

    LOG_INF("SEN0344 handler started, interval=%d ms", read_interval_ms);
    return 0;
}

int sen0344_handler_stop(void)
{
    if (!running) {
        return -EALREADY;
    }

    running = false;

    /* Cancel pending work */
    k_work_cancel_delayable(&heartrate_work);

    LOG_INF("SEN0344 handler stopped");
    return 0;
}

int sen0344_handler_get_data(heartrate_data_t *data)
{
    if (!data) {
        return -EINVAL;
    }

    k_mutex_lock(&data_mutex, K_FOREVER);
    *data = latest_data;
    k_mutex_unlock(&data_mutex);

    return 0;
}

void sen0344_handler_set_callback(heartrate_callback_t callback)
{
    data_callback = callback;
}

int sen0344_handler_set_interval(uint32_t interval_ms)
{
    if (interval_ms < 1000) {
        LOG_WRN("Interval too short, minimum is 1000 ms");
        return -EINVAL;
    }

    read_interval_ms = interval_ms;
    LOG_INF("SEN0344 read interval set to %d ms", read_interval_ms);
    return 0;
}
