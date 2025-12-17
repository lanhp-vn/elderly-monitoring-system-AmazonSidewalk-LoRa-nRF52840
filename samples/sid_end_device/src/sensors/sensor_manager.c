/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Sensor Manager Implementation
 * Coordinates GPS, IMU, and Heart Rate sensors
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "sensor_manager.h"
#include "gps_handler.h"
#include "mpu6050_handler.h"
#include "sen0344_handler.h"

LOG_MODULE_REGISTER(sensor_manager, LOG_LEVEL_INF);

/* ==========================================================================
 * Private Data
 * ========================================================================== */

/* I2C bus mutex for coordinating sensor access */
static struct k_mutex i2c_mutex;

/* Snapshot mutex */
static struct k_mutex snapshot_mutex;

/* Registered callbacks */
static fall_detected_callback_t fall_callback = NULL;
static heartrate_callback_t heartrate_callback = NULL;

/* Initialization state */
static bool initialized = false;

/* ==========================================================================
 * Internal Callback Wrappers
 * ========================================================================== */

static void internal_fall_callback(const sensor_snapshot_t *snapshot)
{
    LOG_INF("Fall detected by sensor manager");

    if (fall_callback) {
        /* Enrich snapshot with all sensor data */
        sensor_snapshot_t full_snapshot;
        sensor_manager_get_snapshot(&full_snapshot);

        /* Preserve fall detection info */
        full_snapshot.fall_state = snapshot->fall_state;
        full_snapshot.fall_detected = snapshot->fall_detected;

        fall_callback(&full_snapshot);
    }
}

static void internal_heartrate_callback(const heartrate_data_t *data)
{
    if (heartrate_callback) {
        heartrate_callback(data);
    }
}

/* ==========================================================================
 * Public API
 * ========================================================================== */

int sensor_manager_init(void)
{
    int ret;

    if (initialized) {
        LOG_WRN("Sensor manager already initialized");
        return 0;
    }

    LOG_INF("Initializing sensor manager...");

    /* Initialize mutexes */
    k_mutex_init(&i2c_mutex);
    k_mutex_init(&snapshot_mutex);

    /* Initialize GPS handler */
    ret = gps_handler_init();
    if (ret != 0) {
        LOG_ERR("GPS handler init failed: %d", ret);
        /* Continue - GPS may be optional */
    }

    /* Initialize MPU6050 handler */
    ret = mpu6050_handler_init();
    if (ret != 0) {
        LOG_ERR("MPU6050 handler init failed: %d", ret);
        /* Continue - may use simulated data */
    }

    /* Initialize SEN0344 handler */
    ret = sen0344_handler_init();
    if (ret != 0) {
        LOG_ERR("SEN0344 handler init failed: %d", ret);
        /* Continue - may use simulated data */
    }

    /* Register internal callbacks */
    mpu6050_handler_set_fall_callback(internal_fall_callback);
    sen0344_handler_set_callback(internal_heartrate_callback);

    initialized = true;
    LOG_INF("Sensor manager initialized");
    return 0;
}

int sensor_manager_start(void)
{
    int ret;

    if (!initialized) {
        LOG_ERR("Sensor manager not initialized");
        return -EINVAL;
    }

    LOG_INF("Starting sensor data collection...");

    /* Start GPS */
    ret = gps_handler_start();
    if (ret != 0 && ret != -EALREADY) {
        LOG_WRN("GPS start failed: %d", ret);
    }

    /* Start MPU6050 */
    ret = mpu6050_handler_start();
    if (ret != 0 && ret != -EALREADY) {
        LOG_WRN("MPU6050 start failed: %d", ret);
    }

    /* Start SEN0344 */
    ret = sen0344_handler_start();
    if (ret != 0 && ret != -EALREADY) {
        LOG_WRN("SEN0344 start failed: %d", ret);
    }

    LOG_INF("Sensor data collection started");
    return 0;
}

int sensor_manager_stop(void)
{
    if (!initialized) {
        return -EINVAL;
    }

    LOG_INF("Stopping sensor data collection...");

    gps_handler_stop();
    mpu6050_handler_stop();
    sen0344_handler_stop();

    LOG_INF("Sensor data collection stopped");
    return 0;
}

int sensor_manager_get_snapshot(sensor_snapshot_t *snapshot)
{
    if (!snapshot) {
        return -EINVAL;
    }

    if (!initialized) {
        LOG_ERR("Sensor manager not initialized");
        return -EINVAL;
    }

    k_mutex_lock(&snapshot_mutex, K_FOREVER);

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->timestamp_ms = k_uptime_get();

    /* Get GPS data */
    gps_handler_get_data(&snapshot->gps);

    /* Get IMU data */
    mpu6050_handler_get_data(&snapshot->imu);
    snapshot->fall_state = mpu6050_handler_get_fall_state();
    snapshot->fall_detected = mpu6050_handler_check_fall_detected();

    /* Get heart rate data */
    sen0344_handler_get_data(&snapshot->heartrate);

    k_mutex_unlock(&snapshot_mutex);

    return 0;
}

void sensor_manager_register_fall_callback(fall_detected_callback_t callback)
{
    fall_callback = callback;
    LOG_INF("Fall detection callback registered");
}

void sensor_manager_register_heartrate_callback(heartrate_callback_t callback)
{
    heartrate_callback = callback;
    LOG_INF("Heart rate callback registered");
}

void sensor_manager_i2c_lock(void)
{
    k_mutex_lock(&i2c_mutex, K_FOREVER);
}

void sensor_manager_i2c_unlock(void)
{
    k_mutex_unlock(&i2c_mutex);
}
