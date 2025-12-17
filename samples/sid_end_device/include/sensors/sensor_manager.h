/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Sensor Manager API - coordinates all sensors
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize all sensors
 * @return 0 on success, negative errno on failure
 */
int sensor_manager_init(void);

/**
 * @brief Start all sensor data collection
 * @return 0 on success, negative errno on failure
 */
int sensor_manager_start(void);

/**
 * @brief Stop all sensor data collection
 * @return 0 on success, negative errno on failure
 */
int sensor_manager_stop(void);

/**
 * @brief Get snapshot of all sensor data
 * @param snapshot Pointer to store snapshot
 * @return 0 on success, negative errno on failure
 */
int sensor_manager_get_snapshot(sensor_snapshot_t *snapshot);

/**
 * @brief Register callback for fall detection events
 * @param callback Function to call when fall is detected
 */
void sensor_manager_register_fall_callback(fall_detected_callback_t callback);

/**
 * @brief Register callback for heart rate data updates
 * @param callback Function to call on heart rate update
 */
void sensor_manager_register_heartrate_callback(heartrate_callback_t callback);

/**
 * @brief Lock I2C bus for exclusive access
 * Used by sensor handlers to coordinate I2C access
 */
void sensor_manager_i2c_lock(void);

/**
 * @brief Unlock I2C bus
 */
void sensor_manager_i2c_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H */
