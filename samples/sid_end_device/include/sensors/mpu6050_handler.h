/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * MPU6050 IMU Handler API with fall detection
 */

#ifndef MPU6050_HANDLER_H
#define MPU6050_HANDLER_H

#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize MPU6050 handler
 * @return 0 on success, negative errno on failure
 */
int mpu6050_handler_init(void);

/**
 * @brief Start IMU data collection and fall detection
 * @return 0 on success, negative errno on failure
 */
int mpu6050_handler_start(void);

/**
 * @brief Stop IMU data collection
 * @return 0 on success, negative errno on failure
 */
int mpu6050_handler_stop(void);

/**
 * @brief Get latest IMU data
 * @param data Pointer to store IMU data
 * @return 0 on success, negative errno on failure
 */
int mpu6050_handler_get_data(imu_data_t *data);

/**
 * @brief Get current fall detection state
 * @return Current fall state
 */
fall_state_t mpu6050_handler_get_fall_state(void);

/**
 * @brief Check and clear fall detected flag
 * @return true if fall was detected since last check
 */
bool mpu6050_handler_check_fall_detected(void);

/**
 * @brief Set callback for IMU data updates
 * @param callback Function to call on IMU data update
 */
void mpu6050_handler_set_callback(imu_data_callback_t callback);

/**
 * @brief Set callback for fall detection events
 * @param callback Function to call when fall is detected
 */
void mpu6050_handler_set_fall_callback(fall_detected_callback_t callback);

/**
 * @brief Set IMU sampling rate
 * @param rate_hz Sampling rate in Hz (max 100)
 * @return 0 on success, negative errno on failure
 */
int mpu6050_handler_set_rate(uint32_t rate_hz);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_HANDLER_H */
