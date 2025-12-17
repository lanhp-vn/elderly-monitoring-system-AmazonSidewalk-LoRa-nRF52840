/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * SEN0344 Heart Rate and SpO2 Sensor Handler API
 */

#ifndef SEN0344_HANDLER_H
#define SEN0344_HANDLER_H

#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SEN0344 handler
 * @return 0 on success, negative errno on failure
 */
int sen0344_handler_init(void);

/**
 * @brief Start heart rate/SpO2 data collection
 * @return 0 on success, negative errno on failure
 */
int sen0344_handler_start(void);

/**
 * @brief Stop heart rate/SpO2 data collection
 * @return 0 on success, negative errno on failure
 */
int sen0344_handler_stop(void);

/**
 * @brief Get latest heart rate data
 * @param data Pointer to store heart rate data
 * @return 0 on success, negative errno on failure
 */
int sen0344_handler_get_data(heartrate_data_t *data);

/**
 * @brief Set callback for heart rate data updates
 * @param callback Function to call on data update
 */
void sen0344_handler_set_callback(heartrate_callback_t callback);

/**
 * @brief Set measurement interval
 * @param interval_ms Measurement interval in milliseconds
 * @return 0 on success, negative errno on failure
 */
int sen0344_handler_set_interval(uint32_t interval_ms);

#ifdef __cplusplus
}
#endif

#endif /* SEN0344_HANDLER_H */
