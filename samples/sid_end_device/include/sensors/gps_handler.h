/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * GPS Handler API for NEO-6M module
 */

#ifndef GPS_HANDLER_H
#define GPS_HANDLER_H

#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize GPS handler
 * @return 0 on success, negative errno on failure
 */
int gps_handler_init(void);

/**
 * @brief Start GPS data collection
 * @return 0 on success, negative errno on failure
 */
int gps_handler_start(void);

/**
 * @brief Stop GPS data collection
 * @return 0 on success, negative errno on failure
 */
int gps_handler_stop(void);

/**
 * @brief Get latest GPS data
 * @param data Pointer to store GPS data
 * @return 0 on success, negative errno on failure
 */
int gps_handler_get_data(gps_data_t *data);

/**
 * @brief Check if GPS has a valid fix
 * @return true if GPS has fix, false otherwise
 */
bool gps_handler_has_fix(void);

/**
 * @brief Set callback for GPS data updates
 * @param callback Function to call on GPS data update
 */
void gps_handler_set_callback(gps_callback_t callback);

/**
 * @brief Set GPS update interval
 * @param interval_ms Update interval in milliseconds
 * @return 0 on success, negative errno on failure
 */
int gps_handler_set_interval(uint32_t interval_ms);

#ifdef __cplusplus
}
#endif

#endif /* GPS_HANDLER_H */
