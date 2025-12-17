/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Shared data types for sensors variant
 */

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * GPS Data Types
 * ========================================================================== */

typedef struct {
    bool has_fix;
    double latitude;
    double longitude;
    int32_t altitude_mm;
    uint32_t speed_mm_s;
    uint16_t bearing_decideg;
    uint8_t satellites;
    uint8_t fix_quality;
    int64_t timestamp_ms;
} gps_data_t;

/* Callback type for GPS data updates */
typedef void (*gps_callback_t)(const gps_data_t *data);

/* ==========================================================================
 * IMU Data Types
 * ========================================================================== */

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_magnitude_g;
    float gyro_magnitude_rad_s;
    int64_t timestamp_ms;
} imu_data_t;

/* Fall detection states */
typedef enum {
    FALL_STATE_NORMAL = 0,
    FALL_STATE_FREE_FALL,
    FALL_STATE_IMPACT
} fall_state_t;

/* Callback type for IMU data updates */
typedef void (*imu_data_callback_t)(const imu_data_t *data);

/* ==========================================================================
 * Heart Rate / SpO2 Data Types
 * ========================================================================== */

typedef struct {
    uint8_t spo2_percent;        /* Blood oxygen saturation (0-100%), 0 = no finger */
    uint32_t heart_rate_bpm;     /* Heart rate in BPM, 0 = no finger */
    int16_t temp_centidegrees;   /* Temperature in centidegrees C */
    bool valid;                  /* True if reading is valid (spo2 > 0 && bpm > 0) */
    int64_t timestamp_ms;
} heartrate_data_t;

/* Callback type for heart rate data updates */
typedef void (*heartrate_callback_t)(const heartrate_data_t *data);

/* ==========================================================================
 * Combined Sensor Snapshot
 * ========================================================================== */

typedef struct {
    gps_data_t gps;
    imu_data_t imu;
    heartrate_data_t heartrate;
    fall_state_t fall_state;
    bool fall_detected;
    int64_t timestamp_ms;
} sensor_snapshot_t;

/* Forward declaration for fall detection callback */
typedef void (*fall_detected_callback_t)(const sensor_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* DATA_TYPES_H */
