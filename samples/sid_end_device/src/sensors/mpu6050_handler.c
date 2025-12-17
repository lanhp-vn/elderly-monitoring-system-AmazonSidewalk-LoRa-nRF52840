/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * MPU6050 IMU Handler Implementation
 * 50Hz sampling with 3-phase fall detection algorithm
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "mpu6050_handler.h"
#include "sensor_manager.h"

LOG_MODULE_REGISTER(mpu6050_handler, LOG_LEVEL_INF);

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/* Device reference from devicetree */
#if DT_NODE_EXISTS(DT_NODELABEL(mpu6050))
#define MPU6050_NODE DT_NODELABEL(mpu6050)
#define MPU6050_AVAILABLE 1
#else
#define MPU6050_AVAILABLE 0
#endif

/* Thread configuration */
#define IMU_THREAD_STACK_SIZE   1536
#define IMU_THREAD_PRIORITY     5       /* Preemptive priority */

/* Sampling configuration */
#define DEFAULT_SAMPLE_RATE_HZ  50
#define MIN_SAMPLE_PERIOD_MS    10      /* Max 100Hz */

/* Fall detection thresholds */
#define ACCEL_FREE_FALL_G       0.35f   /* Weightlessness threshold */
#define ACCEL_IMPACT_G          2.4f    /* Impact threshold */
#define GYRO_IMPACT_RAD_S       4.19f   /* ~240 deg/s rotation threshold */
#define ACCEL_STILL_MIN_G       0.8f    /* Lying still minimum */
#define ACCEL_STILL_MAX_G       1.2f    /* Lying still maximum */
#define GYRO_STILL_RAD_S        0.5f    /* Stillness rotation threshold */

/* Fall detection timing (ms) */
#define FREE_FALL_WINDOW_MS     1000    /* Max free-fall duration */
#define IMPACT_WINDOW_MS        200     /* Impact detection window */
#define POST_FALL_DURATION_MS   5000    /* Post-fall verification timeout */
#define POST_FALL_STILL_SAMPLES 150     /* Samples needed for still (3s @ 50Hz) */
#define FREE_FALL_MIN_DURATION  100     /* Minimum free-fall time */

/* Gravity constant for conversion */
#define GRAVITY_MS2             9.80665f

/* ==========================================================================
 * Private Data
 * ========================================================================== */

#if MPU6050_AVAILABLE
/* Device handle */
static const struct device *mpu6050_dev = DEVICE_DT_GET(MPU6050_NODE);
#endif

/* Thread structures */
K_THREAD_STACK_DEFINE(imu_thread_stack, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;
static k_tid_t imu_thread_id;

/* Synchronization */
static struct k_mutex data_mutex;
static volatile bool running = false;

/* Latest sensor data */
static imu_data_t latest_data;

/* Fall detection state */
static fall_state_t fall_state = FALL_STATE_NORMAL;
static int64_t free_fall_start_time = 0;
static int64_t impact_start_time = 0;
static uint32_t post_impact_still_count = 0;
static volatile bool fall_detected_flag = false;

/* Callbacks */
static imu_data_callback_t data_callback = NULL;
static fall_detected_callback_t fall_callback = NULL;

/* Configuration */
static uint32_t sample_period_ms = 1000 / DEFAULT_SAMPLE_RATE_HZ;

/* Debug logging counter */
static uint32_t sample_count = 0;

/* ==========================================================================
 * Fall Detection Algorithm
 * ========================================================================== */

static void process_fall_detection(const imu_data_t *data)
{
    int64_t now = data->timestamp_ms;
    float accel_g = data->accel_magnitude_g;
    float gyro_rad = data->gyro_magnitude_rad_s;

    switch (fall_state) {
    case FALL_STATE_NORMAL:
        /* Check for free-fall (weightlessness) */
        if (accel_g < ACCEL_FREE_FALL_G) {
            fall_state = FALL_STATE_FREE_FALL;
            free_fall_start_time = now;
            LOG_DBG("Free-fall detected: accel=%.2fg", (double)accel_g);
        }
        break;

    case FALL_STATE_FREE_FALL: {
        int64_t elapsed = now - free_fall_start_time;

        /* Check for impact after free-fall */
        if (accel_g > ACCEL_IMPACT_G && gyro_rad > GYRO_IMPACT_RAD_S) {
            fall_state = FALL_STATE_IMPACT;
            impact_start_time = now;
            post_impact_still_count = 0;
            LOG_WRN("Impact detected: accel=%.2fg, gyro=%.2frad/s",
                    (double)accel_g, (double)gyro_rad);
        }
        /* Timeout: return to normal if no impact */
        else if (elapsed > FREE_FALL_WINDOW_MS) {
            fall_state = FALL_STATE_NORMAL;
            LOG_DBG("Free-fall timeout, returning to normal");
        }
        /* Early recovery: person caught themselves */
        else if (accel_g > ACCEL_STILL_MIN_G && elapsed > FREE_FALL_MIN_DURATION) {
            fall_state = FALL_STATE_NORMAL;
            LOG_DBG("Free-fall recovery detected");
        }
        break;
    }

    case FALL_STATE_IMPACT: {
        int64_t elapsed = now - impact_start_time;

        /* Check if person is lying still */
        bool is_still = (accel_g >= ACCEL_STILL_MIN_G &&
                        accel_g <= ACCEL_STILL_MAX_G &&
                        gyro_rad < GYRO_STILL_RAD_S);

        if (is_still) {
            post_impact_still_count++;

            /* Fall confirmed if still for required duration */
            if (post_impact_still_count >= POST_FALL_STILL_SAMPLES) {
                LOG_ERR("*** FALL CONFIRMED ***");
                fall_detected_flag = true;

                /* Invoke fall callback if registered */
                if (fall_callback) {
                    sensor_snapshot_t snapshot = {0};
                    snapshot.imu = *data;
                    snapshot.fall_state = fall_state;
                    snapshot.fall_detected = true;
                    snapshot.timestamp_ms = now;
                    fall_callback(&snapshot);
                }

                /* Reset state */
                fall_state = FALL_STATE_NORMAL;
                post_impact_still_count = 0;
            }
        } else {
            /* Movement detected - reduce still counter */
            if (post_impact_still_count > 0) {
                post_impact_still_count--;
            }
        }

        /* Timeout: person recovered (got up) */
        if (elapsed > POST_FALL_DURATION_MS) {
            fall_state = FALL_STATE_NORMAL;
            post_impact_still_count = 0;
            LOG_INF("Post-fall timeout, person appears to have recovered");
        }
        break;
    }
    }
}

/* ==========================================================================
 * IMU Thread
 * ========================================================================== */

static void imu_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    imu_data_t data;

    LOG_INF("IMU thread started, rate=%dHz", 1000 / sample_period_ms);

    while (running) {
        int64_t start = k_uptime_get();

#if MPU6050_AVAILABLE
        int ret;
        struct sensor_value accel[3], gyro[3];

        /* Lock I2C bus for exclusive access */
        sensor_manager_i2c_lock();

        /* Fetch sensor data */
        ret = sensor_sample_fetch(mpu6050_dev);
        if (ret != 0) {
            sensor_manager_i2c_unlock();
            LOG_WRN("Failed to fetch MPU6050 sample: %d", ret);
            k_sleep(K_MSEC(sample_period_ms));
            continue;
        }

        /* Get accelerometer data */
        ret = sensor_channel_get(mpu6050_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        if (ret != 0) {
            sensor_manager_i2c_unlock();
            LOG_WRN("Failed to get accel data: %d", ret);
            k_sleep(K_MSEC(sample_period_ms));
            continue;
        }

        /* Get gyroscope data */
        ret = sensor_channel_get(mpu6050_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
        if (ret != 0) {
            sensor_manager_i2c_unlock();
            LOG_WRN("Failed to get gyro data: %d", ret);
            k_sleep(K_MSEC(sample_period_ms));
            continue;
        }

        /* Release I2C bus */
        sensor_manager_i2c_unlock();

        /* Convert to float values */
        data.accel_x = sensor_value_to_float(&accel[0]);
        data.accel_y = sensor_value_to_float(&accel[1]);
        data.accel_z = sensor_value_to_float(&accel[2]);
        data.gyro_x = sensor_value_to_float(&gyro[0]);
        data.gyro_y = sensor_value_to_float(&gyro[1]);
        data.gyro_z = sensor_value_to_float(&gyro[2]);
#else
        /* Simulated data when hardware not available */
        data.accel_x = 0.0f;
        data.accel_y = 0.0f;
        data.accel_z = GRAVITY_MS2;  /* 1g downward */
        data.gyro_x = 0.0f;
        data.gyro_y = 0.0f;
        data.gyro_z = 0.0f;
#endif

        data.timestamp_ms = k_uptime_get();

        /* Calculate magnitudes */
        data.accel_magnitude_g = sqrtf(data.accel_x * data.accel_x +
                                       data.accel_y * data.accel_y +
                                       data.accel_z * data.accel_z) / GRAVITY_MS2;

        data.gyro_magnitude_rad_s = sqrtf(data.gyro_x * data.gyro_x +
                                          data.gyro_y * data.gyro_y +
                                          data.gyro_z * data.gyro_z);

        /* Update shared data */
        k_mutex_lock(&data_mutex, K_FOREVER);
        latest_data = data;
        k_mutex_unlock(&data_mutex);

        /* Process fall detection */
        process_fall_detection(&data);

        /* Invoke data callback if registered */
        if (data_callback) {
            data_callback(&data);
        }

        /* Debug logging (every second) */
        sample_count++;
        if (sample_count % (1000 / sample_period_ms) == 0) {
            LOG_DBG("IMU: accel=%.2fg, gyro=%.2frad/s, state=%d",
                    (double)data.accel_magnitude_g, (double)data.gyro_magnitude_rad_s, fall_state);
        }

        /* Maintain sample rate */
        int64_t elapsed = k_uptime_get() - start;
        if (elapsed < sample_period_ms) {
            k_sleep(K_MSEC(sample_period_ms - elapsed));
        }
    }

    LOG_INF("IMU thread stopped");
}

/* ==========================================================================
 * Public API
 * ========================================================================== */

int mpu6050_handler_init(void)
{
    /* Initialize mutex */
    k_mutex_init(&data_mutex);

    /* Initialize state */
    fall_state = FALL_STATE_NORMAL;
    fall_detected_flag = false;
    sample_count = 0;

#if MPU6050_AVAILABLE
    /* Check device readiness */
    if (!device_is_ready(mpu6050_dev)) {
        LOG_WRN("MPU6050 device not ready - using simulated data");
    } else {
        LOG_INF("MPU6050 device ready");
    }
#else
    LOG_WRN("MPU6050 not available in devicetree - using simulated data");
#endif

    LOG_INF("MPU6050 handler initialized");
    return 0;
}

int mpu6050_handler_start(void)
{
    if (running) {
        LOG_WRN("MPU6050 handler already running");
        return -EALREADY;
    }

    running = true;

    /* Create IMU thread */
    imu_thread_id = k_thread_create(&imu_thread_data, imu_thread_stack,
                                    IMU_THREAD_STACK_SIZE,
                                    imu_thread_entry,
                                    NULL, NULL, NULL,
                                    IMU_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(imu_thread_id, "imu_thread");

    LOG_INF("MPU6050 handler started");
    return 0;
}

int mpu6050_handler_stop(void)
{
    if (!running) {
        return -EALREADY;
    }

    running = false;

    /* Wait for thread to exit */
    k_thread_join(&imu_thread_data, K_MSEC(100));

    LOG_INF("MPU6050 handler stopped");
    return 0;
}

int mpu6050_handler_get_data(imu_data_t *data)
{
    if (!data) {
        return -EINVAL;
    }

    k_mutex_lock(&data_mutex, K_FOREVER);
    *data = latest_data;
    k_mutex_unlock(&data_mutex);

    return 0;
}

fall_state_t mpu6050_handler_get_fall_state(void)
{
    return fall_state;
}

bool mpu6050_handler_check_fall_detected(void)
{
    bool detected = fall_detected_flag;
    fall_detected_flag = false;
    return detected;
}

void mpu6050_handler_set_callback(imu_data_callback_t callback)
{
    data_callback = callback;
}

void mpu6050_handler_set_fall_callback(fall_detected_callback_t callback)
{
    fall_callback = callback;
}

int mpu6050_handler_set_rate(uint32_t rate_hz)
{
    if (rate_hz == 0 || rate_hz > 100) {
        return -EINVAL;
    }

    sample_period_ms = 1000 / rate_hz;
    LOG_INF("IMU sample rate set to %d Hz (period=%d ms)", rate_hz, sample_period_ms);
    return 0;
}
