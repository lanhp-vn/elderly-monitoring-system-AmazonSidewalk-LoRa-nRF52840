/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * NEO-6M GPS Handler Implementation
 * Uses Zephyr GNSS subsystem with configurable update interval
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/logging/log.h>

#include "gps_handler.h"

LOG_MODULE_REGISTER(gps_handler, LOG_LEVEL_INF);

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/* GNSS device from devicetree - use conditional compilation */
#if DT_NODE_EXISTS(DT_ALIAS(gnss))
#define GNSS_DEV DEVICE_DT_GET(DT_ALIAS(gnss))
#define GNSS_AVAILABLE 1
#else
#define GNSS_AVAILABLE 0
#endif

/* Default update interval (5 seconds) */
#define DEFAULT_UPDATE_INTERVAL_MS  5000

/* ==========================================================================
 * Private Data
 * ========================================================================== */

/* Synchronization */
static struct k_mutex data_mutex;
static volatile bool running = false;

/* Latest GPS data */
static gps_data_t latest_data;

/* Callback */
static gps_callback_t user_callback = NULL;

/* Rate limiting */
static uint32_t update_interval_ms = DEFAULT_UPDATE_INTERVAL_MS;
static int64_t last_callback_time = 0;

/* ==========================================================================
 * Simulated GPS (for testing without hardware)
 * ========================================================================== */

#ifdef CONFIG_SENSORS_ENABLE_SIMULATED_GPS

static void simulate_gps_data(void)
{
    k_mutex_lock(&data_mutex, K_FOREVER);

    latest_data.has_fix = true;
    latest_data.latitude = (double)CONFIG_SENSORS_SIMULATED_LATITUDE / 100000.0;
    latest_data.longitude = (double)CONFIG_SENSORS_SIMULATED_LONGITUDE / 100000.0;
    latest_data.altitude_mm = 100000;  /* 100m */
    latest_data.speed_mm_s = 0;
    latest_data.bearing_decideg = 0;
    latest_data.satellites = 8;
    latest_data.fix_quality = 1;
    latest_data.timestamp_ms = k_uptime_get();

    k_mutex_unlock(&data_mutex);
}

#endif /* CONFIG_SENSORS_ENABLE_SIMULATED_GPS */

/* ==========================================================================
 * GNSS Data Callback (from Zephyr GNSS subsystem)
 * ========================================================================== */

#if GNSS_AVAILABLE

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
    ARG_UNUSED(dev);

    if (!running) {
        return;
    }

    int64_t now = k_uptime_get();

    /* Rate limiting: only process if interval has elapsed */
    if ((now - last_callback_time) < update_interval_ms) {
        return;
    }

    gps_data_t parsed = {0};

    /* Check fix status */
    if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
        parsed.has_fix = true;

        /* Convert coordinates from nano-degrees to decimal degrees */
        parsed.latitude = (double)data->nav_data.latitude / 1e9;
        parsed.longitude = (double)data->nav_data.longitude / 1e9;

        /* Altitude in millimeters */
        parsed.altitude_mm = data->nav_data.altitude;

        /* Speed in mm/s */
        parsed.speed_mm_s = data->nav_data.speed;

        /* Bearing in 1/10 degree units */
        parsed.bearing_decideg = data->nav_data.bearing;

        /* Satellite info */
        parsed.satellites = data->info.satellites_cnt;
        parsed.fix_quality = data->info.fix_quality;
    } else {
        parsed.has_fix = false;
        parsed.satellites = data->info.satellites_cnt;
    }

    parsed.timestamp_ms = now;

    /* Update shared data */
    k_mutex_lock(&data_mutex, K_FOREVER);
    latest_data = parsed;
    k_mutex_unlock(&data_mutex);

    /* Update last callback time */
    last_callback_time = now;

    /* Invoke user callback if registered */
    if (user_callback) {
        user_callback(&parsed);
    }

    /* Log GPS status */
    if (parsed.has_fix) {
        LOG_INF("GPS: %.6f, %.6f, alt=%dm, sats=%d",
                parsed.latitude, parsed.longitude,
                parsed.altitude_mm / 1000,
                parsed.satellites);
    } else {
        LOG_DBG("GPS: No fix (satellites=%d)", parsed.satellites);
    }
}

/* Register callback with Zephyr GNSS subsystem */
GNSS_DATA_CALLBACK_DEFINE(GNSS_DEV, gnss_data_cb);

#endif /* GNSS_AVAILABLE */

/* ==========================================================================
 * Public API
 * ========================================================================== */

int gps_handler_init(void)
{
    /* Initialize mutex */
    k_mutex_init(&data_mutex);

    /* Initialize state */
    last_callback_time = 0;
    memset(&latest_data, 0, sizeof(latest_data));

#if GNSS_AVAILABLE
    /* Check GNSS device readiness */
    if (!device_is_ready(GNSS_DEV)) {
        LOG_WRN("GNSS device not ready - using simulated GPS");
#ifdef CONFIG_SENSORS_ENABLE_SIMULATED_GPS
        simulate_gps_data();
#endif
    } else {
        LOG_INF("GPS handler initialized with GNSS device");
    }
#else
    LOG_WRN("GNSS not available in devicetree");
#ifdef CONFIG_SENSORS_ENABLE_SIMULATED_GPS
    LOG_INF("Using simulated GPS coordinates");
    simulate_gps_data();
#endif
#endif

    LOG_INF("GPS handler initialized");
    return 0;
}

int gps_handler_start(void)
{
    if (running) {
        LOG_WRN("GPS handler already running");
        return -EALREADY;
    }

    running = true;
    last_callback_time = 0;  /* Allow immediate first update */

#ifdef CONFIG_SENSORS_ENABLE_SIMULATED_GPS
    /* Update simulated data on start */
    simulate_gps_data();
#endif

    LOG_INF("GPS handler started, update interval=%d ms", update_interval_ms);
    return 0;
}

int gps_handler_stop(void)
{
    if (!running) {
        return -EALREADY;
    }

    running = false;

    LOG_INF("GPS handler stopped");
    return 0;
}

int gps_handler_get_data(gps_data_t *data)
{
    if (!data) {
        return -EINVAL;
    }

#ifdef CONFIG_SENSORS_ENABLE_SIMULATED_GPS
    /* Refresh simulated data */
    simulate_gps_data();
#endif

    k_mutex_lock(&data_mutex, K_FOREVER);
    *data = latest_data;
    k_mutex_unlock(&data_mutex);

    return 0;
}

bool gps_handler_has_fix(void)
{
    bool has_fix;

    k_mutex_lock(&data_mutex, K_FOREVER);
    has_fix = latest_data.has_fix;
    k_mutex_unlock(&data_mutex);

    return has_fix;
}

void gps_handler_set_callback(gps_callback_t callback)
{
    user_callback = callback;
}

int gps_handler_set_interval(uint32_t interval_ms)
{
    if (interval_ms < 1000) {
        LOG_WRN("GPS interval too short, minimum is 1000 ms");
        return -EINVAL;
    }

    update_interval_ms = interval_ms;
    LOG_INF("GPS update interval set to %d ms", update_interval_ms);
    return 0;
}
