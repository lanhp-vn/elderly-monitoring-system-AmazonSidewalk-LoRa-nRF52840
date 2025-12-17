/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Sensors Variant Application
 * Health Monitoring System with automatic LoRa switching
 */

#include <app.h>
#include <sidewalk.h>
#include <app_ble_config.h>
#include <app_subGHz_config.h>
#include <sid_hal_reset_ifc.h>
#include <sid_hal_memory_ifc.h>
#if defined(CONFIG_GPIO)
#include <state_notifier/notifier_gpio.h>
#endif
#if defined(CONFIG_LOG)
#include <state_notifier/notifier_log.h>
#endif
#include <buttons.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

#include "sensors/sensor_manager.h"
#include "sensors/critical_msg_buffer.h"
#include "sensors/data_types.h"

LOG_MODULE_REGISTER(sensors_app, CONFIG_SIDEWALK_LOG_LEVEL);

/* ==========================================================================
 * Configuration
 * ========================================================================== */

#define REGULAR_MSG_INTERVAL_MS     CONFIG_SENSORS_REGULAR_INTERVAL_MS
#define LORA_SWITCH_DELAY_MS        CONFIG_SENSORS_LORA_SWITCH_DELAY_MS
#define SPO2_EMERGENCY_THRESHOLD    CONFIG_SENSORS_SPO2_THRESHOLD
#define BPM_LOW_THRESHOLD           CONFIG_SENSORS_BPM_LOW_THRESHOLD
#define BPM_HIGH_THRESHOLD          CONFIG_SENSORS_BPM_HIGH_THRESHOLD
#define VITAL_DEBOUNCE_COUNT        3
#define PAYLOAD_MAX_SIZE            CONFIG_SENSORS_PAYLOAD_MAX_SIZE

/* ==========================================================================
 * Private Data
 * ========================================================================== */

/* Link mask for initial BLE connection */
static uint32_t persistent_link_mask = SID_LINK_TYPE_1;

/* Sidewalk readiness flag */
volatile bool sidewalk_ready = false;

/* LoRa switch state machine */
static struct {
    bool registered;
    bool time_synced;
    bool ble_up;
    bool switch_done;
    bool switch_scheduled;
} lora_state;

/* Vital signs monitoring */
static struct {
    uint8_t consecutive_abnormal;
    int64_t last_alert_time;
} vital_state;

/* Regular report timer */
static struct k_timer regular_report_timer;
static struct k_work regular_report_work;

/* LoRa switch delayed work */
static struct k_work_delayable lora_switch_work;

/* Emergency handling work */
static struct k_work emergency_work;
static struct k_mutex emergency_mutex;
static sensor_snapshot_t emergency_snapshot;
static bool emergency_pending = false;

/* ==========================================================================
 * Forward Declarations
 * ========================================================================== */

static void check_lora_switch_conditions(const struct sid_status *status);
static void on_fall_detected(const sensor_snapshot_t *snapshot);
static void on_heartrate_data(const heartrate_data_t *data);

/* ==========================================================================
 * Payload Formatting
 * ========================================================================== */

static int format_sensor_payload(char *buf, size_t buf_size, char msg_type,
                                 const sensor_snapshot_t *snapshot)
{
    char fall_char;
    int32_t lat_int, lon_int;

    /* Convert fall state to character */
    switch (snapshot->fall_state) {
    case FALL_STATE_FREE_FALL:
        fall_char = 'F';
        break;
    case FALL_STATE_IMPACT:
        fall_char = 'I';
        break;
    default:
        fall_char = 'N';
        break;
    }

    /* GPS coordinates: 0,0 if no fix, otherwise integer * 100000 */
    if (!snapshot->gps.has_fix) {
        lat_int = 0;
        lon_int = 0;
    } else {
        lat_int = (int32_t)(snapshot->gps.latitude * 100000);
        lon_int = (int32_t)(snapshot->gps.longitude * 100000);
    }

    /* Get timestamp in seconds */
    uint32_t ts_seconds = (uint32_t)(k_uptime_get() / 1000);

    /* Battery percentage - stub for now */
    uint8_t battery_pct = 100;

    return snprintf(buf, buf_size,
        "{\"t\":\"%c\",\"d\":%d,\"f\":\"%c\",\"b\":%u,\"s\":%u,\"T\":%d,\"la\":%d,\"lo\":%d,\"B\":%u,\"ts\":%u}",
        msg_type,
        CONFIG_SENSORS_DEVICE_ID,
        fall_char,
        snapshot->heartrate.heart_rate_bpm,    /* 0 if no finger */
        snapshot->heartrate.spo2_percent,      /* 0 if no finger */
        (int)snapshot->heartrate.temp_centidegrees,
        lat_int,
        lon_int,
        battery_pct,
        ts_seconds);
}

/* ==========================================================================
 * Regular Message Sending (Fire-and-Forget)
 * ========================================================================== */

static void send_regular_message(void)
{
    if (!sidewalk_ready) {
        LOG_DBG("Sidewalk not ready, skipping regular message");
        return;
    }

    /* Get current sensor snapshot */
    sensor_snapshot_t snapshot;
    int ret = sensor_manager_get_snapshot(&snapshot);
    if (ret != 0) {
        LOG_ERR("Failed to get sensor snapshot: %d", ret);
        return;
    }

    /* Allocate message structure */
    sidewalk_msg_t *msg = sid_hal_malloc(sizeof(sidewalk_msg_t));
    if (!msg) {
        LOG_ERR("Failed to allocate message structure");
        return;
    }
    memset(msg, 0, sizeof(*msg));

    /* Format payload */
    char payload[PAYLOAD_MAX_SIZE];
    int len = format_sensor_payload(payload, sizeof(payload), 'R', &snapshot);
    if (len <= 0 || len >= sizeof(payload)) {
        LOG_ERR("Failed to format payload");
        sid_hal_free(msg);
        return;
    }

    /* Allocate and copy payload */
    msg->msg.size = len;
    msg->msg.data = sid_hal_malloc(msg->msg.size);
    if (!msg->msg.data) {
        LOG_ERR("Failed to allocate payload buffer");
        sid_hal_free(msg);
        return;
    }
    memcpy(msg->msg.data, payload, msg->msg.size);

    /* Configure message descriptor */
    msg->desc.type = SID_MSG_TYPE_NOTIFY;
    msg->desc.link_type = SID_LINK_TYPE_ANY;
    msg->desc.link_mode = SID_LINK_MODE_CLOUD;

    /* Fire and forget - send the message */
    ret = sidewalk_event_send(SID_EVENT_SEND_MSG, msg);
    if (ret) {
        LOG_ERR("Failed to send regular message: %d", ret);
        sid_hal_free(msg->msg.data);
        sid_hal_free(msg);
    } else {
        LOG_INF("Regular message sent: %s", payload);
        application_state_sending(&global_state_notifier, true);
    }
}

static void regular_report_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    send_regular_message();
}

static void regular_report_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_work_submit(&regular_report_work);
}

/* ==========================================================================
 * LoRa Auto-Switch Logic
 * ========================================================================== */

static void lora_switch_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (lora_state.switch_done) {
        return;
    }

    LOG_INF("=== Switching to BLE + LoRa ===");
    lora_state.switch_done = true;

    /* Send link switch event with BLE + LoRa mask */
    uint32_t new_mask = SID_LINK_TYPE_1 | SID_LINK_TYPE_3;
    sidewalk_event_send(SID_EVENT_LINK_SWITCH, (void *)(uintptr_t)new_mask);
}

static void check_lora_switch_conditions(const struct sid_status *status)
{
#ifdef CONFIG_SENSORS_AUTO_LORA_SWITCH
    /* Update state from status */
    lora_state.registered =
        (status->detail.registration_status == SID_STATUS_REGISTERED);
    lora_state.time_synced =
        (status->detail.time_sync_status == SID_STATUS_TIME_SYNCED);
    lora_state.ble_up =
        (status->detail.link_status_mask & SID_LINK_TYPE_1) != 0;

    /* Check if all conditions are met */
    if (!lora_state.switch_done &&
        !lora_state.switch_scheduled &&
        lora_state.registered &&
        lora_state.time_synced &&
        lora_state.ble_up) {

        LOG_INF("LoRa switch conditions met - scheduling in %d ms",
                LORA_SWITCH_DELAY_MS);
        lora_state.switch_scheduled = true;

        k_work_schedule(&lora_switch_work, K_MSEC(LORA_SWITCH_DELAY_MS));
    }
#endif
}

/* ==========================================================================
 * Emergency and Vital Signs Handling
 * ========================================================================== */

static void emergency_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    sensor_snapshot_t local_snapshot;

    k_mutex_lock(&emergency_mutex, K_FOREVER);
    if (!emergency_pending) {
        k_mutex_unlock(&emergency_mutex);
        return;
    }
    local_snapshot = emergency_snapshot;
    emergency_pending = false;
    k_mutex_unlock(&emergency_mutex);

    /* Enqueue emergency message to critical buffer */
    int ret = critical_buffer_enqueue(MSG_TYPE_EMERGENCY, &local_snapshot);
    if (ret != 0) {
        LOG_ERR("Failed to enqueue emergency message: %d", ret);
    }
}

static void on_fall_detected(const sensor_snapshot_t *snapshot)
{
    LOG_ERR("*** FALL DETECTED - Sending Emergency Alert ***");

    k_mutex_lock(&emergency_mutex, K_FOREVER);
    emergency_snapshot = *snapshot;
    emergency_pending = true;
    k_mutex_unlock(&emergency_mutex);

    k_work_submit(&emergency_work);
}

static void on_heartrate_data(const heartrate_data_t *data)
{
    /* Skip if no valid reading (spo2 == 0 && bpm == 0 means no finger) */
    if (!data->valid) {
        return;
    }

    bool is_abnormal = false;

    /* Check SpO2 */
    if (data->spo2_percent < SPO2_EMERGENCY_THRESHOLD) {
        LOG_WRN("Low SpO2 detected: %u%%", data->spo2_percent);
        is_abnormal = true;
    }

    /* Check heart rate */
    if (data->heart_rate_bpm < BPM_LOW_THRESHOLD) {
        LOG_WRN("Low BPM detected: %u", data->heart_rate_bpm);
        is_abnormal = true;
    } else if (data->heart_rate_bpm > BPM_HIGH_THRESHOLD) {
        LOG_WRN("High BPM detected: %u", data->heart_rate_bpm);
        is_abnormal = true;
    }

    /* Debounce logic to avoid false positives */
    if (is_abnormal) {
        vital_state.consecutive_abnormal++;
        if (vital_state.consecutive_abnormal >= VITAL_DEBOUNCE_COUNT) {
            /* Prevent rapid-fire alerts - min 30 seconds between alerts */
            int64_t now = k_uptime_get();
            if ((now - vital_state.last_alert_time) > 30000) {
                LOG_ERR("*** VITAL SIGNS ABNORMAL - Sending Emergency ***");

                sensor_snapshot_t snapshot;
                sensor_manager_get_snapshot(&snapshot);

                k_mutex_lock(&emergency_mutex, K_FOREVER);
                emergency_snapshot = snapshot;
                emergency_pending = true;
                k_mutex_unlock(&emergency_mutex);

                k_work_submit(&emergency_work);

                vital_state.last_alert_time = now;
            }
            vital_state.consecutive_abnormal = 0;
        }
    } else {
        vital_state.consecutive_abnormal = 0;
    }
}

/* ==========================================================================
 * Sidewalk Callbacks
 * ========================================================================== */

static void on_sidewalk_event(bool in_isr, void *context)
{
    int err = sidewalk_event_send(SID_EVENT_SIDEWALK, NULL);
    if (err) {
        LOG_ERR("Send event err %d", err);
    }
}

static void on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc,
                                     const struct sid_msg *msg, void *context)
{
    LOG_HEXDUMP_INF((uint8_t *)msg->data, msg->size, "Message received");
    application_state_receiving(&global_state_notifier, true);
    application_state_receiving(&global_state_notifier, false);
}

static void on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc, void *context)
{
    LOG_INF("Message sent successfully (id=%u)", msg_desc->id);
    application_state_sending(&global_state_notifier, false);

    /* Check if this is a critical buffer message */
    if (critical_buffer_owns_message(msg_desc->id)) {
        critical_buffer_on_sent(msg_desc->id);
        return;
    }

    /* Regular message - free the buffer */
    sidewalk_msg_t *message = get_message_buffer(msg_desc->id);
    if (message == NULL) {
        LOG_DBG("Message buffer not found for id %u", msg_desc->id);
        return;
    }
    sid_hal_free(message->msg.data);
    sid_hal_free(message);
}

static void on_sidewalk_send_error(sid_error_t error,
                                   const struct sid_msg_desc *msg_desc,
                                   void *context)
{
    LOG_ERR("Message send error %d (id=%u)", (int)error, msg_desc->id);
    application_state_sending(&global_state_notifier, false);

    /* Check if this is a critical buffer message */
    if (critical_buffer_owns_message(msg_desc->id)) {
        critical_buffer_on_error(msg_desc->id);
        return;
    }

    /* Regular message - free the buffer */
    sidewalk_msg_t *message = get_message_buffer(msg_desc->id);
    if (message == NULL) {
        LOG_DBG("Message buffer not found for id %u", msg_desc->id);
        return;
    }
    sid_hal_free(message->msg.data);
    sid_hal_free(message);
}

static void on_sidewalk_factory_reset(void *context)
{
    ARG_UNUSED(context);
    LOG_INF("Factory reset notification received");

    /* Reset LoRa switch state */
    lora_state.switch_done = false;
    lora_state.switch_scheduled = false;

    if (sid_hal_reset(SID_HAL_RESET_NORMAL)) {
        LOG_WRN("Cannot reboot");
    }
}

static void on_sidewalk_status_changed(const struct sid_status *status, void *context)
{
    int err = 0;
    uint32_t new_link_mask = status->detail.link_status_mask;

    struct sid_status *new_status = sid_hal_malloc(sizeof(struct sid_status));
    if (new_status) {
        memcpy(new_status, status, sizeof(struct sid_status));
        err = sidewalk_event_send(SID_EVENT_NEW_STATUS, new_status);
        if (err) {
            sid_hal_free(new_status);
        }
    }

    /* Update sidewalk_ready flag */
    sidewalk_ready = (status->state == SID_STATE_READY ||
                      status->state == SID_STATE_SECURE_CHANNEL_READY);

    /* Trigger pending critical messages when ready */
    if (sidewalk_ready) {
        critical_buffer_trigger_send();
    }

    switch (status->state) {
    case SID_STATE_READY:
    case SID_STATE_SECURE_CHANNEL_READY:
        application_state_connected(&global_state_notifier, true);
        LOG_INF("Sidewalk ready");
        break;
    case SID_STATE_NOT_READY:
        application_state_connected(&global_state_notifier, false);
        LOG_INF("Sidewalk not ready");
        break;
    case SID_STATE_ERROR:
        application_state_error(&global_state_notifier, true);
        LOG_ERR("Sidewalk error state");
        break;
    }

    application_state_registered(&global_state_notifier,
                                 status->detail.registration_status == SID_STATUS_REGISTERED);
    application_state_time_sync(&global_state_notifier,
                                status->detail.time_sync_status == SID_STATUS_TIME_SYNCED);

    LOG_INF("Device %sregistered, Time Sync %s, Link: {BLE:%s, FSK:%s, LoRa:%s}",
            (SID_STATUS_REGISTERED == status->detail.registration_status) ? "" : "not ",
            (SID_STATUS_TIME_SYNCED == status->detail.time_sync_status) ? "OK" : "FAIL",
            (new_link_mask & SID_LINK_TYPE_1) ? "Up" : "Down",
            (new_link_mask & SID_LINK_TYPE_2) ? "Up" : "Down",
            (new_link_mask & SID_LINK_TYPE_3) ? "Up" : "Down");

    /* Check LoRa switch conditions */
    check_lora_switch_conditions(status);

    if (err) {
        LOG_ERR("Send event err %d", err);
    }
}

/* ==========================================================================
 * Button Handlers
 * ========================================================================== */

static void app_button_handler(uint32_t event)
{
    if (event == SID_EVENT_SEND_MSG) {
        /* BTN1 short press: Send Help message */
        LOG_INF("*** HELP BUTTON PRESSED - Sending Help Request ***");

        sensor_snapshot_t snapshot;
        int ret = sensor_manager_get_snapshot(&snapshot);
        if (ret == 0) {
            critical_buffer_enqueue(MSG_TYPE_HELP, &snapshot);
        } else {
            /* Send with NULL snapshot - buffer will get fresh data */
            critical_buffer_enqueue(MSG_TYPE_HELP, NULL);
        }
        return;
    }

    sidewalk_event_send((sidewalk_event_t)event, NULL);
}

static int app_buttons_init(void)
{
    /* BTN1 short: Help request (panic button) */
    button_set_action_short_press(DK_BTN1, app_button_handler, SID_EVENT_SEND_MSG);
    /* BTN1 long: Nordic DFU mode */
    button_set_action_long_press(DK_BTN1, app_button_handler, SID_EVENT_NORDIC_DFU);
    /* BTN2 short: Connect request */
    button_set_action_short_press(DK_BTN2, app_button_handler, SID_EVENT_CONNECT);
    /* BTN2 long: Factory reset */
    button_set_action_long_press(DK_BTN2, app_button_handler, SID_EVENT_FACTORY_RESET);
    /* BTN3: Manual link switch (for testing) */
    button_set_action(DK_BTN3, app_button_handler, SID_EVENT_LINK_SWITCH);

    return buttons_init();
}

/* ==========================================================================
 * Initialization
 * ========================================================================== */

static void sensors_init(void)
{
    int ret;

    /* Initialize sensor manager */
    ret = sensor_manager_init();
    if (ret != 0) {
        LOG_ERR("Sensor manager init failed: %d", ret);
    }

    /* Initialize critical message buffer */
    ret = critical_buffer_init();
    if (ret != 0) {
        LOG_ERR("Critical buffer init failed: %d", ret);
    }

    /* Register callbacks */
    sensor_manager_register_fall_callback(on_fall_detected);
    sensor_manager_register_heartrate_callback(on_heartrate_data);

    /* Start sensor data collection */
    ret = sensor_manager_start();
    if (ret != 0) {
        LOG_ERR("Sensor manager start failed: %d", ret);
    }

    /* Initialize LoRa switch work */
    k_work_init_delayable(&lora_switch_work, lora_switch_work_handler);

    /* Initialize emergency handling */
    k_mutex_init(&emergency_mutex);
    k_work_init(&emergency_work, emergency_work_handler);

    /* Initialize regular report timer (5 minute interval) */
    k_work_init(&regular_report_work, regular_report_work_handler);
    k_timer_init(&regular_report_timer, regular_report_timer_handler, NULL);

    /* Initialize vital state */
    memset(&vital_state, 0, sizeof(vital_state));
    memset(&lora_state, 0, sizeof(lora_state));

    LOG_INF("Sensors variant initialized (interval=%d ms)", REGULAR_MSG_INTERVAL_MS);
}

static void start_regular_reports(void)
{
    /* Start periodic regular reports */
    k_timer_start(&regular_report_timer,
                  K_MSEC(REGULAR_MSG_INTERVAL_MS),
                  K_MSEC(REGULAR_MSG_INTERVAL_MS));

    LOG_INF("Regular reports started (every %d seconds)",
            REGULAR_MSG_INTERVAL_MS / 1000);
}

/* ==========================================================================
 * Public API
 * ========================================================================== */

void app_start(void)
{
    if (app_buttons_init()) {
        LOG_ERR("Cannot init buttons");
    }

#if defined(CONFIG_GPIO)
    state_watch_init_gpio(&global_state_notifier);
#endif
#if defined(CONFIG_LOG)
    state_watch_init_log(&global_state_notifier);
#endif
    application_state_working(&global_state_notifier, true);

    /* Initialize sensors subsystem */
    sensors_init();

    /* Start regular sensor reports */
    start_regular_reports();

    /* Setup Sidewalk context */
    static sidewalk_ctx_t sid_ctx = { 0 };

    static struct sid_event_callbacks event_callbacks = {
        .context = &sid_ctx,
        .on_event = on_sidewalk_event,
        .on_msg_received = on_sidewalk_msg_received,
        .on_msg_sent = on_sidewalk_msg_sent,
        .on_send_error = on_sidewalk_send_error,
        .on_status_changed = on_sidewalk_status_changed,
        .on_factory_reset = on_sidewalk_factory_reset,
    };

    sid_ctx.config = (struct sid_config){
        .link_mask = persistent_link_mask,
        .callbacks = &event_callbacks,
        .link_config = app_get_ble_config(),
        .sub_ghz_link_config = app_get_sub_ghz_config(),
    };

    sidewalk_start(&sid_ctx);
}
