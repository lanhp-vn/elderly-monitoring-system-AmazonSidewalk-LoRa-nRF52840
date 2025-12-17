/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Critical Message Ring Buffer Implementation
 * Manages Emergency and Help messages with tracking and retries
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <sid_hal_memory_ifc.h>
#include <sidewalk.h>
#include <stdio.h>

#include "critical_msg_buffer.h"
#include "sensor_manager.h"

LOG_MODULE_REGISTER(critical_buffer, LOG_LEVEL_INF);

/* Configuration from Kconfig */
#define BUFFER_SIZE         CONFIG_SENSORS_CRITICAL_BUFFER_SIZE
#define MSG_TTL_MS          (CONFIG_SENSORS_CRITICAL_MSG_TTL * 1000)
#define MAX_RETRIES         CONFIG_SENSORS_CRITICAL_MSG_RETRIES
#define RETRY_DELAY_MS      5000
#define PAYLOAD_MAX_SIZE    CONFIG_SENSORS_PAYLOAD_MAX_SIZE

/* Internal slot structure */
typedef struct {
    msg_slot_state_t state;
    message_type_t type;
    uint16_t sidewalk_msg_id;
    uint8_t retry_count;
    int64_t created_time_ms;
    sensor_snapshot_t snapshot;
    char payload[PAYLOAD_MAX_SIZE];
    uint16_t payload_len;
} critical_msg_slot_t;

/* Buffer state */
static struct {
    critical_msg_slot_t slots[BUFFER_SIZE];
    struct k_mutex mutex;
    struct k_work send_work;
    struct k_work_delayable retry_work;
    bool initialized;
    uint8_t active_count;
} buffer;

/* External dependencies */
extern volatile bool sidewalk_ready;

/* Forward declarations */
static int find_free_slot(void);
static int find_oldest_slot(void);
static int format_critical_payload(char *buf, size_t buf_size, message_type_t type,
                                   const sensor_snapshot_t *snapshot);
static int send_slot_message(int slot_idx);

/* ==========================================================================
 * Payload Formatting
 * ========================================================================== */

static int format_critical_payload(char *buf, size_t buf_size, message_type_t type,
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

    /* GPS coordinates: -1 if no fix, otherwise integer * 100000 */
    if (!snapshot->gps.has_fix) {
        lat_int = -1;
        lon_int = -1;
    } else {
        lat_int = (int32_t)(snapshot->gps.latitude * 100000);
        lon_int = (int32_t)(snapshot->gps.longitude * 100000);
    }

    /* Get timestamp in seconds */
    uint32_t ts_seconds = (uint32_t)(k_uptime_get() / 1000);

    /* Battery percentage - stub for now (100%) */
    uint8_t battery_pct = 100;

    return snprintf(buf, buf_size,
        "{\"t\":\"%c\",\"d\":%d,\"f\":\"%c\",\"b\":%u,\"s\":%u,\"T\":%d,\"la\":%d,\"lo\":%d,\"B\":%u,\"ts\":%u}",
        (char)type,
        CONFIG_SENSORS_DEVICE_ID,
        fall_char,
        snapshot->heartrate.heart_rate_bpm,
        snapshot->heartrate.spo2_percent,
        snapshot->heartrate.temp_centidegrees,
        lat_int,
        lon_int,
        battery_pct,
        ts_seconds);
}

/* ==========================================================================
 * Slot Management
 * ========================================================================== */

static int find_free_slot(void)
{
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (buffer.slots[i].state == MSG_SLOT_FREE) {
            return i;
        }
    }
    return -1;
}

static int find_oldest_slot(void)
{
    int oldest_idx = -1;
    int64_t oldest_time = INT64_MAX;

    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (buffer.slots[i].state != MSG_SLOT_FREE &&
            buffer.slots[i].created_time_ms < oldest_time) {
            oldest_time = buffer.slots[i].created_time_ms;
            oldest_idx = i;
        }
    }
    return oldest_idx;
}

static int find_slot_by_msg_id(uint16_t msg_id)
{
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (buffer.slots[i].state == MSG_SLOT_SENT &&
            buffer.slots[i].sidewalk_msg_id == msg_id) {
            return i;
        }
    }
    return -1;
}

/* ==========================================================================
 * Message Sending
 * ========================================================================== */

static int send_slot_message(int slot_idx)
{
    critical_msg_slot_t *slot = &buffer.slots[slot_idx];

    if (!sidewalk_ready) {
        LOG_DBG("Sidewalk not ready, deferring send");
        return -EAGAIN;
    }

    /* Allocate sidewalk message structure */
    sidewalk_msg_t *msg = sid_hal_malloc(sizeof(sidewalk_msg_t));
    if (!msg) {
        LOG_ERR("Failed to allocate sidewalk_msg_t");
        return -ENOMEM;
    }
    memset(msg, 0, sizeof(*msg));

    /* Allocate and copy payload */
    msg->msg.size = slot->payload_len;
    msg->msg.data = sid_hal_malloc(msg->msg.size);
    if (!msg->msg.data) {
        LOG_ERR("Failed to allocate payload buffer");
        sid_hal_free(msg);
        return -ENOMEM;
    }
    memcpy(msg->msg.data, slot->payload, msg->msg.size);

    /* Configure message descriptor */
    msg->desc.type = SID_MSG_TYPE_NOTIFY;
    msg->desc.link_type = SID_LINK_TYPE_ANY;
    msg->desc.link_mode = SID_LINK_MODE_CLOUD;

    /* Send via sidewalk event */
    int err = sidewalk_event_send(SID_EVENT_SEND_MSG, msg);
    if (err) {
        LOG_ERR("Failed to send critical message: %d", err);
        sid_hal_free(msg->msg.data);
        sid_hal_free(msg);
        return err;
    }

    /* Update slot state - message ID will be updated when we can track it */
    slot->state = MSG_SLOT_SENT;
    slot->sidewalk_msg_id = msg->desc.id;

    LOG_INF("Critical message %c sent (slot=%d, id=%u, retry=%d)",
            (char)slot->type, slot_idx, msg->desc.id, slot->retry_count);

    return 0;
}

/* ==========================================================================
 * Work Handlers
 * ========================================================================== */

static void send_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&buffer.mutex, K_FOREVER);

    int64_t now = k_uptime_get();

    for (int i = 0; i < BUFFER_SIZE; i++) {
        critical_msg_slot_t *slot = &buffer.slots[i];

        if (slot->state == MSG_SLOT_FREE) {
            continue;
        }

        /* Check TTL */
        if ((now - slot->created_time_ms) > MSG_TTL_MS) {
            LOG_WRN("Critical message TTL expired (slot=%d, type=%c)",
                    i, (char)slot->type);
            slot->state = MSG_SLOT_FREE;
            buffer.active_count--;
            continue;
        }

        /* Send pending or retry messages */
        if (slot->state == MSG_SLOT_PENDING || slot->state == MSG_SLOT_RETRY) {
            int err = send_slot_message(i);
            if (err == -EAGAIN) {
                /* Sidewalk not ready, will retry later */
                break;
            }
            /* Other errors: slot stays in current state for retry */
        }
    }

    k_mutex_unlock(&buffer.mutex);
}

static void retry_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    /* Trigger send work to process retries */
    k_work_submit(&buffer.send_work);
}

/* ==========================================================================
 * Public API
 * ========================================================================== */

int critical_buffer_init(void)
{
    if (buffer.initialized) {
        return 0;
    }

    k_mutex_init(&buffer.mutex);
    k_work_init(&buffer.send_work, send_work_handler);
    k_work_init_delayable(&buffer.retry_work, retry_work_handler);

    /* Clear all slots */
    memset(buffer.slots, 0, sizeof(buffer.slots));
    buffer.active_count = 0;
    buffer.initialized = true;

    LOG_INF("Critical message buffer initialized (size=%d, TTL=%ds, retries=%d)",
            BUFFER_SIZE, CONFIG_SENSORS_CRITICAL_MSG_TTL, MAX_RETRIES);

    return 0;
}

int critical_buffer_enqueue(message_type_t type, const sensor_snapshot_t *snapshot)
{
    if (!buffer.initialized) {
        LOG_ERR("Buffer not initialized");
        return -EINVAL;
    }

    sensor_snapshot_t local_snapshot;

    /* Capture current snapshot if none provided */
    if (snapshot == NULL) {
        int ret = sensor_manager_get_snapshot(&local_snapshot);
        if (ret != 0) {
            LOG_ERR("Failed to get sensor snapshot: %d", ret);
            return ret;
        }
        snapshot = &local_snapshot;
    }

    k_mutex_lock(&buffer.mutex, K_FOREVER);

    /* Find a free slot or evict oldest */
    int slot_idx = find_free_slot();
    if (slot_idx < 0) {
        /* Buffer full - evict oldest */
        slot_idx = find_oldest_slot();
        if (slot_idx < 0) {
            k_mutex_unlock(&buffer.mutex);
            LOG_ERR("No slot available");
            return -ENOMEM;
        }
        LOG_WRN("Buffer full - evicting oldest message (slot=%d)", slot_idx);
        buffer.active_count--;
    }

    critical_msg_slot_t *slot = &buffer.slots[slot_idx];

    /* Fill the slot */
    slot->state = MSG_SLOT_PENDING;
    slot->type = type;
    slot->retry_count = 0;
    slot->created_time_ms = k_uptime_get();
    slot->snapshot = *snapshot;
    slot->sidewalk_msg_id = 0;

    /* Format payload immediately to capture timestamp */
    slot->payload_len = format_critical_payload(
        slot->payload, PAYLOAD_MAX_SIZE, type, snapshot);

    if (slot->payload_len <= 0 || slot->payload_len >= PAYLOAD_MAX_SIZE) {
        slot->state = MSG_SLOT_FREE;
        k_mutex_unlock(&buffer.mutex);
        LOG_ERR("Failed to format payload");
        return -EINVAL;
    }

    buffer.active_count++;

    LOG_INF("Critical message %c enqueued (slot=%d, count=%d): %s",
            (char)type, slot_idx, buffer.active_count, slot->payload);

    k_mutex_unlock(&buffer.mutex);

    /* Trigger send */
    k_work_submit(&buffer.send_work);

    return 0;
}

int critical_buffer_on_sent(uint16_t msg_id)
{
    k_mutex_lock(&buffer.mutex, K_FOREVER);

    int slot_idx = find_slot_by_msg_id(msg_id);
    if (slot_idx < 0) {
        k_mutex_unlock(&buffer.mutex);
        return -ENOENT;
    }

    critical_msg_slot_t *slot = &buffer.slots[slot_idx];

    LOG_INF("Critical message %c delivered (slot=%d, id=%u)",
            (char)slot->type, slot_idx, msg_id);

    slot->state = MSG_SLOT_FREE;
    buffer.active_count--;

    k_mutex_unlock(&buffer.mutex);
    return 0;
}

int critical_buffer_on_error(uint16_t msg_id)
{
    k_mutex_lock(&buffer.mutex, K_FOREVER);

    int slot_idx = find_slot_by_msg_id(msg_id);
    if (slot_idx < 0) {
        k_mutex_unlock(&buffer.mutex);
        return -ENOENT;
    }

    critical_msg_slot_t *slot = &buffer.slots[slot_idx];

    slot->retry_count++;

    if (slot->retry_count >= MAX_RETRIES) {
        LOG_ERR("Critical message %c failed after %d retries (slot=%d)",
                (char)slot->type, MAX_RETRIES, slot_idx);
        slot->state = MSG_SLOT_FREE;
        buffer.active_count--;
    } else {
        LOG_WRN("Critical message %c send failed, scheduling retry %d/%d",
                (char)slot->type, slot->retry_count, MAX_RETRIES);
        slot->state = MSG_SLOT_RETRY;

        /* Schedule retry with delay */
        k_work_schedule(&buffer.retry_work, K_MSEC(RETRY_DELAY_MS));
    }

    k_mutex_unlock(&buffer.mutex);
    return 0;
}

void critical_buffer_process(void)
{
    k_work_submit(&buffer.send_work);
}

bool critical_buffer_owns_message(uint16_t msg_id)
{
    bool owns = false;

    k_mutex_lock(&buffer.mutex, K_FOREVER);

    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (buffer.slots[i].state == MSG_SLOT_SENT &&
            buffer.slots[i].sidewalk_msg_id == msg_id) {
            owns = true;
            break;
        }
    }

    k_mutex_unlock(&buffer.mutex);
    return owns;
}

uint8_t critical_buffer_count(void)
{
    uint8_t count;

    k_mutex_lock(&buffer.mutex, K_FOREVER);
    count = buffer.active_count;
    k_mutex_unlock(&buffer.mutex);

    return count;
}

void critical_buffer_trigger_send(void)
{
    k_work_submit(&buffer.send_work);
}
