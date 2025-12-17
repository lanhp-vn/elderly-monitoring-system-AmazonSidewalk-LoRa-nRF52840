/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Critical Message Ring Buffer
 * Manages Emergency and Help messages with tracking and retries
 */

#ifndef CRITICAL_MSG_BUFFER_H
#define CRITICAL_MSG_BUFFER_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Message types */
typedef enum {
    MSG_TYPE_REGULAR = 'R',
    MSG_TYPE_EMERGENCY = 'E',
    MSG_TYPE_HELP = 'H'
} message_type_t;

/* Slot states for lifecycle tracking */
typedef enum {
    MSG_SLOT_FREE = 0,     /* Available for use */
    MSG_SLOT_PENDING,      /* Filled, waiting to be sent */
    MSG_SLOT_SENT,         /* Submitted to Sidewalk, waiting callback */
    MSG_SLOT_RETRY         /* Send failed, pending retry */
} msg_slot_state_t;

/**
 * @brief Initialize the critical message buffer
 *
 * Must be called before any other critical buffer functions.
 * Initializes mutex, work items, and clears all slots.
 *
 * @return 0 on success, negative errno on failure
 */
int critical_buffer_init(void);

/**
 * @brief Enqueue a critical message (Emergency or Help)
 *
 * Captures the current sensor snapshot and queues it for sending.
 * If buffer is full, the oldest message is evicted.
 *
 * Thread-safe: can be called from any context except ISR.
 *
 * @param type Message type (MSG_TYPE_EMERGENCY or MSG_TYPE_HELP)
 * @param snapshot Sensor data snapshot at event time (NULL to capture current)
 * @return 0 on success, negative errno on failure
 */
int critical_buffer_enqueue(message_type_t type, const sensor_snapshot_t *snapshot);

/**
 * @brief Notify buffer that a message was sent successfully
 *
 * Called from on_sidewalk_msg_sent callback.
 * Frees the slot associated with the message ID.
 *
 * @param msg_id Sidewalk message ID from callback
 * @return 0 if found and freed, -ENOENT if not found
 */
int critical_buffer_on_sent(uint16_t msg_id);

/**
 * @brief Notify buffer that a message send failed
 *
 * Called from on_sidewalk_send_error callback.
 * Schedules retry if retry count not exceeded, otherwise frees slot.
 *
 * @param msg_id Sidewalk message ID from callback
 * @return 0 if found, -ENOENT if not found
 */
int critical_buffer_on_error(uint16_t msg_id);

/**
 * @brief Process pending messages in the buffer
 *
 * Called from work handler to send pending/retry messages.
 * Should be called when Sidewalk becomes ready.
 */
void critical_buffer_process(void);

/**
 * @brief Check if a message ID belongs to critical buffer
 *
 * Used to determine if a callback should be routed to critical buffer
 * or handled as a regular fire-and-forget message.
 *
 * @param msg_id Sidewalk message ID to check
 * @return true if message is tracked by critical buffer
 */
bool critical_buffer_owns_message(uint16_t msg_id);

/**
 * @brief Get current count of messages in buffer
 *
 * @return Number of active messages (PENDING + SENT + RETRY)
 */
uint8_t critical_buffer_count(void);

/**
 * @brief Trigger processing of pending messages
 *
 * Submits work to system workqueue to process buffer.
 * Safe to call from any context.
 */
void critical_buffer_trigger_send(void);

#ifdef __cplusplus
}
#endif

#endif /* CRITICAL_MSG_BUFFER_H */
