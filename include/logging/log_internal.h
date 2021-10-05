/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_LOGGING_LOG_INTERNAL_H_
#define ZEPHYR_INCLUDE_LOGGING_LOG_INTERNAL_H_

#include <zephyr/types.h>
#include <sys/__assert.h>
#include <logging/log_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Header contains declarations of functions used internally in the logging,
 * shared between various portions of logging subsystem. Functions are internal
 * not intended to be used outside, including logging backends.
 */

/** @brief Indicate to the log core that one log message has been dropped.
 */
void z_log_dropped(void);

/** @brief Read and clear current drop indications counter.
 *
 * @return Dropped count.
 */
uint32_t z_log_dropped_read_and_clear(void);

/** @brief Check if there are any pending drop notifications.
 *
 * @retval true Pending unreported drop indications.
 * @retval false No pending unreported drop indications.
 */
bool z_log_dropped_pending(void);

/** @brief Free allocated buffer.
 *
 * @param buf Buffer.
 */
void z_log_free(void *buf);

/* Initialize runtime filters */
void z_log_runtime_filters_init(void);

/* Notify log_core that a backend was enabled. */
void z_log_notify_backend_enabled(void);

/** @brief Get pointer to the filter set of the log source.
 *
 * @param source_id Source ID.
 *
 * @return Pointer to the filter set.
 */
static inline uint32_t *z_log_dynamic_filters_get(uint32_t source_id)
{
	return &__log_dynamic_start[source_id].filters;
}

/** @brief Get number of registered sources. */
static inline uint32_t z_log_sources_count(void)
{
	return log_const_source_id(__log_const_end);
}

/** @brief Initialize module for handling logging message. */
void z_log_msg2_init(void);

/** @brief Commit log message.
 *
 * @param msg Message.
 */
void z_log_msg2_commit(struct log_msg2 *msg);

/** @brief Get pending log message.
 *
 * @param[out] len Message length in bytes is written is @p len is not null.
 *
 * @param Message or null if no pending messages.
 */
union log_msg2_generic *z_log_msg2_claim(void);

/** @brief Free message.
 *
 * @param msg Message.
 */
void z_log_msg2_free(union log_msg2_generic *msg);

/** @brief Check if there are any message pending.
 *
 * @retval true if at least one message is pending.
 * @retval false if no message is pending.
 */
bool z_log_msg2_pending(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_LOGGING_LOG_INTERNAL_H_ */
