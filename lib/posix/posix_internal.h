/*
 * Copyright (c) 2022 Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_LIB_POSIX_POSIX_INTERNAL_H_
#define ZEPHYR_LIB_POSIX_POSIX_INTERNAL_H_

/*
 * Bit used to mark a pthread_mutex_t as initialized. Initialization status is
 * verified (against internal status) in lock / unlock / destroy functions.
 */
#define PTHREAD_MUTEX_MASK_INIT 0x80000000

struct posix_mutex {
	k_tid_t owner;
	uint16_t lock_count;
	int type;
	_wait_q_t wait_q;
};

enum pthread_state {
	/* The thread structure is unallocated and available for reuse. */
	PTHREAD_TERMINATED = 0,
	/* The thread is running and joinable. */
	PTHREAD_JOINABLE = PTHREAD_CREATE_JOINABLE,
	/* The thread is running and detached. */
	PTHREAD_DETACHED = PTHREAD_CREATE_DETACHED,
	/* A joinable thread exited and its return code is available. */
	PTHREAD_EXITED
};

struct posix_thread {
	struct k_thread thread;

	/* List of keys that thread has called pthread_setspecific() on */
	sys_slist_t key_list;

	/* Exit status */
	void *retval;

	/* Pthread cancellation */
	int cancel_state;
	int cancel_pending;
	struct k_spinlock cancel_lock;

	/* Pthread State */
	enum pthread_state state;
	pthread_mutex_t state_lock;
	pthread_cond_t state_cond;
};

struct posix_thread *to_posix_thread(pthread_t pthread);

/* get and possibly initialize a posix_mutex */
struct posix_mutex *to_posix_mutex(pthread_mutex_t *mu);

/* get a previously initialized posix_mutex */
struct posix_mutex *get_posix_mutex(pthread_mutex_t mut);

static inline bool is_pthread_mutex_initialized(pthread_mutex_t mut)
{
	return (mut & PTHREAD_MUTEX_MASK_INIT) != 0;
}

static inline pthread_mutex_t mark_pthread_mutex_initialized(pthread_mutex_t mut)
{
	return mut | PTHREAD_MUTEX_MASK_INIT;
}

static inline pthread_mutex_t mark_pthread_mutex_uninitialized(pthread_mutex_t mut)
{
	return mut & ~PTHREAD_MUTEX_MASK_INIT;
}

#endif
