/*
 * Copyright (c) 2024, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <malloc.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

static inline size_t mi_usage(const struct mallinfo *mi)
{
	return mi->uordblks;
}

static inline bool mi_gt(const struct mallinfo *a, const struct mallinfo *b)
{
	return mi_usage(a) > mi_usage(b);
}

ZTEST(libc_common, test_mallinfo)
{
	uint8_t *data;
	struct mallinfo mi_then;
	struct mallinfo mi_now;

	if (IS_ENABLED(CONFIG_NEWLIB_LIBC)) {
		/* Newlib is not supported at this time */
		ztest_test_skip();
	}

	mi_then = mallinfo();
	data = malloc(42);
	mi_now = mallinfo();
	zassert_true(mi_gt(&mi_now, &mi_then));

	mi_then = mi_now;
	free(data);
	mi_now = mallinfo();
	zassert_true(mi_gt(&mi_then, &mi_now));
}
