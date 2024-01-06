/*
 * Copyright (c) 2024, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <malloc.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

size_t mi_usage(const struct mallinfo *mi)
{
	return mi->uordblks;
}

#define _mi_op(_a, _b, _op) (mi_usage(_a) _op mi_usage(_b))

#define _decl_op(_type, _name, _op)                                                                \
	static inline _type _name(const struct mallinfo *_a, const struct mallinfo *_b)            \
	{                                                                                          \
		return _mi_op(_a, _b, _op);                                                        \
	}

_decl_op(bool, mi_gt, >);

ZTEST(mallinfo, test_mallinfo)
{
	uint8_t *data;
	struct mallinfo mi_then;
	struct mallinfo mi_now;

	mi_then = mallinfo();
	data = malloc(42);
	mi_now = mallinfo();
	zassert_true(mi_gt(&mi_now, &mi_then));

	mi_then = mi_now;
	free(data);
	mi_now = mallinfo();
	zassert_true(mi_gt(&mi_then, &mi_now));
}

ZTEST_SUITE(mallinfo, NULL, NULL, NULL, NULL, NULL);
