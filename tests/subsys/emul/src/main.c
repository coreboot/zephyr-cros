/*
 * Copyright 2022 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/emul.h>
#include <ztest.h>
#include <ztest_assert.h>
#include <ztest_test_new.h>

#define TEST_ACCEL DT_NODELABEL(test_bmi)

ZTEST(emul, test_emul_dt_get)
{
	const struct emul *dev_emul = emul_get_binding(DT_NODE_FULL_NAME(TEST_ACCEL));
	/* This variable is static to verify that the result of EMUL_DT_GET is a
	 * compile-time constant.
	 */
	static const struct emul *emul_static = EMUL_DT_GET(TEST_ACCEL);

	zassert_not_null(dev_emul, "emul_get_binding returned NULL");
	zassert_equal(emul_static, dev_emul,
		      "EMUL_DT_GET returned %p but emul_get_binding returned %p", emul_static,
		      dev_emul);
}

ZTEST_SUITE(emul, NULL, NULL, NULL, NULL, NULL);
