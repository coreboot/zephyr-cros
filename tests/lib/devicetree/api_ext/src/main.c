/*
 * Copyright (c) 2021, Commonwealth Scientific and Industrial Research
 * Organisation (CSIRO) ABN 41 687 119 230.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <devicetree.h>
#include <device.h>
#include <drivers/gpio.h>

#include <linker/devicetree_regions.h>

#define TEST_SRAM1      DT_NODELABEL(test_sram1)
#define TEST_SRAM2      DT_NODELABEL(test_sram2)

static void test_linker_regions(void)
{
	zassert_true(!strcmp(LINKER_DT_NODE_REGION_NAME(TEST_SRAM1), "SRAM_REGION"), "");
	zassert_true(!strcmp(LINKER_DT_NODE_REGION_NAME(TEST_SRAM2), "/test/sram@20001000"), "");
}

void test_main(void)
{
	ztest_test_suite(devicetree_api_ext,
			 ztest_unit_test(test_linker_regions)
			 );
	ztest_run_test_suite(devicetree_api_ext);
}
