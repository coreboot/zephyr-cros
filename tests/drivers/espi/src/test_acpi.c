/*
 * Copyright 2021 Google LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/espi.h>
#include <drivers/espi_emul.h>
#include <ztest.h>

static void test_acpi_shared_memory(void)
{
	const struct device *espi_dev = device_get_binding(DT_LABEL(DT_NODELABEL(espi0)));
	struct espi_cfg cfg = {
		.channel_caps = ESPI_CHANNEL_VWIRE | ESPI_CHANNEL_PERIPHERAL,
	};
	uintptr_t host_shm, peripheral_shm;

	zassert_not_null(espi_dev, NULL);

	zassert_ok(espi_config(espi_dev, &cfg), NULL);

	host_shm = emul_espi_host_get_acpi_shm(espi_dev);
	zassert_not_equal(host_shm, 0, NULL);

	zassert_ok(espi_read_lpc_request(espi_dev, EACPI_GET_SHARED_MEMORY,
					 (uint32_t *)&peripheral_shm),
		   NULL);

	zassert_equal(host_shm, peripheral_shm, NULL);
}

ztest_test_suite(acpi, ztest_unit_test(test_acpi_shared_memory));

void test_main(void)
{
	ztest_run_test_suite(acpi);
}
