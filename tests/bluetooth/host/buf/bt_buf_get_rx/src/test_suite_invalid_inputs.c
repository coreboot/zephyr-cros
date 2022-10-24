/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/buf.h>
#include "host_mocks/assert.h"

ZTEST_SUITE(test_bt_buf_get_rx_invalid_input, NULL, NULL, NULL, NULL, NULL);

/*
 *  Test passing invalid buffer type to bt_buf_get_rx()
 *
 *  Constraints:
 *   - Use invalid buffer type 'BT_BUF_CMD'
 *
 *  Expected behaviour:
 *   - An assertion should be raised as an invalid parameter was used
 */
ZTEST(test_bt_buf_get_rx_invalid_input, test_invalid_input_type_bt_buf_cmd)
{
	expect_assert();
	bt_buf_get_rx(BT_BUF_CMD, Z_TIMEOUT_TICKS(1000));
}

/*
 *  Test passing invalid buffer type to bt_buf_get_rx()
 *
 *  Constraints:
 *   - Use invalid buffer type 'BT_BUF_ACL_OUT'
 *
 *  Expected behaviour:
 *   - An assertion should be raised as an invalid parameter was used
 */
ZTEST(test_bt_buf_get_rx_invalid_input, test_invalid_input_type_bt_buf_acl_out)
{
	expect_assert();
	bt_buf_get_rx(BT_BUF_ACL_OUT, Z_TIMEOUT_TICKS(1000));
}

/*
 *  Test passing invalid buffer type to bt_buf_get_rx()
 *
 *  Constraints:
 *   - Use invalid buffer type 'BT_BUF_ISO_OUT'
 *
 *  Expected behaviour:
 *   - An assertion should be raised as an invalid parameter was used
 */
ZTEST(test_bt_buf_get_rx_invalid_input, test_invalid_input_type_bt_buf_iso_out)
{
	expect_assert();
	bt_buf_get_rx(BT_BUF_ISO_OUT, Z_TIMEOUT_TICKS(1000));
}

/*
 *  Test passing invalid buffer type to bt_buf_get_rx()
 *
 *  Constraints:
 *   - Use invalid buffer type 'BT_BUF_H4'
 *
 *  Expected behaviour:
 *   - An assertion should be raised as an invalid parameter was used
 */
ZTEST(test_bt_buf_get_rx_invalid_input, test_invalid_input_type_bt_buf_h4)
{
	expect_assert();
	bt_buf_get_rx(BT_BUF_H4, Z_TIMEOUT_TICKS(1000));
}
