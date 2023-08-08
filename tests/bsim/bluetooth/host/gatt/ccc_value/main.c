/* Copyright (c) 2023 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <argparse.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>

#include "testlib/adv.h"
#include "testlib/att_read.h"
#include "testlib/att_write.h"
#include "bs_macro.h"
#include "bs_sync.h"
#include "testlib/conn_ref.h"
#include "testlib/conn_wait.h"
#include "testlib/connect.h"
#include "testlib/log_utils.h"
#include "testlib/scan.h"
#include "testlib/security.h"

/* This test uses system asserts to fail tests. */
BUILD_ASSERT(__ASSERT_ON);

#define CENTRAL_1_DEVICE_NBR  0
#define CENTRAL_2_DEVICE_NBR  1
#define PERIPHERAL_DEVICE_NBR 2

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define UUID_1                                                                                     \
	BT_UUID_DECLARE_128(0xdb, 0x1f, 0xe2, 0x52, 0xf3, 0xc6, 0x43, 0x66, 0xb3, 0x92, 0x5d,      \
			    0xc6, 0xe7, 0xc9, 0x59, 0x9d)

#define UUID_2                                                                                     \
	BT_UUID_DECLARE_128(0x3f, 0xa4, 0x7f, 0x44, 0x2e, 0x2a, 0x43, 0x05, 0xab, 0x38, 0x07,      \
			    0x8d, 0x16, 0xbf, 0x99, 0xf1)

static atomic_t ccc_cfg_value;
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_INF("New CCC cfg value: %u", value);
	atomic_set(&ccc_cfg_value, value);
}

static struct bt_gatt_attr attrs[] = {
	BT_GATT_PRIMARY_SERVICE(UUID_1),
	BT_GATT_CHARACTERISTIC(UUID_2, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service svc = {
	.attrs = attrs,
	.attr_count = ARRAY_SIZE(attrs),
};

static void bs_sync_all_log(char *log_msg)
{
	/* Everyone meets here. */
	bt_testlib_bs_sync_all();

	if (get_device_nbr() == 0) {
		LOG_WRN("Sync point: %s", log_msg);
	}

	/* Everyone waits for d0 to finish logging. */
	bt_testlib_bs_sync_all();
}

static inline void bt_enable_quiet(void)
{
	bt_testlib_log_level_set("bt_hci_core", LOG_LEVEL_ERR);
	bt_testlib_log_level_set("bt_id", LOG_LEVEL_ERR);

	EXPECT_ZERO(bt_enable(NULL));

	bt_testlib_log_level_set("bt_hci_core", LOG_LEVEL_INF);
	bt_testlib_log_level_set("bt_id", LOG_LEVEL_INF);
}

void the_test(void)
{
	bool central_1 = (get_device_nbr() == CENTRAL_1_DEVICE_NBR);
	bool central_2 = (get_device_nbr() == CENTRAL_2_DEVICE_NBR);
	bool peripheral = (get_device_nbr() == PERIPHERAL_DEVICE_NBR);
	bt_addr_le_t adva;
	struct bt_conn *conn = NULL;

	if (peripheral) {
		EXPECT_ZERO(bt_gatt_service_register(&svc));
	}

	bt_enable_quiet();

	if (peripheral) {
		EXPECT_ZERO(bt_set_name("peripheral"));
		EXPECT_ZERO(bt_testlib_adv_conn(
			&conn, BT_ID_DEFAULT,
			(BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_FORCE_NAME_IN_AD)));
	}

	if (central_1) {
		uint16_t ccc_handle;
		uint16_t chrc_end_handle;
		uint16_t chrc_value_handle;
		uint16_t svc_end_handle;
		uint16_t svc_handle;
		uint8_t ccc_value_buf[2];

		/* Scan and connect */

		EXPECT_ZERO(bt_testlib_scan_find_name(&adva, "peripheral"));
		EXPECT_ZERO(bt_testlib_connect(&adva, &conn));

		/* Bond */
		EXPECT_ZERO(bt_testlib_secure(conn, BT_SECURITY_L2));

		/* Find CCC handle */
		EXPECT_ZERO(bt_testlib_gatt_discover_primary(&svc_handle, &svc_end_handle, conn,
							     UUID_1, 1, 0xffff));

		LOG_INF("svc_handle: %u, svc_end_handle: %u", svc_handle, svc_end_handle);

		EXPECT_ZERO(bt_testlib_gatt_discover_characteristic(
			&chrc_value_handle, &chrc_end_handle, NULL, conn, UUID_2, (svc_handle + 1),
			svc_end_handle));

		LOG_INF("chrc_value_handle: %u, chrc_end_handle: %u", chrc_value_handle,
			chrc_end_handle);

		EXPECT_ZERO(bt_testlib_att_read_by_type_sync(NULL, NULL, &ccc_handle, NULL, conn, 0,
							     BT_UUID_GATT_CCC, chrc_value_handle,
							     chrc_end_handle));

		LOG_INF("ccc_handle: %u", ccc_handle);

		/* Write to CCC */
		sys_put_le16(BT_GATT_CCC_NOTIFY, ccc_value_buf);
		EXPECT_ZERO(bt_testlib_att_write(conn, 0, ccc_handle, ccc_value_buf,
						 sizeof(ccc_value_buf)));

		/* Disconnect */
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		bt_testlib_conn_unref(&conn);
	}

	if (peripheral) {
		bt_testlib_wait_disconnected(conn);
		bt_testlib_conn_unref(&conn);

		/* According to its own API documentation, the cfg value shall
		 * be zero when disconnected, even if there exists a trusted
		 * relationship with a non-zero CCC value.
		 */
		EXPECT_ZERO(atomic_get(&ccc_cfg_value));
	}

	bs_sync_all_log("Test Complete");
	PASS("Test complete\n");
}
