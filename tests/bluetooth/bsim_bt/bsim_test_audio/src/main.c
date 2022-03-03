/*
 * Copyright (c) 2020-2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bstests.h"

extern struct bst_test_list *test_vcs_install(struct bst_test_list *tests);
extern struct bst_test_list *test_vcs_client_install(struct bst_test_list *tests);
extern struct bst_test_list *test_mics_install(struct bst_test_list *tests);
extern struct bst_test_list *test_mics_client_install(struct bst_test_list *tests);
extern struct bst_test_list *test_csis_install(struct bst_test_list *tests);
extern struct bst_test_list *test_csis_client_install(struct bst_test_list *tests);
extern struct bst_test_list *test_mcs_install(struct bst_test_list *tests);
extern struct bst_test_list *test_mcc_install(struct bst_test_list *tests);
extern struct bst_test_list *test_media_controller_install(struct bst_test_list *tests);
extern struct bst_test_list *test_unicast_client_install(struct bst_test_list *tests);
extern struct bst_test_list *test_unicast_server_install(struct bst_test_list *tests);
extern struct bst_test_list *test_broadcast_source_install(struct bst_test_list *tests);
extern struct bst_test_list *test_broadcast_sink_install(struct bst_test_list *tests);

bst_test_install_t test_installers[] = {
	test_vcs_install,
	test_vcs_client_install,
	test_mics_install,
	test_mics_client_install,
	test_csis_install,
	test_csis_client_install,
	test_mcs_install,
	test_mcc_install,
	test_media_controller_install,
	test_unicast_client_install,
	test_unicast_server_install,
	test_broadcast_source_install,
	test_broadcast_sink_install,
	NULL
};

void main(void)
{
	bst_main();
}
