/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <malloc.h>

#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <kernel_internal.h>
#if defined(CONFIG_LOG_RUNTIME_FILTERING)
#include <zephyr/logging/log_ctrl.h>
#endif

static int cmd_app_heap(const struct shell *sh, size_t argc, char **argv)
{
#if defined(CONFIG_BOARD_NATIVE_POSIX)
#if defined(__GLIBC__)
#if __GLIBC__ == 2 && __GLIBC_MINOR__ < 33
	/* mallinfo() was deprecated in glibc 2.33 and removed in 2.34. */
	struct mallinfo mi = mallinfo();
#else
	struct mallinfo2 mi = mallinfo2();
#endif /* CONFIG_BOARD_NATIVE_POSIX && __GLIBC__ == 2 && __GLIBC_MINOR__ < 33 */
#else
	struct mallinfo mi = mallinfo();
#endif
#else
	struct mallinfo mi = mallinfo();
#endif

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Heap size: %zu bytes", (size_t)mi.arena);
	shell_print(sh, "  used: %zu bytes", (size_t)mi.uordblks);
	shell_print(sh, "  free: %zu bytes", (size_t)mi.fordblks);
	shell_print(sh, "  max used: %zu bytes", (size_t)mi.usmblks);
	shell_print(sh, "  free fastbin: %zu bytes", (size_t)mi.fsmblks);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_app,
	SHELL_CMD(heap, NULL, "app heap", cmd_app_heap),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(app, &sub_app, "application commands", NULL);
