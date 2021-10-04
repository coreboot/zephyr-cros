/*
 * Copyright (c) 2019 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARM64 specific kernel interface header
 *
 * This header contains the ARM64 specific kernel interface.  It is
 * included by the kernel interface architecture-abstraction header
 * (include/arm64/cpu.h)
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM64_ARCH_H_
#define ZEPHYR_INCLUDE_ARCH_ARM64_ARCH_H_

/* Add include for DTS generated information */
#include <devicetree.h>

#include <arch/arm64/thread.h>
#include <arch/arm64/exc.h>
#include <arch/arm64/irq.h>
#include <arch/arm64/misc.h>
#include <arch/arm64/asm_inline.h>
#include <arch/arm64/cpu.h>
#include <arch/arm64/macro.inc>
#include <arch/arm64/sys_io.h>
#include <arch/arm64/timer.h>
#include <arch/arm64/error.h>
#include <arch/arm64/mm.h>
#include <arch/arm64/thread_stack.h>
#include <arch/common/addr_types.h>
#include <arch/common/sys_bitops.h>
#include <arch/common/ffs.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE

#include <sys/slist.h>

struct arch_mem_domain {
#ifdef CONFIG_ARM_MMU
	struct arm_mmu_ptables ptables;
#endif
	sys_snode_t node;
};

#endif /* _ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_ARM64_ARCH_H_ */
