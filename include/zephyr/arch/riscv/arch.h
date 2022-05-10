/*
 * Copyright (c) 2016 Jean-Paul Etienne <fractalclone@gmail.com>
 * Contributors: 2018 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief RISCV specific kernel interface header
 * This header contains the RISCV specific kernel interface.  It is
 * included by the generic kernel interface header (arch/cpu.h)
 */

#ifndef ZEPHYR_INCLUDE_ARCH_RISCV_ARCH_H_
#define ZEPHYR_INCLUDE_ARCH_RISCV_ARCH_H_

#include <zephyr/arch/riscv/thread.h>
#include <zephyr/arch/riscv/exp.h>
#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/arch/common/ffs.h>
#if defined(CONFIG_USERSPACE)
#include <zephyr/arch/riscv/syscall.h>
#endif /* CONFIG_USERSPACE */
#include <zephyr/irq.h>
#include <zephyr/sw_isr_table.h>
#include <soc.h>
#include <zephyr/devicetree.h>
#include <zephyr/arch/riscv/csr.h>

/* stacks, for RISCV architecture stack should be 16byte-aligned */
#define ARCH_STACK_PTR_ALIGN  16

#ifdef CONFIG_PMP_STACK_GUARD
#define Z_RISCV_PMP_ALIGN CONFIG_PMP_STACK_GUARD_MIN_SIZE
#define Z_RISCV_STACK_GUARD_SIZE  Z_RISCV_PMP_ALIGN
#else
#define Z_RISCV_PMP_ALIGN 4
#define Z_RISCV_STACK_GUARD_SIZE  0
#endif

/* Kernel-only stacks have the following layout if a stack guard is enabled:
 *
 * +------------+ <- thread.stack_obj
 * | Guard      | } Z_RISCV_STACK_GUARD_SIZE
 * +------------+ <- thread.stack_info.start
 * | Kernel     |
 * | stack      |
 * |            |
 * +............|
 * | TLS        | } thread.stack_info.delta
 * +------------+ <- thread.stack_info.start + thread.stack_info.size
 */
#ifdef CONFIG_PMP_STACK_GUARD
#define ARCH_KERNEL_STACK_RESERVED	Z_RISCV_STACK_GUARD_SIZE
#define ARCH_KERNEL_STACK_OBJ_ALIGN \
		MAX(Z_RISCV_PMP_ALIGN, ARCH_STACK_PTR_ALIGN)
#endif

#ifdef CONFIG_USERSPACE
/* Any thread running In user mode will have full access to the region denoted
 * by thread.stack_info.
 *
 * Thread-local storage is at the very highest memory locations of this area.
 * Memory for TLS and any initial random stack pointer offset is captured
 * in thread.stack_info.delta.
 */
#ifdef CONFIG_PMP_STACK_GUARD
#ifdef CONFIG_MPU_REQUIRES_POWER_OF_TWO_ALIGNMENT
/* Use defaults for everything. The privilege elevation stack is located
 * in another area of memory generated at build time by gen_kobject_list.py
 *
 * +------------+ <- thread.arch.priv_stack_start
 * | Guard      | } Z_RISCV_STACK_GUARD_SIZE
 * +------------+
 * | Priv Stack | } CONFIG_PRIVILEGED_STACK_SIZE - Z_RISCV_STACK_GUARD_SIZE
 * +------------+ <- thread.arch.priv_stack_start +
 *                   CONFIG_PRIVILEGED_STACK_SIZE
 *
 * +------------+ <- thread.stack_obj = thread.stack_info.start
 * | Thread     |
 * | stack      |
 * |            |
 * +............|
 * | TLS        | } thread.stack_info.delta
 * +------------+ <- thread.stack_info.start + thread.stack_info.size
 */
#define ARCH_THREAD_STACK_SIZE_ADJUST(size) \
		Z_POW2_CEIL(ROUND_UP((size), Z_RISCV_PMP_ALIGN))
#define ARCH_THREAD_STACK_OBJ_ALIGN(size) \
		ARCH_THREAD_STACK_SIZE_ADJUST(size)
#define ARCH_THREAD_STACK_RESERVED		0
#else /* !CONFIG_MPU_REQUIRES_POWER_OF_TWO_ALIGNMENT */
/* The stack object will contain the PMP guard, the privilege stack, and then
 * the stack buffer in that order:
 *
 * +------------+ <- thread.stack_obj
 * | Guard      | } Z_RISCV_STACK_GUARD_SIZE
 * +------------+ <- thread.arch.priv_stack_start
 * | Priv Stack | } CONFIG_PRIVILEGED_STACK_SIZE
 * +------------+ <- thread.stack_info.start
 * | Thread     |
 * | stack      |
 * |            |
 * +............|
 * | TLS        | } thread.stack_info.delta
 * +------------+ <- thread.stack_info.start + thread.stack_info.size
 */
#define ARCH_THREAD_STACK_RESERVED	(Z_RISCV_STACK_GUARD_SIZE + \
					 CONFIG_PRIVILEGED_STACK_SIZE)
#define ARCH_THREAD_STACK_OBJ_ALIGN(size)	Z_RISCV_PMP_ALIGN
/* We need to be able to exactly cover the stack buffer with an PMP region,
 * so round its size up to the required granularity of the PMP
 */
#define ARCH_THREAD_STACK_SIZE_ADJUST(size) \
		(ROUND_UP((size), Z_RISCV_PMP_ALIGN))

#endif /* CONFIG_MPU_REQUIRES_POWER_OF_TWO_ALIGNMENT */
#else /* !CONFIG_PMP_STACK_GUARD */
#ifdef CONFIG_MPU_REQUIRES_POWER_OF_TWO_ALIGNMENT
/* Use defaults for everything. The privilege elevation stack is located
 * in another area of memory generated at build time by gen_kobject_list.py
 *
 * +------------+ <- thread.arch.priv_stack_start
 * | Priv Stack | } Z_KERNEL_STACK_LEN(CONFIG_PRIVILEGED_STACK_SIZE)
 * +------------+
 *
 * +------------+ <- thread.stack_obj = thread.stack_info.start
 * | Thread     |
 * | stack      |
 * |            |
 * +............|
 * | TLS        | } thread.stack_info.delta
 * +------------+ <- thread.stack_info.start + thread.stack_info.size
 */
#define ARCH_THREAD_STACK_SIZE_ADJUST(size) \
		Z_POW2_CEIL(ROUND_UP((size), Z_RISCV_PMP_ALIGN))
#define ARCH_THREAD_STACK_OBJ_ALIGN(size) \
		ARCH_THREAD_STACK_SIZE_ADJUST(size)
#define ARCH_THREAD_STACK_RESERVED		0
#else /* !CONFIG_MPU_REQUIRES_POWER_OF_TWO_ALIGNMENT */
/* Userspace enabled, but supervisor stack guards are not in use */

/* Reserved area of the thread object just contains the privilege stack:
 *
 * +------------+ <- thread.stack_obj = thread.arch.priv_stack_start
 * | Priv Stack | } CONFIG_PRIVILEGED_STACK_SIZE
 * +------------+ <- thread.stack_info.start
 * | Thread     |
 * | stack      |
 * |            |
 * +............|
 * | TLS        | } thread.stack_info.delta
 * +------------+ <- thread.stack_info.start + thread.stack_info.size
 */
#define ARCH_THREAD_STACK_RESERVED		CONFIG_PRIVILEGED_STACK_SIZE
#define ARCH_THREAD_STACK_SIZE_ADJUST(size) \
		(ROUND_UP((size), Z_RISCV_PMP_ALIGN))
#define ARCH_THREAD_STACK_OBJ_ALIGN(size)	Z_RISCV_PMP_ALIGN

#endif /* CONFIG_MPU_REQUIRES_POWER_OF_TWO_ALIGNMENT */
#endif /* CONFIG_PMP_STACK_GUARD */

#else /* !CONFIG_USERSPACE */

#ifdef CONFIG_PMP_STACK_GUARD
/* Reserve some memory for the stack guard.
 * This is just a minimally-sized region at the beginning of the stack
 * object, which is programmed to produce an exception if written to.
 *
 * +------------+ <- thread.stack_obj
 * | Guard      | } Z_RISCV_STACK_GUARD_SIZE
 * +------------+ <- thread.stack_info.start
 * | Thread     |
 * | stack      |
 * |            |
 * +............|
 * | TLS        | } thread.stack_info.delta
 * +------------+ <- thread.stack_info.start + thread.stack_info.size
 */
#define ARCH_THREAD_STACK_RESERVED		Z_RISCV_STACK_GUARD_SIZE
#define ARCH_THREAD_STACK_OBJ_ALIGN(size)	Z_RISCV_PMP_ALIGN
/* Default for ARCH_THREAD_STACK_SIZE_ADJUST */
#else /* !CONFIG_PMP_STACK_GUARD */
/* No stack guard, no userspace, Use defaults for everything. */
#endif /* CONFIG_PMP_STACK_GUARD */
#endif /* CONFIG_USERSPACE */

#ifdef CONFIG_64BIT
#define RV_REGSIZE 8
#define RV_REGSHIFT 3
#else
#define RV_REGSIZE 4
#define RV_REGSHIFT 2
#endif

/* Common mstatus bits. All supported cores today have the same
 * layouts.
 */

#define MSTATUS_IEN     (1UL << 3)
#define MSTATUS_MPP_M   (3UL << 11)
#define MSTATUS_MPIE_EN (1UL << 7)
#define MSTATUS_FS_INIT (1UL << 13)
#define MSTATUS_FS_MASK ((1UL << 13) | (1UL << 14))


/* This comes from openisa_rv32m1, but doesn't seem to hurt on other
 * platforms:
 * - Preserve machine privileges in MPP. If you see any documentation
 *   telling you that MPP is read-only on this SoC, don't believe its
 *   lies.
 * - Enable interrupts when exiting from exception into a new thread
 *   by setting MPIE now, so it will be copied into IE on mret.
 */
#define MSTATUS_DEF_RESTORE (MSTATUS_MPP_M | MSTATUS_MPIE_EN)

#ifndef _ASMLANGUAGE
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STACK_ROUND_UP(x) ROUND_UP(x, ARCH_STACK_PTR_ALIGN)

/* macros convert value of its argument to a string */
#define DO_TOSTR(s) #s
#define TOSTR(s) DO_TOSTR(s)

/* concatenate the values of the arguments into one */
#define DO_CONCAT(x, y) x ## y
#define CONCAT(x, y) DO_CONCAT(x, y)

/* Kernel macros for memory attribution
 * (access permissions and cache-ability).
 *
 * The macros are to be stored in k_mem_partition_attr_t
 * objects. The format of a k_mem_partition_attr_t object
 * is an uint8_t composed by configuration register flags
 * located in arch/riscv/include/core_pmp.h
 */

/* Read-Write access permission attributes */
#define K_MEM_PARTITION_P_RW_U_RW ((k_mem_partition_attr_t) \
	{PMP_R | PMP_W})
#define K_MEM_PARTITION_P_RW_U_RO ((k_mem_partition_attr_t) \
	{PMP_R})
#define K_MEM_PARTITION_P_RW_U_NA ((k_mem_partition_attr_t) \
	{0})
#define K_MEM_PARTITION_P_RO_U_RO ((k_mem_partition_attr_t) \
	{PMP_R})
#define K_MEM_PARTITION_P_RO_U_NA ((k_mem_partition_attr_t) \
	{0})
#define K_MEM_PARTITION_P_NA_U_NA ((k_mem_partition_attr_t) \
	{0})

/* Execution-allowed attributes */
#define K_MEM_PARTITION_P_RWX_U_RWX ((k_mem_partition_attr_t) \
	{PMP_R | PMP_W | PMP_X})
#define K_MEM_PARTITION_P_RX_U_RX ((k_mem_partition_attr_t) \
	{PMP_R | PMP_X})

/* Typedef for the k_mem_partition attribute */
typedef struct {
	uint8_t pmp_attr;
} k_mem_partition_attr_t;

struct arch_mem_domain {
	unsigned int pmp_update_nr;
};

void arch_irq_enable(unsigned int irq);
void arch_irq_disable(unsigned int irq);
int arch_irq_is_enabled(unsigned int irq);
void arch_irq_priority_set(unsigned int irq, unsigned int prio);
void z_irq_spurious(const void *unused);

#if defined(CONFIG_RISCV_HAS_PLIC)
#define ARCH_IRQ_CONNECT(irq_p, priority_p, isr_p, isr_param_p, flags_p) \
{ \
	Z_ISR_DECLARE(irq_p, 0, isr_p, isr_param_p); \
	arch_irq_priority_set(irq_p, priority_p); \
}
#elif defined(CONFIG_NUCLEI_ECLIC)
void nuclei_eclic_irq_priority_set(unsigned int irq, unsigned int prio, unsigned int flags);
#define ARCH_IRQ_CONNECT(irq_p, priority_p, isr_p, isr_param_p, flags_p) \
{ \
	Z_ISR_DECLARE(irq_p, 0, isr_p, isr_param_p); \
	nuclei_eclic_irq_priority_set(irq_p, priority_p, flags_p); \
}
#else
#define ARCH_IRQ_CONNECT(irq_p, priority_p, isr_p, isr_param_p, flags_p) \
{ \
	Z_ISR_DECLARE(irq_p, 0, isr_p, isr_param_p); \
}
#endif

/*
 * use atomic instruction csrrc to lock global irq
 * csrrc: atomic read and clear bits in CSR register
 */
static ALWAYS_INLINE unsigned int arch_irq_lock(void)
{
	unsigned int key;

	__asm__ volatile ("csrrc %0, mstatus, %1"
			  : "=r" (key)
			  : "rK" (MSTATUS_IEN)
			  : "memory");

	return key;
}

/*
 * use atomic instruction csrs to unlock global irq
 * csrs: atomic set bits in CSR register
 */
static ALWAYS_INLINE void arch_irq_unlock(unsigned int key)
{
	__asm__ volatile ("csrs mstatus, %0"
			  :
			  : "r" (key & MSTATUS_IEN)
			  : "memory");
}

static ALWAYS_INLINE bool arch_irq_unlocked(unsigned int key)
{
	return (key & MSTATUS_IEN) != 0;
}

static ALWAYS_INLINE void arch_nop(void)
{
	__asm__ volatile("nop");
}

extern uint32_t sys_clock_cycle_get_32(void);

static inline uint32_t arch_k_cycle_get_32(void)
{
	return sys_clock_cycle_get_32();
}

extern uint64_t sys_clock_cycle_get_64(void);

static inline uint64_t arch_k_cycle_get_64(void)
{
	return sys_clock_cycle_get_64();
}

#include <zephyr/arch/riscv/error.h>

#ifdef __cplusplus
}
#endif

#endif /*_ASMLANGUAGE */

#if defined(CONFIG_SOC_FAMILY_RISCV_PRIVILEGE)
#include <zephyr/arch/riscv/riscv-privilege/asm_inline.h>
#endif


#endif
