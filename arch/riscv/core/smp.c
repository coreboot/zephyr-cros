/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <kernel.h>
#include <ksched.h>

volatile struct {
	arch_cpustart_t fn;
	void *arg;
} riscv_cpu_init[CONFIG_MP_NUM_CPUS];

volatile uintptr_t riscv_cpu_wake_flag;
volatile void *riscv_cpu_sp;

void arch_start_cpu(int cpu_num, k_thread_stack_t *stack, int sz,
		    arch_cpustart_t fn, void *arg)
{
	riscv_cpu_init[cpu_num].fn = fn;
	riscv_cpu_init[cpu_num].arg = arg;

	riscv_cpu_sp = Z_THREAD_STACK_BUFFER(stack) + sz;
	riscv_cpu_wake_flag = cpu_num;

	while (riscv_cpu_wake_flag != 0U) {
		;
	}
}

void z_riscv_secondary_cpu_init(int cpu_num)
{
#if defined(CONFIG_RISCV_SOC_INTERRUPT_INIT)
	soc_interrupt_init();
#endif
#ifdef CONFIG_PMP_STACK_GUARD
	z_riscv_configure_interrupt_stack_guard();
#endif
#ifdef CONFIG_SMP
	irq_enable(RISCV_MACHINE_SOFT_IRQ);
#endif
	riscv_cpu_init[cpu_num].fn(riscv_cpu_init[cpu_num].arg);
}

#ifdef CONFIG_SMP
static uintptr_t *get_hart_msip(int hart_id)
{
#ifdef CONFIG_64BIT
	return (uintptr_t *)(uint64_t)(RISCV_MSIP_BASE + (hart_id * 4));
#else
	return (uintptr_t *)(RISCV_MSIP_BASE + (hart_id * 4));
#endif
}

void arch_sched_ipi(void)
{
	unsigned int key;
	uint32_t i;
	uint8_t id;

	key = arch_irq_lock();

	id = _current_cpu->id;
	for (i = 0U; i < CONFIG_MP_NUM_CPUS; i++) {
		if (i != id) {
			volatile uint32_t *r = (uint32_t *)get_hart_msip(i);
			*r = 1U;
		}
	}

	arch_irq_unlock(key);
}

static void sched_ipi_handler(const void *unused)
{
	ARG_UNUSED(unused);

	volatile uint32_t *r = (uint32_t *)get_hart_msip(_current_cpu->id);
	*r = 0U;

	z_sched_ipi();
}

static int riscv_smp_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	IRQ_CONNECT(RISCV_MACHINE_SOFT_IRQ, 0, sched_ipi_handler, NULL, 0);
	irq_enable(RISCV_MACHINE_SOFT_IRQ);

	return 0;
}

SYS_INIT(riscv_smp_init, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif /* CONFIG_SMP */
