/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <init.h>
#include <kernel.h>
#include <kernel_structs.h>
#include <toolchain.h>
#include <sys/__assert.h>
#include <sys/sys_io.h>

#include <xtensa/config/core-isa.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(soc_mp, CONFIG_SOC_LOG_LEVEL);

#include <zsr.h>
#include <cavs-idc.h>
#include <soc.h>
#include <arch/xtensa/cache.h>
#include <cavs-shim.h>
#include <cavs-mem.h>
#include <cpu_init.h>

struct cpustart_rec {
	uint32_t        cpu;
	arch_cpustart_t	fn;
	void            *arg;
};

static struct cpustart_rec start_rec;

char *z_mp_stack_top;

/* Vestigial silliness: An old mechanism for core startup would embed
 * a "manifest" of code to copy to LP-SRAM at startup (vs. the tiny
 * trampoline we use here).  This was constructed in the linker
 * script, and the first word would encode the number of entries.  As
 * it happens, SOF still emits the code to copy this data, so it needs
 * to see this symbol point to a zero.
 */
uint32_t _loader_storage_manifest_start;

/* Simple array of CPUs that are active and available for an IPI.  The
 * IDC interrupt is ALSO used to bring a CPU out of reset, so we need
 * to be absolutely sure we don't try to IPI a CPU that isn't ready to
 * start, or else we'll launch it into garbage and crash the DSP.
 */
bool soc_cpus_active[CONFIG_MP_NUM_CPUS];

/* Tiny assembly stub for calling z_mp_entry() on the auxiliary CPUs.
 * Mask interrupts, clear the register window state and set the stack
 * pointer.  This represents the minimum work required to run C code
 * safely.
 *
 * Note that alignment is absolutely required: the IDC protocol passes
 * only the upper 30 bits of the address to the second CPU.
 */
__asm__(".align 4                   \n\t"
	".global z_soc_mp_asm_entry \n\t"
	"z_soc_mp_asm_entry:        \n\t"
	"  movi  a0, 0x4002f        \n\t" /* WOE | UM | INTLEVEL(max) */
	"  wsr   a0, PS             \n\t"
	"  movi  a0, 0              \n\t"
	"  wsr   a0, WINDOWBASE     \n\t"
	"  movi  a0, 1              \n\t"
	"  wsr   a0, WINDOWSTART    \n\t"
	"  rsync                    \n\t"
	"  movi  a1, z_mp_stack_top \n\t"
	"  l32i  a1, a1, 0          \n\t"
	"  call4 z_mp_entry         \n\t");

__imr void z_mp_entry(void)
{
	cpu_early_init();

	/* We don't know what the boot ROM (on pre-2.5 DSPs) might
	 * have touched and we don't care.  Make sure it's not in our
	 * local cache to be flushed accidentally later.
	 *
	 * Note that technically this is dropping our own (cached)
	 * stack memory, which we don't have a guarantee the compiler
	 * isn't using yet.  Manual inspection of generated code says
	 * we're safe, but really we need a better solution here.
	 */
	if (!IS_ENABLED(CONFIG_SOC_SERIES_INTEL_CAVS_V25)) {
		z_xtensa_cache_flush_inv_all();
	}

	/* Set up the CPU pointer. */
	_cpu_t *cpu = &_kernel.cpus[start_rec.cpu];

	__asm__ volatile("wsr %0, " ZSR_CPU_STR :: "r"(cpu));

	soc_mp_startup(start_rec.cpu);
	soc_cpus_active[start_rec.cpu] = true;
	start_rec.fn(start_rec.arg);
	__ASSERT(false, "arch_start_cpu() handler should never return");
}

bool arch_cpu_active(int cpu_num)
{
	return soc_cpus_active[cpu_num];
}

void arch_start_cpu(int cpu_num, k_thread_stack_t *stack, int sz,
		    arch_cpustart_t fn, void *arg)
{
	__ASSERT_NO_MSG(!soc_cpus_active[cpu_num]);

	start_rec.cpu = cpu_num;
	start_rec.fn = fn;
	start_rec.arg = arg;

	z_mp_stack_top = Z_THREAD_STACK_BUFFER(stack) + sz;

	soc_start_core(cpu_num);
}

/* Fallback stub for external SOF code */
__imr int cavs_idc_smp_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}
