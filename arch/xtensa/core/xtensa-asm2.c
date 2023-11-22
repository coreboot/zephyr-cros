/*
 * Copyright (c) 2017, Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <xtensa-asm2.h>
#include <zephyr/kernel.h>
#include <ksched.h>
#include <zephyr/kernel_structs.h>
#include <kernel_internal.h>
#include <kswap.h>
#include <_soc_inthandlers.h>
#include <zephyr/toolchain.h>
#include <zephyr/logging/log.h>
#include <offsets.h>
#include <zsr.h>
#include <zephyr/arch/common/exc_handle.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

extern char xtensa_arch_except_epc[];
extern char xtensa_arch_kernel_oops_epc[];

#ifdef CONFIG_USERSPACE
Z_EXC_DECLARE(z_xtensa_user_string_nlen);

static const struct z_exc_handle exceptions[] = {
	Z_EXC_HANDLE(z_xtensa_user_string_nlen)
};

#ifdef CONFIG_THREAD_LOCAL_STORAGE
/*
 * Per-thread (TLS) variable indicating whether execution is in user mode.
 */
__thread uint32_t is_user_mode;
#endif

#endif /* CONFIG_USERSPACE */

void *xtensa_init_stack(struct k_thread *thread, int *stack_top,
			void (*entry)(void *, void *, void *),
			void *arg1, void *arg2, void *arg3)
{
	void *ret;
	_xtensa_irq_stack_frame_a11_t *frame;
#ifdef CONFIG_USERSPACE
	struct z_xtensa_thread_stack_header *header =
		(struct z_xtensa_thread_stack_header *)thread->stack_obj;

	thread->arch.psp = header->privilege_stack +
		sizeof(header->privilege_stack);
#endif

	/* Not-a-cpu ID Ensures that the first time this is run, the
	 * stack will be invalidated.  That covers the edge case of
	 * restarting a thread on a stack that had previously been run
	 * on one CPU, but then initialized on this one, and
	 * potentially run THERE and not HERE.
	 */
	thread->arch.last_cpu = -1;

	/* We cheat and shave 16 bytes off, the top four words are the
	 * A0-A3 spill area for the caller of the entry function,
	 * which doesn't exist.  It will never be touched, so we
	 * arrange to enter the function with a CALLINC of 1 and a
	 * stack pointer 16 bytes above the top, so its ENTRY at the
	 * start will decrement the stack pointer by 16.
	 */
	const int bsasz = sizeof(*frame) - 16;

	frame = (void *)(((char *) stack_top) - bsasz);

	(void)memset(frame, 0, bsasz);

	frame->bsa.ps = PS_WOE | PS_UM | PS_CALLINC(1);
#ifdef CONFIG_USERSPACE
	if ((thread->base.user_options & K_USER) == K_USER) {
		frame->bsa.pc = (uintptr_t)arch_user_mode_enter;
	} else {
		frame->bsa.pc = (uintptr_t)z_thread_entry;
	}
#else
	frame->bsa.pc = (uintptr_t)z_thread_entry;
#endif

#if XCHAL_HAVE_THREADPTR
#ifdef CONFIG_THREAD_LOCAL_STORAGE
	frame->bsa.threadptr = thread->tls;
#elif CONFIG_USERSPACE
	frame->bsa.threadptr = (uintptr_t)((thread->base.user_options & K_USER) ? thread : NULL);
#endif
#endif

	/* Arguments to z_thread_entry().  Remember these start at A6,
	 * which will be rotated into A2 by the ENTRY instruction that
	 * begins the C function.  And A4-A7 and A8-A11 are optional
	 * quads that live below the BSA!
	 */
	frame->a7 = (uintptr_t)arg1;  /* a7 */
	frame->a6 = (uintptr_t)entry; /* a6 */
	frame->a5 = 0;                /* a5 */
	frame->a4 = 0;                /* a4 */

	frame->a11 = 0;                /* a11 */
	frame->a10 = 0;                /* a10 */
	frame->a9  = (uintptr_t)arg3;  /* a9 */
	frame->a8  = (uintptr_t)arg2;  /* a8 */

	/* Finally push the BSA pointer and return the stack pointer
	 * as the handle
	 */
	frame->ptr_to_bsa = (void *)&frame->bsa;
	ret = &frame->ptr_to_bsa;

	return ret;
}

void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack,
		     char *stack_ptr, k_thread_entry_t entry,
		     void *p1, void *p2, void *p3)
{
	thread->switch_handle = xtensa_init_stack(thread,
						  (int *)stack_ptr, entry,
						  p1, p2, p3);
#ifdef CONFIG_KERNEL_COHERENCE
	__ASSERT((((size_t)stack) % XCHAL_DCACHE_LINESIZE) == 0, "");
	__ASSERT((((size_t)stack_ptr) % XCHAL_DCACHE_LINESIZE) == 0, "");
	sys_cache_data_flush_and_invd_range(stack, (char *)stack_ptr - (char *)stack);
#endif
}

void z_irq_spurious(const void *arg)
{
	int irqs, ie;

	ARG_UNUSED(arg);

	__asm__ volatile("rsr.interrupt %0" : "=r"(irqs));
	__asm__ volatile("rsr.intenable %0" : "=r"(ie));
	LOG_ERR(" ** Spurious INTERRUPT(s) %p, INTENABLE = %p",
		(void *)irqs, (void *)ie);
	z_xtensa_fatal_error(K_ERR_SPURIOUS_IRQ, NULL);
}

void z_xtensa_dump_stack(const z_arch_esf_t *stack)
{
	_xtensa_irq_stack_frame_raw_t *frame = (void *)stack;
	_xtensa_irq_bsa_t *bsa = frame->ptr_to_bsa;
	uintptr_t num_high_regs;
	int reg_blks_remaining;

	/* Calculate number of high registers. */
	num_high_regs = (uint8_t *)bsa - (uint8_t *)frame + sizeof(void *);
	num_high_regs /= sizeof(uintptr_t);

	/* And high registers are always comes in 4 in a block. */
	reg_blks_remaining = (int)num_high_regs / 4;

	LOG_ERR(" **  A0 %p  SP %p  A2 %p  A3 %p",
		(void *)bsa->a0,
		(void *)((char *)bsa + sizeof(*bsa)),
		(void *)bsa->a2, (void *)bsa->a3);

	if (reg_blks_remaining > 0) {
		reg_blks_remaining--;

		LOG_ERR(" **  A4 %p  A5 %p  A6 %p  A7 %p",
			(void *)frame->blks[reg_blks_remaining].r0,
			(void *)frame->blks[reg_blks_remaining].r1,
			(void *)frame->blks[reg_blks_remaining].r2,
			(void *)frame->blks[reg_blks_remaining].r3);
	}

	if (reg_blks_remaining > 0) {
		reg_blks_remaining--;

		LOG_ERR(" **  A8 %p  A9 %p A10 %p A11 %p",
			(void *)frame->blks[reg_blks_remaining].r0,
			(void *)frame->blks[reg_blks_remaining].r1,
			(void *)frame->blks[reg_blks_remaining].r2,
			(void *)frame->blks[reg_blks_remaining].r3);
	}

	if (reg_blks_remaining > 0) {
		reg_blks_remaining--;

		LOG_ERR(" ** A12 %p A13 %p A14 %p A15 %p",
			(void *)frame->blks[reg_blks_remaining].r0,
			(void *)frame->blks[reg_blks_remaining].r1,
			(void *)frame->blks[reg_blks_remaining].r2,
			(void *)frame->blks[reg_blks_remaining].r3);
	}

#if XCHAL_HAVE_LOOPS
	LOG_ERR(" ** LBEG %p LEND %p LCOUNT %p",
		(void *)bsa->lbeg,
		(void *)bsa->lend,
		(void *)bsa->lcount);
#endif

	LOG_ERR(" ** SAR %p", (void *)bsa->sar);
}

static inline unsigned int get_bits(int offset, int num_bits, unsigned int val)
{
	int mask;

	mask = BIT(num_bits) - 1;
	val = val >> offset;
	return val & mask;
}

static void print_fatal_exception(void *print_stack, int cause,
				  bool is_dblexc, uint32_t depc)
{
	void *pc;
	uint32_t ps, vaddr;
	_xtensa_irq_bsa_t *bsa = (void *)*(int **)print_stack;

	ps = bsa->ps;
	pc = (void *)bsa->pc;

	__asm__ volatile("rsr.excvaddr %0" : "=r"(vaddr));

	LOG_ERR(" ** FATAL EXCEPTION%s", (is_dblexc ? " (DOUBLE)" : ""));
	LOG_ERR(" ** CPU %d EXCCAUSE %d (%s)",
		arch_curr_cpu()->id, cause,
		z_xtensa_exccause(cause));
	LOG_ERR(" **  PC %p VADDR %p", pc, (void *)vaddr);

	if (is_dblexc) {
		LOG_ERR(" **  DEPC %p", (void *)depc);
	}

#ifdef CONFIG_USERSPACE
	LOG_ERR(" **  THREADPTR %p", (void *)bsa->threadptr);
#endif /* CONFIG_USERSPACE */

	LOG_ERR(" **  PS %p", (void *)bsa->ps);
	LOG_ERR(" **    (INTLEVEL:%d EXCM: %d UM:%d RING:%d WOE:%d OWB:%d CALLINC:%d)",
		get_bits(0, 4, ps), get_bits(4, 1, ps),
		get_bits(5, 1, ps), get_bits(6, 2, ps),
		get_bits(18, 1, ps),
		get_bits(8, 4, ps), get_bits(16, 2, ps));
}

static ALWAYS_INLINE void usage_stop(void)
{
#ifdef CONFIG_SCHED_THREAD_USAGE
	z_sched_usage_stop();
#endif
}

#ifdef CONFIG_MULTITHREADING
void *z_arch_get_next_switch_handle(struct k_thread *interrupted)
{
	return _current_cpu->nested <= 1 ?
		z_get_next_switch_handle(interrupted) : interrupted;
}
#else
void *z_arch_get_next_switch_handle(struct k_thread *interrupted)
{
	return interrupted;
}
#endif /* CONFIG_MULTITHREADING */

static inline void *return_to(void *interrupted)
{
	return z_arch_get_next_switch_handle(interrupted);
}

#if defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING)
int arch_float_disable(struct k_thread *thread)
{
	/* xtensa always has FPU enabled so cannot be disabled */
	return -ENOTSUP;
}

int arch_float_enable(struct k_thread *thread, unsigned int options)
{
	/* xtensa always has FPU enabled so nothing to do here */
	return 0;
}
#endif /* CONFIG_FPU && CONFIG_FPU_SHARING */

/* The wrapper code lives here instead of in the python script that
 * generates _xtensa_handle_one_int*().  Seems cleaner, still kind of
 * ugly.
 *
 * This may be unused depending on number of interrupt levels
 * supported by the SoC.
 */
#define DEF_INT_C_HANDLER(l)				\
__unused void *xtensa_int##l##_c(void *interrupted_stack)	\
{							   \
	uint32_t irqs, intenable, m;			   \
	usage_stop();					   \
	__asm__ volatile("rsr.interrupt %0" : "=r"(irqs)); \
	__asm__ volatile("rsr.intenable %0" : "=r"(intenable)); \
	irqs &= intenable;					\
	while ((m = _xtensa_handle_one_int##l(irqs))) {		\
		irqs ^= m;					\
		__asm__ volatile("wsr.intclear %0" : : "r"(m)); \
	}							\
	return return_to(interrupted_stack);		\
}

#if XCHAL_NMILEVEL >= 2
DEF_INT_C_HANDLER(2)
#endif

#if XCHAL_NMILEVEL >= 3
DEF_INT_C_HANDLER(3)
#endif

#if XCHAL_NMILEVEL >= 4
DEF_INT_C_HANDLER(4)
#endif

#if XCHAL_NMILEVEL >= 5
DEF_INT_C_HANDLER(5)
#endif

#if XCHAL_NMILEVEL >= 6
DEF_INT_C_HANDLER(6)
#endif

#if XCHAL_NMILEVEL >= 7
DEF_INT_C_HANDLER(7)
#endif

static inline DEF_INT_C_HANDLER(1)

/* C handler for level 1 exceptions/interrupts.  Hooked from the
 * DEF_EXCINT 1 vector declaration in assembly code.  This one looks
 * different because exceptions and interrupts land at the same
 * vector; other interrupt levels have their own vectors.
 */
void *xtensa_excint1_c(int *interrupted_stack)
{
	int cause;
	_xtensa_irq_bsa_t *bsa = (void *)*(int **)interrupted_stack;
	bool is_fatal_error = false;
	bool is_dblexc = false;
	uint32_t ps;
	void *pc, *print_stack = (void *)interrupted_stack;
	uint32_t depc = 0;

	__asm__ volatile("rsr.exccause %0" : "=r"(cause));

#ifdef CONFIG_XTENSA_MMU
	__asm__ volatile("rsr.depc %0" : "=r"(depc));

	is_dblexc = (depc != 0U);
#endif /* CONFIG_XTENSA_MMU */

	switch (cause) {
	case EXCCAUSE_LEVEL1_INTERRUPT:
		if (!is_dblexc) {
			return xtensa_int1_c(interrupted_stack);
		}
		break;
#ifndef CONFIG_USERSPACE
	/* Syscalls are handled earlier in assembly if MMU is enabled.
	 * So we don't need this here.
	 */
	case EXCCAUSE_SYSCALL:
		/* Just report it to the console for now */
		LOG_ERR(" ** SYSCALL PS %p PC %p",
			(void *)bsa->ps, (void *)bsa->pc);
		z_xtensa_dump_stack(interrupted_stack);

		/* Xtensa exceptions don't automatically advance PC,
		 * have to skip the SYSCALL instruction manually or
		 * else it will just loop forever
		 */
		bsa->pc += 3;
		break;
#endif /* !CONFIG_USERSPACE */
	default:
		ps = bsa->ps;
		pc = (void *)bsa->pc;

#ifdef CONFIG_USERSPACE
		/* If the faulting address is from one of the known
		 * exceptions that should not be fatal, return to
		 * the fixup address.
		 */
		for (int i = 0; i < ARRAY_SIZE(exceptions); i++) {
			if ((pc >= exceptions[i].start) &&
			    (pc < exceptions[i].end)) {
				bsa->pc = (uintptr_t)exceptions[i].fixup;

				goto fixup_out;
			}
		}
#endif /* CONFIG_USERSPACE */

		/* Default for exception */
		int reason = K_ERR_CPU_EXCEPTION;
		is_fatal_error = true;

		/* We need to distinguish between an ill in xtensa_arch_except,
		 * e.g for k_panic, and any other ill. For exceptions caused by
		 * xtensa_arch_except calls, we also need to pass the reason_p
		 * to z_xtensa_fatal_error. Since the ARCH_EXCEPT frame is in the
		 * BSA, the first arg reason_p is stored at the A2 offset.
		 * We assign EXCCAUSE the unused, reserved code 63; this may be
		 * problematic if the app or new boards also decide to repurpose
		 * this code.
		 *
		 * Another intentionally ill is from xtensa_arch_kernel_oops.
		 * Kernel OOPS has to be explicity raised so we can simply
		 * set the reason and continue.
		 */
		if (cause == EXCCAUSE_ILLEGAL) {
			if (pc == (void *)&xtensa_arch_except_epc) {
				cause = 63;
				__asm__ volatile("wsr.exccause %0" : : "r"(cause));
				reason = bsa->a2;
			} else if (pc == (void *)&xtensa_arch_kernel_oops_epc) {
				cause = 64; /* kernel oops */
				reason = K_ERR_KERNEL_OOPS;

				/* A3 contains the second argument to
				 * xtensa_arch_kernel_oops(reason, ssf)
				 * where ssf is the stack frame causing
				 * the kernel oops.
				 */
				print_stack = (void *)bsa->a3;
			}
		}

		if (reason != K_ERR_KERNEL_OOPS) {
			print_fatal_exception(print_stack, cause, is_dblexc, depc);
		}

		/* FIXME: legacy xtensa port reported "HW" exception
		 * for all unhandled exceptions, which seems incorrect
		 * as these are software errors.  Should clean this
		 * up.
		 */
		z_xtensa_fatal_error(reason,
				     (void *)print_stack);
		break;
	}

#ifdef CONFIG_XTENSA_MMU
	switch (cause) {
	case EXCCAUSE_LEVEL1_INTERRUPT:
#ifndef CONFIG_USERSPACE
	case EXCCAUSE_SYSCALL:
#endif /* !CONFIG_USERSPACE */
		is_fatal_error = false;
		break;
	default:
		is_fatal_error = true;
		break;
	}
#endif /* CONFIG_XTENSA_MMU */

	if (is_dblexc || is_fatal_error) {
		uint32_t ignore;

		/* We are going to manipulate _current_cpu->nested manually.
		 * Since the error is fatal, for recoverable errors, code
		 * execution must not return back to the current thread as
		 * it is being terminated (via above z_xtensa_fatal_error()).
		 * So we need to prevent more interrupts coming in which
		 * will affect the nested value as we are going outside of
		 * normal interrupt handling procedure.
		 *
		 * Setting nested to 1 has two effects:
		 * 1. Force return_to() to choose a new thread.
		 *    Since the current thread is being terminated, it will
		 *    not be chosen again.
		 * 2. When context switches to the newly chosen thread,
		 *    nested must be zero for normal code execution,
		 *    as that is not in interrupt context at all.
		 *    After returning from this function, the rest of
		 *    interrupt handling code will decrement nested,
		 *    resulting it being zero before switching to another
		 *    thread.
		 */
		__asm__ volatile("rsil %0, %1"
				: "=r" (ignore) : "i"(XCHAL_NMILEVEL));

		_current_cpu->nested = 1;
	}

#ifdef CONFIG_XTENSA_MMU
#ifdef CONFIG_USERSPACE
fixup_out:
#endif
	if (is_dblexc) {
		__asm__ volatile("wsr.depc %0" : : "r"(0));
	}
#endif /* CONFIG_XTENSA_MMU */


	return return_to(interrupted_stack);
}

#if defined(CONFIG_GDBSTUB)
void *xtensa_debugint_c(int *interrupted_stack)
{
	extern void z_gdb_isr(z_arch_esf_t *esf);

	z_gdb_isr((void *)interrupted_stack);

	return return_to(interrupted_stack);
}
#endif

int z_xtensa_irq_is_enabled(unsigned int irq)
{
	uint32_t ie;

	__asm__ volatile("rsr.intenable %0" : "=r"(ie));

	return (ie & (1 << irq)) != 0U;
}

#ifdef CONFIG_XTENSA_MORE_SPIN_RELAX_NOPS
/* Some compilers might "optimize out" (i.e. remove) continuous NOPs.
 * So force no optimization to avoid that.
 */
__no_optimization
void arch_spin_relax(void)
{
#define NOP1(_, __) __asm__ volatile("nop.n;");
	LISTIFY(CONFIG_XTENSA_NUM_SPIN_RELAX_NOPS, NOP1, (;))
#undef NOP1
}
#endif /* CONFIG_XTENSA_MORE_SPIN_RELAX_NOPS */

#ifdef CONFIG_USERSPACE
FUNC_NORETURN void arch_user_mode_enter(k_thread_entry_t user_entry,
					void *p1, void *p2, void *p3)
{
	struct k_thread *current = _current;
	size_t stack_end;

	/* Transition will reset stack pointer to initial, discarding
	 * any old context since this is a one-way operation
	 */
	stack_end = Z_STACK_PTR_ALIGN(current->stack_info.start +
				      current->stack_info.size -
				      current->stack_info.delta);

	z_xtensa_userspace_enter(user_entry, p1, p2, p3,
				 stack_end, current->stack_info.start);

	CODE_UNREACHABLE;
}
#endif /* CONFIG_USERSPACE */
