/* $Id: r4k_switch.S,v 1.5 1998/06/18 05:52:22 davem Exp $
 * r4k_switch.S: R4xx0 specific task switching code.
 *
 * Copyright (C) 1994, 1995, 1996 by Ralf Baechle and Andreas Busse
 *
 * Multi-cpu abstraction and macros for easier reading:
 * Copyright (C) 1996 David S. Miller (dm@engr.sgi.com)
 */
#include <asm/asm.h>
#include <asm/bootinfo.h>
#include <asm/cachectl.h>
#include <asm/fpregdef.h>
#include <asm/mipsconfig.h>
#include <asm/mipsregs.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/processor.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>

#include <asm/asmmacro.h>

	.set	noreorder
	.set	mips3
	.align	5
	LEAF(r4xx0_resume)
	lui	t5, %hi(current_set)
	lw	t0, %lo(current_set)(t5)
	mfc0	t1, CP0_STATUS
	nop
	sw	t1, THREAD_STATUS(t0)
	ori	t2, t1, 0x1f
	xori	t2, t2, 0x1e
	mtc0	t2, CP0_STATUS
	CPU_SAVE_NONSCRATCH(t0)
	sll	t2, t1, 2		# Save floating point state
	bgez	t2, 2f
	 sw	ra, THREAD_REG31(t0)
	cfc1	t1,  fcr31
	swc1	$f0, (THREAD_FPU + 0x00)(t0)
	swc1	$f1, (THREAD_FPU + 0x04)(t0)
	swc1	$f2, (THREAD_FPU + 0x08)(t0)
	swc1	$f3, (THREAD_FPU + 0x0c)(t0)
	swc1	$f4, (THREAD_FPU + 0x10)(t0)
	swc1	$f5, (THREAD_FPU + 0x14)(t0)
	swc1	$f6, (THREAD_FPU + 0x18)(t0)
	swc1	$f7, (THREAD_FPU + 0x1c)(t0)
	swc1	$f8, (THREAD_FPU + 0x20)(t0)
	swc1	$f9, (THREAD_FPU + 0x24)(t0)
	swc1	$f10, (THREAD_FPU + 0x28)(t0)
	swc1	$f11, (THREAD_FPU + 0x2c)(t0)
	swc1	$f12, (THREAD_FPU + 0x30)(t0)
	swc1	$f13, (THREAD_FPU + 0x34)(t0)
	swc1	$f14, (THREAD_FPU + 0x38)(t0)
	swc1	$f15, (THREAD_FPU + 0x3c)(t0)
	swc1	$f16, (THREAD_FPU + 0x40)(t0)
	swc1	$f17, (THREAD_FPU + 0x44)(t0)
	swc1	$f18, (THREAD_FPU + 0x48)(t0)
	swc1	$f19, (THREAD_FPU + 0x4c)(t0)
	swc1	$f20, (THREAD_FPU + 0x50)(t0)
	swc1	$f21, (THREAD_FPU + 0x54)(t0)
	swc1	$f22, (THREAD_FPU + 0x58)(t0)
	swc1	$f23, (THREAD_FPU + 0x5c)(t0)
	swc1	$f24, (THREAD_FPU + 0x60)(t0)
	swc1	$f25, (THREAD_FPU + 0x64)(t0)
	swc1	$f26, (THREAD_FPU + 0x68)(t0)
	swc1	$f27, (THREAD_FPU + 0x6c)(t0)
	swc1	$f28, (THREAD_FPU + 0x70)(t0)
	swc1	$f29, (THREAD_FPU + 0x74)(t0)
	swc1	$f30, (THREAD_FPU + 0x78)(t0)
	swc1	$f31, (THREAD_FPU + 0x7c)(t0)
	sw	t1,   (THREAD_FPU + 0x80)(t0)
2:
	sw	a0, %lo(current_set)(t5)
	lw	a3, TASK_MM(a0)
	lw	a2, THREAD_STATUS(a0)
	lw	v1, MM_PGD(a3)
	lw	a3, MM_CONTEXT(a3)
	ori	t1, a2, 1		# restore fpu, pipeline magic
	andi	a3, a3, 0xff
	xori	t1, t1, 1
	mtc0	a3, CP0_ENTRYHI
	mtc0	v1, CP0_TAGLO
	mtc0	t1, CP0_STATUS
	sll	t0, a2, 2
	bgez	t0, 2f
	 nop
	lwc1	$f0, (THREAD_FPU + 0x00)(a0)
	lwc1	$f1, (THREAD_FPU + 0x04)(a0)
	lwc1	$f2, (THREAD_FPU + 0x08)(a0)
	lwc1	$f3, (THREAD_FPU + 0x0c)(a0)
	lwc1	$f4, (THREAD_FPU + 0x10)(a0)
	lwc1	$f5, (THREAD_FPU + 0x14)(a0)
	lwc1	$f6, (THREAD_FPU + 0x18)(a0)
	lwc1	$f7, (THREAD_FPU + 0x1c)(a0)
	lwc1	$f8, (THREAD_FPU + 0x20)(a0)
	lwc1	$f9, (THREAD_FPU + 0x24)(a0)
	lwc1	$f10, (THREAD_FPU + 0x28)(a0)
	lwc1	$f11, (THREAD_FPU + 0x2c)(a0)
	lwc1	$f12, (THREAD_FPU + 0x30)(a0)
	lwc1	$f13, (THREAD_FPU + 0x34)(a0)
	lwc1	$f14, (THREAD_FPU + 0x38)(a0)
	lwc1	$f15, (THREAD_FPU + 0x3c)(a0)
	lwc1	$f16, (THREAD_FPU + 0x40)(a0)
	lwc1	$f17, (THREAD_FPU + 0x44)(a0)
	lwc1	$f18, (THREAD_FPU + 0x48)(a0)
	lwc1	$f19, (THREAD_FPU + 0x4c)(a0)
	lwc1	$f20, (THREAD_FPU + 0x50)(a0)
	lwc1	$f21, (THREAD_FPU + 0x54)(a0)
	lwc1	$f22, (THREAD_FPU + 0x58)(a0)
	lwc1	$f23, (THREAD_FPU + 0x5c)(a0)
	lwc1	$f24, (THREAD_FPU + 0x60)(a0)
	lwc1	$f25, (THREAD_FPU + 0x64)(a0)
	lwc1	$f26, (THREAD_FPU + 0x68)(a0)
	lwc1	$f27, (THREAD_FPU + 0x6c)(a0)
	lwc1	$f28, (THREAD_FPU + 0x70)(a0)
	lwc1	$f29, (THREAD_FPU + 0x74)(a0)
	lwc1	$f30, (THREAD_FPU + 0x78)(a0)
	lwc1	$f31, (THREAD_FPU + 0x7c)(a0)
	lw	t0,  (THREAD_FPU + 0x80)(a0)
	ctc1	t0,  fcr31
2:
	CPU_RESTORE_NONSCRATCH(a0)
	lw	t0, THREAD_KSP(a0)
	sw	t0, kernelsp
	jr	ra
	 mtc0	a2, CP0_STATUS
	END(r4xx0_resume)
