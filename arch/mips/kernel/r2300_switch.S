/* $Id: r2300_switch.S,v 1.2 1997/11/30 05:50:02 davem Exp $
 * r2300_switch.S: R3000/R2000 specific task switching code.
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

/* XXX The following is fucking losing... find a better way dave. */
MODE_ALIAS	=	0x00e0			# uncachable, dirty, valid

	.text
	.set	mips3
/*
 * Code necessary to switch tasks on an Linux/MIPS machine.
 * FIXME: We don't need to disable interrupts anymore.
 */
	.align	5
	LEAF(r2300_resume)
	lw	t0,current_set
	mfc0	t1,CP0_STATUS		# Save status register
	addu	t0,a1			# Add tss offset
	sw	t1,THREAD_STATUS(t0)
	ori	t2,t1,0x1f		# Disable interrupts
	xori	t2,0x1e
	mtc0	t2,CP0_STATUS
	CPU_SAVE_NONSCRATCH(t0)
	sll	t2,t1,2			# Save floating point state
	bgez	t2,2f
	 sw	ra,THREAD_REG31(t0)
	sll	t2,t1,5
	bgez	t2,1f
	 swc1	$f0, (THREAD_FPU + 0x00)(t0)
	FPU_SAVE_16ODD(t0)
1:
	FPU_SAVE_16EVEN(t0, t1)
2:
	addu	a0,a1			# Add tss offset
	mfc0	t0, CP0_TAGLO
	li	t1,TLB_ROOT		# get PFN
	mtc0	t1,CP0_ENTRYHI
	mtc0	zero,CP0_INDEX
	srl	t0,12			# PFN is 12 bits west
	ori	t0,MODE_ALIAS		# want uncachable, dirty, valid
	mtc0	t0,CP0_ENTRYLO0
	lw	a2,THREAD_STATUS(a0)
	tlbwi

	/* Flush TLB. */
	mfc0	t3,CP0_STATUS		# disable interrupts...
	ori	t4,t3,1
	xori	t4,1
	mtc0	t4,CP0_STATUS
	lw      t0,mips_tlb_entries
	mtc0	zero,CP0_ENTRYLO0
1:
	subu	t0,1
	mtc0	t0,CP0_INDEX
	lui	t1,0x0008
	or	t1,t0,t1
	sll	t1,12
	mtc0	t1,CP0_ENTRYHI
	bne	t2,t0,1b
	 tlbwi

	ori	t1,a2,1			# Restore FPU, pipeline magic
	xori	t1,1
	mtc0	t1,CP0_STATUS
	sll	t0,a2,2
	bgez	t0,2f
	 sll	t0,a2,5
	bgez	t0,1f
	 lwc1	$f0, (THREAD_FPU + 0x00)(a0)
	FPU_RESTORE_16ODD(a0)
1:
	FPU_RESTORE_16EVEN(a0, t0)
2:
	CPU_RESTORE_NONSCRATCH(a0)
	lw	t0,THREAD_KSP(a0)		# Restore status register
	sw	t0,kernelsp
	jr	ra
	 mtc0	a2,CP0_STATUS
	END(r2300_resume)


