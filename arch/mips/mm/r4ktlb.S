/* $Id: r4ktlb.S,v 1.3 1997/12/13 04:04:40 davem Exp $
 * r4ktlb.S: r4xx0 TLB operations
 *
 * Copyright (C) 1997 David S. Miller (davem@dm.cobaltmicro.com)
 */

#include <asm/asm.h>
#include <asm/regdef.h>

	.text
	.align	4
	.set	mips3
	.set	noat
	.set	noreorder

	.globl	__flush_tlb_page	/* a0=mm->context, a1=page */
__flush_tlb_page:
	subu	t0, zero, (4096 << 1)	/* (PAGE_MASK << 1)		*/
	mfc0	t9, $12			/* save_flags			*/
	andi	a0, a0, 0xff		/* ctx &= 0xff			*/
	mfc0	t8, $10			/* oldpid = get_entryhi()	*/
	ori	t1, t9, 0x01
	xori	t1, t1, 0x01
	mtc0	t1, $12			/* cli				*/
	and	a1, a1, t0		/* page &= (PAGE_MASK << 1)	*/
	or	t3, a1, a0		/* ehi = (page | ctx)		*/
	mtc0	t3, $10			/* set_entryhi(ehi)		*/
	nop				/* CP0 hazard			*/
	tlbp				/* Entry in there at all?	*/
	nop				/* CP0 hazard			*/
	mfc0	t4, $0			/* get_index()			*/
	lui	t5, %hi(0x80000000)	/* ehi2 = KSEG0			*/
	bltz	t4, 1f			/* if not found, exit		*/
	 nop
	mtc0	zero, $2		/* set_entrylo0(0)		*/
	mtc0	zero, $3		/* set_entrylo1(0)		*/
	mtc0	t5, $10			/* set_entryhi(ehi2)		*/
	nop				/* CP0 hazard			*/
	tlbwi				/* tlb_write_indexed()		*/
	nop				/* CP0 hazard			*/
	nop				/* CP0 hazard			*/
1:	mtc0	t8, $10			/* set_entryhi(oldpid)		*/
	nop
	mtc0	t9, $12			/* restore_flags		*/
	nop
	j	ra			/* return to caller		*/
	 nop				/* CP0 hazard			*/

	.globl	__flush_tlb_range
__flush_tlb_range:	/* a0=mm->context, a1=start, a2=end */
	subu	t6, zero, (4096 << 1)	/* t6 = PAGE_MASK << 1		*/
	mfc0	t9, $12			/* save_flags			*/
	and	a1, a1, t6		/* start &= t6			*/
	addu	a2, a2, (8192 - 1)	/* end += ((PAGE_SIZE << 1) - 1)*/
	mfc0	t8, $10			/* oldpid = get_entryhi()	*/
	and	a2, a2, t6		/* end &= t6			*/
	ori	t0, t9, 0x01
	xori	t0, t0, 0x01
	mtc0	t0, $12			/* cli				*/
	sltu	AT, a2, a1		/* AT = (start > end)		*/
	beq	a2, a1, 3f		/* if(start==end) goto out	*/
	 lui	t5, %hi(0x80000000)	/* t5 = KSEG0			*/
	bne	AT, zero, 3f		/* if (AT != 0) goto out	*/
	 andi	a0, a0, 0xff		/* context &= 0xff		*/
1:	or	t0, a1, a0		/* t0 = (start | context)	*/
	addu	a1, a1, (4096 << 1)	/* start += (PAGE_SIZE << 1)	*/
	mtc0	t0, $10			/* set_entryhi(t0)		*/
	nop				/* CP0 Hazard			*/
	tlbp				/* Entry in there at all?	*/
	nop				/* CP0 Hazard			*/
	mfc0	t1, $0			/* idx = get_index()		*/
	nop				/* CP0 Hazard			*/
	bltz	t1, 2f			/* if(idx < 0) continue		*/
	 nop				/* CP0 Hazard			*/
	mtc0	zero, $2		/* set_entrylo0(0)		*/
	mtc0	zero, $3		/* set_entrylo1(0)		*/
	mtc0	t5, $10			/* set_entryhi(t5)		*/
	nop				/* CP0 Hazard			*/
	tlbwi				/* tlb_write_indexed()		*/
	nop				/* CP0 Hazard			*/
	nop				/* CP0 Hazard			*/
2:	bne	a1, a2, 1b		/* while(start != end)		*/
	 nop				/* dslot			*/
3:	mtc0	t8, $10			/* set_entryhi(oldpid)		*/
	nop				/* CP0 Hazard			*/
	mtc0	t9, $12			/* restore_flags		*/
	nop				/* CP0 Hazard			*/
	j	ra			/* return to caller		*/
	 nop				/* CP0 Hazard			*/

	.globl	flush_tlb_all
flush_tlb_all:
	mfc0	t9, $12			/* save_flags()			*/
	mfc0	t8, $10			/* oldpid = get_entryhi()	*/
	move	t0, zero		/* entry = 0			*/
	ori	t1, t9, 0x01
	xori	t1, t1, 0x01
	mtc0	t1, $12			/* cli()			*/
	lui	t5, %hi(0x80000000)	/* tmp=KSEG0			*/
	mtc0	zero, $2		/* set_entrylo0(0)		*/
	mtc0	zero, $3		/* set_entrylo1(0)		*/
	mtc0	t5, $10			/* set_entryhi(tmp)		*/
1:	mtc0	t0, $0			/* set_index(entry)		*/
	nop				/* CP0 Hazard			*/
	tlbwi				/* tlb_write_indexed()		*/
	nop				/* CP0 Hazard			*/
	add	t0, t0, 0x01		/* entry++			*/
	slt	AT, t0, 48		/* AT = (entry < NTLB_ENTRIES)	*/
	bne	AT, zero, 1b		/* if(AT) keep looping		*/
	 nop				/* dslot			*/
	mtc0	t8, $10			/* set_entryhi(oldpid)		*/
	nop				/* CP0 Hazard			*/
	mtc0	t9, $12			/* restore_flags()		*/
	nop				/* CP0 Hazard			*/
	j	ra			/* return to caller		*/
	 nop				/* CP0 Hazard			*/
