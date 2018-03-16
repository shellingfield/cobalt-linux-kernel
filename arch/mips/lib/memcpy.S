/* $Id: memcpy.S,v 1.4 1998/06/10 05:16:21 ingo Exp $
 * memcpy.S: High performance memcpy for MIPS.
 *
 * Copyright (C) 1997 David S. Miller (davem@dm.cobaltmicro.com)
 *               1998 Ingo Molnar (mingo@hal.cobaltmicro.com)
 */

#include <asm/asm.h>
#include <asm/regdef.h>
#include <asm/cacheops.h>

	.text
	.globl	bcopy
	.globl	memmove
	.globl	__memcpy
	.globl	memcpy

	.set	noreorder
	.set	noat
	.set	mips3
	.align	6
bcopy:	move	a3, a0
	move	a0, a1
	b	memcpy
	 move	a1, a3
	nop
	nop
	nop
	nop

word_align:
	andi	AT, a1, 0x1
	beq	AT, zero, 1f
	 andi	AT, a1, 0x2
	lbu	t0, 0x00(a1)
	addu	a1, a1, 0x01
	subu	a2, a2, 0x01
	sb	t0, 0x00(a0)
	bne	AT, zero, 2f

	 addu	a0, a0, 0x01
1:	lhu	t0, 0x00(a1)
	addu	a1, a1, 0x02
	subu	a2, a2, 0x02
	sh	t0, 0x00(a0)
	b	2f
	 addu	a0, a0, 0x02
	nop

memmove:
__memcpy:	/* a0=dst, a1=src, a2=len */
memcpy:	subu	t2, zero, 0x40
	subu	AT, a1, a0
	sltu	t8, a2, 16
	bne	t8, zero, small_chunk
	 andi	AT, AT, 0x3
	bne	AT, zero, non_aligned
	 andi	AT, a1, 0x3
	bne	AT, zero, word_align

2:	 and	t8, a2, t2
	beq	t8, zero, noloop
	 move	a3, a2

/*
 * align dst to cacheline
 *
 * it's not a problem to have some overhead here,
 * we already know that this copy is going to be long
 */

        andi    AT, a0, 0x1f
        beq     AT, zero, unrloop
         subu   AT, zero, 0x20
        and     AT, AT, a0
        addu    AT, AT, 0x20

preloop:
        lw      t0, 0x00(a1)
        addu    a1, a1, 0x04
        sw      t0, 0x00(a0)
        addu   a0, a0, 0x04
        bne     a0, AT, preloop
         subu    a2, a2, 0x04

        and     t8, a2, t2
        beq     t8, zero, noloop
         move   a3, a2

unrloop:lw	t0, 0x00(a1)
	lw	t1, 0x04(a1)
	lw	t2, 0x08(a1)
	lw	t3, 0x0c(a1)
	addu	a0, a0, 32

	lw	t4, 0x10(a1)
	lw	t5, 0x14(a1)
	cache   Create_Dirty_Excl_D, -0x20(a0)
	lw	t6, 0x18(a1)
	lw	t7, 0x1c(a1)
	subu	t8, t8, 32
	sw	t0, -0x20(a0)
	sw	t1, -0x1c(a0)
	sw	t2, -0x18(a0)

	sw	t3, -0x14(a0)
	addu	a1, a1, 32
	sw	t4, -0x10(a0)
	sw	t5, -0x0c(a0)
	sw	t6, -0x08(a0)
	bne	t8, zero, unrloop
	 sw	t7, -0x04(a0)
noloop:	andi	t8, a3, 0x30

	lui	t5, %hi(dtblend)
	bne	t8, zero, ddevice
	 andi	t0, a3, 0x08
ckdword:beq	t0, zero, ckword
	 andi	t1, a3, 0x4
	lw	t2, 0x00(a1)
	addu	a0, a0, 0x8
	lw	t3, 0x04(a1)

	sw	t2, (-0x08)(a0)
	addu	a1, a1, 0x8
	sw	t3, (-0x04)(a0)
ckword:	beq	t1, zero, ckhword
	 andi	t1, a3, 0x2
	lw	t2, 0x00(a1)
	addu	a1, a1, 0x4
	sw	t2, 0x00(a0)

	addu	a0, a0, 0x4
ckhword:beq	t1, zero, ckbyte
	 andi	t1, a3, 0x1
	lhu	t2, 0x00(a1)
	addu	a1, a1, 0x2
	sh	t2, 0x00(a0)
	addu	a0, a0, 0x2
ckbyte:	beq	t1, zero, finish

	 nop
	lbu	t2, 0x00(a1)
	sb	t2, 0x00(a0)
finish:	jr	ra
	 move	v0, t9
ddevice:addiu	t5, t5, %lo(dtblend)	/* Duff it baby... */
	sll	t4, t8, 1
	subu	t5, t5, t4

	addu	a1, t8, a1
	jr	t5
	 addu	a0, t8, a0
	lw	t2, -0x30(a1)
	lw	t3, -0x2c(a1)
	lw	t4, -0x28(a1)
	lw	t5, -0x24(a1)
	sw	t2, -0x30(a0)

	sw	t3, -0x2c(a0)
	sw	t4, -0x28(a0)
	sw	t5, -0x24(a0)
	lw	t2, -0x20(a1)
	lw	t3, -0x1c(a1)
	lw	t4, -0x18(a1)
	lw	t5, -0x14(a1)
	sw	t2, -0x20(a0)

	sw	t3, -0x1c(a0)
	sw	t4, -0x18(a0)
	sw	t5, -0x14(a0)
	lw	t2, -0x10(a1)
	lw	t3, -0x0c(a1)
	lw	t4, -0x08(a1)
	lw	t5, -0x04(a1)
	sw	t2, -0x10(a0)

	sw	t3, -0x0c(a0)
	sw	t4, -0x08(a0)
	b	ckdword
dtblend: sw	t5, -0x04(a0)	/* Yes, the label belongs here. */

#ifdef __MIPSEB__
#define	lw_lo	lwl
#define lw_hi	lwr
#else
#define	lw_lo	lwr
#define lw_hi	lwl
#endif

non_aligned:
	andi	AT, a0, 0x3
	beq	AT, zero, 2f
	 ori	t4, zero, 0x4
	subu	t4, t4, AT
	subu	a2, a2, t4
	addu	t4, t4, a0
1:	lbu	t0, 0x00(a1)
	addu	a1, a1, 0x01
	addu	a0, a0, 0x01
	bne	a0, t4, 1b
	 sb	t0, -0x01(a0)
2:
	addu	AT, zero, -16		/* 0xfffffff0 */
	and	t4, a2, AT
	beq	t4, zero, 8f
	 addu	t4, t4, a0

16:	lw_lo	t0, 0x00(a1)
	lw_hi	t0, 0x03(a1)
	lw_lo	t1, 0x04(a1)
	lw_hi	t1, 0x07(a1)
	sw	t0, 0x00(a0)
	lw_lo	t2, 0x08(a1)
	lw_hi	t2, 0x0b(a1)
	sw	t1, 0x04(a0)
	lw_lo	t3, 0x0c(a1)
	lw_hi	t3, 0x0f(a1)
	sw	t2, 0x08(a0)
	sw	t3, 0x0c(a0)
	addu	a0, a0, 0x10
	bne	t4, a0, 16b
	 addu	a1, a1, 0x10

8:	andi	AT, a2, 0x08
	beq	AT, zero, 4f
	 nop
	lw_lo	t0, 0x00(a1)
	lw_hi	t0, 0x03(a1)
	lw_lo	t1, 0x04(a1)
	lw_hi	t1, 0x07(a1)
	sw	t0, 0x00(a0)
	sw	t1, 0x04(a0)
	addu	a1, a1, 0x08
	addu	a0, a0, 0x08

4:	andi	AT, a2, 0x04
	beq	AT, zero, small_chunk
	 andi	a2, a2, 0x03
	lw_lo	t0, 0x00(a1)
	lw_hi	t0, 0x03(a1)
	sw	t0, 0x00(a0)
	addu	a1, a1, 0x04
	addu	a0, a0, 0x04

small_chunk:
	beq	a2, zero, finish
	 addu	t4, a1, a2
smloop:	lbu	t0, 0x00(a1)
	addu	a1, a1, 0x01
	sb	t0, 0x00(a0)
	bne	a1, t4, smloop
	 addu	a0, a0, 0x01

	b	finish
	 nop
