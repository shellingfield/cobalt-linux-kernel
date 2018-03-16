/* $Id: bzero.S,v 1.3 1997/12/13 04:04:40 davem Exp $
 * bzero.S: Optimized MIPS bzero.
 *
 * Copyright (C) 1997 David S. Miller (davem@dm.cobaltmicro.com)
 */

#include <asm/asm.h>
#include <asm/regdef.h>

	.text
	.align	4

	.globl	__bzero

	.set	noreorder
	.set	noat
	.set	mips3
dword_align:
	andi	AT, a0, 0x1
	beq	AT, zero, 1f
	 andi	AT, a0, 0x2
	sb	zero, 0x00(a0)
	addu	a0, a0, 0x1
	subu	a1, a1, 0x1
	andi	AT, a0, 0x2
1:	beq	AT, zero, 2f

	 nop
	sh	zero, 0x00(a0)
	subu	a1, a1, 0x2
	addu	a0, a0, 0x2
2:	andi	AT, a0, 0x4
	beq	AT, zero, 3f
	 nop
	sw	zero, 0x00(a0)

	subu	a1, a1, 0x4
	b	3f
	 addu	a0, a0, 0x4
	nop
	nop
	nop
	nop
	nop

__bzero:sltiu	t0, a1, 8
	bne	t0, zero, bzero_short
	 andi	AT, a0, 0x7
	addu	t1, zero, -32
	bne	AT, zero, dword_align
3:	 and	t0, a1, t1
	beq	t0, zero, noloop
	 andi	AT, a1, 0x10

unrloop:sd	zero, 0x00(a0)
	sd	zero, 0x08(a0)
	subu	t0, t0, 0x20
	sd	zero, 0x10(a0)
	sd	zero, 0x18(a0)
	bne	t0, zero, unrloop
	 addu	a0, a0, 0x20
noloop:	beq	AT, zero, ckdword

	 andi	AT, a1, 0x08
	sd	zero, 0x00(a0)
	sd	zero, 0x08(a0)
	addu	a0, a0, 0x10
ckdword:beq	AT, zero, ckword
	 andi	AT, a1, 0x04
	sd	zero, 0x00(a0)
	addu	a0, a0, 0x08

ckword:	beq	AT, zero, ckhword
	 andi	AT, a1, 0x02
	sw	zero, 0x00(a0)
	addu	a0, a0, 0x04
ckhword:beq	AT, zero, ckbyte
	 andi	AT, a1, 0x01
	sh	zero, 0x00(a0)
	addu	a0, a0, 0x02

ckbyte:	beq	AT, zero, finish
	 nop
	sb	zero, 0x00(a0)
finish:	jr	ra
	 nop
bzero_short:
	beq	a1, zero, finish
	 nop
sloop:	sb	zero, 0x00(a0)

	subu	a1, a1, 0x1
	bne	a1, zero, sloop
	 addu	a0, a0, 0x1
	b	finish
	 nop
	nop
	nop
	nop
