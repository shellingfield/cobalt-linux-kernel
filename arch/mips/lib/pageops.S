/* $Id: pageops.S,v 1.2 1997/12/13 04:04:40 davem Exp $
 * pageops.S: Copy/Clear/Fill whole pages quickly.
 *
 * Copyright (C) 1997 David S. Miller (davem@dm.cobaltmicro.com)
 */

#include <asm/asm.h>
#include <asm/regdef.h>
#include <asm/cacheops.h>

	.text
	.set	noat
	.set	noreorder
	.set	mips3

	.align	4
	.globl	clear_page
clear_page:	/* a0=page */
	addiu	AT, a0, 4096
clrloop:cache	Create_Dirty_Excl_D, 0x00(a0)
	sd	zero, 0x00(a0)
	sd	zero, 0x08(a0)
	sd	zero, 0x10(a0)
	sd	zero, 0x18(a0)
	cache	Create_Dirty_Excl_D, 0x20(a0)
	addiu	a0, a0, 0x40
	sd	zero, -0x20(a0)
	sd	zero, -0x18(a0)
	sd	zero, -0x10(a0)
	bne	AT, a0, clrloop
	 sd	zero, -0x08(a0)
	j	ra
	 nop

	.globl	copy_page
copy_page:	/* a0=dest, a1=source */
	mfc0	t9, $12
	addiu	AT, a0, 4096
	nop
	ori	t0, t9, 0x01
	xori	t0, t0, 0x01
	mtc0	t0, $12
	nop
	nop
cploop:	ld	t0, 0x00(a1)
	ld	t1, 0x08(a1)
	ld	t2, 0x10(a1)
	ld	t3, 0x18(a1)
	ld	t4, 0x20(a1)
	ld	t5, 0x28(a1)
	ld	t6, 0x30(a1)
	ld	t7, 0x38(a1)
	addiu	a1, a1, 0x40
	cache	Create_Dirty_Excl_D, 0x00(a0)
	sd	t0, 0x00(a0)
	sd	t1, 0x08(a0)
	sd	t2, 0x10(a0)
	sd	t3, 0x18(a0)
	cache	Create_Dirty_Excl_D, 0x20(a0)
	addiu	a0, a0, 0x40
	sd	t4, -0x20(a0)
	sd	t5, -0x18(a0)
	sd	t6, -0x10(a0)
	bne	AT, a0, cploop
	 sd	t7, -0x08(a0)
	mtc0	t9, $12
	nop
	j	ra
	 nop

	.globl	pgd_init
pgd_init:	/* a0=pgdp */
	la	a1, invalid_pte_table
	addiu	AT, a0, 4096
piloop:	cache	Create_Dirty_Excl_D, 0x00(a0)
	sw	a1, 0x00(a0)
	sw	a1, 0x04(a0)
	sw	a1, 0x08(a0)
	sw	a1, 0x0c(a0)
	addiu	a0, a0, 0x20
	sw	a1, -0x10(a0)
	sw	a1, -0x0c(a0)
	sw	a1, -0x08(a0)
	bne	AT, a0, piloop
	 sw	a1, -0x04(a0)
	j	ra
	 nop
