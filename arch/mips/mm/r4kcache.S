/* $Id: r4kcache.S,v 1.5 1998/11/17 11:08:24 davem Exp $
 * r4kcache.S: Optimized R4k cache flush operations.
 *
 * Copyright (C) 1997, 1998 David S. Miller (davem@dm.cobaltmicro.com)
 */

#include <linux/config.h>

#include <asm/asm.h>
#include <asm/regdef.h>
#include <asm/cacheops.h>

	.text
	.align	4
	.set	noat
	.set	noreorder
	.set	mips3

	/* NOTE: flush_page_to_ram is implemented in terms of
	 *       the following (ie, __flush_page_current(page, 0))
	 */
	.globl	__flush_cache_page_current
__flush_cache_page_current:	/* a0=page, a1=text */
	subu	t1, zero, 4096
	and	a0, a0, t1
	addu	t0, a0, 4096
1:	cache	Hit_Writeback_Inv_D, 0x00(a0)
	cache	Hit_Writeback_Inv_D, 0x20(a0)
	cache	Hit_Writeback_Inv_D, 0x40(a0)
	cache	Hit_Writeback_Inv_D, 0x60(a0)
	addu	a0, a0, 0x80
	bne	a0, t0, 1b
	 nop
	beq	a1, zero, 3f
	 subu	a0, a0, 4096
2:	cache	Hit_Invalidate_I, 0x00(a0)
	cache	Hit_Invalidate_I, 0x20(a0)
	cache	Hit_Invalidate_I, 0x40(a0)
	cache	Hit_Invalidate_I, 0x60(a0)
	addu	a0, a0, 0x80
	bne	a0, t0, 2b
	 nop
3:	j	ra
	 nop

	.globl	__flush_cache_page_other
__flush_cache_page_other:	/* a0=page, a1=text */
	subu	t1, zero, 4096
	and	a0, a0, t1
	addu	t0, a0, 4096
1:	cache	Index_Writeback_Inv_D, 0x00(a0)
	cache	Index_Writeback_Inv_D, 0x20(a0)
	cache	Index_Writeback_Inv_D, 0x40(a0)
	cache	Index_Writeback_Inv_D, 0x60(a0)
	addu	a0, a0, 0x80
	bne	a0, t0, 1b
	 nop
	beq	a1, zero, 3f
	 subu	a0, a0, 4096
2:	cache	Index_Invalidate_I, 0x00(a0)
	cache	Index_Invalidate_I, 0x20(a0)
	cache	Index_Invalidate_I, 0x40(a0)
	cache	Index_Invalidate_I, 0x60(a0)
	addu	a0, a0, 0x80
	bne	a0, t0, 2b
	 nop
3:	j	ra
	 nop

	.globl	flush_cache_all
flush_cache_all:
	lui	t0, %hi(0x80000000)
	lw	t1, dcache_size
	addu	t1, t0, t1
1:	cache	Index_Writeback_Inv_D, 0x00(t0)
	cache	Index_Writeback_Inv_D, 0x20(t0)
	cache	Index_Writeback_Inv_D, 0x40(t0)
	cache	Index_Writeback_Inv_D, 0x60(t0)
	cache	Index_Invalidate_I, 0x00(t0)
	cache	Index_Invalidate_I, 0x20(t0)
	cache	Index_Invalidate_I, 0x40(t0)
	cache	Index_Invalidate_I, 0x60(t0)
	addu	t0, t0, 0x80
	bne	t0, t1, 1b
	 nop
	j	ra
	 nop

	.globl	sys_cacheflush
sys_cacheflush:			/* a0=addr, a1=bytes, a2=cachemask */
	addu	a1, a0, a1
	ori	a0, a0, 0x1f
	add	a1, a1, 0x1f
	xori	a0, a0, 0x1f
	ori	a1, a1, 0x1f
	xori	a1, a1, 0x1f
	move	t0, a0
	and	t4, a2, 0x02	/* DCACHE bit */
	beq	t4, zero, 2f
	 move	t1, a1
1:	cache	Hit_Writeback_Inv_D, 0x00(a0)
	addu	a0, a0, 0x20
	bne	a0, a1, 1b
	 nop
	move	a0, t0
2:	and	t4, a2, 0x01	/* ICACHE bit */
	beq	t4, zero, 4f
	 move	a1, t1
3:	cache	Hit_Invalidate_I, 0x00(a0)
	addu	a0, a0, 0x20
	bne	a0, a1, 3b
	 nop
4:	j	ra
	 move	v0, zero

	.globl	flush_cache_pre_dma_out
flush_cache_pre_dma_out:	/* a0=addr, a1=size */
	addu	a1, a0, a1
	ori	a0, a0, 0x1f
	add	a1, a1, 0x1f
	xori	a0, a0, 0x1f
	ori	a1, a1, 0x1f
	xori	a1, a1, 0x1f
1:	cache	Hit_Writeback_Inv_D, 0x00(a0)
	addu	a0, a0, 0x20
	bne	a0, a1, 1b
	 nop
	j	ra
	 nop

	.globl	flush_cache_post_dma_in
flush_cache_post_dma_in:
	addu	a1, a0, a1
	ori	a0, a0, 0x1f
	add	a1, a1, 0x1f
	xori	a0, a0, 0x1f
	ori	a1, a1, 0x1f
	xori	a1, a1, 0x1f
1:	cache	Hit_Invalidate_D, 0x00(a0)
	addu	a0, a0, 0x20
	bne	a0, a1, 1b
	 nop
	j	ra
	 nop
