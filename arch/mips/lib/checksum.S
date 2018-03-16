/* $Id: checksum.S,v 1.2 1997/11/19 22:43:56 davem Exp $
 * checksum.S: MIPS optimized checksum code.
 *
 * Copyright (C) 1997 David S. Miller (davem@dm.cobaltmicro.com)
 */

#include <asm/asm.h>
#include <asm/regdef.h>

	.text
	.align	4

	.set	noat
	.set	noreorder

csum_partial_end_cruft:
	beq	t0, zero, 1f
	 andi	AT, a1, 0x04
	lw	t2, 0x00(a0)
	lw	t3, 0x04(a0)
	addu	a0, a0, 0x08
	addu	t2, t2, a2
	sltu	t4, t2, a2
	addu	t2, t2, t4
	addu	t3, t3, t2
	sltu	t5, t3, t2
	addu	a2, t3, t5
1:	beq	AT, zero, 1f
	 andi	a1, a1, 0x03
	lw	t2, 0x00(a0)
	addu	a0, a0, 0x04
	addu	t2, t2, a2
	sltu	t3, t2, a2
	addu	a2, t2, t3
1:	beq	a1, zero, 1f
	 subu	t7, a1, 0x01
	bne	t7, zero, 2f
	 subu	AT, a1, 0x02
	b	4f
	 move	t4, zero
2:	lhu	t4, 0x00(a0)
	beq	AT, zero, 6f
	 addu	a0, a0, 0x02
4:	beq	t7, zero, 9f
	 lbu	t5, 0x00(a0)
	sll	t5, t5, 16
9:	or	t4, t5, t4
6:	addu	t4, t4, a2
	sltu	t3, t4, a2
	addu	a2, t4, t3
1:	jr	ra
	 move	v0, a2

csum_partial_fix_alignment:
	sltu	AT, a1, 0x06
	bne	AT, zero, cpte
	 andi	t0, a1, 0x0f
	lhu	t2, 0x00(a0)
	subu	a1, a1, 0x02
	addu	a0, a0, 0x02
	addu	t2, t2, a2
	sltu	t3, t2, a2
	addu	t2, t2, t3
	sll	AT, t2, 16
	addu	t2, t2, AT
	sltu	AT, t2, AT
	srl	t2, t2, 16
	b	cpa
	 addu	a2, t2, AT

	.globl	csum_partial
csum_partial:	/* a0=buf, a1=len, a2=sum */
	andi	AT, a0, 0x2
	bne	AT, zero, csum_partial_fix_alignment
	 subu	t1, zero, 0x20
cpa:	and	t1, a1, t1
	beq	t1, zero, 3f
	 nop
unrloop:lw	t2,  0x00(a0)
	addu	a0, a0, 0x20
	lw	t3, -0x1c(a0)
	subu	t1, t1, 0x20
	lw	t4, -0x18(a0)
	addu	t2, t2, a2
	lw	t5, -0x14(a0)
	sltu	t7, t2, a2
	lw	t6, -0x10(a0)
	addu	a2, t2, t3
	sltu	t8, a2, t2
	addu	t4, t4, a2
	sltu	t9, t4, a2
	addu	a2, t4, t5
	sltu	a3, a2, t5
	addu	t0, t7, t8
	lw	t7, -0x0c(a0)
	addu	t0, t0, t9
	addu	t0, t0, a3
	lw	t8, -0x08(a0)
	addu	t6, t6, a2
	sltu	t2, t6, a2
	lw	t9, -0x04(a0)
	addu	a2, t6, t7
	sltu	t3, a2, t7
	addu	t8, t8, a2
	sltu	t4, t8, a2
	addu	a2, t8, t9
	sltu	t5, a2, t9
	addu	t0, t0, t2
	addu	t0, t0, t3
	addu	t0, t0, t4
	addu	t0, t0, t5
	addu	t0, t0, a2
	sltu	a2, t0, a2
	bne	t1, zero, unrloop
	 addu	a2, a2, t0
3:	andi	t1, a1, 0x10
	beq	t1, zero, cpte
	 andi	t0, a1, 0x0f

	lw	t2,  0x00(a0)
	addu	a0, a0, 0x10
	lw	t3, -0x0c(a0)
	addu	a2, a2, t2
	sltu	t6, a2, t2
	lw	t4, -0x08(a0)
	addu	a2, a2, t3
	sltu	t7, a2, t3
	lw	t5, -0x04(a0)
	addu	a2, a2, t4
	sltu	t8, a2, t4
	addu	a2, a2, t5
	sltu	t9, a2, t5
	addu	t6, t6, t7
	addu	t6, t6, t8
	addu	t6, t6, t9
	addu	t6, t6, a2
	sltu	t2, t6, a2
	addu	a2, t6, t2

cpte:	bne	t0, zero, csum_partial_end_cruft
	 andi	t0, a1, 0x08
cpout:	jr	ra
	 move	v0, a2

cc_end_cruft:
	beq	AT, zero, 1f
	 andi	AT, a2, 0x04
	lw	t2, 0x00(a0)
	lw	t3, 0x04(a0)
	addu	a0, a0, 0x08
	addu	a1, a1, 0x08
	sw	t2, -0x08(a1)
	addu	t2, t2, a3
	sltu	t4, t2, a3
	addu	t2, t2, t4
	sw	t3, -0x04(a1)
	addu	t3, t3, t2
	sltu	t5, t3, t2
	addu	a3, t3, t5
1:	beq	AT, zero, 1f
	 andi	a2, a2, 0x03
	lw	t2, 0x00(a0)
	addu	a0, a0, 0x04
	addu	a1, a1, 0x04
	addu	t3, t2, a3
	sw	t2, -0x04(a1)
	sltu	t4, t3, a3
	addu	a3, t3, t4
1:	beq	a2, zero, 1f
	 subu	t7, a2, 0x01
	bne	t7, zero, 2f
	 subu	AT, a2, 0x02
	b	4f
	 move	t4, zero
2:	lhu	t4, 0x00(a0)
	addu	a0, a0, 0x02
	addu	a1, a1, 0x02
	beq	AT, zero, 6f
	 sh	t4, -0x02(a1)
4:	lbu	t5, 0x00(a0)
	beq	t7, zero, 9f
	 sb	t5, 0x00(a1)
	sll	t5, t5, 16
9:	or	t4, t5, t4
6:	addu	t4, t4, a3
	sltu	t3, t4, a3
	addu	a3, t4, t3
1:	jr	ra
	 move	v0, a3

cc_word_align:
	sltu	AT, a2, 0x06
	bne	AT, zero, ccte
	 andi	t0, a2, 0x0f
	andi	AT, a0, 0x01
	bne	AT, zero, ccslow
	 nop
	lhu	t2,  0x00(a0)
	addu	a0, a0, 0x02
	addu	a1, a1, 0x02
	subu	a2, a2, 0x02
	addu	t4, t2, a3
	sltu	t3, t4, a3
	addu	t4, t3, t4
	sll	t5, t4, 16
	addu	t4, t4, t5
	sh	t2, -0x02(a1)
	sltu	t5, t4, t5
	srl	t4, t4, 16
	addu	a3, t4, t5
	b	3f
	 and	t1, a2, t1

	.globl	csum_partial_copy
csum_partial_copy:	/* a0=src, a1=dst, a2=len, a3=sum */
	xor	AT, a0, a1
	andi	AT, AT, 0x03
	bne	AT, zero, ccslow
	 andi	AT, a0, 0x03
	bne	AT, zero, cc_word_align
	 subu	t1, zero, 0x20
	and	t1, a2, t1
3:	beq	t1, zero, 3f
	 nop
cploop:	lw	t2,  0x00(a0)
	addu	a0, a0, 0x20
	lw	t3, -0x1c(a0)
	subu	t1, t1, 0x20
	lw	t4, -0x18(a0)
	sw	t2,  0x00(a1)
	addu	a1, a1, 0x20
	addu	t2, t2, a3
	lw	t5, -0x14(a0)
	sltu	t7, t2, a3
	lw	t6, -0x10(a0)
	addu	a3, t2, t3
	sw	t3, -0x1c(a1)
	sltu	t8, a3, t2
	sw	t4, -0x18(a1)
	addu	t4, t4, a3
	sltu	t9, t4, a3
	addu	a3, t4, t5
	sw	t5, -0x14(a1)
	sltu	AT, a3, t5
	addu	t0, t7, t8
	lw	t7, -0x0c(a0)
	addu	t0, t0, t9
	sw	t6, -0x10(a1)
	addu	t0, t0, AT
	lw	t8, -0x08(a0)
	addu	t6, t6, a3
	sw	t7, -0x0c(a1)
	sltu	t2, t6, a3
	lw	t9, -0x04(a0)
	addu	a3, t6, t7
	sw	t8, -0x08(a1)
	sltu	t3, a3, t7
	addu	t8, t8, a3
	sltu	t4, t8, a3
	addu	a3, t8, t9
	sw	t9, -0x04(a1)
	sltu	t5, a3, t9
	addu	t0, t0, t2
	addu	t0, t0, t3
	addu	t0, t0, t4
	addu	t0, t0, t5
	addu	t0, t0, a3
	sltu	a3, t0, a3
	bne	t1, zero, cploop
	 addu	a3, a3, t0
3:	andi	AT, a2, 0x10
	bne	AT, zero, cc16
	 andi	t0, a2, 0x0f
ccte:	bne	t0, zero, cc_end_cruft
	 andi	AT, a2, 0x08
	jr	ra
	 move	v0, a3
cc16:	lw	t1,  0x00(a0)
	addu	a0, a0, 0x10
	lw	t2, -0x0c(a0)
	lw	t3, -0x08(a0)
	addu	t5, a3, t1
	sw	t1,  0x00(a1)
	sltu	t6, t5, a3
	addu	a1, a1, 0x10
	lw	t4, -0x04(a0)
	sw	t2, -0x0c(a1)
	addu	t2, t2, t5
	sltu	t7, t2, t5
	sw	t3, -0x08(a1)
	addu	t5, t2, t3
	sltu	t8, t5, t2
	sw	t4, -0x04(a1)
	addu	t4, t4, t5
	sltu	t9, t4, t5
	addu	t6, t6, t7
	addu	t6, t6, t8
	addu	t6, t6, t9
	addu	t4, t4, t6
	sltu	t5, t4, t6
	b	ccte
	 addu	a3, t4, t5

ccslow:
	andi	t9, a0, 0x01
	beq	a2, zero, 4f
	 move	t5, zero

	beq	t9, zero, 1f
	 nop

	subu	a2, a2, 0x01
	lbu	t5, 0x00(a0)
	addu	a0, a0, 0x01
	sb	t5, 0x00(a1)
	addu	a1, a1, 0x01
	sll	t5, t5, 0x08

1:	srl	t4, a2, 0x01
	beq	t4, zero, 3f
	 andi	AT, a2, 0x01
	andi	t0, a0, 0x02
	beq	t0, zero, 1f
	 nop

	lhu	t6, 0x00(a0)
	addu	a0, a0, 0x02
	subu	a2, a2, 0x02
	subu	t4, t4, 0x01
	srl	t7, t6, 0x08
	sb	t6, 0x00(a1)
	sb	t7, 0x01(a1)
	addu	t5, t5, t6
	addu	a1, a1, 0x02

1:	srl	t4, t4, 0x01
	beq	t4, zero, 2f
	 andi	AT, a2, 0x02

5:	lw	t0, 0x00(a0)
	addu	a0, a0, 0x04
	subu	t4, t4, 0x01
	srl	t1, t0, 0x08
	srl	t2, t0, 0x10
	sb	t0, 0x00(a1)
	srl	t3, t0, 0x18
	sb	t1, 0x01(a1)
	sb	t2, 0x02(a1)
	addu	t7, t0, t5
	sb	t3, 0x03(a1)
	sltu	t6, t7, t5
	addu	a1, a1, 0x04
	bne	t4, zero, 5b
	 addu	t5, t6, t7

	sll	t2, t5, 16
	srl	t5, t5, 16
	srl	t2, t2, 16
	addu	t5, t2, t5

2:	beq	AT, zero, 3f
	 andi	AT, a2, 0x01

	lhu	t6, 0x00(a0)
	addu	a0, a0, 0x02
	srl	t7, t6, 0x08
	sb	t6, 0x00(a1)
	addu	t5, t5, t6
	sb	t7, 0x01(a1)
	addu	a1, a1, 0x02

3:	beq	AT, zero, 1f
	 nop

	lbu	t2, 0x00(a0)
	sb	t2, 0x00(a1)
	addu	t5, t5, t2

1:	sll	AT, t5, 16
	addu	t5, t5, AT
	sltu	AT, t5, AT
	srl	t5, t5, 16
	addu	t5, t5, AT
	beq	t9, zero, 4f
	 srl	t6, t5, 8
	andi	t7, t5, 0xff
	andi	t6, t6, 0xff
	sll	t7, t7, 8
	or	t5, t6, t7

4:	addu	t7, t5, a3
	sltu	t8, t7, a3
	j	ra
	 addu	v0, t7, t8
