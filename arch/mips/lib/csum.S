#include <asm/addrspace.h>
#include <asm/asm.h>
#include <asm/regdef.h>

/*
 * Compute kernel code checksum to check kernel code against corruption
 * (Ancient debugging trash ...)
 */
		LEAF(csum1)
		li	t0,KSEG0
		la	t1,KSEG0+0x80
		move	v0,zero
1:		lw	t2,(t0)
		addiu	t0,4
		addiu	t2,0x5555
		bne	t0,t1,1b
		xor	v0,t2
		jr	ra
		END(csum1)

		LEAF(csum2)
		li	t0,KSEG1
		la	t1,KSEG1+0x80
		move	v0,zero
1:		lw	t2,(t0)
		addiu	t0,4
		addiu	t2,0x5555
		bne	t0,t1,1b
		xor	v0,t2
		jr	ra
		END(csum2)
