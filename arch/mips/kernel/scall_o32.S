/*
 * Handle ABI O32 style syscalls.
 *
 * Copyright (C) 1997 by Ralf Baechle
 *
 * $Id: scall_o32.S,v 1.3 1997/12/22 21:36:50 davem Exp $
 */
#include <asm/asm.h>
#include <linux/errno.h>
#include <asm/mipsregs.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>
#include <asm/unistd.h>

/* This duplicates the definition from <linux/sched.h> */
#define PF_TRACESYS	0x00000020	/* tracing system calls */

/* This duplicates the definition from <linux/mm.h> */
#define VERIFY_READ	0

/* This duplicates the definition from <asm/signal.h> */
#define SIGILL		4		/* Illegal instruction (ANSI).  */

/* Highest syscall used of any syscall flavour */
#define MAX_SYSCALL_NO	__NR_Linux + __NR_Linux_syscalls

	.align  5
NESTED(handle_sys, PT_SIZE, sp)
	.set	noat
	SAVE_ALL
	STI
	.set	at

	/*
	 * By convention "li v0,<syscallno>" is always preceeding
	 * the syscall instruction.  So if we're in a delay slot
	 * userland is screwed up.
	 */
	lw	t0, PT_CAUSE(sp)	# delay slot?
	lw	t1, PT_EPC(sp)		# skip syscall on return
	bltz	t0, sigill_and_out

	sltiu	t0, v0, MAX_SYSCALL_NO + 1 # check syscall number
	addiu	t1, 4			# skip to next instruction
	beqz	t0, illegal_syscall
	sw	t1, PT_EPC(sp)

	/* XXX Put both in one cacheline, should save a bit. */
	sll	t0, v0, 2
	lw	s0, sys_call_table(t0)	# syscall routine
	lbu	s1, sys_narg_table(v0)	# number of arguments
	beqz	s0, illegal_syscall;

	subu	t0, s1, 5		# 5 or more arguments?
	lw	s7, current_set		# syscall tracing enabled?
	bgezal	t0, stackargs		# This clobbers s3...

	lw	s3, TASK_FLAGS(s7)
	andi	s3, PF_TRACESYS
	bnez	s3, trace_a_syscall

	jalr	s0			# Do The Real Thing (TM)

	li	t0, -EMAXERRNO - 1	# error?
	sltu	t0, t0, v0
	sw	t0, PT_R7(sp)		# set error flag
	beqz	t0, 1f

	negu	v0			# error
	sw	v0, PT_R0(sp)		# set flag for syscall restarting
1:	sw	v0, PT_R2(sp)		# result
	j	ret_from_sys_call

/* ------------------------------------------------------------------------ */

trace_a_syscall:
	/* Stash args away temporarily. */
	move	s1, a0
	move	s2, a1
	move	s3, a2
	move	s4, a3

	jal	syscall_trace

	/* Now, move original args back into place. */
	move	a0, s1
	move	a1, s2
	move	a2, s3
	move	a3, s4

	jalr	s0			# Do The Real Thing (TM)

	li	t0, -EMAXERRNO - 1	# error?
	sltu	t0, t0, v0
	sw	t0, PT_R7(sp)		# set error flag
	beqz	t0, 1f

	negu	v0			# error
	sw	v0, PT_R0(sp)		# set flag for syscall restarting
1:	sw	v0, PT_R2(sp)		# result

	jal	syscall_trace
	j	ret_from_sys_call

/* ------------------------------------------------------------------------ */

	/*
	 * More than four arguments.  Try to deal with it by copying the
	 * stack arguments from the user stack to the kernel stack.
	 * This Sucks (TM).
	 */
stackargs:
	move	s3, ra			# save return address

	li	a0, VERIFY_READ
	lw	a1, PT_R29(sp)		# get old user stack pointer
	subu	s2, s1, 4
	sll	a2, s2, 2		# stack valid?
	jal	verify_area
	bltz	v0, bad_stack		# -> sp is bad

	lw	t0, PT_R29(sp)		# get old user stack pointer
	la	t1, 1f			# copy 1 to 3 arguments
	sll	s2, s2, 3
	subu	t1, s2
	jr	t1

	/* Ok, copy the args from the luser stack to the kernel stack */
	lw	t1, 20(t0)		# argument #6 from usp
	sw	t1, 20(sp)
	lw	t1, 16(t0)		# argument #5 from usp
	sw	t1, 16(sp)
1:

	lw	a0, PT_R4(sp)		# restore arg regs from frame
	lw	a1, PT_R5(sp)
	lw	a2, PT_R6(sp)
	lw	a3, PT_R7(sp)

	jr	s3			# go back

	/*
	 * The stackpointer for a call with more than 4 arguments is bad.
	 */
bad_stack:
	negu	v0				# error
	sw	v0, PT_R0(sp)
	sw	v0, PT_R2(sp)
	li	t0, 1				# set error flag
	sw	t0, PT_R7(sp)
	j	ret_from_sys_call

	/*
	 * The system call does not exist in this kernel
	 */
illegal_syscall:
	li	v0, ENOSYS			# error
	sw	v0, PT_R2(sp)
	li	t0, 1				# set error flag
	sw	t0, PT_R7(sp)
	j	ret_from_sys_call

sigill_and_out:
	li	t0, -1				# not a sys call
	REG_S	t0, PT_OR2(sp)
	li	a0, SIGILL
	lw	a2, current_set
	jal	force_sig
	j	ret_from_sys_call
	END(handle_sys)
