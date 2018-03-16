/*
 * Handle unaligned accesses by emulation.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1996 by Ralf Baechle
 *
 * This file contains exception handler for address error exception with the
 * special capability to execute faulting instructions in software.  The
 * handler does not try to handle the case when the program counter points
 * to an address not aligned to a word boundary.
 *
 * Putting data to unaligned addresses is a bad practice even on Intel where
 * only the performance is affected.  Much worse is that such code is non-
 * portable.  Due to several programs that die on MIPS due to alignment
 * problems I decieded to implement this handler anyway though I originally
 * didn't intend to do this at all for user code.
 *
 * For now I enable fixing of address errors by default to make life easier.
 * I however intend to disable this somewhen in the future when the alignment
 * problems with user programs have been fixed.  For programmers this is the
 * right way to go.
 *
 * Fixing address errors is a per process option.  The option is inherited
 * across fork(2) and execve(2) calls.  If you really want to use the
 * option in your user programs - I discourage the use of the software
 * emulation strongly - use the following code in your userland stuff:
 *
 * #include <sys/sysmips.h>
 *
 * ...
 * sysmips(MIPS_FIXADE, x);
 * ...
 *
 * The parameter x is 0 for disabeling software emulation.  Set bit 0 for
 * enabeling software emulation and bit 1 for enabeling printing debug
 * messages into syslog to aid finding address errors in programs.
 *
 * The logging feature is a addition over RISC/os and IRIX where only the
 * values 0 and 1 are acceptable values for x.  I'll probably remove this
 * hack later on.
 *
 * Below a little program to play around with this feature.
 *
 * #include <stdio.h>
 * #include <asm/sysmips.h>
 * 
 * struct foo {
 *         unsigned char bar[8];
 * };
 *
 * main(int argc, char *argv[])
 * {
 *         struct foo x = {0, 1, 2, 3, 4, 5, 6, 7};
 *         unsigned int *p = (unsigned int *) (x.bar + 3);
 *         int i;
 *
 *         if (argc > 1)
 *                 sysmips(MIPS_FIXADE, atoi(argv[1]));
 *
 *         printf("*p = %08lx\n", *p);
 *
 *         *p = 0xdeadface;
 *
 *         for(i = 0; i <= 7; i++)
 *         printf("%02x ", x.bar[i]);
 *         printf("\n");
 * }
 *
 * Until I've written the code to handle branch delay slots it may happen
 * that the kernel receives an ades/adel instruction from an insn in a
 * branch delay slot but is unable to handle this case.  The kernel knows
 * this fact and therefore will kill the process.  For most code you can
 * fix this temporarily by compiling with flags -fno-delayed-branch -Wa,-O0.
 *
 * Coprozessor loads are not supported; I think this case is unimportant
 * in the practice.
 *
 * TODO: Handle ndc (attempted store to doubleword in uncached memory)
 *       exception for the R6000.
 */
#include <linux/mm.h>
#include <linux/signal.h>
#include <asm/branch.h>
#include <asm/inst.h>
#include <asm/segment.h>
#include <asm/unaligned.h>

/*
 * Don't inline the functions for better debugging.
 */
/*#define __inline__*/

/*
 * Damned ...  There is just too much bad code with that Intel desease in
 * it out there.   So we really need to support this :-(
 */
static __inline__ unsigned long
load_mem(unsigned long addr, unsigned int size)
{
	switch(size) {
	case 2: return ldw_u((unsigned short *) addr);
	case 4: return ldl_u((unsigned int *) addr);
#if 0
#if (_MIPS_ISA == _MIPS_ISA_MIPS3) || (_MIPS_ISA == _MIPS_ISA_MIPS4)
	case 8: return ldq_u((unsigned long long *) addr);
#endif
#endif
	}

	/* Unreached */
	return 0;
}

static __inline__ void
store_mem(unsigned long addr, unsigned int size, unsigned long val)
{
	switch(size) {
	case 2: stw_u(val, (unsigned short *) addr);
		return;
	case 4: stl_u(val, (unsigned int *) addr);
		return;
#if 0
#if (_MIPS_ISA == _MIPS_ISA_MIPS3) || (_MIPS_ISA == _MIPS_ISA_MIPS4)
	case 8: stq_u(val, (unsigned long long *) addr);
		return;
#endif
#endif
	}
}

/*
 * Bummer: accesses by the kernel can be assumed to be good.  However
 * we need to call verify_area to check userspace addresses for sanity.
 * This sucks, but as I'm doing this here I'm too conservative to pull
 * the 2.1.x exception handling into this kernel.
 */
static __inline__ void
emulate_load_store_insn(struct pt_regs *regs, unsigned long addr, unsigned long pc)
{
	union mips_instruction insn;
	int size, signop;
	unsigned long value;
	int kernelmode = pc & 0x80000000;

	/*
	 * Assumes we're not in a branch delay slot.
	 */
	insn.word = get_user((unsigned int *)pc);
	switch (insn.i_format.opcode) {
	/*
	 * These are instruction that a compiler doesn't generate.  We
	 * can assume therefore that the code is MIPS-aware and
	 * really buggy.  Emulating these instructions would break the
	 * semantics anyway.
	 */
	case ll_op:
	case lld_op:
	case sc_op:
	case scd_op:

	/*
	 * For these instructions the only way to create an address
	 * error is an attempted access to kernel/supervisor address
	 * space.
	 */
	case ldl_op:
	case ldr_op:
	case lwl_op:
	case lwr_op:
	case sdl_op:
	case sdr_op:
	case swl_op:
	case swr_op:
	case lb_op:
	case lbu_op:
	case sb_op:
		force_sig(SIGBUS, current);
		return;

	/*
	 * The remaining opcodes are the ones that are really of interrest.
	 */
	case lh_op:	size = 2; signop = 1; goto do_reg_load;
	case lhu_op:	size = 2; signop = 0; goto do_reg_load;
	case lw_op:	size = 4; signop = 1; goto do_reg_load;
	case lwu_op:	size = 4; signop = 0; goto do_reg_load;
	case ld_op:	size = 8; signop = 1;
do_reg_load:
		if (!kernelmode &&
		    verify_area(VERIFY_READ, (void *)addr, size) < 0) {
			force_sig(SIGSEGV, current);
			return;
		}
		value = load_mem(addr, size);
		if (!signop) {
			/* This is an unsigned load */
			if (size == 2) value &= 0xffffUL;
			else if (size == 4) value &= 0xffffffffUL;
		}
		regs->regs[insn.i_format.rt] = value;
		regs->regs[insn.i_format.rt] = value;
		return;
	case sh_op:	size = 2; goto do_reg_store;
	case sw_op:	size = 4; goto do_reg_store;
	case sd_op:	size = 8;
do_reg_store:
		if (!kernelmode &&
		    verify_area(VERIFY_WRITE, (void *)addr, size) < 0) {
			force_sig(SIGSEGV, current);
			return;
		}
		store_mem(addr, size, regs->regs[insn.i_format.rt]);
		return;

	case lwc1_op:
		size = 8; goto do_fp_load;
	case ldc1_op:
		size = 4;

do_fp_load:
		if (!kernelmode &&
		    verify_area(VERIFY_READ, (void *)addr, size) < 0) {
			force_sig(SIGSEGV, current);
			return;
		}

		regs->cp0_status |= ST0_CU1;
#define FPLD_ASM(reg)	\
		value = load_mem(addr, 4);				\
		__asm__( "\tlwc1\t$f"#reg",%0\n": : "m" (value));	\
			if (insn.i_format.opcode == lwc1_op || (reg) & 1) \
				break;					\
			else addr += 4;

		switch(insn.i_format.rt) {
		    case 0: FPLD_ASM(0); case 1: FPLD_ASM(1);
		    case 2: FPLD_ASM(2); case 3: FPLD_ASM(3);
		    case 4: FPLD_ASM(4); case 5: FPLD_ASM(5);
		    case 6: FPLD_ASM(6); case 7: FPLD_ASM(7);
		    case 8: FPLD_ASM(8); case 9: FPLD_ASM(9);
		    case 10: FPLD_ASM(10); case 11: FPLD_ASM(11);
		    case 12: FPLD_ASM(12); case 13: FPLD_ASM(13);
		    case 14: FPLD_ASM(14); case 15: FPLD_ASM(15);
		    case 16: FPLD_ASM(16); case 17: FPLD_ASM(17);
		    case 18: FPLD_ASM(18); case 19: FPLD_ASM(19);
		    case 20: FPLD_ASM(20); case 21: FPLD_ASM(21);
		    case 22: FPLD_ASM(22); case 23: FPLD_ASM(23);
		    case 24: FPLD_ASM(24); case 25: FPLD_ASM(25);
		    case 26: FPLD_ASM(26); case 27: FPLD_ASM(27);
		    case 28: FPLD_ASM(28); case 29: FPLD_ASM(29);
		    case 30: FPLD_ASM(30); case 31: FPLD_ASM(31);
		}
		return;

	case swc1_op:
		size=4; goto do_fp_store;
	case sdc1_op:
		size=8;

do_fp_store:
		if (!kernelmode &&
		    verify_area(VERIFY_READ, (void *)addr, size) < 0) {
			force_sig(SIGSEGV, current);
			return;
		}

		regs->cp0_status |= ST0_CU1;
#define FPST_ASM(reg)	\
		__asm__( "\tswc1\t$f"#reg",%0\n": : "m" (value));	\
			store_mem(addr, 4, value);			\
			if (insn.i_format.opcode == swc1_op || (reg) & 1) \
				break;					\
			else addr += 4;

		switch(insn.i_format.rt) {
		    case 0: FPST_ASM(0); case 1: FPST_ASM(1);
		    case 2: FPST_ASM(2); case 3: FPST_ASM(3);
		    case 4: FPST_ASM(4); case 5: FPST_ASM(5);
		    case 6: FPST_ASM(6); case 7: FPST_ASM(7);
		    case 8: FPST_ASM(8); case 9: FPST_ASM(9);
		    case 10: FPST_ASM(10); case 11: FPST_ASM(11);
		    case 12: FPST_ASM(12); case 13: FPST_ASM(13);
		    case 14: FPST_ASM(14); case 15: FPST_ASM(15);
		    case 16: FPST_ASM(16); case 17: FPST_ASM(17);
		    case 18: FPST_ASM(18); case 19: FPST_ASM(19);
		    case 20: FPST_ASM(20); case 21: FPST_ASM(21);
		    case 22: FPST_ASM(22); case 23: FPST_ASM(23);
		    case 24: FPST_ASM(24); case 25: FPST_ASM(25);
		    case 26: FPST_ASM(26); case 27: FPST_ASM(27);
		    case 28: FPST_ASM(28); case 29: FPST_ASM(29);
		    case 30: FPST_ASM(30); case 31: FPST_ASM(31);
		}
		return;

	case lwc2_op:
	case ldc2_op:
	case swc2_op:
	case sdc2_op:
		/*
		 * These are the coprozessor 2 load/stores.  The current
		 * implementations don't use cp2 and cp2 should always be
		 * disabled in c0_status.  So send SIGILL.
		 */
	default:
		/*
		 * Pheeee...  We encountered an yet unknown instruction ...
		 */
		printk("[%s:%d] AIEEE emulate_store_insn at %08lx\n",
		       current->comm, current->pid, regs->cp0_epc);
		force_sig(SIGILL, current);
	}
}

static __inline__ void
fix_ade(struct pt_regs *regs, unsigned long pc)
{
	if(user_mode(regs)) {
		if(verify_area(VERIFY_READ, (void *)pc, 4) < 0) {
			force_sig(SIGSEGV, current);
			return;
		}
	}

	/*
	 * Did we catch a fault trying to load an instruction?
	 */
	if (regs->cp0_badvaddr == pc) {
		/*
		 * This fault occured during an attempt to load an instruction.
		 */
		force_sig(SIGBUS, current);
	}

	/*
	 * Ok, this wasn't a failed instruction load.  The CPU was capable of
	 * reading the instruction and faulted after this.  So we don't need
	 * to verify_area the address of the instrucion.  We still don't
	 * know whether the address used was legal and therefore need to do
	 * verify_area().  The CPU already did the checking for legal
	 * instructions for us, so we don't need to do this.
	 */
	emulate_load_store_insn(regs, regs->cp0_badvaddr, pc);
}

#define is_kernel_address(x) ((x) & 0x80000000)

void do_ade(struct pt_regs *regs)
{
	extern unsigned long unaligned_instructions;
	unsigned long logpc, pc = regs->cp0_epc;
	char adels;

	unaligned_instructions++;
	adels = (((regs->cp0_cause & CAUSEF_EXCCODE) >> CAUSEB_EXCCODE) == 4)
	  ? 'l' : 's';
	if (current->tss.mflags & MF_LOGADE) {
		logpc = pc;
		if (regs->cp0_cause & CAUSEF_BD)
			logpc += 4;
		printk(KERN_DEBUG
		       "Caught ade%c in '%s' at 0x%08lx accessing 0x%08lx.\n",
		       adels, current->comm, logpc, regs->cp0_badvaddr);
	}
	compute_return_epc(regs);
	if(current->tss.mflags & MF_FIXADE) {
		pc += ((regs->cp0_cause & CAUSEF_BD) ? 4 : 0);
		fix_ade(regs, pc);
		return;
	}
	force_sig(SIGBUS, current);
}
