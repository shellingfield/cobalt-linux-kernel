/*
 *  linux/arch/mips/kernel/process.c
 *
 *  Copyright (C) 1995 Ralf Baechle
 *
 *  Modified for R3000/DECStation support by Paul M. Antoine 1995, 1996
 *
 * This file handles the architecture-dependent parts of initialization,
 * though it does not yet currently fully support the DECStation,
 * or R3000 - PMA.
 */
#include <linux/config.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/ptrace.h>
#include <linux/malloc.h>
#include <linux/mman.h>
#include <linux/sys.h>
#include <linux/user.h>
#include <linux/a.out.h>

#include <asm/bootinfo.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/mipsregs.h>
#include <asm/processor.h>
#include <asm/segment.h>
#include <asm/stackframe.h>
#include <asm/io.h>
#include <asm/elf.h>
#include <asm/reboot.h>
#ifdef CONFIG_SGI
#include <asm/sgialib.h>
#endif

int active_ds = USER_DS;

asmlinkage void ret_from_sys_call(void);

void hard_reset_now(void)
{
	machine_halt();
}

void copy_thread(int nr, unsigned long clone_flags, unsigned long usp,
                 struct task_struct * p, struct pt_regs * regs)
{
	struct pt_regs * childregs;
	long childksp;

	childksp = p->kernel_stack_page + KERNEL_STACK_SIZE;

	/* set up new TSS. */
	childregs = (struct pt_regs *) childksp -1;
	*childregs = *regs;
	childregs->regs[7] = 0;	/* Clear error flag */
	if(current->personality == PER_LINUX) {
		childregs->regs[2] = 0;	/* Child gets zero as return value */
		regs->regs[2] = p->pid;
	} else {
		/* Under IRIX things are a little different. */
		childregs->regs[2] = 0;
		childregs->regs[3] = 1;
		regs->regs[2] = p->pid;
		regs->regs[3] = 0;
	}
	if (childregs->cp0_status & ST0_CU0) {
		childregs->regs[29] = childksp;
		p->tss.current_ds = KERNEL_DS;
	} else {
		if (usp & 7)
			printk("warning: pid %d called clone() with "
			       "misaligned stack pointer 0x%x\n", 
				current->pid, usp);
		childregs->regs[29] = usp;
		p->tss.current_ds = USER_DS;
	}
	p->tss.ksp = childksp;
	p->tss.reg29 = (unsigned long) childregs;
	p->tss.reg31 = (unsigned long) ret_from_sys_call;

	/*
	 * New tasks loose permission to use the fpu. This accelerates context
	 * switching for most programs since they don't use the fpu.
	 */
	p->tss.cp0_status = read_32bit_cp0_register(CP0_STATUS) &
                            ~(ST0_CU3|ST0_CU2|ST0_CU1|ST0_KSU|ST0_ERL|ST0_EXL);
	childregs->cp0_status &= ~(ST0_CU3|ST0_CU2|ST0_CU1);
	p->mm->context = 0;
}

/* Fill in the fpu structure for a core dump.. */
int dump_fpu(struct pt_regs *regs, elf_fpregset_t *r)
{
	/* We actually store the FPU info in the task->tss
	 * area.
	 */
	if(regs->cp0_status & ST0_CU1) {
		memcpy(r, &current->tss.fpu, sizeof(current->tss.fpu));
		return 1;
	}
	return 0; /* Task didn't use the fpu at all. */
}

/* Fill in the user structure for a core dump.. */
void dump_thread(struct pt_regs *regs, struct user *dump)
{
	dump->magic = CMAGIC;
	dump->start_code  = current->mm->start_code;
	dump->start_data  = current->mm->start_data;
	dump->start_stack = regs->regs[29] & ~(PAGE_SIZE - 1);
	dump->u_tsize = (current->mm->end_code - dump->start_code) >> PAGE_SHIFT;
	dump->u_dsize = (current->mm->brk + (PAGE_SIZE - 1) - dump->start_data) >> PAGE_SHIFT;
	dump->u_ssize =
		(current->mm->start_stack - dump->start_stack + PAGE_SIZE - 1) >> PAGE_SHIFT;
	memcpy(&dump->regs[0], regs, sizeof(struct pt_regs));
	memcpy(&dump->regs[EF_SIZE/4], &current->tss.fpu, sizeof(current->tss.fpu));
}
