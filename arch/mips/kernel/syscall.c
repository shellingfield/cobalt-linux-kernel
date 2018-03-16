/*
 * MIPS specific syscall handling functions and syscalls
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1995, 1996 by Ralf Baechle
 *
 * TODO:  Implement the compatibility syscalls.
 *        Don't waste that much memory for empty entries in the syscall
 *        table.
 *
 * $Id: syscall.c,v 1.4 1998/02/16 22:03:35 davem Exp $
 */

#include <linux/config.h>
#include <linux/linkage.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/smp_lock.h>
#include <linux/mman.h>
#include <linux/sched.h>
#include <linux/utsname.h>
#include <linux/unistd.h>
#include <asm/branch.h>
#include <asm/ptrace.h>
#include <asm/signal.h>
#include <asm/segment.h>
#include <asm/page.h>
#include <asm/pgtable.h>

typedef asmlinkage int (*syscall_t)(void *a0,...);
extern syscall_t sys_call_table[];
extern unsigned char sys_narg_table[];

asmlinkage int sys_pipe(struct pt_regs regs)
{
	int fd[2];
	int error;

	error = do_pipe(fd);
	if (error)
		return error;
	regs.regs[3] = fd[1];
	return fd[0];
}

asmlinkage unsigned long sys_mmap(unsigned long addr, size_t len, int prot,
                                  int flags, int fd, off_t offset)
{
	struct file * file = NULL;
	unsigned long res;

	if (!(flags & MAP_ANONYMOUS)) {
		if (fd >= NR_OPEN || !(file = current->files->fd[fd]))
			return -EBADF;
	}
	flags &= ~(MAP_EXECUTABLE | MAP_DENYWRITE);

	res = do_mmap(file, addr, len, prot, flags, offset);

	return res;
}

asmlinkage int sys_idle(void)
{
        int ret = -EPERM;

	if (current->pid != 0)
		goto out;
	/* endless idle loop with no priority at all */
	current->priority = -100;
	current->counter = -100;
	for (;;) {
		/*
		 * R4[36]00 have wait, R4[04]00 don't.
		 * FIXME: We should save power by reducing the clock where
		 *        possible.  Thiss will cut down the power consuption
		 *        of R4200 systems to about 1/16th of normal, the
		 *        same for logic clocked with the processor generated
		 *        clocks.
		 */
		if (wait_available && !need_resched)
			__asm__(".set\tmips3\n\t"
				"nop\n\t"
				"nop\n\t"
				"wait\n\t"
				"nop\n\t"
				"nop\n\t"
				".set\tmips0");
		run_task_queue(&tq_scheduler);
		schedule();

		/* Free up page table caches when they become too large. */

#define PGTCACHE_HIGH_WATER		50
#define PGTCACHE_LOW_WATER		25

		if(pgtable_cache_size > PGTCACHE_LOW_WATER) {
			do {	if(pgd_quicklist)
					free_page((unsigned long) get_pgd_fast());
				if(pte_quicklist)
					free_page((unsigned long) get_pte_fast());
			} while(pgtable_cache_size > PGTCACHE_HIGH_WATER);
		}

#undef PGTCACHE_HIGH_WATER
#undef PGTCACHE_LOW_WATER

	}
out:
	return ret;
}

asmlinkage int sys_fork(struct pt_regs regs)
{
	return do_fork(SIGCHLD, regs.regs[29], &regs);
}

asmlinkage int sys_clone(struct pt_regs regs)
{
	unsigned long clone_flags;
	unsigned long newsp;
	int res;

	clone_flags = regs.regs[4];
	newsp = regs.regs[5];
	if (!newsp)
		newsp = regs.regs[29];
	res = do_fork(clone_flags, newsp, &regs);
	return res;
}

/*
 * sys_execve() executes a new program.
 */
asmlinkage int sys_execve(struct pt_regs regs)
{
	int error;
	char * filename;

	error = getname((char *) (long)regs.regs[4], &filename);
	if (error)
		return error;
	error = do_execve(filename, (char **) (long)regs.regs[5],
	                  (char **) (long)regs.regs[6], &regs);
	putname(filename);
	return error;
}

/*
 * Do the indirect syscall syscall.
 * XXX
 */
asmlinkage int sys_syscall(struct pt_regs regs)
{
	syscall_t syscall;
	unsigned long syscallnr = regs.regs[4];
	unsigned long a0, a1, a2, a3, a4, a5, a6;
	int nargs, errno;

	if (syscallnr > __NR_Linux + __NR_Linux_syscalls)
		return -ENOSYS;

	syscall = sys_call_table[syscallnr];
	nargs = sys_narg_table[syscallnr];
	/*
	 * Prevent stack overflow by recursive
	 * syscall(__NR_syscall, __NR_syscall,...);
	 */
	if (syscall == (syscall_t) sys_syscall)
		return -EINVAL;

	if (syscall == NULL)
		return -ENOSYS;

	if (nargs > 3) {
		unsigned long usp = regs.regs[29];
		unsigned long *sp = (unsigned long *) usp;
		if (usp & 3) {
			printk("unaligned usp -EFAULT\n");
			force_sig(SIGSEGV, current);
			return -EFAULT;
		}
		errno = verify_area(VERIFY_READ, (void *) (usp + 16),
		                    (nargs - 3) * sizeof(unsigned long));
		if(errno)
			return -EFAULT;
		switch(nargs) {
		case 7:
			a3 = sp[4]; a4 = sp[5]; a5 = sp[6]; a6 = sp[7];
			break;
		case 6:
			a3 = sp[4]; a4 = sp[5]; a5 = sp[6]; a6 = 0;
			break;
		case 5:
			a3 = sp[4]; a4 = sp[5]; a5 = a6 = 0;
			break;
		case 4:
			a3 = sp[4]; a4 = a5 = a6 = 0;
			break;

		default:
			a3 = a4 = a5 = a6 = 0;
			break;
		}
	} else {
		a3 = a4 = a5 = a6 = 0;
	}
	a0 = regs.regs[5]; a1 = regs.regs[6]; a2 = regs.regs[7];
	return syscall((void *)a0, a1, a2, a3, a4, a5, a6);
}


/*
 * Build the string table for the builtin "poor man's strace".
 */
#ifdef CONF_PRINT_SYSCALLS
#define SYS(fun, narg) #fun,
static char *sfnames[] = {
#include "syscalls.h"
};
#endif

#if defined(CONFIG_BINFMT_IRIX) && defined(CONF_DEBUG_IRIX)
#define SYS(fun, narg) #fun,
static char *irix_sys_names[] = {
#include "irix5sys.h"
};
#endif
