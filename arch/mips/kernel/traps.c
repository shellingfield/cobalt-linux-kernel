/*
 * arch/mips/kernel/traps.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright 1994, 1995, 1996, 1997 by Ralf Baechle
 * Modified for R3000 by Paul M. Antoine, 1995, 1996
 *
 * $Id: traps.c,v 1.6 1999/06/16 03:24:28 cjohnson Exp $
 */
#include <linux/config.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/smp_lock.h>

#include <asm/branch.h>
#include <asm/cachectl.h>
#include <asm/jazz.h>
#include <asm/vector.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/bootinfo.h>
#include <asm/watch.h>
#include <asm/system.h>
#include <asm/segment.h>

#ifdef CONFIG_SGI
#include <asm/sgialib.h>
#endif

#undef CONF_DEBUG_EXCEPTIONS

static inline void console_verbose(void)
{
	extern int console_loglevel;
	console_loglevel = 15;
}

/*
 * Machine specific interrupt handlers
 */
extern asmlinkage void acer_pica_61_handle_int(void);
extern asmlinkage void decstation_handle_int(void);
extern asmlinkage void deskstation_rpc44_handle_int(void);
extern asmlinkage void deskstation_tyne_handle_int(void);
extern asmlinkage void mips_magnum_4000_handle_int(void);

extern asmlinkage void r4k_handle_mod(void);
extern asmlinkage void r2300_handle_mod(void);
extern asmlinkage void r4k_handle_tlbl(void);
extern asmlinkage void r2300_handle_tlbl(void);
extern asmlinkage void r4k_handle_tlbs(void);
extern asmlinkage void r2300_handle_tlbs(void);
extern asmlinkage void handle_adel(void);
extern asmlinkage void handle_ades(void);
extern asmlinkage void handle_ibe(void);
extern asmlinkage void handle_dbe(void);
extern asmlinkage void handle_sys(void);
extern asmlinkage void handle_bp(void);
extern asmlinkage void handle_ri(void);
extern asmlinkage void handle_cpu(void);
extern asmlinkage void handle_ov(void);
extern asmlinkage void handle_tr(void);
extern asmlinkage void handle_vcei(void);
extern asmlinkage void handle_fpe(void);
extern asmlinkage void handle_vced(void);
extern asmlinkage void handle_watch(void);
extern asmlinkage void handle_reserved(void);

static char *cpu_names[] = CPU_NAMES;

char watch_available = 0;
char dedicated_iv_available = 0;

void (*ibe_board_handler)(struct pt_regs *regs);
void (*dbe_board_handler)(struct pt_regs *regs);

int kstack_depth_to_print = 24;

/*
 * These constant is for searching for possible module text segments.
 * MODULE_RANGE is a guess of how much space is likely to be vmalloced.
 */
#define MODULE_RANGE (8*1024*1024)

/*
 * This routine abuses get_user()/put_user() to reference pointers
 * with at least a bit of error checking ...
 */
void show_registers(char * str, struct pt_regs * regs, long err)
{
	int	i;
	int	*stack;
	u32	*sp, *pc, addr, module_start, module_end;
	extern	char start_kernel, _etext;

	sp = (u32 *)regs->regs[29];
	pc = (u32 *)regs->cp0_epc;

	show_regs(regs);

	/*
	 * Dump the stack
	 */
	printk("Process %s (pid: %d, stackpage=%08lx)\nStack: ",
		current->comm, current->pid, (unsigned long)current);
	for(i=0;i<5;i++)
		printk("%08x ", *sp++);
	stack = (int *) sp;

	for(i=0; i < kstack_depth_to_print; i++) {
		unsigned int stackdata;

		if (((u32) stack & (PAGE_SIZE -1)) == 0)
			break;
		if (i && ((i % 8) == 0))
			printk("\n       ");
		stackdata = get_user(stack++);
		printk("%08x ", stackdata);
	}
	printk("\nCall Trace: ");
	stack = (int *)sp;
	i = 1;
	module_start = VMALLOC_START;
	module_end = module_start + MODULE_RANGE;
	while (((unsigned long)stack & (PAGE_SIZE -1)) != 0) {
		addr = get_user(stack++);
		/*
		 * If the address is either in the text segment of the
		 * kernel, or in the region which contains vmalloc'ed
		 * memory, it *may* be the address of a calling
		 * routine; if so, print it so that someone tracing
		 * down the cause of the crash will be able to figure
		 * out the call path that was taken.
		 */
		if (((addr >= (u32) &start_kernel) &&
		     (addr <= (u32) &_etext)) ||
		    ((addr >= module_start) && (addr <= module_end))) {
			if (i && ((i % 8) == 0))
				printk("\n       ");
			printk("%08x ", addr);
			i++;
		}
	}

	printk("\nCode : ");
	if ((KSEGX(pc) == KSEG0 || KSEGX(pc) == KSEG1) &&
	    (((unsigned long) pc & 3) == 0))
	{
		for(i=0;i<5;i++)
			printk("%08x ", *pc++);
		printk("\n");
	}
	else
		printk("(Bad address in epc)\n");
	do_exit(SIGSEGV);
}

void die_if_kernel(const char * str, struct pt_regs * regs, long err)
{
	/*
	 * Just return if in user mode.
	 * XXX
	 */
#if (_MIPS_ISA == _MIPS_ISA_MIPS1) || (_MIPS_ISA == _MIPS_ISA_MIPS2)
	if (!((regs)->cp0_status & 0x4))
		return;
#endif
#if (_MIPS_ISA == _MIPS_ISA_MIPS3) || (_MIPS_ISA == _MIPS_ISA_MIPS4)
	if (!(regs->cp0_status & 0x18))
		return;
#endif
	console_verbose();
	printk("%s: %04lx\n", str, err & 0xffff);
	show_regs(regs);
	do_exit(SIGSEGV);
}

static void default_be_board_handler(struct pt_regs *regs)
{
	/*
	 * Assume it would be too dangerous to continue ...
	 */
	force_sig(SIGBUS, current);
}

void do_ibe(struct pt_regs *regs)
{
	ibe_board_handler(regs);
}

void do_dbe(struct pt_regs *regs)
{
	dbe_board_handler(regs);
}

void do_ov(struct pt_regs *regs)
{
#ifdef CONF_DEBUG_EXCEPTIONS
	show_regs(regs);
#endif
	compute_return_epc(regs);
	force_sig(SIGFPE, current);
}

void do_fpe(struct pt_regs *regs)
{
	force_sig(SIGFPE, current);
}

static inline int get_insn_opcode(struct pt_regs *regs, unsigned int *opcode)
{
	unsigned int *epc;

	epc = (unsigned int *) (unsigned long) regs->cp0_epc;
	if (regs->cp0_cause & CAUSEF_BD)
		epc += 4;

	if (verify_area(VERIFY_READ, epc, 4)) {
		force_sig(SIGSEGV, current);
		return 1;
	}
	*opcode = *epc;

	return 0;
}

static inline void
do_bp_and_tr(struct pt_regs *regs, char *exc, unsigned int trapcode)
{
	/*
	 * (A short test says that IRIX 5.3 sends SIGTRAP for all break
	 * insns, even for break codes that indicate arithmetic failures.
	 * Wiered ...)
	 */
	force_sig(SIGTRAP, current);
#ifdef CONF_DEBUG_EXCEPTIONS
	show_regs(regs);
#endif
}

void do_bp(struct pt_regs *regs)
{
	unsigned int opcode, bcode;

	/*
	 * There is the ancient bug in the MIPS assemblers that the break
	 * code starts left to bit 16 instead to bit 6 in the opcode.
	 * Gas is bug-compatible ...
	 */
#ifdef CONF_DEBUG_EXCEPTIONS
	printk("BREAKPOINT at %08lx\n", regs->cp0_epc);
#endif
	if (get_insn_opcode(regs, &opcode))
		return;
	bcode = ((opcode >> 16) & ((1 << 20) - 1));

	do_bp_and_tr(regs, "bp", bcode);
}

void do_tr(struct pt_regs *regs)
{
	unsigned int opcode, bcode;

	if (get_insn_opcode(regs, &opcode))
		return;
	bcode = ((opcode >> 6) & ((1 << 20) - 1));

	do_bp_and_tr(regs, "tr", bcode);
}

void do_ri(struct pt_regs *regs)
{
#ifdef CONF_DEBUG_EXCEPTIONS
	show_regs(regs);
#endif
	printk("[%s:%d] Illegal instruction at %08lx ra=%08lx\n",
	       current->comm, current->pid, regs->cp0_epc, regs->regs[31]);
	compute_return_epc(regs);
	force_sig(SIGILL, current);
}

void do_cpu(struct pt_regs *regs)
{
	unsigned int cpid;

	cpid = (regs->cp0_cause >> CAUSEB_CE) & 3;
	if (cpid == 1) {
		memset(&current->tss.fpu, 0, sizeof(union mips_fpu_union));
		regs->cp0_status |= ST0_CU1;
		return;
	}
	force_sig(SIGILL, current);
}

void do_vcei(struct pt_regs *regs)
{
	/*
	 * Only possible on R4[04]00[SM]C. No handler because I don't have
	 * such a cpu.  Theory says this exception doesn't happen.
	 */
	panic("Caught VCEI exception - should not happen");
}

void do_vced(struct pt_regs *regs)
{
	/*
	 * Only possible on R4[04]00[SM]C. No handler because I don't have
	 * such a cpu.  Theory says this exception doesn't happen.
	 */
	panic("Caught VCE exception - should not happen");
}

void do_watch(struct pt_regs *regs)
{
	/*
	 * We use the watch exception where available to detect stack
	 * overflows.
	 */
	show_regs(regs);
	panic("Caught WATCH exception - probably caused by stack overflow.");
}

void do_reserved(struct pt_regs *regs)
{
	/*
	 * Game over - no way to handle this if it ever occurs.
	 * Most probably caused by a new unknown cpu type or
	 * after another deadly hard/software error.
	 */
	panic("Caught reserved exception - should not happen.");
}

static inline void watch_init(unsigned long cputype)
{
	switch(cputype) {
	case CPU_R10000:
	case CPU_R4000MC:
	case CPU_R4400MC:
	case CPU_R4000SC:
	case CPU_R4400SC:
	case CPU_R4000PC:
	case CPU_R4400PC:
	case CPU_R4200:
	case CPU_R4300:
		set_except_vector(23, handle_watch);
		watch_available = 1;
		break;
	}
}

/*
 * Some MIPS CPUs have a dedicated interrupt vector which reduces the
 * interrupt processing overhead.  Use it where available.
 * FIXME: more CPUs than just the Nevada have this feature.
 */
static inline void setup_dedicated_int(void)
{
	extern void except_vec4(void);
	switch(mips_cputype) {
	case CPU_NEVADA:
		memcpy((void *)(KSEG0 + 0x200), except_vec4, 8);
		set_cp0_cause(CAUSEF_IV, CAUSEF_IV);
		dedicated_iv_available = 1;
	}
}

unsigned long exception_handlers[32];

/*
 * As a side effect of the way this is implemented we're limited
 * to interrupt handlers in the address range from
 * KSEG0 <= x < KSEG0 + 256mb on the Nevada.  Oh well ...
 */
void set_except_vector(int n, void *addr)
{
	unsigned handler = (unsigned long) addr;
	exception_handlers[n] = handler;
	if (n == 0 && dedicated_iv_available) {
		*(volatile u32 *)(KSEG0+0x200) = 0x08000000 |
		                                 (0x03ffffff & (handler >> 2));
		flush_icache_range(KSEG0+0x200, KSEG0 + 0x204);
	}
}

typedef asmlinkage int (*syscall_t)(void *a0,...);

asmlinkage void (*save_fp_context)(struct sigcontext *sc);
extern asmlinkage void r4k_save_fp_context(struct sigcontext *sc);
extern asmlinkage void r2300_save_fp_context(struct sigcontext *sc);
extern asmlinkage void r6000_save_fp_context(struct sigcontext *sc);

asmlinkage void (*restore_fp_context)(struct sigcontext *sc);
extern asmlinkage void r4k_restore_fp_context(struct sigcontext *sc);
extern asmlinkage void r2300_restore_fp_context(struct sigcontext *sc);
extern asmlinkage void r6000_restore_fp_context(struct sigcontext *sc);

extern asmlinkage void r4xx0_resume(void *tsk);
extern asmlinkage void r2300_resume(void *tsk);

void trap_init(void)
{
	extern char except_vec0_nevada, except_vec0_r4000;
	extern except_vec0_r4600, except_vec0_r2300;
	extern char except_vec1_generic, except_vec2_generic;
	extern char except_vec3_generic, except_vec3_r4000;
	unsigned long i;

	if(mips_machtype == MACH_MIPS_MAGNUM_4000 ||
	   mips_machtype == MACH_DESKSTATION_RPC44 ||
	   mips_machtype == MACH_SNI_RM200_PCI)
		EISA_bus = 1;

	/* Copy the generic exception handler code to it's final destination. */
	memcpy((void *)(KSEG0 + 0x80), &except_vec1_generic, 0x80);
	memcpy((void *)(KSEG0 + 0x100), &except_vec2_generic, 0x80);
	memcpy((void *)(KSEG0 + 0x180), &except_vec3_generic, 0x80);

	/*
	 * Setup default vectors
	 */
	for(i = 0; i <= 31; i++)
		set_except_vector(i, handle_reserved);

	/*
	 * Only some CPUs have the watch exceptions or a dedicated
	 * interrupt vector.
	 */
	watch_init(mips_cputype);
	setup_dedicated_int();

	/*
	 * Handling the following exceptions depends mostly of the cpu type
	 */
	switch(mips_cputype) {
	case CPU_R10000:
		/*
		 * The R10000 is in most aspects similar to the R4400.  It
		 * should get some special optimizations.
		 */
		write_32bit_cp0_register(CP0_FRAMEMASK, 0);
		set_cp0_status(ST0_XX, ST0_XX);
		/*
		 * The R10k might even work for Linux/MIPS - but we're paranoid
		 * and refuse to run until this is tested on real silicon
		 */
		panic("CPU too expensive - making holiday in the ANDES!");
		break;
	case CPU_R4000MC:
	case CPU_R4400MC:
	case CPU_R4000SC:
	case CPU_R4400SC:
		/* XXX The following won't work because we _cannot_
		 * XXX perform any load/store before the VCE handler.
		 */
		set_except_vector(14, handle_vcei);
		set_except_vector(31, handle_vced);
	case CPU_R4000PC:
	case CPU_R4400PC:
	case CPU_R4200:
	case CPU_R4300:
     /* case CPU_R4640: */
	case CPU_R4600:
        case CPU_R5000:
        case CPU_NEVADA:
		if(mips_cputype == CPU_NEVADA) {
			memcpy((void *)KSEG0, &except_vec0_nevada, 0x80);
		} else if (mips_cputype == CPU_R4600)
			memcpy((void *)KSEG0, &except_vec0_r4600, 0x80);
		else
			memcpy((void *)KSEG0, &except_vec0_r4000, 0x80);

		/*
		 * The idea is that this special r4000 general exception
		 * vector will check for VCE exceptions before calling
		 * out of the exception array.  XXX TODO
		 */
		memcpy((void *)(KSEG0 + 0x100), (void *) KSEG0, 0x80);
		memcpy((void *)(KSEG0 + 0x180), &except_vec3_r4000, 0x80);

		save_fp_context = r4k_save_fp_context;
		restore_fp_context = r4k_restore_fp_context;
		resume = r4xx0_resume;
		set_except_vector(1, r4k_handle_mod);
		set_except_vector(2, r4k_handle_tlbl);
		set_except_vector(3, r4k_handle_tlbs);
		set_except_vector(4, handle_adel);
		set_except_vector(5, handle_ades);

		/*
		 * The following two are signaled by onboard hardware and
		 * should get board specific handlers to get maximum
		 * available information.
		 */
		set_except_vector(6, handle_ibe);
		set_except_vector(7, handle_dbe);

		ibe_board_handler = default_be_board_handler;
		dbe_board_handler = default_be_board_handler;

		set_except_vector(8, handle_sys);
		set_except_vector(9, handle_bp);
		set_except_vector(10, handle_ri);
		set_except_vector(11, handle_cpu);
		set_except_vector(12, handle_ov);
		set_except_vector(13, handle_tr);
		set_except_vector(15, handle_fpe);
		break;

	case CPU_R6000:
	case CPU_R6000A:
		save_fp_context = r6000_save_fp_context;
		restore_fp_context = r6000_restore_fp_context;
#if 0
		/*
		 * The R6000 is the only R-series CPU that features a machine
		 * check exception (similar to the R4000 cache error) and
		 * unaligned ldc1/sdc1 exception.  The handlers have not been
		 * written yet.  Well, anyway there is no R6000 machine on the
		 * current list of targets for Linux/MIPS.
		 * (Duh, crap, there is someone with a tripple R6k machine)
		 */
		set_except_vector(14, handle_mc);
		set_except_vector(15, handle_ndc);
#endif
	case CPU_R2000:
	case CPU_R3000:
	case CPU_R3000A:
		memcpy((void *)KSEG0, &except_vec0_r2300, 0x80);
		save_fp_context = r2300_save_fp_context;
		restore_fp_context = r2300_restore_fp_context;
		resume = r2300_resume;
		set_except_vector(1, r2300_handle_mod);
		set_except_vector(2, r2300_handle_tlbl);
		set_except_vector(3, r2300_handle_tlbs);
		set_except_vector(4, handle_adel);
		set_except_vector(5, handle_ades);
		/*
		 * The Data Bus Error/ Instruction Bus Errors are signaled
		 * by external hardware.  Therefore these two expection have
		 * board specific handlers.
		 */
		set_except_vector(6, handle_ibe);
		set_except_vector(7, handle_dbe);
		ibe_board_handler = default_be_board_handler;
		dbe_board_handler = default_be_board_handler;

		set_except_vector(8, handle_sys);
		set_except_vector(9, handle_bp);
		set_except_vector(10, handle_ri);
		set_except_vector(11, handle_cpu);
		set_except_vector(12, handle_ov);
		set_except_vector(13, handle_tr);
		set_except_vector(15, handle_fpe);
		break;
	case CPU_R3041:
	case CPU_R3051:
	case CPU_R3052:
	case CPU_R3081:
	case CPU_R3081E:
	case CPU_R8000:
		printk("Detected unsupported CPU type %s.\n",
			cpu_names[mips_cputype]);
		panic("Can't handle CPU");
		break;

	case CPU_UNKNOWN:
	default:
		panic("Unknown CPU type");
	}
	flush_icache_range(KSEG0, KSEG0 + 0x200);
}
