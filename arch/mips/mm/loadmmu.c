/*
 * loadmmu.c: Setup cpu/cache specific function ptrs at boot time.
 *
 * Copyright (C) 1996 David S. Miller (dm@engr.sgi.com)
 *
 * $Id: loadmmu.c,v 1.4 1997/11/25 04:19:27 davem Exp $
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mm.h>

#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/bootinfo.h>
#include <asm/sgialib.h>

/* Miscellaneous. */
void (*show_regs)(struct pt_regs *);
asmlinkage void (*resume)(void *tsk);

extern void ld_mmu_r2300(void);
extern void ld_mmu_r4xx0(void);
extern void ld_mmu_r6000(void);
extern void ld_mmu_tfp(void);
extern void ld_mmu_andes(void);

void loadmmu(void)
{
	switch(mips_cputype) {
	case CPU_R2000:
	case CPU_R3000:
		printk("Loading R[23]00 MMU routines.\n");
		ld_mmu_r2300();
		break;

	case CPU_R4000PC:
	case CPU_R4000SC:
	case CPU_R4000MC:
	case CPU_R4200:
	case CPU_R4300:
	case CPU_R4400PC:
	case CPU_R4400SC:
	case CPU_R4400MC:
	case CPU_R4600:
	case CPU_R4640:
	case CPU_R4650:
	case CPU_R4700:
	case CPU_R5000:
	case CPU_R5000A:
	case CPU_NEVADA:
		printk("Loading R4000 MMU routines.\n");
		ld_mmu_r4xx0();
		break;

	case CPU_R6000:
	case CPU_R6000A:
		printk("Loading R6000 MMU routines.\n");
		ld_mmu_r6000();
		break;

	case CPU_R8000:
		printk("Loading TFP MMU routines.\n");
		ld_mmu_tfp();
		break;

	case CPU_R10000:
		printk("Loading R10000 MMU routines.\n");
		ld_mmu_andes();
		break;

	default:
		/* XXX We need an generic routine in the MIPS port
		 * XXX to jabber stuff onto the screen on all machines
		 * XXX before the console is setup.  The ARCS prom
		 * XXX routines look good for this, but only the SGI
		 * XXX code has a full library for that at this time.
		 */
		panic("Yeee, unsupported mmu/cache architecture.");
	}
}
