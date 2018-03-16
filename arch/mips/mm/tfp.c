/* $Id: tfp.c,v 1.4 1997/11/25 04:19:27 davem Exp $
 * tfp.c: MMU and cache routines specific to the r8000 (TFP).
 *
 * Copyright (C) 1996 David S. Miller (dm@engr.sgi.com)
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mm.h>

#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/sgialib.h>

extern unsigned long mips_tlb_entries;

void ld_mmu_tfp(void)
{
	flush_cache_all();
	flush_tlb_all();
}

