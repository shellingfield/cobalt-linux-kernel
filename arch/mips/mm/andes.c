/* $Id: andes.c,v 1.4 1997/11/25 04:19:26 davem Exp $
 * andes.c: MMU and cache operations for the R10000 (ANDES).
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

void ld_mmu_andes(void)
{
	flush_cache_all();
	flush_tlb_all();
}
