/*
 * r4xx0.c: R4000 processor variant specific MMU/Cache routines.
 *
 * Copyright (C) 1996 David S. Miller (dm@engr.sgi.com)
 *
 * $Id: r4xx0.c,v 1.4 1997/11/25 04:19:27 davem Exp $
 */
#include <linux/config.h>

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/autoconf.h>

#include <asm/sgi.h>
#include <asm/sgimc.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/bootinfo.h>
#include <asm/sgialib.h>
#include <asm/mmu_context.h>

/* CP0 hazard avoidance. */
#define BARRIER __asm__ __volatile__(".set noreorder\n\t" \
				     "nop; nop; nop; nop; nop; nop;\n\t" \
				     ".set reorder\n\t")

/* Primary cache parameters. */
int icache_size, dcache_size; /* Size in bytes */
int ic_lsize, dc_lsize;       /* LineSize in bytes */

/* Secondary cache (if present) parameters. */
static scache_size, sc_lsize;        /* Again, in bytes */

#include <asm/cacheops.h>
#include <asm/r4kcache.h>

#undef DEBUG_CACHE

/*
 * On processors with QED R4600 style two set assosicative cache
 * this is the bit which selects the way in the cache for the
 * indexed cachops.
 */
#define icache_waybit (icache_size >> 1)
#define dcache_waybit (dcache_size >> 1)

#undef DEBUG_TLB
#undef DEBUG_TLBUPDATE

#define NTLB_ENTRIES       48  /* Fixed on all R4XX0 variants... */
#define NTLB_ENTRIES_HALF  24  /* Fixed on all R4XX0 variants... */

#ifdef DEBUG_TLBUPDATE
static unsigned long ehi_debug[NTLB_ENTRIES];
static unsigned long el0_debug[NTLB_ENTRIES];
static unsigned long el1_debug[NTLB_ENTRIES];
#endif

static void r4k_show_regs(struct pt_regs * regs)
{
	/* Saved main processor registers. */
	printk("$0 : %08lx %08lx %08lx %08lx\n",
	       0UL, regs->regs[1], regs->regs[2], regs->regs[3]);
	printk("$4 : %08lx %08lx %08lx %08lx\n",
               regs->regs[4], regs->regs[5], regs->regs[6], regs->regs[7]);
	printk("$8 : %08lx %08lx %08lx %08lx\n",
	       regs->regs[8], regs->regs[9], regs->regs[10], regs->regs[11]);
	printk("$12: %08lx %08lx %08lx %08lx\n",
               regs->regs[12], regs->regs[13], regs->regs[14], regs->regs[15]);
	printk("$16: %08lx %08lx %08lx %08lx\n",
	       regs->regs[16], regs->regs[17], regs->regs[18], regs->regs[19]);
	printk("$20: %08lx %08lx %08lx %08lx\n",
               regs->regs[20], regs->regs[21], regs->regs[22], regs->regs[23]);
	printk("$24: %08lx %08lx\n",
	       regs->regs[24], regs->regs[25]);
	printk("$28: %08lx %08lx %08lx %08lx\n",
	       regs->regs[28], regs->regs[29], regs->regs[30], regs->regs[31]);

	/* Saved cp0 registers. */
	printk("epc   : %08lx\nStatus: %08lx\nCause : %08lx\n",
	       regs->cp0_epc, regs->cp0_status, regs->cp0_cause);
}

/* Detect and size the various r4k caches. */
static void probe_icache(unsigned long config)
{
	unsigned long tmp;

	tmp = (config >> 9) & 7;
	icache_size = (1 << (12 + tmp));
	if((config >> 5) & 1)
		ic_lsize = 32;
	else
		ic_lsize = 16;

	printk("Primary ICACHE %dK (linesize %d bytes)\n",
	       (int)(icache_size >> 10), (int)ic_lsize);
}

static void probe_dcache(unsigned long config)
{
	unsigned long tmp;

	tmp = (config >> 6) & 7;
	dcache_size = (1 << (12 + tmp));
	if((config >> 4) & 1)
		dc_lsize = 32;
	else
		dc_lsize = 16;

	printk("Primary DCACHE %dK (linesize %d bytes)\n",
	       (int)(dcache_size >> 10), (int)dc_lsize);
}

static int probe_scache_eeprom(unsigned long config)
{
#ifdef CONFIG_SGI
	volatile unsigned int *cpu_control;
	unsigned short cmd = 0xc220;
	unsigned long data = 0;
	int i, n;

#ifdef __MIPSEB__
	cpu_control = (volatile unsigned int *) KSEG1ADDR(0x1fa00034);
#else
	cpu_control = (volatile unsigned int *) KSEG1ADDR(0x1fa00030);
#endif
#define DEASSERT(bit) (*(cpu_control) &= (~(bit)))
#define ASSERT(bit) (*(cpu_control) |= (bit))
#define DELAY  for(n = 0; n < 100000; n++) __asm__ __volatile__("")
	DEASSERT(SGIMC_EEPROM_PRE);
	DEASSERT(SGIMC_EEPROM_SDATAO);
	DEASSERT(SGIMC_EEPROM_SECLOCK);
	DEASSERT(SGIMC_EEPROM_PRE);
	DELAY;
	ASSERT(SGIMC_EEPROM_CSEL); ASSERT(SGIMC_EEPROM_SECLOCK);
	for(i = 0; i < 11; i++) {
		if(cmd & (1<<15))
			ASSERT(SGIMC_EEPROM_SDATAO);
		else
			DEASSERT(SGIMC_EEPROM_SDATAO);
		DEASSERT(SGIMC_EEPROM_SECLOCK);
		ASSERT(SGIMC_EEPROM_SECLOCK);
		cmd <<= 1;
	}
	DEASSERT(SGIMC_EEPROM_SDATAO);
	for(i = 0; i < (sizeof(unsigned short) * 8); i++) {
		unsigned int tmp;

		DEASSERT(SGIMC_EEPROM_SECLOCK);
		DELAY;
		ASSERT(SGIMC_EEPROM_SECLOCK);
		DELAY;
		data <<= 1;
		tmp = *cpu_control;
		if(tmp & SGIMC_EEPROM_SDATAI)
			data |= 1;
	}
	DEASSERT(SGIMC_EEPROM_SECLOCK);
	DEASSERT(SGIMC_EEPROM_CSEL);
	ASSERT(SGIMC_EEPROM_PRE);
	ASSERT(SGIMC_EEPROM_SECLOCK);
	data <<= PAGE_SHIFT;
	printk("R4600/R5000 SCACHE size %dK ", (int) (data >> 10));
	switch(mips_cputype) {
	case CPU_R4600:
	case CPU_R4640:
		sc_lsize = 32;
		break;

	default:
		sc_lsize = 128;
		break;
	}
	printk("linesize %d bytes\n", sc_lsize);
	scache_size = data;
	if(data) {
		unsigned long addr, tmp1, tmp2;

		/* Enable r4600/r5000 cache.  But flush it first. */
		for(addr = KSEG0; addr < (KSEG0 + dcache_size);
		    addr += dc_lsize)
			flush_dcache_line_indexed(addr);
		for(addr = KSEG0; addr < (KSEG0 + icache_size);
		    addr += ic_lsize)
			flush_icache_line_indexed(addr);
		for(addr = KSEG0; addr < (KSEG0 + scache_size);
		    addr += sc_lsize)
			flush_scache_line_indexed(addr);

		/* R5000 scache enable is in CP0 config, on R4600 variants
		 * the scache is enable by the memory mapped cache controller.
		 */
		if(mips_cputype == CPU_R5000) {
			unsigned long config;

			config = read_32bit_cp0_register(CP0_CONFIG);
			config |= 0x1000;
			write_32bit_cp0_register(CP0_CONFIG, config);
		} else {
			/* This is really cool... */
			printk("Enabling R4600 SCACHE\n");
			__asm__ __volatile__("
			.set noreorder
			.set mips3
			mfc0	%2, $12
			nop; nop; nop; nop;
			li	%1, 0x80
			mtc0	%1, $12
			nop; nop; nop; nop;
			li	%0, 0x1
			dsll	%0, 31
			lui	%1, 0x9000
			dsll32	%1, 0
			or	%0, %1, %0
			sb	$0, 0(%0)
			mtc0	$0, $12
			nop; nop; nop; nop;
			mtc0	%2, $12
			nop; nop; nop; nop;
			.set mips0
			.set reorder
		        " : "=r" (tmp1), "=r" (tmp2), "=r" (addr));
		}

		return 1;
	} else {
		if(mips_cputype == CPU_R5000)
			return -1;
		else
			return 0;
	}
#else
	/*
	 * XXX For now we don't panic and assume that existing chipset
	 * controlled caches are setup correnctly and are completly
	 * transparent.  Works fine for those MIPS machines I know.
	 * Morituri the salutant ...
	 */
	return 0;

	panic("Cannot probe SCACHE on this machine.");
#endif
}

/* If you even _breathe_ on this function, look at the gcc output
 * and make sure it does not pop things on and off the stack for
 * the cache sizing loop that executes in KSEG1 space or else
 * you will crash and burn badly.  You have been warned.
 */
static int probe_scache(unsigned long config)
{
	extern unsigned long stext;
	unsigned long flags, addr, begin, end, pow2;
	int tmp;

	tmp = ((config >> 17) & 1);
	if(tmp)
		return 0;
	tmp = ((config >> 22) & 3);
	switch(tmp) {
	case 0:
		sc_lsize = 16;
		break;
	case 1:
		sc_lsize = 32;
		break;
	case 2:
		sc_lsize = 64;
		break;
	case 3:
		sc_lsize = 128;
		break;
	}

	begin = (unsigned long) &stext;
	begin &= ~((4 * 1024 * 1024) - 1);
	end = begin + (4 * 1024 * 1024);

	/* This is such a bitch, you'd think they would make it
	 * easy to do this.  Away you daemons of stupidity!
	 */
	save_and_cli(flags);

	/* Fill each size-multiple cache line with a valid tag. */
	pow2 = (64 * 1024);
	for(addr = begin; addr < end; addr = (begin + pow2)) {
		unsigned long *p = (unsigned long *) addr;
		__asm__ __volatile__("nop" : : "r" (*p)); /* whee... */
		pow2 <<= 1;
	}

	/* Load first line with zero (therefore invalid) tag. */
	set_taglo(0);
	set_taghi(0);
	__asm__ __volatile__("nop; nop; nop; nop;"); /* avoid the hazard */
	__asm__ __volatile__("\n\t.set noreorder\n\t"
			     ".set mips3\n\t"
			     "cache 8, (%0)\n\t"
			     ".set mips0\n\t"
			     ".set reorder\n\t" : : "r" (begin));
	__asm__ __volatile__("\n\t.set noreorder\n\t"
			     ".set mips3\n\t"
			     "cache 9, (%0)\n\t"
			     ".set mips0\n\t"
			     ".set reorder\n\t" : : "r" (begin));
	__asm__ __volatile__("\n\t.set noreorder\n\t"
			     ".set mips3\n\t"
			     "cache 11, (%0)\n\t"
			     ".set mips0\n\t"
			     ".set reorder\n\t" : : "r" (begin));

	/* Now search for the wrap around point. */
	pow2 = (128 * 1024);
	tmp = 0;
	for(addr = (begin + (128 * 1024)); addr < (end); addr = (begin + pow2)) {
		__asm__ __volatile__("\n\t.set noreorder\n\t"
				     ".set mips3\n\t"
				     "cache 7, (%0)\n\t"
				     ".set mips0\n\t"
				     ".set reorder\n\t" : : "r" (addr));
		__asm__ __volatile__("nop; nop; nop; nop;"); /* hazard... */
		if(!get_taglo())
			break;
		pow2 <<= 1;
	}
	restore_flags(flags);
	addr -= begin;
	printk("Secondary cache sized at %dK linesize %d\n", (int) (addr >> 10),
	       sc_lsize);
	scache_size = addr;
	return 1;
}

static void setup_noscache_funcs(void)
{
}

static void setup_scache_funcs(void)
{
}

typedef int (*probe_func_t)(unsigned long);
static probe_func_t probe_scache_kseg1;

void ld_mmu_r4xx0(void)
{
	unsigned long cfg = read_32bit_cp0_register(CP0_CONFIG);
	int sc_present = 0;

	printk("CPU revision is: %08x\n", read_32bit_cp0_register(CP0_PRID));

	set_cp0_config(CONFIG_CM_CMASK, CONFIG_CM_CACHABLE_NONCOHERENT);
//set_cp0_config(CONFIG_CM_CMASK, CONFIG_CM_CACHABLE_WA);
//set_cp0_config(CONFIG_CM_CMASK, CONFIG_CM_CACHABLE_NO_WA);

	probe_icache(cfg);
	probe_dcache(cfg);

	switch(mips_cputype) {
	case CPU_R4000PC:
	case CPU_R4000SC:
	case CPU_R4000MC:
	case CPU_R4400PC:
	case CPU_R4400SC:
	case CPU_R4400MC:
		probe_scache_kseg1 = (probe_func_t) (KSEG1ADDR(&probe_scache));
		sc_present = probe_scache_kseg1(cfg);
		break;

	case CPU_R4600:
	case CPU_R4640:
	case CPU_R4700:
	case CPU_R5000:	/* XXX: We don't handle the true R5000 SCACHE */
	case CPU_NEVADA:
		probe_scache_kseg1 = (probe_func_t)
			(KSEG1ADDR(&probe_scache_eeprom));
		sc_present = probe_scache_eeprom(cfg);

		/* Try using tags if eeprom gives us bogus data. */
		if(sc_present == -1) {
			probe_scache_kseg1 =
				(probe_func_t) (KSEG1ADDR(&probe_scache));
			sc_present = probe_scache_kseg1(cfg);
		}
		break;
	};

	if(sc_present == 1
	   && (mips_cputype == CPU_R4000SC
               || mips_cputype == CPU_R4000MC
               || mips_cputype == CPU_R4400SC
               || mips_cputype == CPU_R4400MC)) {
		/* Has a true secondary cache. */
		setup_scache_funcs();
	} else {
		/* Lacks true secondary cache. */
		setup_noscache_funcs();
		if((mips_cputype != CPU_R5000)) { /* XXX */
			;
		}
	}

	show_regs = r4k_show_regs;

	flush_cache_all();
	write_32bit_cp0_register(CP0_WIRED, 0);

	/*
	 * You should never change this register:
	 *   - On R4600 1.7 the tlbp never hits for pages smaller than
	 *     the value in the c0_pagemask register.
	 *   - The entire mm handling assumes the c0_pagemask register to
	 *     be set for 4kb pages.
	 */
	write_32bit_cp0_register(CP0_PAGEMASK, PM_4K);
	flush_tlb_all();
}
