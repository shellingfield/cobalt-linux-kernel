/*
 * Export MIPS-specific functions needed for loadable modules.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1996, 1997 by Ralf Baechle
 *
 * $Id: ksyms.c,v 1.3 1997/11/25 04:19:26 davem Exp $
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/interrupt.h>

#include <asm/checksum.h>
#include <asm/dma.h>
#include <asm/floppy.h>
#include <asm/io.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/sgihpc.h>
#include <asm/segment.h>

static struct symbol_table arch_symbol_table = {
#include <linux/symtab_begin.h>
	X(EISA_bus),

	/*
	 * String functions
	 */
	XNOVERS(bcopy),
	XNOVERS(memcmp),
	XNOVERS(memset),
	XNOVERS(memcpy),
	XNOVERS(memmove),
	XNOVERS(strcat),
	XNOVERS(strchr),
	XNOVERS(strlen),
	XNOVERS(strncat),
	XNOVERS(strnlen),
	XNOVERS(strrchr),
	XNOVERS(strtok),

	X(clear_page),

	/*
	 * Userspace access stuff.
	 */
	X(active_ds),

	/* Networking helper routines. */
	X(csum_partial_copy),

	/*
	 * Functions to control caches.
	 */
	/* X(flush_page_to_ram),    no longer needed -DaveM */
	X(fd_cacheflush),
	X(flush_cache_all),

	/*
	 * Base address of ports for Intel style I/O.
	 */
	X(mips_io_port_base),

	/*
	 * Architecture specific stuff.
	 */
#ifdef CONFIG_MIPS_JAZZ
	X(vdma_alloc),
	X(vdma_free),
	X(vdma_log2phys),
#endif

#ifdef CONFIG_SGI
	X(hpc3c0),
#endif

	/*
	 * Kernel hacking ...
	 */
#ifdef CONFIG_MIPS_FPE_MODULE
	X(force_sig),
	X(__compute_return_epc),
	X(register_fpe),
	X(unregister_fpe),
#endif
#include <linux/symtab_end.h>
};

void arch_syms_export(void)
{
	register_symtab(&arch_symbol_table);
}
