
/*
 *  linux/include/asm-mips/ide.h
 *
 *  Copyright (C) 1994-1996  Linus Torvalds & authors
 */

/*
 *  This file contains the MIPS architecture specific IDE code.
 */

#ifndef __ASM_MIPS_IDE_H
#define __ASM_MIPS_IDE_H

#ifdef __KERNEL__

#include <linux/config.h>

typedef unsigned long ide_ioreg_t;

#ifdef CONFIG_BLK_DEV_COBALT_SECONDARY
#define MAX_HWIFS 2
#else
#define MAX_HWIFS 1
#endif

static __inline__ int ide_default_irq(int index)
{
	switch (index) {
		case 0: return 14;	/* We're cheating */
#ifdef CONFIG_BLK_DEV_COBALT_SECONDARY
		case 1: return 15;	/* VIA is in native mode by default. */
#endif
		default:
			return 0;
	}
}

static __inline__ ide_ioreg_t ide_default_io_base(int index)
{
	switch (index) {
		case 0:	return 0x100001f0;
#ifdef CONFIG_BLK_DEV_COBALT_SECONDARY
		case 1: return 0x10000170;
#endif
		default:
			return 0;
	}
}

#endif /* __KERNEL__ */

#endif /* __ASM_MIPS_IDE_H */
