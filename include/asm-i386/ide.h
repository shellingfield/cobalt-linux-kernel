/*
 *  linux/include/asm-i386/ide.h
 *
 *  Copyright (C) 1994-1996  Linus Torvalds & authors
 */

/*
 *  This file contains the MIPS architecture specific IDE code.
 */

#ifndef __ASM_I386_IDE_H
#define __ASM_I386_IDE_H

#ifdef __KERNEL__

typedef unsigned short ide_ioreg_t;

#define MAX_HWIFS	4	/* an arbitrary, but realistic limit */

static __inline__ int ide_default_irq(int base)
{
	switch (base) {
		case 0: return 14;
		case 1: return 15;
		case 2: return 11;
		case 3: return 10;
		default:
			return 0;
	}
}

static __inline__ ide_ioreg_t ide_default_io_base(int index)
{
	switch (index) {
		case 0:	return 0x1f0;
		case 1:	return 0x170;
		case 2: return 0x1e8;
		case 3: return 0x168;
		default:
			return 0;
	}
}

#endif /* __KERNEL__ */

#endif /* __ASM_I386_IDE_H */
