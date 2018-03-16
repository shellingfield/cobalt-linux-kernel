/*
 * Lowlevel hardware stuff for the MIPS based Cobalt microservers.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1997 Cobalt Microserver
 * Copyright (C) 1997 Ralf Baechle
 *
 * $Id: cobalt.h,v 1.1 1997/10/27 23:26:47 davem Exp $
 */
#ifndef __ASM_MIPS_COBALT_H 
#define __ASM_MIPS_COBALT_H 

/*
 * Base address of I/O ports
 */
#define COBALT_LOCAL_IO_SPACE	0xa0000000

/*
 * COBALT interrupt enable bits
 */
#define COBALT_IE_PCI		(1 << 0)
#define COBALT_IE_FLOPPY	(1 << 1)
#define COBALT_IE_KEYBOARD	(1 << 2)
#define COBALT_IE_SERIAL1	(1 << 3)
#define COBALT_IE_SERIAL2	(1 << 4)
#define COBALT_IE_PARALLEL	(1 << 5)
#define COBALT_IE_GPIO		(1 << 6)
#define COBALT_IE_RTC		(1 << 7)

/*
 * PCI defines
 */
#define COBALT_IE_ETHERNET	(1 << 7)
#define COBALT_IE_SCSI		(1 << 7)

/*
 * COBALT Interrupt Level definitions.
 * These should match the request IRQ id's.
 */
#define COBALT_TIMER_IRQ	0
#define COBALT_KEYBOARD_IRQ	1
#define COBALT_ETHERNET_IRQ	13
#define COBALT_SCC_IRQ		4
#define COBALT_SERIAL2_IRQ	4
#define COBALT_PARALLEL_IRQ	5
#define COBALT_FLOPPY_IRQ	6 /* needs to be consistent with floppy driver! */
#define COBALT_SCSI_IRQ		7

/*
 * Access the R4030 DMA and I/O Controller
 */
#ifndef __LANGUAGE_ASSEMBLY__

extern inline void r4030_delay(void)
{
__asm__ __volatile__(
	".set\tnoreorder\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	"nop\n\t"
	".set\treorder");
}

extern inline unsigned short r4030_read_reg16(unsigned addr)
{
	unsigned short ret = *((volatile unsigned short *)addr);
	r4030_delay();
	return ret;
}

extern inline unsigned int r4030_read_reg32(unsigned addr)
{
	unsigned int ret = *((volatile unsigned int *)addr);
	r4030_delay();
	return ret;
}

extern inline void r4030_write_reg16(unsigned addr, unsigned val)
{
	*((volatile unsigned short *)addr) = val;
	r4030_delay();
}

extern inline unsigned int r4030_write_reg32(unsigned addr, unsigned val)
{
	*((volatile unsigned int *)addr) = val;
	r4030_delay();
}

#endif /* !LANGUAGE_ASSEMBLY__ */

/*
 * Handling the VIA ISA host bridge.
 */

#define	VIA_DELAY()						\
{								\
	unsigned char ctr;					\
	for (ctr=0;ctr<1;ctr++);				\
}

#define VIA_PORT_WRITE(x,y)					\
{								\
	*((volatile unsigned char *) (0xb0000000 | x)) = y;	\
	VIA_DELAY();						\
}

#define VIA_PORT_READ(x)	(*((unsigned char *) (0xB0000000 | (x))))

#define RESET_VIA_TIMER()					\
	asm("sb\t%1,0x70(%0)\n\t"				\
	    "lb\$0,0x71(%0)"					\
	    : /* No outputs */					\
	    : "r" (0xb0000000), "i" (0x0c));

#endif /* __ASM_MIPS_COBALT_H */
