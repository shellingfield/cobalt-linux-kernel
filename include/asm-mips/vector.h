/*
 * include/asm-mips/vector.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1995, 1996, 1997 by Ralf Baechle
 */
#ifndef __ASM_MIPS_VECTOR_H
#define __ASM_MIPS_VECTOR_H

/*
 * These vector structures are not very good to maintain - they'd probably
 * grow to at leat three times the size - so I'll remove 'em and replace
 * the with lots of ordinary variables.
 */
extern void (*irq_setup)(void);
#ifdef __KERNEL__
extern asmlinkage void (*fd_cacheflush)(const void *addr, size_t size);
#endif
/*
 * This structure defines how to access various features of
 * different machine types and how to access them.
 */
struct feature {
	/*
	 * How to access the floppy controller's ports.
	 */
	unsigned char (*fd_inb)(unsigned int port);
	void (*fd_outb)(unsigned char value, unsigned int port);
	/*
	 * How to access the floppy DMA functions.
	 */
	void (*fd_enable_dma)(int channel);
	void (*fd_disable_dma)(int channel);
	int (*fd_request_dma)(int channel);
	void (*fd_free_dma)(int channel);
	void (*fd_clear_dma_ff)(int channel);
	void (*fd_set_dma_mode)(int channel, char mode);
	void (*fd_set_dma_addr)(int channel, unsigned int a);
	void (*fd_set_dma_count)(int channel, unsigned int count);
	int (*fd_get_dma_residue)(int channel);
	void (*fd_enable_irq)(int irq);
	void (*fd_disable_irq)(int irq);
	/*
	 * How to access the RTC register of the DS1287?
	 */
	unsigned char (*rtc_read_data)(unsigned long addr);
	void (*rtc_write_data)(unsigned char data, unsigned long addr);
};

extern struct feature *feature;

#endif /* __ASM_MIPS_VECTOR_H */
