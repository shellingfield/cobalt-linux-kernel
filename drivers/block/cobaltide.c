/* $Id: cobaltide.c,v 1.17 1999/01/28 11:34:14 davem Exp $
 * cobaltide.c: Support for DMA mode on IDE controller found on
 *              Cobalt Microserver CPU27 boards.
 *
 * Copyright (C) 1997 David S. Miller (davem@dm.cobaltmicro.com)
 */

/* Based almost entirely on Triton driver with MIPS hackery. */

#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>
#include <linux/pci.h>
#include <linux/bios32.h>

#include <asm/io.h>
#include <asm/dma.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/cacheops.h>

#include "ide.h"

/* Define this to get debugging info about DMA mode enabling etc. */
#undef DEBUG_DMA

/* Define this when we get UDMA working... -DaveM */

#define COBALT_UDMA

/*
 * Our Physical Region Descriptor (PRD) table should be large enough
 * to handle the biggest I/O request we are likely to see.  Since requests
 * can have no more than 256 sectors, and since the typical blocksize is
 * two sectors, we could get by with a limit of 128 entries here for the
 * usual worst case.  Most requests seem to include some contiguous blocks,
 * further reducing the number of table entries required.
 *
 * The driver reverts to PIO mode for individual requests that exceed
 * this limit (possible with 512 byte blocksizes, eg. MSDOS f/s), so handling
 * 100% of all crazy scenarios here is not necessary.
 *
 * As it turns out though, we must allocate a full 4KB page for this,
 * so the two PRD tables (ide0 & ide1) will each get half of that,
 * allowing each to have about 256 entries (8 bytes each) from this.
 */
#define PRD_BYTES	8
#define PRD_ENTRIES	(PAGE_SIZE / (2 * PRD_BYTES))
#define DEFAULT_BMIBA	0xcc00		/* in case ROM did not init it */

/*
 * dma_intr() is the handler for disk read/write DMA interrupts
 */
static void dma_intr (ide_drive_t *drive)
{
	byte stat, dma_stat;
	int i;
	struct request *rq = HWGROUP(drive)->rq;
	ide_ioreg_t dma_base = HWIF(drive)->dma_base;

	dma_stat = inb(dma_base+2);		/* get DMA status */
	outb(inb(dma_base)&~1, dma_base);	/* stop DMA operation */
	stat = GET_STAT();			/* get drive status */
	if (OK_STAT(stat,DRIVE_READY,drive->bad_wstat|DRQ_STAT)) {
		if ((dma_stat & 7) == 4) {	/* verify good DMA status */
			rq = HWGROUP(drive)->rq;
			for (i = rq->nr_sectors; i > 0;) {
				i -= rq->current_nr_sectors;
				ide_end_request(1, HWGROUP(drive));
			}
			return;
		}
		printk("%s: bad DMA status: 0x%02x\n", drive->name, dma_stat);
	}
	sti();
	ide_error(drive, "dma_intr", stat);
}

/*
 * build_dmatable() prepares a dma request.
 * Returns 0 if all went okay, returns 1 otherwise.
 */
static int build_dmatable (ide_drive_t *drive, unsigned int reading)
{
	struct request *rq = HWGROUP(drive)->rq;
	struct buffer_head *bh = rq->bh;
	unsigned long size, vaddr, addr, *table = HWIF(drive)->dmatable;
	unsigned int count = 0;

	do {
		/*
		 * Determine addr and size of next buffer area.  We assume that
		 * individual virtual buffers are always composed linearly in
		 * physical memory.  For example, we assume that any 8kB buffer
		 * is always composed of two adjacent physical 4kB pages rather
		 * than two possibly non-adjacent physical 4kB pages.
		 */
		if (bh == NULL) {  /* paging and tape requests have (rq->bh == NULL) */
			addr = virt_to_bus (rq->buffer);
#ifdef CONFIG_BLK_DEV_IDETAPE
			if (drive->media == ide_tape)
				size = drive->tape.pc->request_transfer;
			else
#endif /* CONFIG_BLK_DEV_IDETAPE */	
			size = rq->nr_sectors << 9;
		} else {
			/* group sequential buffers into one large buffer */
			addr = virt_to_bus (bh->b_data);
			size = bh->b_size;
			while ((bh = bh->b_reqnext) != NULL) {
				if ((addr + size) != virt_to_bus (bh->b_data))
					break;
				size += bh->b_size;
			}
		}

		/* Flush the cache for the DMA operation, if reading this is
		 * going TO memory so we don't care about data bits in the
		 * cache right now.
		 */
		vaddr = (unsigned long)bus_to_virt(addr);
		if(!reading)
		__asm__ __volatile__("
	.set noreorder
	.set mips3
1:	cache %2, 0x00(%1)
	cache %2, 0x20(%1)
	cache %2, 0x40(%1)
	cache %2, 0x60(%1)
	addu %1, %1, 0x80
	bne %1, %3, 1b
	 nop	
	.set mips0
	.set reorder
"		: "=r" (vaddr)
		: "0" (vaddr), "i" (Hit_Writeback_Inv_D),
		  "r" (vaddr + size));
		else
		__asm__ __volatile__("
	.set noreorder
	.set mips3
1:	cache %2, 0x00(%1)
	cache %2, 0x20(%1)
	cache %2, 0x40(%1)
	cache %2, 0x60(%1)
	addu %1, %1, 0x80
	bne %1, %3, 1b
	 nop	
	.set mips0
	.set reorder
"		: "=r" (vaddr)
		: "0" (vaddr), "i" (Hit_Invalidate_D),
		  "r" (vaddr + size));

		/*
		 * Fill in the dma table, without crossing any 64kB boundaries.
		 * We assume 16-bit alignment of all blocks.
		 */
		while (size) {
			if (++count >= PRD_ENTRIES) {
				printk("%s: DMA table too small\n", drive->name);
				return 1; /* revert to PIO for this request */
			} else {
				unsigned long bcount = 0x10000 - (addr & 0xffff);
				if (bcount > size)
					bcount = size;
				*table++ = addr;
				*table++ = bcount & 0xffff;
				addr += bcount;
				size -= bcount;
			}
		}
	} while (bh != NULL);
	if (count) {
		*--table |= 0x80000000;	/* set End-Of-Table (EOT) bit */
		return 0;
	}
	printk("%s: empty DMA table?\n", drive->name);
	return 1;	/* let the PIO routines handle this weirdness */
}

/*
 * good_dma_drives() lists the model names (from "hdparm -i")
 * of drives which do not support mword2 DMA but which are
 * known to work fine with this interface under Linux.
 */
const char *good_dma_drives[] = {"Micropolis 2112A",
				 "CONNER CTMA 4000",
				 "CONNER CTT8000-A",
				 NULL};

static __inline__ int wait_for_ready(ide_drive_t *drive)
{
	int timeout = 100000;

	while(--timeout) {
		byte stat = GET_STAT();

		if(!(stat & BUSY_STAT) && !(stat & ERR_STAT)) {
			if(stat & READY_STAT)
				break;
		}
		if(stat & ERR_STAT)
			return 1;
	}
	if(timeout == 0)
		return 1;

	/* Success. */
	return 0;
}

/* Enable DMA or UDMA mode on a target. */
#define IDE_SETXFER		0x03
#define IDE_SETFEATURE		0xef

static int cobaltide_do_setfeature(ide_drive_t *drive, byte command)
{
	OUT_BYTE(IDE_SETXFER, IDE_FEATURE_REG);
	OUT_BYTE(command, IDE_NSECTOR_REG);
	if(wait_for_ready(drive))
		return 1;
	OUT_BYTE(IDE_SETFEATURE, IDE_COMMAND_REG);
	if(wait_for_ready(drive))
		return 1;

	/* Success. */
	return 0;
}

#define IDE_DMA2_ENABLE		0x22

static int cobaltide_dma2_enable(ide_drive_t *drive)
{
	unsigned int cfgword, ret = 0;
	unsigned long flags;
	byte bus, fn;

	save_flags(flags);
	cli();

	if(cobaltide_do_setfeature(drive, IDE_DMA2_ENABLE)) {
		ret = 1;
		goto out;
	}

	bus = 0;
	fn = PCI_DEVFN(9, 1);

	pcibios_read_config_dword(bus, fn, 0x40, &cfgword);

	/* Configure prefetching. */
	cfgword |= (0x3 << 14);

	/* Set FIFO thresholds. */
	cfgword &= 0x90ffffff;
	cfgword |= 0x2a000000;

	/* Don't use IDE command snooping to enable/disable mastering. */
	cfgword &= ~(1 << 18);

	pcibios_write_config_dword(bus, fn, 0x40, cfgword);

	pcibios_read_config_dword(bus, fn, 0x44, &cfgword);

	/* Enable read retries. */
	cfgword |= 0x00000008;

	/* Make FIFO flushes happen on interrupts and end of sector. */
	cfgword &= 0xff0cffff;
	cfgword |= 0x00a00000;

	pcibios_write_config_dword(bus, fn, 0x44, cfgword);

	/* And finally set mode2 timing. */
	pcibios_read_config_dword(bus, fn, 0x48, &cfgword);
	cfgword &= ~(0xff000000 >> (drive->select.b.unit * 8));
	cfgword |=  (0x31000000 >> (drive->select.b.unit * 8));
	pcibios_write_config_dword(bus, fn, 0x48, cfgword);

out:
	restore_flags(flags);
	return ret;
}

#ifdef COBALT_UDMA

#define IDE_UDMA2_ENABLE	0x42

static int cobaltide_udma_enable(ide_drive_t *drive)
{
	unsigned int cfgword, ret = 0;
	unsigned long flags;
	byte bus, fn;

	bus = 0;
	fn = PCI_DEVFN(9, 1);

	/* The following sequence must be atomic. */
	save_flags(flags);
	cli();

	/* XXX buffering, FIFO allotment, read retries, FIFO flushing,
	 * XXX prefetching, etc.?
	 */

	pcibios_read_config_dword(bus, fn, 0x50, &cfgword);

	cfgword &= ~(0xe3000000 >> (8 * drive->select.b.unit));

	pcibios_write_config_dword(bus, fn, 0x50, cfgword);

	if(cobaltide_do_setfeature(drive, IDE_UDMA2_ENABLE))
		ret = 1;

	restore_flags(flags);

	return ret;
}
#endif /* COBALT_UDMA */

static int cobaltide_dma_enable(ide_drive_t *drive, struct hd_driveid *id)
{
	int ret;
#ifdef DEBUG_DMA
	printk("fv:um[%x:%x], ", id->field_valid, id->udma_modes);
#endif
#ifdef COBALT_UDMA
	if((id->field_valid & 0x4) && (id->udma_modes & 0x4)) {
#ifdef DEBUG_DMA
		printk("UltraDMA-Enable, ");
#endif
		ret = cobaltide_udma_enable(drive);
	} else
#endif
	{
#ifdef DEBUG_DMA
		printk("DMA2-Enable, ");
#endif
		ret = cobaltide_dma2_enable(drive);
	}
#ifdef DEBUG_DMA
	printk("fv:um[%x:%x], ", id->field_valid, id->udma_modes);
#endif
	return ret;
}

static int config_drive_for_dma (ide_drive_t *drive)
{
	const char **list;
	struct hd_driveid *id = drive->id;

	if (id && (id->capability & 1)) {
		/* Enable DMA on any drive that supports mword2 DMA */
		if ((id->field_valid & 2) && (id->dma_mword & 0x0004)) {
			if(cobaltide_dma_enable(drive, id) == 0) {
				drive->using_dma = 1;
				return 0;		/* DMA enabled */
			}
			return 1;
		}
		/* Consult the list of known "good" drives */
		list = good_dma_drives;
		while (*list) {
			if (!strcmp(*list++,id->model)) {
				drive->using_dma = 1;
				return 0;	/* DMA enabled */
			}
		}
	}
	return 1;	/* DMA not enabled */
}

static int cobalt_dmaproc (ide_dma_action_t func, ide_drive_t *drive)
{
	ide_ioreg_t dma_base = HWIF(drive)->dma_base;
	unsigned int reading = (1 << 3);

	switch (func) {
		case ide_dma_abort:
			outb(inb(dma_base)&~1, dma_base);	/* stop DMA */
			return 0;
		case ide_dma_check:
			return config_drive_for_dma (drive);
		case ide_dma_write:
			reading = 0;
		case ide_dma_read:
			break;
		case ide_dma_status_bad:
			return ((inb(dma_base+2) & 7) != 4);	/* verify good DMA status */
		case ide_dma_transferred:
			/* XXX (number of bytes actually transferred) */
			return (0);
		case ide_dma_begin:
			outb(inb(dma_base)|1, dma_base);	/* begin DMA */
			return 0;
		default:
			printk("cobalt_dmaproc: unsupported func: %d\n", func);
			return 1;
	}
	if (build_dmatable (drive, reading))
		return 1;
#if 0
	/* XXX TRIGGER LOGIC ANALYZER XXX -DaveM */
	inl(dma_base + 12);
#endif
	outl(virt_to_bus (HWIF(drive)->dmatable), dma_base + 4); /* PRD table */
	outb(reading, dma_base);			/* specify r/w */
	outb(inb(dma_base+2)|0x06, dma_base+2);		/* clear status bits */
#ifdef CONFIG_BLK_DEV_IDEATAPI
	if (drive->media != ide_disk)
		return 0;
#endif /* CONFIG_BLK_DEV_IDEATAPI */	
	ide_set_handler(drive, &dma_intr, WAIT_CMD);	/* issue cmd to drive */
	OUT_BYTE(reading ? WIN_READDMA : WIN_WRITEDMA, IDE_COMMAND_REG);
	outb(inb(dma_base)|1, dma_base);		/* begin DMA */
	return 0;
}

static void init_cobalt_dma (ide_hwif_t *hwif, ide_ioreg_t base)
{
	static unsigned long dmatable = 0;

	printk("    %s: BM-DMA at 0x%08lx-0x%08lx", hwif->name, base, base+7);
	if (check_region(base, 8)) {
		printk(" -- ERROR, PORTS ALREADY IN USE");
	} else {
		request_region(base, 8, "IDE DMA");
		hwif->dma_base = base;
		if (!dmatable) {
			/*
			 * The BM-DMA uses a full 32-bits, so we can
			 * safely use __get_free_page() here instead
			 * of __get_dma_pages() -- no ISA limitations.
			 */
			dmatable = __get_free_page(GFP_KERNEL);
		}
		if (dmatable) {
			flush_cache_pre_dma_out(dmatable,
						(PRD_ENTRIES*PRD_BYTES));
			hwif->dmatable = KSEG1ADDR((unsigned long *) dmatable);
			outl(virt_to_bus((unsigned long *)dmatable), base + 4);
			dmatable += (PRD_ENTRIES * PRD_BYTES);
			hwif->dmaproc  = &cobalt_dmaproc;
		}
	}
	printk("\n");
}

#ifdef DEBUG_DRIVER
static void dump_regs(void)
{
	byte bus, fn;
	byte b;
	unsigned short w;

	bus = 0;
	fn = PCI_DEVFN(9, 1);

	pcibios_read_config_word(bus, fn, PCI_COMMAND, &w);
	printk("IDE: PCI_COMMAND[0x04](%04x) ", w);
	pcibios_read_config_word(bus, fn, PCI_STATUS, &w);
	printk("PCI_STATUS[0x06](%04x)\n", w);

	pcibios_read_config_byte(bus, fn, PCI_CLASS_PROG, &b);
	printk("IDE: PCI_CLASS_PROG[0x09](%02x) ", b);
	pcibios_read_config_byte(bus, fn, PCI_LATENCY_TIMER, &b);
	printk("PCI_LATENCY_TIMER[0x0d](%02x)\n", b);

	pcibios_read_config_byte(bus, fn, 0x40, &b);
	printk("IDE: ChipEnable[0x40](%02x) ", b);
	pcibios_read_config_byte(bus, fn, 0x41, &b);
	printk("IdeConfiguration[0x41](%02x)\n", b);

	pcibios_read_config_byte(bus, fn, 0x43, &b);
	printk("IDE: FIFOConfiguration[0x43](%02x) ", b);
	pcibios_read_config_byte(bus, fn, 0x44, &b);
	printk("MiscControl1[0x44](%02x)\n", b);

	pcibios_read_config_byte(bus, fn, 0x45, &b);
	printk("IDE: MiscControl2[0x45](%02x) ", b);
	pcibios_read_config_byte(bus, fn, 0x46, &b);
	printk("MiscControl3[0x46](%02x)\n", b);

	pcibios_read_config_byte(bus, fn, 0x48, &b);
	printk("IDE: DriveTimingControl[0x48-0x4b](%02x:", b);
	pcibios_read_config_byte(bus, fn, 0x49, &b);
	printk("%02x:", b);
	pcibios_read_config_byte(bus, fn, 0x4a, &b);
	printk("%02x:", b);
	pcibios_read_config_byte(bus, fn, 0x4b, &b);
	printk("%02x) ", b);
	pcibios_read_config_byte(bus, fn, 0x4c, &b);
	printk("AddressSetupTime[0x4c](%02x)\n", b);

	pcibios_read_config_byte(bus, fn, 0x4e, &b);
	printk("IDE: SecondaryPortAccessTiming[0x4e](%02x) ", b);
	pcibios_read_config_byte(bus, fn, 0x4f, &b);
	printk("PrimaryPortAccessTiming[0x4f](%02x)\n", b);

	pcibios_read_config_byte(bus, fn, 0x50, &b);
	printk("IDE: UltraDMA33ExtTimingControl[0x50-0x53](%02x:", b);
	pcibios_read_config_byte(bus, fn, 0x51, &b);
	printk("%02x:", b);
	pcibios_read_config_byte(bus, fn, 0x52, &b);
	printk("%02x:", b);
	pcibios_read_config_byte(bus, fn, 0x53, &b);
	printk("%02x)\n", b);

	pcibios_read_config_word(bus, fn, 0x60, &w);
	printk("IDE: PrimarySectorSize[0x60](%04x) ", w);
	pcibios_read_config_word(bus, fn, 0x68, &w);
	printk("IDE: SecondarySectorSize[0x60](%04x)\n", w);
}
#endif /* DEBUG_DRIVER */

/* ide_init_cobalt() prepares the IDE driver for DMA operation. */
void ide_init_cobalt (void)
{
	byte bus, fn;
	int rc = 0, h;
	int try_again = 1, dma_enabled = 0;
	unsigned short cfgword, galileo_id;
	unsigned int bmiba;
	unsigned char lt;

	printk("ide: Cobalt MicroServer IDE controller.\n");

	/* Hardcoded for now... */
	bus = 0;
	fn = PCI_DEVFN(9,1);

	{
		unsigned int gal_cfg = *((volatile unsigned int *)0xb4000c00);
		printk("GAL CFG[%08x-->%08x]\n",
		       gal_cfg,
		       (gal_cfg & ~0x6) | 0x2);
		*((volatile unsigned int *)0xb4000c00) =
			(gal_cfg & ~0x6) | 0x2;
	}

	/* On all machines prior to Q2, we had the STOP line disconnected
	 * from Galileo to VIA on PCI.  The new Galileo does not function
	 * correctly unless we have it connected.
	 *
	 * Therefore we must set the disconnect/retry cycle values to
	 * something sensible when using the new Galileo.
	 */
	pcibios_read_config_word(0, 0, PCI_DEVICE_ID, &galileo_id);
	if(galileo_id == 0x4146) {
		/* New Galileo, assumes PCI stop line to VIA is connected. */
		*((volatile unsigned int *)0xb4000c04) = 0x00004020;
	} else {
		/* Old Galileo, assumes PCI STOP line to VIA is disconnected. */
		*((volatile unsigned int *)0xb4000c04) = 0x0000ffff;
	}

	/* Enable Bus Mastering and fast back to back. */
	pcibios_read_config_word(bus, fn, PCI_COMMAND, &cfgword);
	cfgword |= (PCI_COMMAND_FAST_BACK | PCI_COMMAND_MASTER);
	pcibios_write_config_word(bus, fn, PCI_COMMAND, cfgword);

#ifdef CONFIG_BLK_DEV_COBALT_SECONDARY
	/* Enable secondary interface.  ROM only enables primary one.  */
	{
		unsigned char iface_enable = 0xb;
		pcibios_write_config_byte(bus, fn, 0x40, iface_enable);
	}
#endif

	/* Set latency timer to reasonable value. */
	pcibios_read_config_byte(bus, fn, PCI_LATENCY_TIMER, &lt);
	if(lt < 64)
		pcibios_write_config_byte(bus, fn, PCI_LATENCY_TIMER, 64);
	pcibios_write_config_byte(bus, fn, PCI_CACHE_LINE_SIZE, 7);

	/* Get the bmiba base address. */
	do {
		pcibios_read_config_dword(bus, fn, 0x20, &bmiba);
		bmiba &= 0xfff0;	/* extract port base address */
		if (bmiba) {
			dma_enabled = 1;
			break;
		} else {
			printk("ide: BM-DMA base register is invalid (0x%08x)\n", bmiba);
			if (inb(DEFAULT_BMIBA) != 0xff || !try_again)
				break;
			printk("ide: setting BM-DMA base register to 0x%08x\n",
			       DEFAULT_BMIBA);
			pcibios_write_config_dword(bus, fn, 0x20, DEFAULT_BMIBA|1);
		}
	} while (try_again--);

	bmiba += 0x10000000;

	/* Save the dma_base port addr for each interface. */
	for (h = 0; h < MAX_HWIFS; ++h) {
		ide_hwif_t *hwif = &ide_hwifs[h];

		hwif->chipset = ide_cobalt;
		if (dma_enabled)
			init_cobalt_dma(hwif, bmiba + (h * 8));
	}

	if (rc)
		printk("ide: pcibios access failed - %s\n",
		       pcibios_strerror(rc));

#ifdef DEBUG_DRIVER
	dump_regs();
#endif /* DEBUG_DRIVER */
}
