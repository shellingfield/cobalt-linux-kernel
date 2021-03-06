/*
 * $Id: b1pci.c,v 1.2 1999/07/07 05:56:11 thockin Exp $
 * 
 * Module for AVM B1 PCI-card.
 * 
 * (c) Copyright 1997 by Carsten Paeth (calle@calle.in-berlin.de)
 * 
 * $Log: b1pci.c,v $
 * Revision 1.2  1999/07/07 05:56:11  thockin
 * * Tue Jul 6 1999  Tim Hockin <thockin@cobaltnet.com>
 *   - Make menuconfig now works
 *
 *   - Using config-sk now builds just about everything as modules
 *     This should make a small enough kernel to use for ROM
 *
 *   - /lib/modules/%{version} is now included by this package
 *
 *   - .config is now included in this package
 *
 *   - Added $(MODROOT) for make modules_install
 *
 *   - ISDN4Linux tree pulled from 2.0.36
 *
 *   - Added PCI IDs for ISDN cards (Fritz Elfert)
 *
 *   - Added strstr symbol export
 *
 *   - Added isdnlog patch from Fritz Elfert
 *
 *   - config-sk now builds ISDN modules by default
 *
 *   - Changed /tmp/kernel to /var/tmp/kernel for BuildRoot
 *
 *   - Added %clean section to specfile
 *
 * Revision 1.2.2.2  1998/01/23 16:49:30  calle
 * added functions for pcmcia cards,
 * avmb1_addcard returns now the controller number.
 *
 * Revision 1.2.2.1  1997/11/26 10:46:57  calle
 * prepared for M1 (Mobile) and T1 (PMX) cards.
 * prepared to set configuration after load to support other D-channel
 * protocols, point-to-point and leased lines.
 *
 * Revision 1.2  1997/05/18 09:24:13  calle
 * added verbose disconnect reason reporting to avmb1.
 * some fixes in capi20 interface.
 * changed info messages for B1-PCI
 *
 * Revision 1.1  1997/03/30 17:10:42  calle
 * added support for AVM-B1-PCI card.
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/bios32.h>
#include <linux/pci.h>
#include <linux/skbuff.h>
#include "compat.h"
#include <linux/capi.h>
#include <linux/b1lli.h>

#ifndef PCI_VENDOR_ID_AVM
#define PCI_VENDOR_ID_AVM	0x1244
#endif

#ifndef PCI_DEVICE_ID_AVM_B1
#define PCI_DEVICE_ID_AVM_B1	0x700
#endif

static char *revision = "$Revision: 1.2 $";

/* ------------------------------------------------------------- */

#ifdef HAS_NEW_SYMTAB
MODULE_AUTHOR("Carsten Paeth <calle@calle.in-berlin.de>");
#endif

/* ------------------------------------------------------------- */

/* ------------------------------------------------------------- */
/* -------- Init & Cleanup ------------------------------------- */
/* ------------------------------------------------------------- */

/*
 * init / exit functions
 */

#ifdef MODULE
#define b1pci_init init_module
#endif

int b1pci_init(void)
{
	char *p;
	char rev[10];
	int rc;
	int pci_index;

	if ((p = strchr(revision, ':'))) {
		strcpy(rev, p + 1);
		p = strchr(rev, '$');
		*p = 0;
	} else
		strcpy(rev, " ??? ");


#ifdef CONFIG_PCI
	if (!pcibios_present()) {
		printk(KERN_ERR "b1pci: no PCI-BIOS present\n");
		return -EIO;
	}

	printk(KERN_INFO "b1pci: revision %s\n", rev);

	for (pci_index = 0; pci_index < 8; pci_index++) {
		unsigned char pci_bus, pci_device_fn;
		unsigned int ioaddr;
		unsigned char irq;

		if (pcibios_find_device (PCI_VENDOR_ID_AVM,
					PCI_DEVICE_ID_AVM_B1, pci_index,
					&pci_bus, &pci_device_fn) != 0) {
			continue;
		}
		pcibios_read_config_byte(pci_bus, pci_device_fn,
				PCI_INTERRUPT_LINE, &irq);
		pcibios_read_config_dword(pci_bus, pci_device_fn,
				PCI_BASE_ADDRESS_1, &ioaddr);
		/* Strip the I/O address out of the returned value */
		ioaddr &= PCI_BASE_ADDRESS_IO_MASK;
		printk(KERN_INFO
			"b1pci: PCI BIOS reports AVM-B1 at i/o %#x, irq %d\n",
			ioaddr, irq);
		if ((rc = avmb1_probecard(ioaddr, irq, AVM_CARDTYPE_B1)) != 0) {
		        printk(KERN_ERR
			"b1pci: no AVM-B1 at i/o %#x, irq %d detected\n",
			ioaddr, irq);
			return rc;
		}
		if ((rc = avmb1_addcard(ioaddr, irq, AVM_CARDTYPE_B1)) < 0)
			return rc;
	}
	return 0;
#else
	printk(KERN_ERR "b1pci: kernel not compiled with PCI.\n");
	return -EIO;
#endif
}

#ifdef MODULE
void cleanup_module(void)
{
}
#endif
