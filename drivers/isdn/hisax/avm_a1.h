/* $Id: avm_a1.h,v 1.2 1999/07/07 05:56:08 thockin Exp $
 *
 * avm_a1.h   Header for AVM A1 (Fritz) ISDN card
 *
 * Author	Karsten Keil (keil@temic-ech.spacenet.de)
 *
 *
 * $Log: avm_a1.h,v $
 * Revision 1.2  1999/07/07 05:56:08  thockin
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
 * Revision 1.2  1997/01/21 22:14:36  keil
 * cleanups
 *
 * Revision 1.1  1996/10/12 21:42:40  keil
 * Initial revision
 *
 *
*/

#define	 AVM_A1_STAT_ISAC	0x01
#define	 AVM_A1_STAT_HSCX	0x02
#define	 AVM_A1_STAT_TIMER	0x04

extern	void avm_a1_report(struct IsdnCardState *sp);
extern  void release_io_avm_a1(struct IsdnCard *card);
extern	int  setup_avm_a1(struct IsdnCard *card);
extern  int  initavm_a1(struct IsdnCardState *sp);
