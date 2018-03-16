/* $Id: teles0.h,v 1.2 1999/07/07 05:56:09 thockin Exp $
 *
 * teles0.h   Header for Teles 16.0 8.0 & compatible
 *
 * Author	Karsten Keil (keil@temic-ech.spacenet.de)
 *
 *
 * $Log: teles0.h,v $
 * Revision 1.2  1999/07/07 05:56:09  thockin
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
 * Revision 1.2  1997/01/21 22:26:52  keil
 * cleanups
 *
 * Revision 1.1  1996/10/13 20:03:48  keil
 * Initial revision
 *
 *
*/

extern	void teles0_report(struct IsdnCardState *sp);
extern  void release_io_teles0(struct IsdnCard *card);
extern	int  setup_teles0(struct IsdnCard *card);
extern  int  initteles0(struct IsdnCardState *sp);
