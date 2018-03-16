/* $Id: siemens.h,v 1.2 1999/07/07 05:56:09 thockin Exp $
 *
 * siemens.h   ISAC and HSCX spezific Macros
 *
 * Author	Karsten Keil (keil@temic-ech.spacenet.de)
 *
 *
 * $Log: siemens.h,v $
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
 * Revision 1.4  1997/01/21 22:24:33  keil
 * cleanups
 *
 * Revision 1.3  1996/12/08 19:48:34  keil
 * adding Monitor channel registers
 *
 * Revision 1.2  1996/10/27 22:24:00  keil
 * HSCX version code removed
 *
 * Revision 1.1  1996/10/12 21:39:39  keil
 * Initial revision
 *
 *
*/


/* All Registers without FIFOs (original Siemens Spec - 20 hex) */

#define ISAC_MASK 0x0
#define ISAC_ISTA 0x0
#define ISAC_STAR 0x1
#define ISAC_CMDR 0x1
#define ISAC_EXIR 0x4
#define ISAC_RBCH 0xa
#define ISAC_ADF2 0x19
#define ISAC_SPCR 0x10
#define ISAC_ADF1 0x18
#define ISAC_CIR0 0x11
#define ISAC_CIX0 0x11
#define ISAC_STCR 0x17
#define ISAC_MODE 0x2
#define ISAC_RSTA 0x7
#define ISAC_RBCL 0x5
#define ISAC_TIMR 0x3
#define ISAC_SQXR 0x1b
#define ISAC_MOSR 0x1a
#define ISAC_MOCR 0x1a
#define ISAC_MOR0 0x12
#define ISAC_MOX0 0x12
#define ISAC_MOR1 0x14
#define ISAC_MOX1 0x14

#define HSCX_ISTA 0x0
#define HSCX_CCR1 0xf
#define HSCX_CCR2 0xc
#define HSCX_TSAR 0x11
#define HSCX_TSAX 0x10
#define HSCX_XCCR 0x12
#define HSCX_RCCR 0x13
#define HSCX_MODE 0x2
#define HSCX_CMDR 0x1
#define HSCX_EXIR 0x4
#define HSCX_XAD1 0x4
#define HSCX_XAD2 0x5
#define HSCX_RAH2 0x7
#define HSCX_RSTA 0x7
#define HSCX_TIMR 0x3
#define HSCX_STAR 0x1
#define HSCX_RBCL 0x5
#define HSCX_XBCH 0xd
#define HSCX_VSTR 0xe
#define HSCX_RLCR 0xe
#define HSCX_MASK 0x0
