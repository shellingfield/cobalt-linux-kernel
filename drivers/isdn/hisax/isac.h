/* $Id: isac.h,v 1.1 1999/07/07 05:56:07 thockin Exp $

 * isac.h   ISAC specific defines
 *
 * Author       Karsten Keil (keil@temic-ech.spacenet.de)
 *
 *
 * $Log: isac.h,v $
 * Revision 1.1  1999/07/07 05:56:07  thockin
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
 * Revision 1.3.2.3  1998/05/27 18:05:41  keil
 * HiSax 3.0
 *
 * Revision 1.3.2.2  1997/11/15 19:01:14  keil
 * ipac changes
 *
 * Revision 1.3.2.1  1997/10/17 22:10:50  keil
 * new files on 2.0
 *
 * Revision 1.3  1997/07/27 21:37:41  keil
 * T3 implemented; supervisor l1timer; B-channel TEST_LOOP
 *
 * Revision 1.2  1997/06/26 11:16:16  keil
 * first version
 *
 *
 */


/* All Registers original Siemens Spec  */

#define ISAC_MASK 0x20
#define ISAC_ISTA 0x20
#define ISAC_STAR 0x21
#define ISAC_CMDR 0x21
#define ISAC_EXIR 0x24
#define ISAC_ADF2 0x39
#define ISAC_SPCR 0x30
#define ISAC_ADF1 0x38
#define ISAC_CIR0 0x31
#define ISAC_CIX0 0x31
#define ISAC_CIR1 0x33
#define ISAC_CIX1 0x33
#define ISAC_STCR 0x37
#define ISAC_MODE 0x22
#define ISAC_RSTA 0x27
#define ISAC_RBCL 0x25
#define ISAC_RBCH 0x2A
#define ISAC_TIMR 0x23
#define ISAC_SQXR 0x3b
#define ISAC_MOSR 0x3a
#define ISAC_MOCR 0x3a
#define ISAC_MOR0 0x32
#define ISAC_MOX0 0x32
#define ISAC_MOR1 0x34
#define ISAC_MOX1 0x34

#define ISAC_RBCH_XAC 0x80

#define ISAC_CMD_TIM	0x0
#define ISAC_CMD_RS	0x1
#define ISAC_CMD_SCZ	0x4
#define ISAC_CMD_SSZ	0x2
#define ISAC_CMD_AR8	0x8
#define ISAC_CMD_AR10	0x9
#define ISAC_CMD_ARL	0xA
#define ISAC_CMD_DUI	0xF

#define ISAC_IND_RS	0x1
#define ISAC_IND_PU	0x7
#define ISAC_IND_DR	0x0
#define ISAC_IND_SD	0x2
#define ISAC_IND_DIS	0x3
#define ISAC_IND_EI	0x6
#define ISAC_IND_RSY	0x4
#define ISAC_IND_ARD	0x8
#define ISAC_IND_TI	0xA
#define ISAC_IND_ATI	0xB
#define ISAC_IND_AI8	0xC
#define ISAC_IND_AI10	0xD
#define ISAC_IND_DID	0xF

extern void ISACVersion(struct IsdnCardState *cs, char *s);
extern void initisac(struct IsdnCardState *cs);
extern void isac_interrupt(struct IsdnCardState *cs, u_char val);
extern void clear_pending_isac_ints(struct IsdnCardState *cs);
