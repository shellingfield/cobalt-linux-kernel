/* $Id: hfc_2bs0.h,v 1.1 1999/07/07 05:56:07 thockin Exp $

 *  specific defines for CCD's HFC 2BS0
 *
 * Author       Karsten Keil (keil@temic-ech.spacenet.de)
 *
 *
 * $Log: hfc_2bs0.h,v $
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
 * Revision 1.1.2.1  1997/10/17 22:10:43  keil
 * new files on 2.0
 *
 * Revision 1.1  1997/09/11 17:31:34  keil
 * Common part for HFC 2BS0 based cards
 *
 *
 */

#define HFC_CTMT	0xe0
#define HFC_CIRM  	0xc0
#define HFC_CIP		0x80
#define HFC_Z1		0x00
#define HFC_Z2		0x08
#define HFC_Z_LOW	0x00
#define HFC_Z_HIGH	0x04
#define HFC_F1_INC	0x28
#define HFC_FIFO_IN	0x2c
#define HFC_F1		0x30
#define HFC_F2		0x34
#define HFC_F2_INC	0x38
#define HFC_FIFO_OUT	0x3c
#define HFC_B1          0x00
#define HFC_B2		0x02
#define HFC_REC		0x01
#define HFC_SEND	0x00
#define HFC_CHANNEL(ch) (ch ? HFC_B2 : HFC_B1)

#define HFC_STATUS	0
#define HFC_DATA	1
#define HFC_DATA_NODEB	2

/* Status (READ) */
#define HFC_BUSY	0x01
#define HFC_TIMINT	0x02
#define HFC_EXTINT	0x04

/* CTMT (Write) */
#define HFC_CLTIMER 0x10
#define HFC_TIM50MS 0x08
#define HFC_TIMIRQE 0x04
#define HFC_TRANSB2 0x02
#define HFC_TRANSB1 0x01

/* CIRM (Write) */
#define HFC_RESET  	0x08
#define HFC_MEM8K	0x10
#define HFC_INTA	0x01
#define HFC_INTB	0x02
#define HFC_INTC	0x03
#define HFC_INTD	0x04
#define HFC_INTE	0x05
#define HFC_INTF	0x06

extern void main_irq_hfc(struct BCState *bcs);
extern void inithfc(struct IsdnCardState *cs);
extern void releasehfc(struct IsdnCardState *cs);
