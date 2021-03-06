/* $Id: isdn_common.h,v 1.4 2000/04/07 04:42:32 cjohnson Exp $

 * header for Linux ISDN subsystem, common used functions and debugging-switches (linklevel).
 *
 * Copyright 1994-1998  by Fritz Elfert (fritz@isdn4linux.de)
 * Copyright 1995,96    by Thinking Objects Software GmbH Wuerzburg
 * Copyright 1995,96    by Michael Hipp (Michael.Hipp@student.uni-tuebingen.de)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * $Log: isdn_common.h,v $
 * Revision 1.4  2000/04/07 04:42:32  cjohnson
 * - Enabled IPAUTOFW and CONFIG_DLCI/CONFIG_DLCI_MODULE
 *
 * - Merged IPPORTFW patch
 *
 * - Merged in latest (11/27/99) ISDN from Fritz Elfert
 *
 * - Added patches to adaptec driver for recent hardware
 *
 * Revision 1.3  1999/07/07 05:56:10  thockin
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
 * Revision 1.6.2.3  1998/11/05 22:11:50  fritz
 * Changed mail-address.
 *
 * Revision 1.6.2.2  1998/10/25 15:48:10  fritz
 * Misc bugfixes and adaptions to new HiSax
 *
 * Revision 1.6.2.1  1998/03/16 09:55:48  cal
 * Merged in TimRu-patches. Still needs validation in conjunction with ABC-patches.
 *
 * Revision 1.6  1997/02/28 02:32:44  fritz
 * Cleanup: Moved some tty related stuff from isdn_common.c
 *          to isdn_tty.c
 * Bugfix:  Bisync protocol did not behave like documented.
 *
 * Revision 1.5  1997/02/10 10:05:45  fritz
 * More changes for Kernel 2.1.X
 * Symbol information moved to isdn_syms.c
 *
 * Revision 1.4  1997/02/03 22:56:50  fritz
 * Removed isdn_writebuf_stub prototype.
 *
 * Revision 1.3  1996/05/19 00:13:05  fritz
 * Removed debug flag.
 *
 * Revision 1.2  1996/04/20 16:20:40  fritz
 * Misc. typos.
 *
 * Revision 1.1  1996/01/10 21:37:19  fritz
 * Initial revision
 *
 */

#undef  ISDN_DEBUG_MODEM_OPEN
#undef  ISDN_DEBUG_MODEM_IOCTL
#undef  ISDN_DEBUG_MODEM_WAITSENT
#undef  ISDN_DEBUG_MODEM_HUP
#undef  ISDN_DEBUG_MODEM_ICALL
#undef  ISDN_DEBUG_MODEM_DUMP
#undef  ISDN_DEBUG_MODEM_VOICE
#undef  ISDN_DEBUG_AT
#undef  ISDN_DEBUG_NET_DUMP
#undef  ISDN_DEBUG_NET_DIAL
#undef  ISDN_DEBUG_NET_ICALL

/* Prototypes */
extern void isdn_MOD_INC_USE_COUNT(void);
extern void isdn_MOD_DEC_USE_COUNT(void);
extern void isdn_free_channel(int di, int ch, int usage);
extern int isdn_command(isdn_ctrl *);
extern void isdn_all_eaz(int di, int ch);
extern int isdn_dc2minor(int di, int ch);
extern void isdn_info_update(void);
extern char *isdn_map_eaz2msn(char *msn, int di);
extern void isdn_timer_ctrl(int tf, int onoff);
extern void isdn_unexclusive_channel(int di, int ch);
extern int isdn_getnum(char **);
extern int isdn_readbchan(int, int, u_char *, u_char *, int, int);
extern int isdn_get_free_channel(int, int, int, int, int);
extern int isdn_writebuf_skb_stub(int, int, struct sk_buff *);
extern int register_isdn(isdn_if * i);
extern void isdn_printk(char *fmt, ...);
#if (LINUX_VERSION_CODE < 0x020111)
extern void isdn_export_syms(void);
#else
#define isdn_export_syms()
#endif
#if defined(ISDN_DEBUG_NET_DUMP) || defined(ISDN_DEBUG_MODEM_DUMP) || defined(CONFIG_ISDN_TIMEOUT_RULES)
extern void isdn_dumppkt(char *, u_char *, int, int);
#endif

#define isdn_putlog(fmt, args...) ( \
	printk(fmt , ## args), \
	isdn_printk(fmt , ## args) \
)
