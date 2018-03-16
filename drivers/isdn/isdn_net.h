/* $Id: isdn_net.h,v 1.3 1999/07/07 05:56:11 thockin Exp $

 * header for Linux ISDN subsystem, network related functions (linklevel).
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
 * $Log: isdn_net.h,v $
 * Revision 1.3  1999/07/07 05:56:11  thockin
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
 * Revision 1.5.2.2  1998/11/05 22:12:09  fritz
 * Changed mail-address.
 *
 * Revision 1.5.2.1  1998/10/25 15:48:29  fritz
 * Misc bugfixes and adaptions to new HiSax
 *
 * Revision 1.5  1997/02/10 20:12:47  fritz
 * Changed interface for reporting incoming calls.
 *
 * Revision 1.4  1997/02/03 23:16:48  fritz
 * Removed isdn_net_receive_callback prototype.
 *
 * Revision 1.3  1997/01/17 01:19:30  fritz
 * Applied chargeint patch.
 *
 * Revision 1.2  1996/04/20 16:29:43  fritz
 * Misc. typos
 *
 * Revision 1.1  1996/02/11 02:35:13  fritz
 * Initial revision
 *
 */

			      /* Definitions for hupflags:                */
#define ISDN_WAITCHARGE  1      /* did not get a charge info yet            */
#define ISDN_HAVECHARGE  2      /* We know a charge info                    */
#define ISDN_CHARGEHUP   4      /* We want to use the charge mechanism      */
#define ISDN_INHUP       8      /* Even if incoming, close after huptimeout */
#define ISDN_MANCHARGE  16      /* Charge Interval manually set             */

extern char *isdn_net_new(char *, struct device *);
extern char *isdn_net_newslave(char *);
extern int isdn_net_rm(char *);
extern int isdn_net_rmall(void);
extern int isdn_net_stat_callback(int, isdn_ctrl *);
extern int isdn_net_setcfg(isdn_net_ioctl_cfg *);
extern int isdn_net_getcfg(isdn_net_ioctl_cfg *);
extern int isdn_net_addphone(isdn_net_ioctl_phone *);
extern int isdn_net_getphones(isdn_net_ioctl_phone *, char *);
extern int isdn_net_delphone(isdn_net_ioctl_phone *);
extern int isdn_net_find_icall(int, isdn_ctrl *, int);
extern void isdn_net_hangup(struct device *);
extern void isdn_net_dial(void);
extern void isdn_net_autohup(void);
extern int isdn_net_force_hangup(char *);
extern int isdn_net_force_dial(char *);
extern isdn_net_dev *isdn_net_findif(char *);
extern int isdn_net_send_skb(struct device *, isdn_net_local *,
			     struct sk_buff *);
extern int isdn_net_rcv_skb(int, struct sk_buff *);
