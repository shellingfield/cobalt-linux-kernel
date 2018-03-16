/* $Id: ix1_micro.h,v 1.2 1999/07/07 05:56:09 thockin Exp $

 * ix1_micro.h  low level stuff for ITK ix1-micro Rev.2 isdn cards
 *
 *              derived from teles3.h from Karsten Keil
 *
 * Copyright (C) 1997 Klaus-Peter Nischke (ITK AG)   (for the modifications
 to the original file teles.c)
 *
 * $Log: ix1_micro.h,v $
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
 * Revision 1.1  1997/01/27 15:42:48  keil
 * first version
 *
 *
 */

/*
   For the modification done by the author the following terms and conditions
   apply (GNU PUBLIC LICENSE)


   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.


   You may contact Klaus-Peter Nischke by email: klaus@nischke.do.eunet.de
   or by conventional mail:

   Klaus-Peter Nischke
   Deusener Str. 287
   44369 Dortmund
   Germany
 */


extern void ix1micro_report(struct IsdnCardState *sp);
extern void release_io_ix1micro(struct IsdnCard *card);
extern int setup_ix1micro(struct IsdnCard *card);
extern int initix1micro(struct IsdnCardState *sp);
