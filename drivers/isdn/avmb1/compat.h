/*
 * $Id: compat.h,v 1.2 1999/07/07 05:56:11 thockin Exp $
 * 
 * Headerfile for Compartibility between different kernel versions
 * 
 * (c) Copyright 1996 by Carsten Paeth (calle@calle.in-berlin.de)
 * 
 * $Log: compat.h,v $
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
 * Revision 1.1.2.1  1998/10/25 14:36:22  fritz
 * Backported from MIPS (Cobalt).
 *
 * Revision 1.1  1997/03/04 21:50:36  calle
 * Frirst version in isdn4linux
 *
 * Revision 2.2  1997/02/12 09:31:39  calle
 * new version
 *
 * Revision 1.1  1997/01/31 10:32:20  calle
 * Initial revision
 *
 * 
 */
#ifndef __COMPAT_H__
#define __COMPAT_H__

#include <linux/version.h>
#include <asm/segment.h>
#include <linux/isdnif.h>

#if LINUX_VERSION_CODE >= 0x020112	/* 2.1.18 */
#define HAS_NEW_SYMTAB
#endif

#endif				/* __COMPAT_H__ */
