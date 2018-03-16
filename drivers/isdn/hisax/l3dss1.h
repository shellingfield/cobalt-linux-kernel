/* $Id: l3dss1.h,v 1.1 1999/07/07 05:56:07 thockin Exp $
 *
 *  DSS1 (Euro) D-channel protocol defines
 *
 * $Log: l3dss1.h,v $
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
 * Revision 1.3.2.3  1998/05/27 18:06:14  keil
 * HiSax 3.0
 *
 * Revision 1.3.2.2  1998/02/03 23:16:10  keil
 * german AOC
 *
 * Revision 1.3.2.1  1997/10/17 22:10:52  keil
 * new files on 2.0
 *
 * Revision 1.3  1997/08/07 17:44:37  keil
 * Fix RESTART
 *
 * Revision 1.2  1997/08/03 14:36:34  keil
 * Implement RESTART procedure
 *
 * Revision 1.1  1997/07/27 21:08:38  keil
 * new
 *
 *
 *
 */
#define T303	4000
#define T304	30000
#define T305	30000
#define T308	4000
#define T310	30000
#define T313	4000
#define T318	4000
#define T319	4000

/*
 * Message-Types
 */

#define MT_ALERTING            0x01
#define MT_CALL_PROCEEDING     0x02
#define MT_CONNECT             0x07
#define MT_CONNECT_ACKNOWLEDGE 0x0f
#define MT_PROGRESS            0x03
#define MT_SETUP               0x05
#define MT_SETUP_ACKNOWLEDGE   0x0d
#define MT_RESUME              0x26
#define MT_RESUME_ACKNOWLEDGE  0x2e
#define MT_RESUME_REJECT       0x22
#define MT_SUSPEND             0x25
#define MT_SUSPEND_ACKNOWLEDGE 0x2d
#define MT_SUSPEND_REJECT      0x21
#define MT_USER_INFORMATION    0x20
#define MT_DISCONNECT          0x45
#define MT_RELEASE             0x4d
#define MT_RELEASE_COMPLETE    0x5a
#define MT_RESTART             0x46
#define MT_RESTART_ACKNOWLEDGE 0x4e
#define MT_SEGMENT             0x60
#define MT_CONGESTION_CONTROL  0x79
#define MT_INFORMATION         0x7b
#define MT_FACILITY            0x62
#define MT_NOTIFY              0x6e
#define MT_STATUS              0x7d
#define MT_STATUS_ENQUIRY      0x75

#define MT_INVALID             0xff

#define IE_BEARER              0x04
#define IE_CAUSE               0x08
#define IE_CALLID              0x10
#define IE_FACILITY            0x1c
#define IE_CALL_STATE          0x14
#define IE_CHANNEL_ID          0x18
#define IE_RESTART_IND         0x79