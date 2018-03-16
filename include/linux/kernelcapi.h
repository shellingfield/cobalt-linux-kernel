/*
 * $Id: kernelcapi.h,v 1.2 1999/07/07 05:56:14 thockin Exp $
 * 
 * Kernel CAPI 2.0 Interface for Linux
 * 
 * (c) Copyright 1997 by Carsten Paeth (calle@calle.in-berlin.de)
 * 
 * $Log: kernelcapi.h,v $
 * Revision 1.2  1999/07/07 05:56:14  thockin
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
 * Revision 1.1  1997/03/04 21:27:33  calle
 * First version in isdn4linux
 *
 * Revision 2.2  1997/02/12 09:31:39  calle
 * new version
 *
 * Revision 1.1  1997/01/31 10:32:20  calle
 * Initial revision
 *
 * 
 */
#ifndef __KERNEL_CAPI_H__
#define __KERNEL_CAPI_H__

#define CAPI_MAXAPPL	20	/*
				   * maximum number of applications 
				 */
#define CAPI_MAXCONTR	4	/*
				   * maximum number of controller 
				 */
#define CAPI_MAXDATAWINDOW	8

#ifdef __KERNEL__

struct capi_interface {
	int (*capi_installed) (void);

	 __u16(*capi_register) (capi_register_params * rparam, __u16 * applidp);
	 __u16(*capi_release) (__u16 applid);
	 __u16(*capi_put_message) (__u16 applid, struct sk_buff * msg);
	 __u16(*capi_get_message) (__u16 applid, struct sk_buff ** msgp);
	 __u16(*capi_set_signal) (__u16 applid,
			      void (*signal) (__u16 applid, __u32 param),
				  __u32 param);
	 __u16(*capi_get_manufacturer) (__u16 contr, __u8 buf[CAPI_MANUFACTURER_LEN]);
	 __u16(*capi_get_version) (__u16 contr, struct capi_version * verp);
	 __u16(*capi_get_serial) (__u16 contr, __u8 serial[CAPI_SERIAL_LEN]);
	 __u16(*capi_get_profile) (__u16 contr, struct capi_profile * profp);

	/*
	 * to init controllers, data is always in user memory
	 */
	int (*capi_manufacturer) (unsigned int cmd, void *data);

};

#define	KCI_CONTRUP	0
#define	KCI_CONTRDOWN	1

struct capi_interface_user {
	char name[20];
	void (*callback) (unsigned int cmd, __u16 contr, void *data);
	struct capi_interface_user *next;
};

struct capi_interface *attach_capi_interface(struct capi_interface_user *);
int detach_capi_interface(struct capi_interface_user *);


#define CAPI_NOERROR                      0x0000

#define CAPI_TOOMANYAPPLS		  0x1001
#define CAPI_LOGBLKSIZETOSMALL	          0x1002
#define CAPI_BUFFEXECEEDS64K 	          0x1003
#define CAPI_MSGBUFSIZETOOSMALL	          0x1004
#define CAPI_ANZLOGCONNNOTSUPPORTED	  0x1005
#define CAPI_REGRESERVED		  0x1006
#define CAPI_REGBUSY 		          0x1007
#define CAPI_REGOSRESOURCEERR	          0x1008
#define CAPI_REGNOTINSTALLED 	          0x1009
#define CAPI_REGCTRLERNOTSUPPORTEXTEQUIP  0x100a
#define CAPI_REGCTRLERONLYSUPPORTEXTEQUIP 0x100b

#define CAPI_ILLAPPNR		          0x1101
#define CAPI_ILLCMDORSUBCMDORMSGTOSMALL   0x1102
#define CAPI_SENDQUEUEFULL		  0x1103
#define CAPI_RECEIVEQUEUEEMPTY	          0x1104
#define CAPI_RECEIVEOVERFLOW 	          0x1105
#define CAPI_UNKNOWNNOTPAR		  0x1106
#define CAPI_MSGBUSY 		          0x1107
#define CAPI_MSGOSRESOURCEERR	          0x1108
#define CAPI_MSGNOTINSTALLED 	          0x1109
#define CAPI_MSGCTRLERNOTSUPPORTEXTEQUIP  0x110a
#define CAPI_MSGCTRLERONLYSUPPORTEXTEQUIP 0x110b

#endif				/* __KERNEL__ */

#endif				/* __KERNEL_CAPI_H__ */
