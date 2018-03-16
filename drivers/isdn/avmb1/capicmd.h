/*
 * $Id: capicmd.h,v 1.2 1999/07/07 05:56:11 thockin Exp $
 * 
 * CAPI 2.0 Interface for Linux
 * 
 * Copyright 1997 by Carsten Paeth (calle@calle.in-berlin.de)
 * 
 * $Log: capicmd.h,v $
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
 * Revision 1.1  1997/03/04 21:50:30  calle
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
#ifndef __CAPICMD_H__
#define __CAPICMD_H__

/*----- CAPI commands -----*/
#define CAPI_ALERT		    0x01
#define CAPI_CONNECT		    0x02
#define CAPI_CONNECT_ACTIVE	    0x03
#define CAPI_CONNECT_B3_ACTIVE	    0x83
#define CAPI_CONNECT_B3 	    0x82
#define CAPI_CONNECT_B3_T90_ACTIVE  0x88
#define CAPI_DATA_B3		    0x86
#define CAPI_DISCONNECT_B3	    0x84
#define CAPI_DISCONNECT 	    0x04
#define CAPI_FACILITY		    0x80
#define CAPI_INFO		    0x08
#define CAPI_LISTEN		    0x05
#define CAPI_MANUFACTURER	    0xff
#define CAPI_RESET_B3		    0x87
#define CAPI_SELECT_B_PROTOCOL	    0x41

/*----- CAPI subcommands -----*/

#define CAPI_REQ    0x80
#define CAPI_CONF   0x81
#define CAPI_IND    0x82
#define CAPI_RESP   0x83

/*----- CAPI combined commands -----*/

#define CAPICMD(cmd,subcmd)	(((cmd)<<8)|(subcmd))

#define CAPI_DISCONNECT_REQ		CAPICMD(CAPI_DISCONNECT,CAPI_REQ)
#define CAPI_DISCONNECT_CONF		CAPICMD(CAPI_DISCONNECT,CAPI_CONF)
#define CAPI_DISCONNECT_IND		CAPICMD(CAPI_DISCONNECT,CAPI_IND)
#define CAPI_DISCONNECT_RESP		CAPICMD(CAPI_DISCONNECT,CAPI_RESP)

#define CAPI_ALERT_REQ			CAPICMD(CAPI_ALERT,CAPI_REQ)
#define CAPI_ALERT_CONF			CAPICMD(CAPI_ALERT,CAPI_CONF)

#define CAPI_CONNECT_REQ		CAPICMD(CAPI_CONNECT,CAPI_REQ)
#define CAPI_CONNECT_CONF		CAPICMD(CAPI_CONNECT,CAPI_CONF)
#define CAPI_CONNECT_IND		CAPICMD(CAPI_CONNECT,CAPI_IND)
#define CAPI_CONNECT_RESP		CAPICMD(CAPI_CONNECT,CAPI_RESP)

#define CAPI_CONNECT_ACTIVE_REQ		CAPICMD(CAPI_CONNECT_ACTIVE,CAPI_REQ)
#define CAPI_CONNECT_ACTIVE_CONF	CAPICMD(CAPI_CONNECT_ACTIVE,CAPI_CONF)
#define CAPI_CONNECT_ACTIVE_IND		CAPICMD(CAPI_CONNECT_ACTIVE,CAPI_IND)
#define CAPI_CONNECT_ACTIVE_RESP	CAPICMD(CAPI_CONNECT_ACTIVE,CAPI_RESP)

#define CAPI_SELECT_B_PROTOCOL_REQ	CAPICMD(CAPI_SELECT_B_PROTOCOL,CAPI_REQ)
#define CAPI_SELECT_B_PROTOCOL_CONF	CAPICMD(CAPI_SELECT_B_PROTOCOL,CAPI_CONF)

#define CAPI_CONNECT_B3_ACTIVE_REQ	CAPICMD(CAPI_CONNECT_B3_ACTIVE,CAPI_REQ)
#define CAPI_CONNECT_B3_ACTIVE_CONF	CAPICMD(CAPI_CONNECT_B3_ACTIVE,CAPI_CONF)
#define CAPI_CONNECT_B3_ACTIVE_IND	CAPICMD(CAPI_CONNECT_B3_ACTIVE,CAPI_IND)
#define CAPI_CONNECT_B3_ACTIVE_RESP	CAPICMD(CAPI_CONNECT_B3_ACTIVE,CAPI_RESP)

#define CAPI_CONNECT_B3_REQ		CAPICMD(CAPI_CONNECT_B3,CAPI_REQ)
#define CAPI_CONNECT_B3_CONF		CAPICMD(CAPI_CONNECT_B3,CAPI_CONF)
#define CAPI_CONNECT_B3_IND		CAPICMD(CAPI_CONNECT_B3,CAPI_IND)
#define CAPI_CONNECT_B3_RESP		CAPICMD(CAPI_CONNECT_B3,CAPI_RESP)


#define CAPI_CONNECT_B3_T90_ACTIVE_IND	CAPICMD(CAPI_CONNECT_B3_T90_ACTIVE,CAPI_IND)
#define CAPI_CONNECT_B3_T90_ACTIVE_RESP	CAPICMD(CAPI_CONNECT_B3_T90_ACTIVE,CAPI_RESP)

#define CAPI_DATA_B3_REQ		CAPICMD(CAPI_DATA_B3,CAPI_REQ)
#define CAPI_DATA_B3_CONF		CAPICMD(CAPI_DATA_B3,CAPI_CONF)
#define CAPI_DATA_B3_IND		CAPICMD(CAPI_DATA_B3,CAPI_IND)
#define CAPI_DATA_B3_RESP		CAPICMD(CAPI_DATA_B3,CAPI_RESP)

#define CAPI_DISCONNECT_B3_REQ		CAPICMD(CAPI_DISCONNECT_B3,CAPI_REQ)
#define CAPI_DISCONNECT_B3_CONF		CAPICMD(CAPI_DISCONNECT_B3,CAPI_CONF)
#define CAPI_DISCONNECT_B3_IND		CAPICMD(CAPI_DISCONNECT_B3,CAPI_IND)
#define CAPI_DISCONNECT_B3_RESP		CAPICMD(CAPI_DISCONNECT_B3,CAPI_RESP)

#define CAPI_RESET_B3_REQ		CAPICMD(CAPI_RESET_B3,CAPI_REQ)
#define CAPI_RESET_B3_CONF		CAPICMD(CAPI_RESET_B3,CAPI_CONF)
#define CAPI_RESET_B3_IND		CAPICMD(CAPI_RESET_B3,CAPI_IND)
#define CAPI_RESET_B3_RESP		CAPICMD(CAPI_RESET_B3,CAPI_RESP)

#define CAPI_LISTEN_REQ			CAPICMD(CAPI_LISTEN,CAPI_REQ)
#define CAPI_LISTEN_CONF		CAPICMD(CAPI_LISTEN,CAPI_CONF)

#define CAPI_MANUFACTURER_REQ		CAPICMD(CAPI_MANUFACTURER,CAPI_REQ)
#define CAPI_MANUFACTURER_CONF		CAPICMD(CAPI_MANUFACTURER,CAPI_CONF)
#define CAPI_MANUFACTURER_IND		CAPICMD(CAPI_MANUFACTURER,CAPI_IND)
#define CAPI_MANUFACTURER_RESP		CAPICMD(CAPI_MANUFACTURER,CAPI_RESP)

#define CAPI_FACILITY_REQ		CAPICMD(CAPI_FACILITY,CAPI_REQ)
#define CAPI_FACILITY_CONF		CAPICMD(CAPI_FACILITY,CAPI_CONF)
#define CAPI_FACILITY_IND		CAPICMD(CAPI_FACILITY,CAPI_IND)
#define CAPI_FACILITY_RESP		CAPICMD(CAPI_FACILITY,CAPI_RESP)

#define CAPI_INFO_REQ			CAPICMD(CAPI_INFO,CAPI_REQ)
#define CAPI_INFO_CONF			CAPICMD(CAPI_INFO,CAPI_CONF)
#define CAPI_INFO_IND			CAPICMD(CAPI_INFO,CAPI_IND)
#define CAPI_INFO_RESP			CAPICMD(CAPI_INFO,CAPI_RESP)

#endif				/* __CAPICMD_H__ */
