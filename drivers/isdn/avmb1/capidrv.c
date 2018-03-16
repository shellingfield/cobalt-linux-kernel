/*
 * $Id: capidrv.c,v 1.1 1997/11/29 02:01:32 davem Exp $
 *
 * ISDN4Linux Driver, using capi20 interface (kernelcapi)
 *
 * Copyright 1997 by Carsten Paeth (calle@calle.in-berlin.de)
 *
 * $Log: capidrv.c,v $
 * Revision 1.1  1997/11/29 02:01:32  davem
 * Merge to 2.0.32
 *
 * Revision 1.3.2.1  1997/07/13 12:16:48  calle
 * bug fix for more than one controller in connect_req.
 *
 * Revision 1.3  1997/05/18 09:24:15  calle
 * added verbose disconnect reason reporting to avmb1.
 * some fixes in capi20 interface.
 * changed info messages for B1-PCI
 *
 * Revision 1.2  1997/03/05 21:19:59  fritz
 * Removed include of config.h (mkdep stated this is unneded).
 *
 * Revision 1.1  1997/03/04 21:50:31  calle
 * Frirst version in isdn4linux
 *
 * Revision 2.2  1997/02/12 09:31:39  calle
 * new version
 *
 * Revision 1.1  1997/01/31 10:32:20  calle
 * Initial revision
 *
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/sched.h>
#include <linux/malloc.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/signal.h>
#include <linux/mm.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/skbuff.h>
#include <linux/isdn.h>
#include <linux/isdnif.h>
#include <linux/capi.h>
#include <linux/kernelcapi.h>

#include "compat.h"
#include "capiutil.h"
#include "capicmd.h"
#include "capidrv.h"

static char *revision = "$Revision: 1.1 $";
int debugmode = 0;

#ifdef HAS_NEW_SYMTAB
MODULE_AUTHOR("Carsten Paeth <calle@calle.in-berlin.de>");
MODULE_PARM(debugmode, "i");
#endif

/* -------- type definitions ----------------------------------------- */


struct capidrv_contr {

	struct capidrv_contr *next;

	__u32 contrnr;
	char name[20];

	/*
	 * for isdn4linux
	 */
	isdn_if interface;
	int myid;

	/*
	 * LISTEN state
	 */
	int state;
	__u32 cipmask;
	__u32 cipmask2;

	/*
	 * ID of capi message sent
	 */
	__u16 msgid;

	/*
	 * B-Channels
	 */
	int nbchan;
	struct capidrv_bchan {
		struct capidrv_contr *contr;
		__u8 msn[ISDN_MSNLEN];
		int l2;
		int l3;
		__u8 num[ISDN_MSNLEN];
		__u8 mynum[ISDN_MSNLEN];
		int si1;
		int si2;
		int incoming;
		int disconnecting;
		struct capidrv_plci {
			struct capidrv_plci *next;
			__u32 plci;
			__u32 ncci;	/* ncci for CONNECT_ACTIVE_IND */
			__u16 msgid;	/* to identfy CONNECT_CONF */
			int chan;
			int state;
			struct capidrv_ncci {
				struct capidrv_ncci *next;
				struct capidrv_plci *plcip;
				__u32 ncci;
				__u16 msgid;	/* to identfy CONNECT_B3_CONF */
				int chan;
				int state;
				int oldstate;
				/* */
				__u16 datahandle;
			} *ncci_list;
		} *plcip;
		struct capidrv_ncci *nccip;
	} *bchans;

	struct capidrv_plci *plci_list;
};

struct capidrv_data {
	__u16 appid;
	int ncontr;
	struct capidrv_contr *contr_list;
};

typedef struct capidrv_plci capidrv_plci;
typedef struct capidrv_ncci capidrv_ncci;
typedef struct capidrv_contr capidrv_contr;
typedef struct capidrv_data capidrv_data;
typedef struct capidrv_bchan capidrv_bchan;

/* -------- data definitions ----------------------------------------- */

static capidrv_data global;
static struct capi_interface *capifuncs;

/* -------- convert functions ---------------------------------------- */

static inline __u32 b1prot(int l2, int l3)
{
	switch (l2) {
	case ISDN_PROTO_L2_X75I:
	case ISDN_PROTO_L2_X75UI:
	case ISDN_PROTO_L2_X75BUI:
		return 0;
	case ISDN_PROTO_L2_HDLC:
	default:
		return 0;
	case ISDN_PROTO_L2_TRANS:
		return 1;
	}
}

static inline __u32 b2prot(int l2, int l3)
{
	switch (l2) {
	case ISDN_PROTO_L2_X75I:
	case ISDN_PROTO_L2_X75UI:
	case ISDN_PROTO_L2_X75BUI:
	default:
		return 0;
	case ISDN_PROTO_L2_HDLC:
		return 1;
	case ISDN_PROTO_L2_TRANS:
		return 0;
	}
}

static inline __u32 b3prot(int l2, int l3)
{
	switch (l2) {
	case ISDN_PROTO_L2_X75I:
	case ISDN_PROTO_L2_X75UI:
	case ISDN_PROTO_L2_X75BUI:
	case ISDN_PROTO_L2_HDLC:
	case ISDN_PROTO_L2_TRANS:
	default:
		return 0;
	}
}

static inline __u16 si2cip(__u8 si1, __u8 si2)
{
	static const __u8 cip[17][5] =
	{
	/*  0  1  2  3  4  */
		{0, 0, 0, 0, 0},	/*0 */
		{16, 16, 4, 26, 16},	/*1 */
		{17, 17, 17, 4, 4},	/*2 */
		{2, 2, 2, 2, 2},	/*3 */
		{18, 18, 18, 18, 18},	/*4 */
		{2, 2, 2, 2, 2},	/*5 */
		{0, 0, 0, 0, 0},	/*6 */
		{2, 2, 2, 2, 2},	/*7 */
		{2, 2, 2, 2, 2},	/*8 */
		{21, 21, 21, 21, 21},	/*9 */
		{19, 19, 19, 19, 19},	/*10 */
		{0, 0, 0, 0, 0},	/*11 */
		{0, 0, 0, 0, 0},	/*12 */
		{0, 0, 0, 0, 0},	/*13 */
		{0, 0, 0, 0, 0},	/*14 */
		{22, 22, 22, 22, 22},	/*15 */
		{27, 27, 27, 28, 27}	/*16 */
	};
	if (si1 > 16)
		si1 = 0;
	if (si2 > 4)
		si2 = 0;

	return (__u16) cip[si1][si2];
}

static inline __u8 cip2si1(__u16 cipval)
{
	static const __u8 si[32] =
	{7, 1, 7, 7, 1, 1, 7, 7,	/*0-7 */
	 7, 1, 0, 0, 0, 0, 0, 0,	/*8-15 */
	 1, 2, 4, 10, 9, 9, 15, 7,	/*16-23 */
	 7, 7, 1, 16, 16, 0, 0, 0};	/*24-31 */

	if (cipval > 31)
		cipval = 0;	/* .... */
	return si[cipval];
}

static inline __u8 cip2si2(__u16 cipval)
{
	static const __u8 si[32] =
	{0, 0, 0, 0, 2, 3, 0, 0,	/*0-7 */
	 0, 3, 0, 0, 0, 0, 0, 0,	/*8-15 */
	 1, 2, 0, 0, 9, 0, 0, 0,	/*16-23 */
	 0, 0, 3, 2, 3, 0, 0, 0};	/*24-31 */

	if (cipval > 31)
		cipval = 0;	/* .... */
	return si[cipval];
}


/* -------- controller managment ------------------------------------- */

static inline capidrv_contr *findcontrbydriverid(int driverid)
{
	capidrv_contr *p = global.contr_list;

	while (p) {
		if (p->myid == driverid)
			return p;
		p = p->next;
	}
	return (capidrv_contr *) 0;
}

static capidrv_contr *findcontrbynumber(__u32 contr)
{
	capidrv_contr *p = global.contr_list;

	while (p) {
		if (p->contrnr == contr)
			return p;
		p = p->next;
	}
	return (capidrv_contr *) 0;
}


/* -------- plci management ------------------------------------------ */

static capidrv_plci *new_plci(capidrv_contr * card, int chan)
{
	capidrv_plci *plcip;

	plcip = (capidrv_plci *) kmalloc(sizeof(capidrv_plci), GFP_ATOMIC);

	if (plcip == 0)
		return 0;

	memset(plcip, 0, sizeof(capidrv_plci));
	plcip->state = ST_PLCI_NONE;
	plcip->plci = 0;
	plcip->msgid = 0;
	plcip->chan = chan;
	plcip->next = card->plci_list;
	card->plci_list = plcip;
	card->bchans[chan].plcip = plcip;

	return plcip;
}

static capidrv_plci *find_plci_by_plci(capidrv_contr * card, __u32 plci)
{
	capidrv_plci *p;
	for (p = card->plci_list; p; p = p->next)
		if (p->plci == plci)
			return p;
	return 0;
}

static capidrv_plci *find_plci_by_msgid(capidrv_contr * card, __u16 msgid)
{
	capidrv_plci *p;
	for (p = card->plci_list; p; p = p->next)
		if (p->msgid == msgid)
			return p;
	return 0;
}

static capidrv_plci *find_plci_by_ncci(capidrv_contr * card, __u32 ncci)
{
	capidrv_plci *p;
	for (p = card->plci_list; p; p = p->next)
		if (p->plci == (ncci & 0xffff))
			return p;
	return 0;
}

static void free_plci(capidrv_contr * card, capidrv_plci * plcip)
{
	capidrv_plci **pp;

	for (pp = &card->plci_list; *pp; pp = &(*pp)->next) {
		if (*pp == plcip) {
			*pp = (*pp)->next;
			card->bchans[plcip->chan].plcip = 0;
			card->bchans[plcip->chan].disconnecting = 0;
			card->bchans[plcip->chan].incoming = 0;
			kfree(plcip);
			return;
		}
	}
	printk(KERN_ERR "capidrv: free_plci %p (0x%x) not found, Huh?\n",
	       plcip, plcip->plci);
}

/* -------- ncci management ------------------------------------------ */

static inline capidrv_ncci *new_ncci(capidrv_contr * card,
				     capidrv_plci * plcip,
				     __u32 ncci)
{
	capidrv_ncci *nccip;

	nccip = (capidrv_ncci *) kmalloc(sizeof(capidrv_ncci), GFP_ATOMIC);

	if (nccip == 0)
		return 0;

	memset(nccip, 0, sizeof(capidrv_ncci));
	nccip->ncci = ncci;
	nccip->state = ST_NCCI_NONE;
	nccip->plcip = plcip;
	nccip->chan = plcip->chan;
	nccip->datahandle = 0;

	nccip->next = plcip->ncci_list;
	plcip->ncci_list = nccip;

	card->bchans[plcip->chan].nccip = nccip;

	return nccip;
}

static inline capidrv_ncci *find_ncci(capidrv_contr * card, __u32 ncci)
{
	capidrv_plci *plcip;
	capidrv_ncci *p;

	if ((plcip = find_plci_by_ncci(card, ncci)) == 0)
		return 0;

	for (p = plcip->ncci_list; p; p = p->next)
		if (p->ncci == ncci)
			return p;
	return 0;
}

static inline capidrv_ncci *find_ncci_by_msgid(capidrv_contr * card,
					       __u32 ncci, __u16 msgid)
{
	capidrv_plci *plcip;
	capidrv_ncci *p;

	if ((plcip = find_plci_by_ncci(card, ncci)) == 0)
		return 0;

	for (p = plcip->ncci_list; p; p = p->next)
		if (p->msgid == msgid)
			return p;
	return 0;
}

static void free_ncci(capidrv_contr * card, struct capidrv_ncci *nccip)
{
	struct capidrv_ncci **pp;

	for (pp = &(nccip->plcip->ncci_list); *pp; pp = &(*pp)->next) {
		if (*pp == nccip) {
			*pp = (*pp)->next;
			break;
		}
	}
	card->bchans[nccip->chan].nccip = 0;
	kfree(nccip);
}

/* -------- convert and send capi message ---------------------------- */

static void send_message(capidrv_contr * card, _cmsg * cmsg)
{
	struct sk_buff *skb;
	size_t len;
	capi_cmsg2message(cmsg, cmsg->buf);
	len = CAPIMSG_LEN(cmsg->buf);
	skb = dev_alloc_skb(len);
        SET_SKB_FREE(skb);
	memcpy(skb_put(skb, len), cmsg->buf, len);
	(*capifuncs->capi_put_message) (global.appid, skb);
}

/* -------- state machine -------------------------------------------- */

struct listenstatechange {
	int actstate;
	int nextstate;
	int event;
};

static struct listenstatechange listentable[] =
{
	{ST_LISTEN_NONE, ST_LISTEN_WAIT_CONF, EV_LISTEN_REQ},
	{ST_LISTEN_ACTIVE, ST_LISTEN_ACTIVE_WAIT_CONF, EV_LISTEN_REQ},
	{ST_LISTEN_WAIT_CONF, ST_LISTEN_NONE, EV_LISTEN_CONF_ERROR},
    {ST_LISTEN_ACTIVE_WAIT_CONF, ST_LISTEN_ACTIVE, EV_LISTEN_CONF_ERROR},
	{ST_LISTEN_WAIT_CONF, ST_LISTEN_NONE, EV_LISTEN_CONF_EMPTY},
      {ST_LISTEN_ACTIVE_WAIT_CONF, ST_LISTEN_NONE, EV_LISTEN_CONF_EMPTY},
	{ST_LISTEN_WAIT_CONF, ST_LISTEN_ACTIVE, EV_LISTEN_CONF_OK},
	{ST_LISTEN_ACTIVE_WAIT_CONF, ST_LISTEN_ACTIVE, EV_LISTEN_CONF_OK},
	{},
};

static void listen_change_state(capidrv_contr * card, int event)
{
	struct listenstatechange *p = listentable;
	while (p->event) {
		if (card->state == p->actstate && p->event == event) {
			if (debugmode)
				printk(KERN_DEBUG "capidrv: listen_change_state %d -> %d\n",
				       card->state, p->nextstate);
			card->state = p->nextstate;
			return;
		}
		p++;
	}
	printk(KERN_ERR "capidrv: listen_change_state state=%d event=%d ????\n",
	       card->state, event);

}

/* ------------------------------------------------------------------ */

static void p0(capidrv_contr * card, capidrv_plci * plci)
{
	isdn_ctrl cmd;

	card->bchans[plci->chan].contr = 0;
	cmd.command = ISDN_STAT_DHUP;
	cmd.driver = card->myid;
	cmd.arg = plci->chan;
	card->interface.statcallb(&cmd);
	free_plci(card, plci);
}

/* ------------------------------------------------------------------ */

struct plcistatechange {
	int actstate;
	int nextstate;
	int event;
	void (*changefunc) (capidrv_contr * card, capidrv_plci * plci);
};

static struct plcistatechange plcitable[] =
{
  /* P-0 */
	{ST_PLCI_NONE, ST_PLCI_OUTGOING, EV_PLCI_CONNECT_REQ, 0},
	{ST_PLCI_NONE, ST_PLCI_ALLOCATED, EV_PLCI_FACILITY_IND_UP, 0},
	{ST_PLCI_NONE, ST_PLCI_INCOMING, EV_PLCI_CONNECT_IND, 0},
  /* P-0.1 */
	{ST_PLCI_OUTGOING, ST_PLCI_NONE, EV_PLCI_CONNECT_CONF_ERROR, p0},
	{ST_PLCI_OUTGOING, ST_PLCI_ALLOCATED, EV_PLCI_CONNECT_CONF_OK, 0},
    {ST_PLCI_OUTGOING, ST_PLCI_DISCONNECTING, EV_PLCI_DISCONNECT_REQ, 0},
 {ST_PLCI_OUTGOING, ST_PLCI_DISCONNECTING, EV_PLCI_FACILITY_IND_DOWN, 0},
  /* P-1 */
      {ST_PLCI_ALLOCATED, ST_PLCI_ACTIVE, EV_PLCI_CONNECT_ACTIVE_IND, 0},
   {ST_PLCI_ALLOCATED, ST_PLCI_DISCONNECTING, EV_PLCI_DISCONNECT_REQ, 0},
{ST_PLCI_ALLOCATED, ST_PLCI_DISCONNECTING, EV_PLCI_FACILITY_IND_DOWN, 0},
    {ST_PLCI_ALLOCATED, ST_PLCI_DISCONNECTED, EV_PLCI_DISCONNECT_IND, 0},
  /* P-ACT */
      {ST_PLCI_ACTIVE, ST_PLCI_DISCONNECTING, EV_PLCI_DISCONNECT_REQ, 0},
   {ST_PLCI_ACTIVE, ST_PLCI_DISCONNECTING, EV_PLCI_FACILITY_IND_DOWN, 0},
	{ST_PLCI_ACTIVE, ST_PLCI_DISCONNECTED, EV_PLCI_DISCONNECT_IND, 0},
  /* P-2 */
    {ST_PLCI_INCOMING, ST_PLCI_DISCONNECTING, EV_PLCI_CONNECT_REJECT, 0},
    {ST_PLCI_INCOMING, ST_PLCI_FACILITY_IND, EV_PLCI_FACILITY_IND_UP, 0},
	{ST_PLCI_INCOMING, ST_PLCI_ACCEPTING, EV_PLCI_CONNECT_RESP, 0},
    {ST_PLCI_INCOMING, ST_PLCI_DISCONNECTING, EV_PLCI_DISCONNECT_REQ, 0},
 {ST_PLCI_INCOMING, ST_PLCI_DISCONNECTING, EV_PLCI_FACILITY_IND_DOWN, 0},
     {ST_PLCI_INCOMING, ST_PLCI_DISCONNECTED, EV_PLCI_DISCONNECT_IND, 0},
  /* P-3 */
{ST_PLCI_FACILITY_IND, ST_PLCI_DISCONNECTING, EV_PLCI_CONNECT_REJECT, 0},
{ST_PLCI_FACILITY_IND, ST_PLCI_ACCEPTING, EV_PLCI_CONNECT_ACTIVE_IND, 0},
{ST_PLCI_FACILITY_IND, ST_PLCI_DISCONNECTING, EV_PLCI_DISCONNECT_REQ, 0},
	{ST_PLCI_FACILITY_IND, ST_PLCI_DISCONNECTING, EV_PLCI_FACILITY_IND_DOWN, 0},
 {ST_PLCI_FACILITY_IND, ST_PLCI_DISCONNECTED, EV_PLCI_DISCONNECT_IND, 0},
  /* P-4 */
      {ST_PLCI_ACCEPTING, ST_PLCI_ACTIVE, EV_PLCI_CONNECT_ACTIVE_IND, 0},
   {ST_PLCI_ACCEPTING, ST_PLCI_DISCONNECTING, EV_PLCI_DISCONNECT_REQ, 0},
{ST_PLCI_ACCEPTING, ST_PLCI_DISCONNECTING, EV_PLCI_FACILITY_IND_DOWN, 0},
    {ST_PLCI_ACCEPTING, ST_PLCI_DISCONNECTED, EV_PLCI_DISCONNECT_IND, 0},
  /* P-5 */
{ST_PLCI_DISCONNECTING, ST_PLCI_DISCONNECTED, EV_PLCI_DISCONNECT_IND, 0},
  /* P-6 */
	{ST_PLCI_DISCONNECTED, ST_PLCI_NONE, EV_PLCI_DISCONNECT_RESP, p0},
	{},
};

static void plci_change_state(capidrv_contr * card, capidrv_plci * plci, int event)
{
	struct plcistatechange *p = plcitable;
	while (p->event) {
		if (plci->state == p->actstate && p->event == event) {
			if (debugmode)
				printk(KERN_DEBUG "capidrv: plci_change_state:0x%x %d -> %d\n",
				  plci->plci, plci->state, p->nextstate);
			plci->state = p->nextstate;
			if (p->changefunc)
				p->changefunc(card, plci);
			return;
		}
		p++;
	}
	printk(KERN_ERR "capidrv: plci_change_state:0x%x state=%d event=%d ????\n",
	       plci->plci, plci->state, event);
}

/* ------------------------------------------------------------------ */

static _cmsg cmsg;

static void n0(capidrv_contr * card, capidrv_ncci * ncci)
{
	isdn_ctrl cmd;

	capi_fill_DISCONNECT_REQ(&cmsg,
				 global.appid,
				 card->msgid++,
				 ncci->plcip->plci,
				 0,	/* BChannelinformation */
				 0,	/* Keypadfacility */
				 0,	/* Useruserdata */
				 0	/* Facilitydataarray */
	);
	send_message(card, &cmsg);
	plci_change_state(card, ncci->plcip, EV_PLCI_DISCONNECT_REQ);

	cmd.command = ISDN_STAT_BHUP;
	cmd.driver = card->myid;
	cmd.arg = ncci->chan;
	card->interface.statcallb(&cmd);
	free_ncci(card, ncci);
}

/* ------------------------------------------------------------------ */

struct nccistatechange {
	int actstate;
	int nextstate;
	int event;
	void (*changefunc) (capidrv_contr * card, capidrv_ncci * ncci);
};

static struct nccistatechange nccitable[] =
{
  /* N-0 */
	{ST_NCCI_NONE, ST_NCCI_OUTGOING, EV_NCCI_CONNECT_B3_REQ, 0},
	{ST_NCCI_NONE, ST_NCCI_INCOMING, EV_NCCI_CONNECT_B3_IND, 0},
  /* N-0.1 */
    {ST_NCCI_OUTGOING, ST_NCCI_ALLOCATED, EV_NCCI_CONNECT_B3_CONF_OK, 0},
      {ST_NCCI_OUTGOING, ST_NCCI_NONE, EV_NCCI_CONNECT_B3_CONF_ERROR, 0},
  /* N-1 */
 {ST_NCCI_INCOMING, ST_NCCI_DISCONNECTING, EV_NCCI_CONNECT_B3_REJECT, 0},
	{ST_NCCI_INCOMING, ST_NCCI_ALLOCATED, EV_NCCI_CONNECT_B3_RESP, 0},
  {ST_NCCI_INCOMING, ST_NCCI_DISCONNECTED, EV_NCCI_DISCONNECT_B3_IND, 0},
 {ST_NCCI_INCOMING, ST_NCCI_DISCONNECTING, EV_NCCI_DISCONNECT_B3_REQ, 0},
  /* N-2 */
   {ST_NCCI_ALLOCATED, ST_NCCI_ACTIVE, EV_NCCI_CONNECT_B3_ACTIVE_IND, 0},
 {ST_NCCI_ALLOCATED, ST_NCCI_DISCONNECTED, EV_NCCI_DISCONNECT_B3_IND, 0},
{ST_NCCI_ALLOCATED, ST_NCCI_DISCONNECTING, EV_NCCI_DISCONNECT_B3_REQ, 0},
  /* N-ACT */
	{ST_NCCI_ACTIVE, ST_NCCI_RESETING, EV_NCCI_RESET_B3_REQ, 0},
    {ST_NCCI_ACTIVE, ST_NCCI_DISCONNECTED, EV_NCCI_DISCONNECT_B3_IND, 0},
   {ST_NCCI_ACTIVE, ST_NCCI_DISCONNECTING, EV_NCCI_DISCONNECT_B3_REQ, 0},
  /* N-3 */
	{ST_NCCI_RESETING, ST_NCCI_ACTIVE, EV_NCCI_RESET_B3_IND, 0},
  {ST_NCCI_RESETING, ST_NCCI_DISCONNECTED, EV_NCCI_DISCONNECT_B3_IND, 0},
 {ST_NCCI_RESETING, ST_NCCI_DISCONNECTING, EV_NCCI_DISCONNECT_B3_REQ, 0},
  /* N-4 */
	{ST_NCCI_DISCONNECTING, ST_NCCI_DISCONNECTED, EV_NCCI_DISCONNECT_B3_IND, 0},
	{ST_NCCI_DISCONNECTING, ST_NCCI_PREVIOUS, EV_NCCI_DISCONNECT_B3_CONF_ERROR, 0},
  /* N-5 */
    {ST_NCCI_DISCONNECTED, ST_NCCI_NONE, EV_NCCI_DISCONNECT_B3_RESP, n0},
	{},
};

static void ncci_change_state(capidrv_contr * card, capidrv_ncci * ncci, int event)
{
	struct nccistatechange *p = nccitable;
	while (p->event) {
		if (ncci->state == p->actstate && p->event == event) {
			if (debugmode)
				printk(KERN_DEBUG "capidrv: ncci_change_state:0x%x %d -> %d\n",
				  ncci->ncci, ncci->state, p->nextstate);
			if (p->nextstate == ST_NCCI_PREVIOUS) {
				ncci->state = ncci->oldstate;
				ncci->oldstate = p->actstate;
			} else {
				ncci->oldstate = p->actstate;
				ncci->state = p->nextstate;
			}
			if (p->changefunc)
				p->changefunc(card, ncci);
			return;
		}
		p++;
	}
	printk(KERN_ERR "capidrv: ncci_change_state:0x%x state=%d event=%d ????\n",
	       ncci->ncci, ncci->state, event);
}

/* ------------------------------------------------------------------- */

static inline int new_bchan(capidrv_contr * card)
{
	int i;
	for (i = 0; i < card->nbchan; i++) {
		if (card->bchans[i].plcip == 0) {
			card->bchans[i].disconnecting = 0;
			return i;
		}
	}
	return -1;
}

/* ------------------------------------------------------------------- */

static void handle_controller(_cmsg * cmsg)
{
	capidrv_contr *card = findcontrbynumber(cmsg->adr.adrController & 0x7f);

	if (!card) {
		printk(KERN_ERR "capidrv: %s from unknown controller 0x%x\n",
		       capi_cmd2str(cmsg->Command, cmsg->Subcommand),
		       cmsg->adr.adrController & 0x7f);
		return;
	}
	switch (CAPICMD(cmsg->Command, cmsg->Subcommand)) {

	case CAPI_LISTEN_CONF:	/* Controller */
		if (debugmode)
			printk(KERN_DEBUG "capidrv: listenconf Info=0x%4x (%s) cipmask=0x%x\n",
			       cmsg->Info, capi_info2str(cmsg->Info), card->cipmask);
		if (cmsg->Info) {
			listen_change_state(card, EV_LISTEN_CONF_ERROR);
		} else if (card->cipmask == 0) {
			listen_change_state(card, EV_LISTEN_CONF_EMPTY);
		} else {
			listen_change_state(card, EV_LISTEN_CONF_OK);
		}
		break;

	case CAPI_MANUFACTURER_IND:	/* Controller */
		goto ignored;
	case CAPI_MANUFACTURER_CONF:	/* Controller */
		goto ignored;
	case CAPI_FACILITY_IND:	/* Controller/plci/ncci */
		goto ignored;
	case CAPI_FACILITY_CONF:	/* Controller/plci/ncci */
		goto ignored;
	case CAPI_INFO_IND:	/* Controller/plci */
		goto ignored;
	case CAPI_INFO_CONF:	/* Controller/plci */
		goto ignored;

	default:
		printk(KERN_ERR "capidrv: got %s from controller 0x%x ???",
		       capi_cmd2str(cmsg->Command, cmsg->Subcommand),
		       cmsg->adr.adrController);
	}
	return;

      ignored:
	printk(KERN_INFO "capidrv: %s from controller 0x%x ignored\n",
	       capi_cmd2str(cmsg->Command, cmsg->Subcommand),
	       cmsg->adr.adrController);
}

static void handle_incoming_call(capidrv_contr * card, _cmsg * cmsg)
{
	capidrv_plci *plcip;
	capidrv_bchan *bchan;
	isdn_ctrl cmd;
	int chan;

	if ((chan = new_bchan(card)) == -1) {
		printk(KERN_ERR "capidrv: incoming call on not existing bchan ?\n");
		return;
	}
	bchan = &card->bchans[chan];
	if ((plcip = new_plci(card, chan)) == 0) {
		printk(KERN_ERR "capidrv: incoming call: no memory, sorry.\n");
		return;
	}
	bchan->incoming = 1;
	plcip->plci = cmsg->adr.adrPLCI;
	plci_change_state(card, plcip, EV_PLCI_CONNECT_IND);

	cmd.command = ISDN_STAT_ICALL;
	cmd.driver = card->myid;
	cmd.arg = chan;
	memset(&cmd.parm.setup, 0, sizeof(cmd.parm.setup));
	strncpy(cmd.parm.setup.phone,
	        cmsg->CallingPartyNumber + 3,
		cmsg->CallingPartyNumber[0] - 2);
	strncpy(cmd.parm.setup.eazmsn,
	        cmsg->CalledPartyNumber + 2,
		cmsg->CalledPartyNumber[0] - 1);
	cmd.parm.setup.si1 = cip2si1(cmsg->CIPValue);
	cmd.parm.setup.si2 = cip2si2(cmsg->CIPValue);
	cmd.parm.setup.plan = cmsg->CallingPartyNumber[1];
	cmd.parm.setup.screen = cmsg->CallingPartyNumber[2];

	printk(KERN_INFO "capidrv: incoming call %s,%d,%d,%s\n", 
			cmd.parm.setup.phone,
			cmd.parm.setup.si1,
			cmd.parm.setup.si2,
			cmd.parm.setup.eazmsn);

	switch (card->interface.statcallb(&cmd)) {
	case 0:
		/* No device matching this call.
		 * and isdn_common.c has send a HANGUP command
		 * which is ignored in state ST_PLCI_INCOMING,
		 * so we send RESP to ignore the call
		 */
		capi_cmsg_answer(cmsg);
		cmsg->Reject = 1;	/* ignore */
		send_message(card, cmsg);
		plci_change_state(card, plcip, EV_PLCI_CONNECT_REJECT);
		printk(KERN_INFO "capidrv: incoming call %s,%d,%d,%s ignored\n",
			cmd.parm.setup.phone,
			cmd.parm.setup.si1,
			cmd.parm.setup.si2,
			cmd.parm.setup.eazmsn);
		break;
	case 1:
		/* At least one device matching this call (RING on ttyI)
		 * HL-driver may send ALERTING on the D-channel in this
		 * case.
		 * really means: RING on ttyI or a net interface
		 * accepted this call already.
		 *
		 * If the call was accepted, state has already changed,
		 * and CONNECT_RESP already sent.
		 */
		if (plcip->state == ST_PLCI_INCOMING) {
			printk(KERN_INFO "capidrv: incoming call %s,%d,%d,%s tty alerting\n",
				cmd.parm.setup.phone,
				cmd.parm.setup.si1,
				cmd.parm.setup.si2,
				cmd.parm.setup.eazmsn);
			capi_fill_ALERT_REQ(cmsg,
					    global.appid,
					    card->msgid++,
					    plcip->plci,	/* adr */
					    0,	/* BChannelinformation */
					    0,	/* Keypadfacility */
					    0,	/* Useruserdata */
					    0	/* Facilitydataarray */
			);
			plcip->msgid = cmsg->Messagenumber;
			send_message(card, cmsg);
		} else {
			printk(KERN_INFO "capidrv: incoming call %s,%d,%d,%s on netdev\n",
				cmd.parm.setup.phone,
				cmd.parm.setup.si1,
				cmd.parm.setup.si2,
				cmd.parm.setup.eazmsn);
		}
		break;

	case 2:		/* Call will be rejected. */
		capi_cmsg_answer(cmsg);
		cmsg->Reject = 2;	/* reject call, normal call clearing */
		send_message(card, cmsg);
		plci_change_state(card, plcip, EV_PLCI_CONNECT_REJECT);
		break;

	default:
		/* An error happened. (Invalid parameters for example.) */
		capi_cmsg_answer(cmsg);
		cmsg->Reject = 8;	/* reject call,
					   destination out of order */
		send_message(card, cmsg);
		plci_change_state(card, plcip, EV_PLCI_CONNECT_REJECT);
		break;
	}
	return;
}

static void handle_plci(_cmsg * cmsg)
{
	capidrv_contr *card = findcontrbynumber(cmsg->adr.adrController & 0x7f);
	capidrv_plci *plcip;
	isdn_ctrl cmd;

	if (!card) {
		printk(KERN_ERR "capidrv: %s from unknown controller 0x%x\n",
		       capi_cmd2str(cmsg->Command, cmsg->Subcommand),
		       cmsg->adr.adrController & 0x7f);
		return;
	}
	switch (CAPICMD(cmsg->Command, cmsg->Subcommand)) {

	case CAPI_DISCONNECT_IND:	/* plci */
		if (cmsg->Reason) {
			printk(KERN_INFO "capidrv: %s reason 0x%x (%s) for plci 0x%x\n",
			   capi_cmd2str(cmsg->Command, cmsg->Subcommand),
			       cmsg->Reason, capi_info2str(cmsg->Reason), cmsg->adr.adrPLCI);
		}
		if (!(plcip = find_plci_by_plci(card, cmsg->adr.adrPLCI))) {
			capi_cmsg_answer(cmsg);
			send_message(card, cmsg);
			goto notfound;
		}
		card->bchans[plcip->chan].disconnecting = 1;
		plci_change_state(card, plcip, EV_PLCI_DISCONNECT_IND);
		capi_cmsg_answer(cmsg);
		send_message(card, cmsg);
		plci_change_state(card, plcip, EV_PLCI_DISCONNECT_RESP);
		break;

	case CAPI_DISCONNECT_CONF:	/* plci */
		if (cmsg->Info) {
			printk(KERN_INFO "capidrv: %s info 0x%x (%s) for plci 0x%x\n",
			   capi_cmd2str(cmsg->Command, cmsg->Subcommand),
			       cmsg->Info, capi_info2str(cmsg->Info), 
			       cmsg->adr.adrPLCI);
		}
		if (!(plcip = find_plci_by_plci(card, cmsg->adr.adrPLCI)))
			goto notfound;

		card->bchans[plcip->chan].disconnecting = 1;
		break;

	case CAPI_ALERT_CONF:	/* plci */
		if (cmsg->Info) {
			printk(KERN_INFO "capidrv: %s info 0x%x (%s) for plci 0x%x\n",
			   capi_cmd2str(cmsg->Command, cmsg->Subcommand),
			       cmsg->Info, capi_info2str(cmsg->Info), 
			       cmsg->adr.adrPLCI);
		}
		break;

	case CAPI_CONNECT_IND:	/* plci */
		handle_incoming_call(card, cmsg);
		break;

	case CAPI_CONNECT_CONF:	/* plci */
		if (cmsg->Info) {
			printk(KERN_INFO "capidrv: %s info 0x%x (%s) for plci 0x%x\n",
			   capi_cmd2str(cmsg->Command, cmsg->Subcommand),
			       cmsg->Info, capi_info2str(cmsg->Info), 
			       cmsg->adr.adrPLCI);
		}
		if (!(plcip = find_plci_by_msgid(card, cmsg->Messagenumber)))
			goto notfound;

		plcip->plci = cmsg->adr.adrPLCI;
		if (cmsg->Info) {
			plci_change_state(card, plcip, EV_PLCI_CONNECT_CONF_ERROR);
		} else {
			plci_change_state(card, plcip, EV_PLCI_CONNECT_CONF_OK);
		}
		break;

	case CAPI_CONNECT_ACTIVE_IND:	/* plci */

		if (!(plcip = find_plci_by_plci(card, cmsg->adr.adrPLCI)))
			goto notfound;

		if (card->bchans[plcip->chan].incoming) {
			capi_cmsg_answer(cmsg);
			send_message(card, cmsg);
			plci_change_state(card, plcip, EV_PLCI_CONNECT_ACTIVE_IND);
		} else {
			capidrv_ncci *nccip;
			capi_cmsg_answer(cmsg);
			send_message(card, cmsg);

			nccip = new_ncci(card, plcip, cmsg->adr.adrPLCI);

			if (!nccip) {
				printk(KERN_ERR "capidrv: no mem for ncci, sorry\n");
				break;	/* $$$$ */
			}
			capi_fill_CONNECT_B3_REQ(cmsg,
						 global.appid,
						 card->msgid++,
						 plcip->plci,	/* adr */
						 0	/* NCPI */
			);
			nccip->msgid = cmsg->Messagenumber;
			send_message(card, cmsg);
			cmd.command = ISDN_STAT_DCONN;
			cmd.driver = card->myid;
			cmd.arg = plcip->chan;
			card->interface.statcallb(&cmd);
			plci_change_state(card, plcip, EV_PLCI_CONNECT_ACTIVE_IND);
			ncci_change_state(card, nccip, EV_NCCI_CONNECT_B3_REQ);
		}
		break;

	case CAPI_INFO_IND:	/* Controller/plci */

		if (!(plcip = find_plci_by_plci(card, cmsg->adr.adrPLCI)))
			goto notfound;

		if (cmsg->InfoNumber == 0x4000) {
			if (cmsg->InfoElement[0] == 4) {
				cmd.command = ISDN_STAT_CINF;
				cmd.driver = card->myid;
				cmd.arg = plcip->chan;
				sprintf(cmd.parm.num, "%lu",
					(unsigned long)
					((__u32) cmsg->InfoElement[1]
				  | ((__u32) (cmsg->InfoElement[2]) << 8)
				 | ((__u32) (cmsg->InfoElement[3]) << 16)
					 | ((__u32) (cmsg->InfoElement[4]) << 24)));
				card->interface.statcallb(&cmd);
				break;
			}
		}
		printk(KERN_ERR "capidrv: %s\n", capi_cmsg2str(cmsg));
		break;

	case CAPI_CONNECT_ACTIVE_CONF:		/* plci */
		goto ignored;
	case CAPI_SELECT_B_PROTOCOL_CONF:	/* plci */
		goto ignored;
	case CAPI_FACILITY_IND:	/* Controller/plci/ncci */
		goto ignored;
	case CAPI_FACILITY_CONF:	/* Controller/plci/ncci */
		goto ignored;

	case CAPI_INFO_CONF:	/* Controller/plci */
		goto ignored;

	default:
		printk(KERN_ERR "capidrv: got %s for plci 0x%x ???",
		       capi_cmd2str(cmsg->Command, cmsg->Subcommand),
		       cmsg->adr.adrPLCI);
	}
	return;
      ignored:
	printk(KERN_INFO "capidrv: %s for plci 0x%x ignored\n",
	       capi_cmd2str(cmsg->Command, cmsg->Subcommand),
	       cmsg->adr.adrPLCI);
	return;
      notfound:
	printk(KERN_ERR "capidrv: %s: plci 0x%x not found\n",
	       capi_cmd2str(cmsg->Command, cmsg->Subcommand),
	       cmsg->adr.adrPLCI);
	return;
}

static void handle_ncci(_cmsg * cmsg)
{
	capidrv_contr *card = findcontrbynumber(cmsg->adr.adrController & 0x7f);
	capidrv_plci *plcip;
	capidrv_ncci *nccip;
	isdn_ctrl cmd;

	if (!card) {
		printk(KERN_ERR "capidrv: %s from unknown controller 0x%x\n",
		       capi_cmd2str(cmsg->Command, cmsg->Subcommand),
		       cmsg->adr.adrController & 0x7f);
		return;
	}
	switch (CAPICMD(cmsg->Command, cmsg->Subcommand)) {

	case CAPI_CONNECT_B3_ACTIVE_IND:	/* ncci */
		if (!(nccip = find_ncci(card, cmsg->adr.adrNCCI)))
			goto notfound;

		capi_cmsg_answer(cmsg);
		send_message(card, cmsg);
		ncci_change_state(card, nccip, EV_NCCI_CONNECT_B3_ACTIVE_IND);

		cmd.command = ISDN_STAT_BCONN;
		cmd.driver = card->myid;
		cmd.arg = nccip->chan;
		card->interface.statcallb(&cmd);

		printk(KERN_INFO "capidrv: chan %d up with ncci 0x%x\n",
		       nccip->chan, nccip->ncci);
		break;

	case CAPI_CONNECT_B3_ACTIVE_CONF:	/* ncci */
		goto ignored;

	case CAPI_CONNECT_B3_IND:	/* ncci */

		plcip = find_plci_by_ncci(card, cmsg->adr.adrNCCI);
		if (plcip) {
			nccip = new_ncci(card, plcip, cmsg->adr.adrNCCI);
			if (nccip) {
				ncci_change_state(card, nccip, EV_NCCI_CONNECT_B3_IND);
				capi_fill_CONNECT_B3_RESP(cmsg,
							  global.appid,
							  card->msgid++,
							  nccip->ncci,	/* adr */
							  0,	/* Reject */
							  0	/* NCPI */
				);
				send_message(card, cmsg);
				ncci_change_state(card, nccip, EV_NCCI_CONNECT_B3_RESP);
				break;
			}
			printk(KERN_ERR "capidrv: no mem for ncci, sorry\n");
		} else {
			printk(KERN_ERR "capidrv: %s: plci for ncci 0x%x not found\n",
			   capi_cmd2str(cmsg->Command, cmsg->Subcommand),
			       cmsg->adr.adrNCCI);
		}
		capi_fill_CONNECT_B3_RESP(cmsg,
					  global.appid,
					  card->msgid++,
					  cmsg->adr.adrNCCI,
					  2,	/* Reject */
					  0	/* NCPI */
		);
		send_message(card, cmsg);
		break;

	case CAPI_CONNECT_B3_CONF:	/* ncci */

		if (!(nccip = find_ncci_by_msgid(card,
						 cmsg->adr.adrNCCI,
						 cmsg->Messagenumber)))
			goto notfound;

		nccip->ncci = cmsg->adr.adrNCCI;
		if (cmsg->Info) {
			printk(KERN_INFO "capidrv: %s info 0x%x (%s) for ncci 0x%x\n",
			   capi_cmd2str(cmsg->Command, cmsg->Subcommand),
			       cmsg->Info, capi_info2str(cmsg->Info), 
			       cmsg->adr.adrNCCI);
		}

		if (cmsg->Info)
			ncci_change_state(card, nccip, EV_NCCI_CONNECT_B3_CONF_ERROR);
		else
			ncci_change_state(card, nccip, EV_NCCI_CONNECT_B3_CONF_OK);
		break;

	case CAPI_CONNECT_B3_T90_ACTIVE_IND:	/* ncci */
		capi_cmsg_answer(cmsg);
		send_message(card, cmsg);
		break;

	case CAPI_DATA_B3_IND:	/* ncci */
		/* handled in handle_data() */
		goto ignored;

	case CAPI_DATA_B3_CONF:	/* ncci */
		if (!(nccip = find_ncci(card, cmsg->adr.adrNCCI)))
			goto notfound;

		cmd.command = ISDN_STAT_BSENT;
		cmd.driver = card->myid;
		cmd.arg = nccip->chan;
		card->interface.statcallb(&cmd);

		break;

	case CAPI_DISCONNECT_B3_IND:	/* ncci */
		if (!(nccip = find_ncci(card, cmsg->adr.adrNCCI)))
			goto notfound;

		card->bchans[nccip->chan].disconnecting = 1;
		ncci_change_state(card, nccip, EV_NCCI_DISCONNECT_B3_IND);
		capi_cmsg_answer(cmsg);
		send_message(card, cmsg);
		ncci_change_state(card, nccip, EV_NCCI_DISCONNECT_B3_RESP);
		break;

	case CAPI_DISCONNECT_B3_CONF:	/* ncci */
		if (!(nccip = find_ncci(card, cmsg->adr.adrNCCI)))
			goto notfound;
		if (cmsg->Info) {
			printk(KERN_INFO "capidrv: %s info 0x%x (%s) for ncci 0x%x\n",
			   capi_cmd2str(cmsg->Command, cmsg->Subcommand),
			       cmsg->Info, capi_info2str(cmsg->Info), 
			       cmsg->adr.adrNCCI);
			ncci_change_state(card, nccip, EV_NCCI_DISCONNECT_B3_CONF_ERROR);
		}
		break;

	case CAPI_RESET_B3_IND:	/* ncci */
		capi_cmsg_answer(cmsg);
		send_message(card, cmsg);
		break;

	case CAPI_RESET_B3_CONF:	/* ncci */
		goto ignored;	/* $$$$ */

	case CAPI_FACILITY_IND:	/* Controller/plci/ncci */
		goto ignored;
	case CAPI_FACILITY_CONF:	/* Controller/plci/ncci */
		goto ignored;

	default:
		printk(KERN_ERR "capidrv: got %s for ncci 0x%x ???",
		       capi_cmd2str(cmsg->Command, cmsg->Subcommand),
		       cmsg->adr.adrNCCI);
	}
	return;
      ignored:
	printk(KERN_INFO "capidrv: %s for ncci 0x%x ignored\n",
	       capi_cmd2str(cmsg->Command, cmsg->Subcommand),
	       cmsg->adr.adrNCCI);
	return;
      notfound:
	printk(KERN_ERR "capidrv: %s: ncci 0x%x not found\n",
	       capi_cmd2str(cmsg->Command, cmsg->Subcommand),
	       cmsg->adr.adrNCCI);
}


static void handle_data(_cmsg * cmsg, struct sk_buff *skb)
{
	capidrv_contr *card = findcontrbynumber(cmsg->adr.adrController & 0x7f);
	capidrv_ncci *nccip;

	if (!card) {
		printk(KERN_ERR "capidrv: %s from unknown controller 0x%x\n",
		       capi_cmd2str(cmsg->Command, cmsg->Subcommand),
		       cmsg->adr.adrController & 0x7f);
		return;
	}
	if (!(nccip = find_ncci(card, cmsg->adr.adrNCCI))) {
		printk(KERN_ERR "capidrv: %s: ncci 0x%x not found\n",
		       capi_cmd2str(cmsg->Command, cmsg->Subcommand),
		       cmsg->adr.adrNCCI);
		kfree_skb(skb, FREE_READ);
		return;
	}
	(void) skb_pull(skb, CAPIMSG_LEN(skb->data));
	card->interface.rcvcallb_skb(card->myid, nccip->chan, skb);
	capi_cmsg_answer(cmsg);
	send_message(card, cmsg);
}

static _cmsg s_cmsg;

static void capidrv_signal(__u16 applid, __u32 dummy)
{
	struct sk_buff *skb = 0;

	while ((*capifuncs->capi_get_message) (global.appid, &skb) == CAPI_NOERROR) {
		capi_message2cmsg(&s_cmsg, skb->data);
		if (debugmode > 1)
			printk(KERN_DEBUG "capidrv_signal: %s\n", capi_cmsg2str(&s_cmsg));

		if (s_cmsg.Command == CAPI_DATA_B3
		    && s_cmsg.Subcommand == CAPI_IND) {
			handle_data(&s_cmsg, skb);
			continue;
		}
		kfree_skb(skb, FREE_READ);
		if ((s_cmsg.adr.adrController & 0xffffff00) == 0)
			handle_controller(&s_cmsg);
		else if ((s_cmsg.adr.adrPLCI & 0xffff0000) == 0)
			handle_plci(&s_cmsg);
		else
			handle_ncci(&s_cmsg);
	}
}

/* ------------------------------------------------------------------- */

static _cmsg cmdcmsg;

static int capidrv_ioctl(isdn_ctrl * c, capidrv_contr * card)
{
	switch (c->arg) {
	default:
		printk(KERN_DEBUG "capidrv: capidrv_ioctl(%ld) called ??\n", c->arg);
		return -EINVAL;
	}
	return -EINVAL;
}

static int capidrv_command(isdn_ctrl * c, capidrv_contr * card)
{
	isdn_ctrl cmd;
	struct capidrv_bchan *bchan;
	struct capidrv_plci *plcip;

	if (c->command == ISDN_CMD_IOCTL)
		return capidrv_ioctl(c, card);

	switch (c->command) {
	case ISDN_CMD_DIAL:{
			__u8 calling[ISDN_MSNLEN + 3];
			__u8 called[ISDN_MSNLEN + 2];

			if (debugmode)
				printk(KERN_DEBUG "capidrv: ISDN_CMD_DIAL(ch=%ld,\"%s,%d,%d,%s\")\n",
					c->arg,
				        c->parm.setup.phone,
				        c->parm.setup.si1,
				        c->parm.setup.si2,
				        c->parm.setup.eazmsn);

			bchan = &card->bchans[c->arg % card->nbchan];

			if (bchan->plcip) {
				printk(KERN_ERR "capidrv: dail ch=%ld,\"%s,%d,%d,%s\" in use (plci=0x%x)\n",
			        	c->arg, 
				        c->parm.setup.phone,
				        c->parm.setup.si1,
				        c->parm.setup.si2,
				        c->parm.setup.eazmsn,
				        bchan->plcip->plci);
				return 0;
			}
			bchan->si1 = c->parm.setup.si1;
			bchan->si2 = c->parm.setup.si2;

			strncpy(bchan->num, c->parm.setup.phone, sizeof(bchan->num));
			strncpy(bchan->mynum, c->parm.setup.eazmsn, sizeof(bchan->mynum));

			calling[0] = strlen(bchan->mynum) + 2;
			calling[1] = 0;
			calling[2] = 0x80;
			strncpy(calling + 3, bchan->mynum, ISDN_MSNLEN);

			called[0] = strlen(bchan->num) + 1;
			called[1] = 0x80;
			strncpy(called + 2, bchan->num, ISDN_MSNLEN);

			capi_fill_CONNECT_REQ(&cmdcmsg,
					      global.appid,
					      card->msgid++,
					      card->contrnr,	/* adr */
					  si2cip(bchan->si1, bchan->si2),	/* cipvalue */
					      called,	/* CalledPartyNumber */
					      calling,	/* CallingPartyNumber */
					      0,	/* CalledPartySubaddress */
					      0,	/* CallingPartySubaddress */
					    b1prot(bchan->l2, bchan->l3),	/* B1protocol */
					    b2prot(bchan->l2, bchan->l3),	/* B2protocol */
					    b3prot(bchan->l2, bchan->l3),	/* B3protocol */
					      0,	/* B1configuration */
					      0,	/* B2configuration */
					      0,	/* B3configuration */
					      0,	/* BC */
					      0,	/* LLC */
					      0,	/* HLC */
					      0,	/* BChannelinformation */
					      0,	/* Keypadfacility */
					      0,	/* Useruserdata */
					      0		/* Facilitydataarray */
			    );
			if ((plcip = new_plci(card, (c->arg % card->nbchan))) == 0) {
				cmd.command = ISDN_STAT_DHUP;
				cmd.driver = card->myid;
				cmd.arg = (c->arg % card->nbchan);
				card->interface.statcallb(&cmd);
				return -1;
			}
			plcip->msgid = cmdcmsg.Messagenumber;
			plci_change_state(card, plcip, EV_PLCI_CONNECT_REQ);
			send_message(card, &cmdcmsg);
			return 0;
		}

	case ISDN_CMD_ACCEPTD:

		if (debugmode)
			printk(KERN_DEBUG "capidrv: ISDN_CMD_ACCEPTD(ch=%ld)\n",
			       c->arg);
		bchan = &card->bchans[c->arg % card->nbchan];

		capi_fill_CONNECT_RESP(&cmdcmsg,
				       global.appid,
				       card->msgid++,
				       bchan->plcip->plci,	/* adr */
				       0,	/* Reject */
				       b1prot(bchan->l2, bchan->l3),	/* B1protocol */
				       b2prot(bchan->l2, bchan->l3),	/* B2protocol */
				       b3prot(bchan->l2, bchan->l3),	/* B3protocol */
				       0,	/* B1configuration */
				       0,	/* B2configuration */
				       0,	/* B3configuration */
				       0,	/* ConnectedNumber */
				       0,	/* ConnectedSubaddress */
				       0,	/* LLC */
				       0,	/* BChannelinformation */
				       0,	/* Keypadfacility */
				       0,	/* Useruserdata */
				       0	/* Facilitydataarray */
		);
		capi_cmsg2message(&cmdcmsg, cmdcmsg.buf);
		plci_change_state(card, bchan->plcip, EV_PLCI_CONNECT_RESP);
		send_message(card, &cmdcmsg);
		return 0;

	case ISDN_CMD_ACCEPTB:
		if (debugmode)
			printk(KERN_DEBUG "capidrv: ISDN_CMD_ACCEPTB(ch=%ld)\n",
			       c->arg);
		return -ENOSYS;

	case ISDN_CMD_HANGUP:
		if (debugmode)
			printk(KERN_DEBUG "capidrv: ISDN_CMD_HANGUP(ch=%ld)\n",
			       c->arg);
		bchan = &card->bchans[c->arg % card->nbchan];

		if (bchan->disconnecting) {
			if (debugmode)
				printk(KERN_DEBUG "capidrv: chan %ld already disconnecting ...\n",
				       c->arg);
			return 0;
		}
		if (bchan->nccip) {
			bchan->disconnecting = 1;
			capi_fill_DISCONNECT_B3_REQ(&cmdcmsg,
						    global.appid,
						    card->msgid++,
						    bchan->nccip->ncci,
						    0	/* NCPI */
			);
			ncci_change_state(card, bchan->nccip, EV_NCCI_DISCONNECT_B3_REQ);
			send_message(card, &cmdcmsg);
		} else if (bchan->plcip) {
			bchan->disconnecting = 1;
			if (bchan->plcip->state == ST_PLCI_INCOMING) {
				/* just ignore, we a called from isdn_status_callback(),
				 * which will return 0 or 2, this is handled by the
				 * CONNECT_IND handler
				 */
			} else {
				capi_fill_DISCONNECT_REQ(&cmdcmsg,
							 global.appid,
							 card->msgid++,
						      bchan->plcip->plci,
							 0,	/* BChannelinformation */
							 0,	/* Keypadfacility */
							 0,	/* Useruserdata */
							 0	/* Facilitydataarray */
				);
				plci_change_state(card, bchan->plcip, EV_PLCI_DISCONNECT_REQ);
				send_message(card, &cmdcmsg);
			}
		}
/* ready */

	case ISDN_CMD_SETL2:
		if (debugmode)
			printk(KERN_DEBUG "capidrv: set L2 on chan %ld to %ld\n",
			       (c->arg & 0xff), (c->arg >> 8));
		bchan = &card->bchans[c->arg % card->nbchan];
		bchan->l2 = (c->arg >> 8);
		return 0;

	case ISDN_CMD_SETL3:
		if (debugmode)
			printk(KERN_DEBUG "capidrv: set L3 on chan %ld to %ld\n",
			       (c->arg & 0xff), (c->arg >> 8));
		bchan = &card->bchans[c->arg % card->nbchan];
		bchan->l3 = (c->arg >> 8);
		return 0;

	case ISDN_CMD_SETEAZ:
		if (debugmode)
			printk(KERN_DEBUG "capidrv: set EAZ \"%s\" on chan %ld\n",
			       c->parm.num, c->arg);
		bchan = &card->bchans[c->arg % card->nbchan];
		strncpy(bchan->msn, c->parm.num, ISDN_MSNLEN);
		return 0;

	case ISDN_CMD_CLREAZ:
		if (debugmode)
			printk(KERN_DEBUG "capidrv: clearing EAZ on chan %ld\n", c->arg);
		bchan = &card->bchans[c->arg % card->nbchan];
		bchan->msn[0] = 0;
		return 0;

	case ISDN_CMD_LOCK:
		if (debugmode > 1)
			printk(KERN_DEBUG "capidrv: ISDN_CMD_LOCK (%ld)\n", c->arg);
		MOD_INC_USE_COUNT;
		break;

	case ISDN_CMD_UNLOCK:
		if (debugmode > 1)
			printk(KERN_DEBUG "capidrv: ISDN_CMD_UNLOCK (%ld)\n", c->arg);
		MOD_DEC_USE_COUNT;
		break;

/* never called */
	case ISDN_CMD_GETL2:
		if (debugmode)
			printk(KERN_DEBUG "capidrv: ISDN_CMD_GETL2\n");
		return -ENODEV;
	case ISDN_CMD_GETL3:
		if (debugmode)
			printk(KERN_DEBUG "capidrv: ISDN_CMD_GETL3\n");
		return -ENODEV;
	case ISDN_CMD_GETEAZ:
		if (debugmode)
			printk(KERN_DEBUG "capidrv: ISDN_CMD_GETEAZ\n");
		return -ENODEV;
	case ISDN_CMD_SETSIL:
		if (debugmode)
			printk(KERN_DEBUG "capidrv: ISDN_CMD_SETSIL\n");
		return -ENODEV;
	case ISDN_CMD_GETSIL:
		if (debugmode)
			printk(KERN_DEBUG "capidrv: ISDN_CMD_GETSIL\n");
		return -ENODEV;
	default:
		printk(KERN_ERR "capidrv: ISDN_CMD_%d, Huh?\n", c->command);
		return -EINVAL;
	}
	return 0;
}

static int if_command(isdn_ctrl * c)
{
	capidrv_contr *card = findcontrbydriverid(c->driver);

	if (card)
		return capidrv_command(c, card);

	printk(KERN_ERR
	     "capidrv: if_command %d called with invalid driverId %d!\n",
	       c->command, c->driver);
	return -ENODEV;
}

static _cmsg sendcmsg;

static int if_sendbuf(int id, int channel, struct sk_buff *skb)
{
	capidrv_contr *card = findcontrbydriverid(id);
	capidrv_bchan *bchan;
	capidrv_ncci *nccip;
	int len = skb->len;
	size_t msglen;
	__u16 errcode;

	if (!card) {
		printk(KERN_ERR "capidrv: if_sendbuf called with invalid driverId %d!\n",
		       id);
		return 0;
	}
	bchan = &card->bchans[channel % card->nbchan];
	nccip = bchan->nccip;
	if (!nccip || nccip->state != ST_NCCI_ACTIVE) {
		printk(KERN_ERR "capidrv: if_sendbuf: %s:%d: chan not up!\n",
		       card->name, channel);
		return 0;
	}
	capi_fill_DATA_B3_REQ(&sendcmsg, global.appid, card->msgid++,
			      nccip->ncci,	/* adr */
			      (__u32) skb->data,	/* Data */
			      skb->len,		/* DataLength */
			      nccip->datahandle++,	/* DataHandle */
			      0	/* Flags */
	    );
	capi_cmsg2message(&sendcmsg, sendcmsg.buf);
	msglen = CAPIMSG_LEN(sendcmsg.buf);
	if (skb_headroom(skb) < msglen) {
		struct sk_buff *nskb = dev_alloc_skb(msglen + skb->len);
		if (!nskb) {
			printk(KERN_ERR "capidrv: if_sendbuf: no memory\n");
			return 0;
		}
#if 0
		printk(KERN_DEBUG "capidrv: only %d bytes headroom\n",
		       skb_headroom(skb));
#endif
                SET_SKB_FREE(nskb);
		memcpy(skb_put(nskb, msglen), sendcmsg.buf, msglen);
		memcpy(skb_put(nskb, skb->len), skb->data, skb->len);
		errcode = (*capifuncs->capi_put_message) (global.appid, nskb);
		switch (errcode) {
		case CAPI_NOERROR:
			dev_kfree_skb(skb, FREE_WRITE);
			return len;
		case CAPI_SENDQUEUEFULL:
			dev_kfree_skb(nskb, FREE_WRITE);
			return 0;
		default:
			return -1;
		}
	} else {
		memcpy(skb_push(skb, msglen), sendcmsg.buf, msglen);
		errcode = (*capifuncs->capi_put_message) (global.appid, skb);
		switch (errcode) {
		case CAPI_NOERROR:
			return len;
		case CAPI_SENDQUEUEFULL:
			return 0;
		default:
			return -1;
		}
	}
}

static int capidrv_addcontr(__u16 contr, struct capi_profile *profp)
{
	capidrv_contr *card;
	isdn_ctrl cmd;
	char id[20];
	int i;

	sprintf(id, "capidrv-%d", contr);
	if (!(card = (capidrv_contr *) kmalloc(sizeof(capidrv_contr), GFP_ATOMIC))) {
		printk(KERN_WARNING
		 "capidrv: (%s) Could not allocate contr-struct.\n", id);
		return -1;
	}
	memset(card, 0, sizeof(capidrv_contr));
	strcpy(card->name, id);
	card->contrnr = contr;
	card->nbchan = profp->nbchannel;
	card->bchans = (capidrv_bchan *) kmalloc(sizeof(capidrv_bchan) * card->nbchan, GFP_ATOMIC);
	if (!card->bchans) {
		printk(KERN_WARNING
		"capidrv: (%s) Could not allocate bchan-structs.\n", id);
		kfree(card);
		return -1;
	}
	card->interface.channels = profp->nbchannel;
	card->interface.maxbufsize = 2048;
	card->interface.command = if_command;
	card->interface.writebuf_skb = if_sendbuf;
	card->interface.writecmd = 0;
	card->interface.readstat = 0;
	card->interface.features = ISDN_FEATURE_L2_X75I |
	    ISDN_FEATURE_L2_X75UI |
	    ISDN_FEATURE_L2_X75BUI |
	    ISDN_FEATURE_L2_HDLC |
	    ISDN_FEATURE_L2_TRANS |
	    ISDN_FEATURE_L3_TRANS |
	    ISDN_FEATURE_P_UNKNOWN;
	card->interface.hl_hdrlen = 22; /* len of DATA_B3_REQ */
	strncpy(card->interface.id, id, sizeof(card->interface.id) - 1);
	card->next = global.contr_list;
	global.contr_list = card;
	global.ncontr++;

	if (!register_isdn(&card->interface)) {
		global.contr_list = global.contr_list->next;
		printk(KERN_ERR "capidrv: Unable to register contr %s\n", id);
		kfree(card->bchans);
		kfree(card);
		return -1;
	}
	card->myid = card->interface.channels;

	memset(card->bchans, 0, sizeof(capidrv_bchan) * card->nbchan);
	for (i = 0; i < card->nbchan; i++) {
		card->bchans[i].contr = card;
	}

	cmd.driver = card->myid;
	cmd.command = ISDN_STAT_RUN;
	card->interface.statcallb(&cmd);

	card->cipmask = 1;	/* any */
	card->cipmask2 = 0;

	capi_fill_LISTEN_REQ(&cmdcmsg, global.appid,
			     card->msgid++,
			     contr,	/* controller */
			     1 << 6,	/* Infomask */
			     card->cipmask,
			     card->cipmask2,
			     0, 0);
	send_message(card, &cmdcmsg);
	listen_change_state(card, EV_LISTEN_REQ);

	printk(KERN_INFO "%s: now up (%d B channels)\n",
		card->name, card->nbchan);

	return 0;
}

static int capidrv_delcontr(__u16 contr)
{
	capidrv_contr **pp, *card;
	isdn_ctrl cmd;
	int i;

	for (pp = &global.contr_list; *pp; pp = &(*pp)->next) {
		if ((*pp)->contrnr == contr)
			break;
	}
	if (!*pp) {
		printk(KERN_ERR "capidrv: delcontr: no contr %u\n", contr);
		return -1;
	}
	card = *pp;
	*pp = (*pp)->next;
	global.ncontr--;

	for (i = 0; i < card->nbchan; i++) {
		if (card->bchans[i].nccip)
			free_ncci(card, card->bchans[i].nccip);
		if (card->bchans[i].plcip)
			free_plci(card, card->bchans[i].plcip);
		if (card->plci_list)
			printk(KERN_ERR "capidrv: bug in free_plci()\n");
	}
	kfree(card->bchans);

	cmd.command = ISDN_STAT_UNLOAD;
	cmd.driver = card->myid;
	card->interface.statcallb(&cmd);

	printk(KERN_INFO "%s: now down.\n", card->name);

	kfree(card);

	return 0;
}


static void lower_callback(unsigned int cmd, __u16 contr, void *data)
{
	switch (cmd) {
	case KCI_CONTRUP:
		(void) capidrv_addcontr(contr, (capi_profile *) data);
		break;
	case KCI_CONTRDOWN:
		(void) capidrv_delcontr(contr);
		break;
	}
}

static struct capi_interface_user cuser = {
	"capidrv",
	lower_callback
};

#ifdef MODULE
#define capidrv_init init_module
#endif

int capidrv_init(void)
{
	struct capi_register_params rparam;
	capi_profile profile;
	char rev[10];
	char *p;
	__u32 ncontr, contr;
	__u16 errcode;

	capifuncs = attach_capi_interface(&cuser);

	if (!capifuncs)
		return -EIO;

#ifndef HAS_NEW_SYMTAB
	/* No symbols to export, hide all symbols */
	register_symtab(NULL);
#endif

	if ((p = strchr(revision, ':'))) {
		strcpy(rev, p + 1);
		p = strchr(rev, '$');
		*p = 0;
	} else
		strcpy(rev, " ??? ");

	rparam.level3cnt = 2;
	rparam.datablkcnt = 8;
	rparam.datablklen = 2048;
	errcode = (*capifuncs->capi_register) (&rparam, &global.appid);
	if (errcode) {
		detach_capi_interface(&cuser);
		return -EIO;
	}

	errcode = (*capifuncs->capi_get_profile) (0, &profile);
	if (errcode != CAPI_NOERROR) {
		(void) (*capifuncs->capi_release) (global.appid);
		detach_capi_interface(&cuser);
		return -EIO;
	}

	(void) (*capifuncs->capi_set_signal) (global.appid, capidrv_signal, 0);

	ncontr = profile.ncontroller;
	for (contr = 1; contr <= ncontr; contr++) {
		errcode = (*capifuncs->capi_get_profile) (contr, &profile);
		if (errcode != CAPI_NOERROR)
			continue;
		(void) capidrv_addcontr(contr, &profile);
	}

	return 0;
}

#ifdef MODULE
void cleanup_module(void)
{
	capidrv_contr *card, *next;
	char rev[10];
	char *p;

	if ((p = strchr(revision, ':'))) {
		strcpy(rev, p + 1);
		p = strchr(rev, '$');
		*p = 0;
	} else {
		strcpy(rev, " ??? ");
	}

	for (card = global.contr_list; card; card = next) {
		next = card->next;
		capidrv_delcontr(card->contrnr);
	}

	(void) (*capifuncs->capi_release) (global.appid);
	detach_capi_interface(&cuser);

	printk(KERN_NOTICE "capidrv: Rev%s: unloaded\n", rev);
}

#endif
