/*
 *		IP_MASQ_IRC irc masquerading module
 *
 *
 * Version:	@(#)ip_masq_irc.c 0.01   03/20/96
 *
 * Author:	Juan Jose Ciarlante
 *		
 *
 * Fixes:
 *	- set NO_DADDR flag in ip_masq_new().
 *	- Added multiple port support (Nigel Metheringham)
 *
 * FIXME:
 *	- detect also previous "PRIVMSG" string ?.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *	
 * Multiple Port Support
 *	The helper can be made to handle up to MAX_MASQ_APP_PORTS (normally 12)
 *	with the port numbers being defined at module load time.  The module
 *	uses the symbol "ports" to define a list of monitored ports, which can
 *	be specified on the insmod command line as
 *		ports=x1,x2,x3...
 *	where x[n] are integer port numbers.  This option can be put into
 *	/etc/conf.modules (or /etc/modules.conf depending on your config)
 *	where modload will pick it up should you use modload to load your
 *	modules.
 *	
 */

#include <linux/module.h>

#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/system.h>
#include <linux/skbuff.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <net/protocol.h>
#include <net/tcp.h>
#include <net/ip_masq.h>

#ifndef DEBUG_CONFIG_IP_MASQ_IRC
#define DEBUG_CONFIG_IP_MASQ_IRC 0
#endif

/* 
 * List of ports (up to MAX_MASQ_APP_PORTS) to be handled by helper
 * First port is set to the default port.
 */
static int ports[MAX_MASQ_APP_PORTS] = {6667}; /* I rely on the trailing items being set to zero */
struct ip_masq_app *masq_incarnations[MAX_MASQ_APP_PORTS];

static int
masq_irc_init_1 (struct ip_masq_app *mapp, struct ip_masq *ms)
{
        MOD_INC_USE_COUNT;
        return 0;
}

static int
masq_irc_done_1 (struct ip_masq_app *mapp, struct ip_masq *ms)
{
        MOD_DEC_USE_COUNT;
        return 0;
}

int
masq_irc_out (struct ip_masq_app *mapp, struct ip_masq *ms, struct sk_buff **skb_p, struct device *dev)
{
        struct sk_buff *skb;
	struct iphdr *iph;
	struct tcphdr *th;
	char *data, *data_limit;
	__u32 s_addr;
	__u16 s_port;
	struct ip_masq *n_ms;
	char buf[20];		/* "m_addr m_port" (dec base)*/
        unsigned buf_len;
	int diff;
        int xtra_args = 0;      /* extra int args wanted after addr */
        char *dcc_p, *addr_beg_p, *addr_end_p;

        skb = *skb_p;
	iph = skb->h.iph;
        th = (struct tcphdr *)&(((char *)iph)[iph->ihl*4]);
        data = (char *)&th[1];

        /*
         *	Hunt irc DCC string, the _shortest_:
         *
         *	strlen("DCC CHAT chat AAAAAAAA P\x01\n")=26
         *	strlen("DCC SEND F AAAAAAAA P S\x01\n")=25
         *		AAAAAAAAA: bound addr (1.0.0.0==16777216, min 8 digits)
         *		P:         bound port (min 1 d )
         *		F:         filename   (min 1 d )
         *		S:         size       (min 1 d ) 
         *		0x01, \n:  terminators
         */

        data_limit = skb->h.raw + skb->len;
        
	while (data < (data_limit - 25) )
	{
		if (memcmp(data,"DCC ",4))  {
			data ++;
			continue;
		}
                
                dcc_p = data;
		data += 4;     /* point to DCC cmd */
                
                if (memcmp(data, "CHAT ", 5) == 0 ||
                    memcmp(data, "SEND ", 5) == 0)
                {
                        /*
                         *	extra arg (file_size) req. for "SEND"
                         */
                        
                        if (*data == 'S') xtra_args++;
                        data += 5;
                }
                else
                        continue;

                /*
                 *	skip next string.
                 */
                
                while( *data++ != ' ')
                        
                        /*
                         *	must still parse, at least, "AAAAAAAA P\x01\n",
                         *      12 bytes left.
                         */
                        if (data > (data_limit-12)) return 0;

                
                addr_beg_p = data;
                
                /*
                 *	client bound address in dec base
                 */
                
 		s_addr = simple_strtoul(data,&data,10);
		if (*data++ !=' ')
			continue;

                /*
                 *	client bound port in dec base
                 */
                
		s_port = simple_strtoul(data,&data,10);
                addr_end_p = data;
                
                /*
                 *	should check args consistency?
                 */
                
                while(xtra_args) {
                        if (*data != ' ')
                                break;
                        data++;
                        simple_strtoul(data,&data,10);
                        xtra_args--;
                }
                
                if (xtra_args != 0) continue;
                
                /*
                 *	terminators.
                 */
                
                if (data[0] != 0x01)
                        continue;
		if (data[1]!='\r' && data[1]!='\n')
			continue;
                
		/*
		 *	Now create an masquerade entry for it
                 * 	must set NO_DPORT and NO_DADDR because
                 *	connection is requested by another client.
		 */
                
		n_ms = ip_masq_new(dev, IPPROTO_TCP,
                                   htonl(s_addr),htons(s_port),
                                   0, 0,
                                   IP_MASQ_F_NO_DPORT|IP_MASQ_F_NO_DADDR
                                   );
		if (n_ms==NULL)
			return 0;

                ip_masq_set_expire(n_ms, ip_masq_expire->tcp_fin_timeout);
                
		/*
		 * Replace the old "address port" with the new one
		 */
                
		buf_len = sprintf(buf,"%lu %u",
                        ntohl(n_ms->maddr),ntohs(n_ms->mport));
                
		/*
		 * Calculate required delta-offset to keep TCP happy
		 */
		
		diff = buf_len - (addr_end_p-addr_beg_p);

#if DEBUG_CONFIG_IP_MASQ_IRC
                *addr_beg_p = '\0';
		printk("masq_irc_out(): '%s' %X:%X detected (diff=%d)\n", dcc_p, s_addr,s_port, diff);
#endif	
		/*
		 *	No shift.
		 */
		 
		if (diff==0) 
		{
			/*
			 * simple case, just copy.
 			 */
 			memcpy(addr_beg_p,buf,buf_len);
 			return 0;
 		}

                *skb_p = ip_masq_skb_replace(skb, GFP_ATOMIC,
                                             addr_beg_p, addr_end_p-addr_beg_p,
                                             buf, buf_len);
                return diff;
	}
	return 0;

}

/*
 *	Main irc object
 *     	You need 1 object per port in case you need
 *	to offer also other used irc ports (6665,6666,etc),
 *	they will share methods but they need own space for
 *	data. 
 */

struct ip_masq_app ip_masq_irc = {
        NULL,			/* next */
	"irc",			/* name */
        0,                      /* type */
        0,                      /* n_attach */
        masq_irc_init_1,        /* init_1 */
        masq_irc_done_1,        /* done_1 */
        masq_irc_out,           /* pkt_out */
        NULL                    /* pkt_in */
};

/*
 * 	ip_masq_irc initialization
 */

int ip_masq_irc_init(void)
{
	int i, j;

	for (i=0; (i<MAX_MASQ_APP_PORTS); i++) {
		if (ports[i]) {
			if ((masq_incarnations[i] = kmalloc(sizeof(struct ip_masq_app),
							    GFP_KERNEL)) == NULL)
				return -ENOMEM;
			memcpy(masq_incarnations[i], &ip_masq_irc, sizeof(struct ip_masq_app));
			if ((j = register_ip_masq_app(masq_incarnations[i], 
						      IPPROTO_TCP, 
						      ports[i]))) {
				return j;
			}
#if DEBUG_CONFIG_IP_MASQ_IRC
			printk("Irc: loaded support on port[%d] = %d\n",
			       i, ports[i]);
#endif
		} else {
			/* To be safe, force the incarnation table entry to NULL */
			masq_incarnations[i] = NULL;
		}
	}
	return 0;
}

/*
 * 	ip_masq_irc fin.
 */

int ip_masq_irc_done(void)
{
	int i, j, k;

	k=0;
	for (i=0; (i<MAX_MASQ_APP_PORTS); i++) {
		if (masq_incarnations[i]) {
			if ((j = unregister_ip_masq_app(masq_incarnations[i]))) {
				k = j;
			} else {
				kfree(masq_incarnations[i]);
				masq_incarnations[i] = NULL;
#if DEBUG_CONFIG_IP_MASQ_IRC
				printk("Irc: unloaded support on port[%d] = %d\n",
				       i, ports[i]);
#endif
			}
		}
	}
	return k;
}


#ifdef MODULE

int init_module(void)
{
        if (ip_masq_irc_init() != 0)
                return -EIO;
        register_symtab(NULL);
        return 0;
}

void cleanup_module(void)
{
        if (ip_masq_irc_done() != 0)
                printk("ip_masq_irc: can't remove module");
}

#endif /* MODULE */
