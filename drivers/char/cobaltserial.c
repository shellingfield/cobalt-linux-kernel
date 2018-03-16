/*
 * cobaltserial.c: Serial port driver for Cobalt Microserver.
 *
 * Derived directly from Mac serial driver, which was
 * derived from: drivers/sbus/char/sunserial.c by Paul Mackerras.
 *
 * Added 16550 support as well for RAQ boards.  -DaveM
 *
 * Copyright (C) 1996 Paul Mackerras (Paul.Mackerras@cs.anu.edu.au)
 * Copyright (C) 1995,1997,1998 David S. Miller (davem@caip.rutgers.edu)
 *
 * $Id: cobaltserial.c,v 1.17 1998/12/11 10:22:39 gid Exp $
 */

#include <linux/config.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/console.h>

#include <asm/bootinfo.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/segment.h>
#include <asm/bitops.h>

#include "cobaltserial.h"
#include <linux/serial.h>
#include <linux/serial_reg.h>

enum cobalt_scc_type {
	COBALT_SCC_TYPE_ZILOG = 0, /* 2700 devel/alpha/beta boards */
        COBALT_SCC_TYPE_16550 = 1, /* all RAQs */
	COBALT_SCC_TYPE_NONE = 2,  /* 2700 production units */
};

/* Things shared by both the Zilog and 16550 specific drivers since
 * never are both active at the same.  -DaveM
 */
DECLARE_TASK_QUEUE(tq_serial);
struct tty_driver serial_driver, callout_driver;
static int serial_refcount;

/* serial subtype definitions */
#define SERIAL_TYPE_NORMAL	1
#define SERIAL_TYPE_CALLOUT	2

/* number of characters left in xmit buffer before we ask for more */
#define WAKEUP_CHARS 256

/* Debugging... DEBUG_INTR is bad to use when one of the zs
 * lines is your console ;(
 */
#undef SERIAL_DEBUG_INTR
#undef SERIAL_DEBUG_OPEN
#undef SERIAL_DEBUG_FLOW

#define _INLINE_ inline


#ifndef MIN
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#endif

/*
 * tmp_buf is used as a temporary buffer by serial_write.  We need to
 * lock it in case the memcpy_fromfs blocks while swapping in a page,
 * and some other program tries to do a serial write at the same time.
 * Since the lock will only come under contention when the system is
 * swapping and available memory is low, it makes sense to share one
 * buffer across all the serial ports, since it significantly saves
 * memory if large numbers of serial ports are open.
 */
static unsigned char tmp_buf[4096]; /* This is cheating */
static struct semaphore tmp_buf_sem = MUTEX;

/* ======================== Zilog-specific portion of the driver ========================== */

/*
 * It would be nice to dynamically allocate everything that
 * depends on NUM_SERIAL, so we could support any number of
 * Z8530s, but for now...
 */
#define ZS_NUM_SERIAL	1		/* Max number of ZS chips supported */
#define ZS_NUM_CHANNELS	(ZS_NUM_SERIAL * 2)	/* 2 channels per chip */

static struct wait_queue * keypress_wait = NULL;

/* This delays in between register accesses to met the settle
 * time requirements of the Zilog chip.
 */
#define ZS_RECOVERY_DELAY	udelay(5)

struct mac_zschannel *zs_kgdbchan;
struct mac_zschannel zs_channels[ZS_NUM_CHANNELS];

struct mac_serial zs_soft[ZS_NUM_CHANNELS];
int zs_channels_found;
struct mac_serial *zs_chain;	/* list of all channels */

struct tty_struct zs_ttys[ZS_NUM_CHANNELS];
/** struct tty_struct *zs_constty; **/

/* Console hooks... */
static int zs_cons_chan = 0;
struct mac_serial *zs_consinfo = 0;
struct mac_zschannel *zs_conschan;

/*
 * Initialization values for when a channel is used for
 * kernel gdb support.
 */
static unsigned char zs_kgdb_regs[16] = {
	0, 0, 0,		/* write 0, 1, 2 */
	(Rx8 | RxENABLE),	/* write 3 */
	(X16CLK | SB1),		/* write 4 */
	(Tx8 | TxENAB | RTS),	/* write 5 */
	0, 0, 0,		/* write 6, 7, 8 */
	(NV),			/* write 9 */
	(NRZ),			/* write 10 */
	(TCBR | RCBR),		/* write 11 */
	1, 0,			/* 38400 baud divisor, write 12 + 13 */
	(BRENABL),		/* write 14 */
	(DCDIE)			/* write 15 */
};

#define ZS_CLOCK         11059200 	/* Z8530 RTxC input clock rate */

static void zs_probe_sccs(void);
static void zs_change_speed(struct mac_serial *info);

static struct tty_struct *zs_serial_table[ZS_NUM_CHANNELS];
static struct termios *zs_serial_termios[ZS_NUM_CHANNELS];
static struct termios *zs_serial_termios_locked[ZS_NUM_CHANNELS];
static inline int zs_serial_paranoia_check(struct mac_serial *info,
					   dev_t device, const char *routine)
{
#ifdef ZS_SERIAL_PARANOIA_CHECK
	static const char *badmagic =
		"Warning: bad magic number for serial struct (%d, %d) in %s\n";
	static const char *badinfo =
		"Warning: null mac_serial for (%d, %d) in %s\n";

	if (!info) {
		printk(badinfo, MAJOR(device), MINOR(device), routine);
		return 1;
	}
	if (info->magic != SERIAL_MAGIC) {
		printk(badmagic, MAJOR(device), MINOR(device), routine);
		return 1;
	}
#endif
	return 0;
}

/*
 * This is used to figure out the divisor speeds and the timeouts
 */
static int zs_baud_table[] = {
	0, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800,
	9600, 19200, 38400, 57600, 115200, 230400, 0 };

/* 
 * Reading and writing Z8530 registers.
 */
static inline unsigned char read_zsreg(struct mac_zschannel *channel,
				       unsigned char reg)
{
	unsigned char retval;

	*channel->control = reg;
	ZS_RECOVERY_DELAY;
	retval = *channel->control;
	ZS_RECOVERY_DELAY;
	return retval;
}

static inline void write_zsreg(struct mac_zschannel *channel,
			       unsigned char reg, unsigned char value)
{
	*channel->control = reg;
	ZS_RECOVERY_DELAY;
	*channel->control = value;
	ZS_RECOVERY_DELAY;
	return;
}

static inline unsigned char read_zsdata(struct mac_zschannel *channel)
{
	unsigned char retval;

	retval = *channel->data;
	ZS_RECOVERY_DELAY;
	return retval;
}

static inline void write_zsdata(struct mac_zschannel *channel,
				unsigned char value)
{
	*channel->data = value;
	ZS_RECOVERY_DELAY;
	return;
}

static inline void load_zsregs(struct mac_zschannel *channel,
			       unsigned char *regs)
{
	ZS_CLEARERR(channel);
	ZS_CLEARFIFO(channel);

	// Reset chip (NOTE: we toast the opposing channel here, but I need to debug this thing) TJS
	write_zsreg(channel, R9, FHWRES);


	/* Load 'em up */
	write_zsreg(channel, R4, regs[R4]);
	write_zsreg(channel, R2, regs[R2]);
	write_zsreg(channel, R3, regs[R3] & ~RxENABLE);
	write_zsreg(channel, R5, regs[R5] & ~TxENAB);
	write_zsreg(channel, R9, regs[R9]);

	write_zsreg(channel, R10, regs[R10]);
	write_zsreg(channel, R11, regs[R11]);
	write_zsreg(channel, R12, regs[R12]);
	write_zsreg(channel, R13, regs[R13]);
	write_zsreg(channel, R14, regs[R14] & ~BRENABL);

	write_zsreg(channel, R14, regs[R14]);
	write_zsreg(channel, R3, regs[R3]);
	write_zsreg(channel, R5, regs[R5]);

	write_zsreg(channel, R1, regs[R1]);
	write_zsreg(channel, R9, regs[R9]);

	return;

}

/* Sets or clears DTR/RTS on the requested line */
static inline void zs_rtsdtr(struct mac_serial *ss, int set)
{
	if (set)
		ss->curregs[5] |= (RTS | DTR);
	else
		ss->curregs[5] &= ~(RTS | DTR);
	write_zsreg(ss->zs_channel, 5, ss->curregs[5]);
	return;
}

static inline void zs_kgdb_chaninit(struct mac_serial *ss, int intson, int bps)
{
	int brg;

	if (intson) {
		zs_kgdb_regs[R1] = INT_ALL_Rx;
		zs_kgdb_regs[R9] |= MIE;
	} else {
		zs_kgdb_regs[R1] = 0;
		zs_kgdb_regs[R9] &= ~MIE;
	}
	brg = BPS_TO_BRG(bps, ZS_CLOCK/16);
	zs_kgdb_regs[R12] = brg;
	zs_kgdb_regs[R13] = brg >> 8;
	load_zsregs(ss->zs_channel, zs_kgdb_regs);
}

/* Utility routines for the Zilog */
static inline int get_zsbaud(struct mac_serial *ss)
{
	struct mac_zschannel *channel = ss->zs_channel;
	int brg;

	if ((ss->curregs[R11] & TCBR) == 0) {
		/* higher rates don't use the baud rate generator */
		return (ss->curregs[R4] & X32CLK)? ZS_CLOCK/32: ZS_CLOCK/16;
	}
	/* The baud rate is split up between two 8-bit registers in
	 * what is termed 'BRG time constant' format in my docs for
	 * the chip, it is a function of the clk rate the chip is
	 * receiving which happens to be constant.
	 */
	brg = (read_zsreg(channel, 13) << 8);
	brg |= read_zsreg(channel, 12);
	return BRG_TO_BPS(brg, (ZS_CLOCK/(ss->clk_divisor)));
}

/* On receive, this clears errors and the receiver interrupts */
static inline void zs_recv_clear(struct mac_zschannel *zsc)
{
	write_zsreg(zsc, 0, ERR_RES);
	write_zsreg(zsc, 0, RES_H_IUS); /* XXX this is unnecessary */
}

/*
 * ------------------------------------------------------------
 * rs_stop() and rs_start()
 *
 * This routines are called before setting or resetting tty->stopped.
 * ------------------------------------------------------------
 */
static void zs_stop(struct tty_struct *tty)
{
	struct mac_serial *info = (struct mac_serial *)tty->driver_data;
	unsigned long flags;

	if (zs_serial_paranoia_check(info, tty->device, "zs_stop"))
		return;

	save_and_cli(flags);
	if (info->curregs[5] & TxENAB) {
		info->curregs[5] &= ~TxENAB;
		write_zsreg(info->zs_channel, 5, info->curregs[5]);
	}
	restore_flags(flags);
}

static void zs_start(struct tty_struct *tty)
{
	struct mac_serial *info = (struct mac_serial *)tty->driver_data;
	unsigned long flags;
	
	if (zs_serial_paranoia_check(info, tty->device, "zs_start"))
		return;
	
	save_flags(flags); cli();
	if (info->xmit_cnt && info->xmit_buf && !(info->curregs[5] & TxENAB)) {
		info->curregs[5] |= TxENAB;
		write_zsreg(info->zs_channel, 5, info->curregs[5]);
	}
	restore_flags(flags);
}

/*
 * Drop into either the boot monitor or kadb upon receiving a break
 * from keyboard/console input. XXX
 */
void batten_down_hatches(void)
{
	panic("batten_down_hatches called.  Implement me.");
}

/*
 * ----------------------------------------------------------------------
 *
 * Here starts the interrupt handling routines.  All of the following
 * subroutines are declared as inline and are folded into
 * rs_interrupt().  They were separated out for readability's sake.
 *
 * 				- Ted Ts'o (tytso@mit.edu), 7-Mar-93
 * -----------------------------------------------------------------------
 */

/*
 * This routine is used by the interrupt handler to schedule
 * processing in the software interrupt portion of the driver.
 */
static _INLINE_ void zs_sched_event(struct mac_serial *info,
				    int event)
{
	info->event |= 1 << event;
	queue_task(&info->tqueue, &tq_serial);
	mark_bh(SERIAL_BH);
}

extern void breakpoint(void);  /* For the KGDB frame character */

static _INLINE_ void zs_receive_chars(struct mac_serial *info,
				   struct pt_regs *regs)
{
	struct tty_struct *tty = info->tty;
	unsigned char ch, stat, flag;

	while ((read_zsreg(info->zs_channel, 0) & Rx_CH_AV) != 0) {

		stat = read_zsreg(info->zs_channel, R1);
		ch = read_zsdata(info->zs_channel);

		if(info->is_cons) {
			if(ch==0) { /* whee, break received */
				batten_down_hatches();
				zs_recv_clear(info->zs_channel);
				return;
			} else if (ch == 1) {
				show_state();
				return;
			} else if (ch == 2) {
				show_buffers();
				return;
			}
			/* It is a 'keyboard interrupt' ;-) */
			wake_up(&keypress_wait);
		}

#ifdef CONFIG_REMOTE_DEBUG	/* KGDB not yet supported */
		/*
		 * Look for kgdb 'stop' character, consult the gdb documentation
		 * for remote target debugging and arch/sparc/kernel/gdb-stub.c
		 * to see how all this works.
		 */
		if ((info->kgdb_channel) && (ch =='\003')) {
			breakpoint();
			continue;
		}
#endif

		if (!tty)
			continue;

		if (tty->flip.count >= TTY_FLIPBUF_SIZE)
			queue_task(&tty->flip.tqueue, &tq_timer);
		tty->flip.count++;
		if (stat & Rx_OVR) {
			flag = TTY_OVERRUN;
			/* reset the error indication */
			write_zsreg(info->zs_channel, 0, ERR_RES);
		} else if (stat & FRM_ERR) {
			/* this error is not sticky */
			flag = TTY_FRAME;
		} else if (stat & PAR_ERR) {
			flag = TTY_PARITY;
			/* reset the error indication */
			write_zsreg(info->zs_channel, 0, ERR_RES);
		} else
			flag = 0;
		*tty->flip.flag_buf_ptr++ = flag;
		*tty->flip.char_buf_ptr++ = ch;

		queue_task(&tty->flip.tqueue, &tq_timer);
	}
}

static void zs_transmit_chars(struct mac_serial *info)
{
	if ((read_zsreg(info->zs_channel, 0) & Tx_BUF_EMP) == 0)
		return;
	info->tx_active = 0;

	if (info->x_char) {
		/* Send next char */
		write_zsdata(info->zs_channel, info->x_char);
		info->x_char = 0;
		info->tx_active = 1;
		return;
	}

	if ((info->xmit_cnt <= 0) || info->tty->stopped || info->tx_stopped) {
		write_zsreg(info->zs_channel, 0, RES_Tx_P);
		return;
	}

	/* Send char */
	write_zsdata(info->zs_channel, info->xmit_buf[info->xmit_tail++]);
	info->xmit_tail = info->xmit_tail & (SERIAL_XMIT_SIZE-1);
	info->xmit_cnt--;
	info->tx_active = 1;

	if (info->xmit_cnt < WAKEUP_CHARS)
		zs_sched_event(info, RS_EVENT_WRITE_WAKEUP);
}

static _INLINE_ void zs_status_handle(struct mac_serial *info)
{
	unsigned char status;

	/* Get status from Read Register 0 */
	status = read_zsreg(info->zs_channel, 0);

	/* Check for DCD transitions */
	if (((status ^ info->read_reg_zero) & DCD) != 0
	    && info->tty && C_CLOCAL(info->tty)) {
		if (status & DCD) {
			wake_up_interruptible(&info->open_wait);
		} else if (!(info->flags & ZILOG_CALLOUT_ACTIVE)) {
			queue_task(&info->tqueue_hangup, &tq_scheduler);
		}
	}

	/* Check for CTS transitions */
	if (info->tty && C_CRTSCTS(info->tty)) {
		/*
		 * For some reason, on the Power Macintosh,
		 * it seems that the CTS bit is 1 when CTS is
		 * *negated* and 0 when it is asserted.
		 * The DCD bit doesn't seem to be inverted
		 * like this.
		 */
		if ((status & CTS) == 0) {
			if (info->tx_stopped) {
				info->tx_stopped = 0;
				if (!info->tx_active)
					zs_transmit_chars(info);
			}
		} else {
			info->tx_stopped = 1;
		}
	}

	/* Clear status condition... */
	write_zsreg(info->zs_channel, 0, RES_EXT_INT);
	info->read_reg_zero = status;
}

/*
 * This is the serial driver's generic interrupt routine
 */
static void zs_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	struct mac_serial *info = (struct mac_serial *) dev_id;
	unsigned char zs_intreg;
	int shift;

	/* NOTE: The read register 3, which holds the irq status,
	 *       does so for both channels on each chip.  Although
	 *       the status value itself must be read from the A
	 *       channel and is only valid when read from channel A.
	 *       Yes... broken hardware...
	 */
#define CHAN_IRQMASK (CHBRxIP | CHBTxIP | CHBEXT)

	if (info->zs_chan_a == info->zs_channel)
		shift = 3;	/* Channel A */
	else
		shift = 0;	/* Channel B */

	for (;;) {
		zs_intreg = read_zsreg(info->zs_chan_a, 3) >> shift;
		if ((zs_intreg & CHAN_IRQMASK) == 0)
			break;

		if (zs_intreg & CHBRxIP)
			zs_receive_chars(info, regs);
		if (zs_intreg & CHBTxIP)
			zs_transmit_chars(info);
		if (zs_intreg & CHBEXT)
			zs_status_handle(info);
	}
}

/*
 * -------------------------------------------------------------------
 * Here ends the serial interrupt routines.
 * -------------------------------------------------------------------
 */

/*
 * This routine is used to handle the "bottom half" processing for the
 * serial driver, known also the "software interrupt" processing.
 * This processing is done at the kernel interrupt level, after the
 * zs_interrupt() has returned, BUT WITH INTERRUPTS TURNED ON.  This
 * is where time-consuming activities which can not be done in the
 * interrupt driver proper are done; the interrupt driver schedules
 * them using zs_sched_event(), and they get done here.
 */
static void zs_do_serial_bh(void)
{
	run_task_queue(&tq_serial);
}

static void zs_do_softint(void *private_)
{
	struct mac_serial	*info = (struct mac_serial *) private_;
	struct tty_struct	*tty;
	
	tty = info->tty;
	if (!tty)
		return;

	if (clear_bit(RS_EVENT_WRITE_WAKEUP, &info->event)) {
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
		    tty->ldisc.write_wakeup)
			(tty->ldisc.write_wakeup)(tty);
		wake_up_interruptible(&tty->write_wait);
	}
}

/*
 * This routine is called from the scheduler tqueue when the interrupt
 * routine has signalled that a hangup has occurred.  The path of
 * hangup processing is:
 *
 * 	serial interrupt routine -> (scheduler tqueue) ->
 * 	do_serial_hangup() -> tty->hangup() -> rs_hangup()
 * 
 */
static void zs_do_serial_hangup(void *private_)
{
	struct mac_serial	*info = (struct mac_serial *) private_;
	struct tty_struct	*tty;
	
	tty = info->tty;
	if (!tty)
		return;

	tty_hangup(tty);
}

static void zs_timer(void)
{
}

static int zs_startup(struct mac_serial * info)
{
	unsigned long flags;

	if (info->flags & ZILOG_INITIALIZED)
		return 0;

	if (!info->xmit_buf) {
		info->xmit_buf = (unsigned char *) get_free_page(GFP_KERNEL);
		if (!info->xmit_buf)
			return -ENOMEM;
	}

	save_flags(flags); cli();

#ifdef SERIAL_DEBUG_OPEN
	printk("starting up ttyS%d (irq %d)...", info->line, info->irq);
#endif

	/*
	 * Clear the receive FIFO.
	 */
	ZS_CLEARFIFO(info->zs_channel);
	info->xmit_fifo_size = 1;

	/*
	 * Clear the interrupt registers.
	 */
	write_zsreg(info->zs_channel, 0, ERR_RES);
	write_zsreg(info->zs_channel, 0, RES_H_IUS);

	/*
	 * Turn on RTS and DTR.
	 */
	zs_rtsdtr(info, 1);

	/*
	 * Finally, enable sequencing and interrupts
	 */
	info->curregs[1] = (info->curregs[1] & ~0x18) | (EXT_INT_ENAB | INT_ALL_Rx | TxINT_ENAB);
	info->curregs[3] |= (RxENABLE | Rx8);
	info->curregs[5] |= (TxENAB | Tx8);
	info->curregs[9] |= (NV | MIE);
	write_zsreg(info->zs_channel, 3, info->curregs[3]);
	write_zsreg(info->zs_channel, 5, info->curregs[5]);
	write_zsreg(info->zs_channel, 9, info->curregs[9]);

	if (info->tty)
		clear_bit(TTY_IO_ERROR, &info->tty->flags);
	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;

	/*
	 * Set the speed of the serial port
	 */
	//zs_change_speed(info);

	/* Save the current value of RR0 */
	info->read_reg_zero = read_zsreg(info->zs_channel, 0);

	info->flags |= ZILOG_INITIALIZED;
	restore_flags(flags);
	return 0;
}

/*
 * This routine will shutdown a serial port; interrupts are disabled, and
 * DTR is dropped if the hangup on close termio flag is on.
 */
static void zs_shutdown(struct mac_serial * info)
{
	unsigned long	flags;

	if (!(info->flags & ZILOG_INITIALIZED))
		return;

#ifdef SERIAL_DEBUG_OPEN
	printk("Shutting down serial port %d (irq %d)....", info->line,
	       info->irq);
#endif
	
	save_flags(flags); cli(); /* Disable interrupts */
	
	if (info->xmit_buf) {
		free_page((unsigned long) info->xmit_buf);
		info->xmit_buf = 0;
	}

	if (info->tty)
		set_bit(TTY_IO_ERROR, &info->tty->flags);

	info->flags &= ~ZILOG_INITIALIZED;
	restore_flags(flags);
}

/*
 * This routine is called to set the UART divisor registers to match
 * the specified baud rate for a serial port.
 */
static void zs_change_speed(struct mac_serial *info)
{
	unsigned short port;
	unsigned cflag;
	int	i;
	int	brg;
	unsigned long flags;

	if (!info->tty || !info->tty->termios)
		return;
	cflag = info->tty->termios->c_cflag;
	if (!(port = info->port))
		return;
	i = cflag & (CBAUD | CBAUDEX);

	save_flags(flags); cli();
	info->zs_baud = zs_baud_table[i];
	info->clk_divisor = 16;

	switch (info->zs_baud) {
	case ZS_CLOCK/16:	/* 230400 */
		info->curregs[4] = X16CLK;
		info->curregs[11] = 0;
		break;
	case ZS_CLOCK/32:	/* 115200 */
		info->curregs[4] = X32CLK;
		info->curregs[11] = 0;
		break;
	default:
		info->curregs[4] = X16CLK;
		info->curregs[11] = TCBR | RCBR;
		brg = BPS_TO_BRG(info->zs_baud, ZS_CLOCK/info->clk_divisor);
		info->curregs[12] = (brg & 255);
		info->curregs[13] = ((brg >> 8) & 255);
		info->curregs[14] = BRENABL;
	}

	/* byte size and parity */
	info->curregs[3] &= ~RxNBITS_MASK;
	info->curregs[5] &= ~TxNBITS_MASK;
	switch (cflag & CSIZE) {
	case CS5:
		info->curregs[3] |= Rx5;
		info->curregs[5] |= Tx5;
		break;
	case CS6:
		info->curregs[3] |= Rx6;
		info->curregs[5] |= Tx6;
		break;
	case CS7:
		info->curregs[3] |= Rx7;
		info->curregs[5] |= Tx7;
		break;
	case CS8:
	default: /* defaults to 8 bits */
		info->curregs[3] |= Rx8;
		info->curregs[5] |= Tx8;
		break;
	}

	info->curregs[4] &= ~(SB_MASK | PAR_ENA | PAR_EVEN);
	if (cflag & CSTOPB) {
		info->curregs[4] |= SB2;
	} else {
		info->curregs[4] |= SB1;
	}
	if (cflag & PARENB) {
		info->curregs[4] |= PAR_ENA;
	}
	if (!(cflag & PARODD)) {
		info->curregs[4] |= PAR_EVEN;
	}

	info->curregs[15] &= ~(DCDIE | CTSIE);
	if (!(cflag & CLOCAL)) {
		info->curregs[15] |= DCDIE;
	}
	if (cflag & CRTSCTS) {
		info->curregs[15] |= CTSIE;
		if ((read_zsreg(info->zs_channel, 0) & CTS) != 0)
			info->tx_stopped = 1;
	} else
		info->tx_stopped = 0;

	/* Load up the new values */
	load_zsregs(info->zs_channel, info->curregs);

	restore_flags(flags);
}

/* This is for console output over ttya/ttyb */
static void zs_put_char(char ch)
{
	struct mac_zschannel *chan = zs_conschan;
	int loops = 0;
	unsigned long flags;

	if(!chan)
		return;

	save_flags(flags); cli();
	while ((read_zsreg(chan, 0) & Tx_BUF_EMP) == 0)
		if (++loops >= 1000000)
			break;
	write_zsdata(chan, ch);
	restore_flags(flags);
}

/* These are for receiving and sending characters under the kgdb
 * source level kernel debugger.
 */
void putDebugChar(char kgdb_char)
{
	struct mac_zschannel *chan = zs_kgdbchan;

	while ((read_zsreg(chan, 0) & Tx_BUF_EMP) == 0)
		udelay(5);
	write_zsdata(chan, kgdb_char);
}

char getDebugChar(void)
{
	struct mac_zschannel *chan = zs_kgdbchan;

	while ((read_zsreg(chan, 0) & Rx_CH_AV) == 0)
		udelay(5);
	return read_zsdata(chan);
}

/*
 * Fair output driver allows a process to speak.
 */
static void zs_fair_output(void)
{
	int left;		/* Output no more than that */
	unsigned long flags;
	struct mac_serial *info = zs_consinfo;
	char c;

	if (info == 0) return;
	if (info->xmit_buf == 0) return;

	save_flags(flags);  cli();
	left = info->xmit_cnt;
	while (left != 0) {
		c = info->xmit_buf[info->xmit_tail];
		info->xmit_tail = (info->xmit_tail+1) & (SERIAL_XMIT_SIZE-1);
		info->xmit_cnt--;
		restore_flags(flags);

		zs_put_char(c);

		save_flags(flags);  cli();
		left = MIN(info->xmit_cnt, left-1);
	}

	restore_flags(flags);
	return;
}

/*
 * zs_console_print is registered for printk.
 */
static void zs_console_print(const char *p)
{
	char c;

	while((c=*(p++)) != 0) {
		if(c == '\n')
			zs_put_char('\r');
		zs_put_char(c);
	}

	/* Comment this if you want to have a strict interrupt-driven output */
	zs_fair_output();

	return;
}

static void zs_flush_chars(struct tty_struct *tty)
{
	struct mac_serial *info = (struct mac_serial *)tty->driver_data;
	unsigned long flags;

	if (zs_serial_paranoia_check(info, tty->device, "zs_flush_chars"))
		return;

	if (info->xmit_cnt <= 0 || tty->stopped || info->tx_stopped ||
	    !info->xmit_buf)
		return;

	/* Enable transmitter */
	save_flags(flags); cli();
	zs_transmit_chars(info);
	restore_flags(flags);
}

static int zs_write(struct tty_struct * tty, int from_user,
		    const unsigned char *buf, int count)
{
	int	c, total = 0;
	struct mac_serial *info = (struct mac_serial *)tty->driver_data;
	unsigned long flags;

	if (zs_serial_paranoia_check(info, tty->device, "zs_write"))
		return 0;

	if (!info || !info->xmit_buf)
		return 0;

	save_flags(flags);
	while (1) {
		cli();		
		c = MIN(count, MIN(SERIAL_XMIT_SIZE - info->xmit_cnt - 1,
				   SERIAL_XMIT_SIZE - info->xmit_head));
		if (c <= 0)
			break;

		if (from_user) {
			down(&tmp_buf_sem);
			memcpy_fromfs(tmp_buf, buf, c);
			c = MIN(c, MIN(SERIAL_XMIT_SIZE - info->xmit_cnt - 1,
				       SERIAL_XMIT_SIZE - info->xmit_head));
			memcpy(info->xmit_buf + info->xmit_head, tmp_buf, c);
			up(&tmp_buf_sem);
		} else
			memcpy(info->xmit_buf + info->xmit_head, buf, c);
		info->xmit_head = (info->xmit_head + c) & (SERIAL_XMIT_SIZE-1);
		info->xmit_cnt += c;
		restore_flags(flags);
		buf += c;
		count -= c;
		total += c;
	}
	if (info->xmit_cnt && !tty->stopped && !info->tx_stopped
	    && !info->tx_active)
		zs_transmit_chars(info);
	restore_flags(flags);
	return total;
}

static int zs_write_room(struct tty_struct *tty)
{
	struct mac_serial *info = (struct mac_serial *)tty->driver_data;
	int	ret;
				
	if (zs_serial_paranoia_check(info, tty->device, "zs_write_room"))
		return 0;
	ret = SERIAL_XMIT_SIZE - info->xmit_cnt - 1;
	if (ret < 0)
		ret = 0;
	return ret;
}

static int zs_chars_in_buffer(struct tty_struct *tty)
{
	struct mac_serial *info = (struct mac_serial *)tty->driver_data;
				
	if (zs_serial_paranoia_check(info, tty->device, "zs_chars_in_buffer"))
		return 0;
	return info->xmit_cnt;
}

static void zs_flush_buffer(struct tty_struct *tty)
{
	struct mac_serial *info = (struct mac_serial *)tty->driver_data;
				
	if (zs_serial_paranoia_check(info, tty->device, "zs_flush_buffer"))
		return;
	cli();
	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;
	sti();
	wake_up_interruptible(&tty->write_wait);
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
	    tty->ldisc.write_wakeup)
		(tty->ldisc.write_wakeup)(tty);
}

/*
 * ------------------------------------------------------------
 * rs_throttle()
 * 
 * This routine is called by the upper-layer tty layer to signal that
 * incoming characters should be throttled.
 * ------------------------------------------------------------
 */
static void zs_throttle(struct tty_struct * tty)
{
	struct mac_serial *info = (struct mac_serial *)tty->driver_data;
	unsigned long flags;
#ifdef SERIAL_DEBUG_THROTTLE
	char	buf[64];
	
	printk("throttle %s: %d....\n", _tty_name(tty, buf),
	       tty->ldisc.chars_in_buffer(tty));
#endif

	if (zs_serial_paranoia_check(info, tty->device, "zs_throttle"))
		return;
	
	if (I_IXOFF(tty)) {
		save_flags(flags); cli();
		info->x_char = STOP_CHAR(tty);
		if (!info->tx_active)
			zs_transmit_chars(info);
		restore_flags(flags);
	}

	if (C_CRTSCTS(tty)) {
		/*
		 * Here we want to turn off the RTS line.  On Macintoshes,
		 * we only get the DTR line, which goes to both DTR and
		 * RTS on the modem.  RTS doesn't go out to the serial
		 * port socket.  So you should make sure your modem is
		 * set to ignore DTR if you're using CRTSCTS.
		 */
		save_flags(flags); cli();
		info->curregs[5] &= ~(DTR | RTS);
		write_zsreg(info->zs_channel, 5, info->curregs[5]);
		restore_flags(flags);
	}
}

static void zs_unthrottle(struct tty_struct * tty)
{
	struct mac_serial *info = (struct mac_serial *)tty->driver_data;
	unsigned long flags;
#ifdef SERIAL_DEBUG_THROTTLE
	char	buf[64];
	
	printk("unthrottle %s: %d....\n", _tty_name(tty, buf),
	       tty->ldisc.chars_in_buffer(tty));
#endif

	if (zs_serial_paranoia_check(info, tty->device, "zs_unthrottle"))
		return;
	
	if (I_IXOFF(tty)) {
		save_flags(flags); cli();
		if (info->x_char)
			info->x_char = 0;
		else {
			info->x_char = START_CHAR(tty);
			if (!info->tx_active)
				zs_transmit_chars(info);
		}
		restore_flags(flags);
	}

	if (C_CRTSCTS(tty)) {
		/* Assert RTS and DTR lines */
		save_flags(flags); cli();
		info->curregs[5] |= DTR | RTS;
		write_zsreg(info->zs_channel, 5, info->curregs[5]);
		restore_flags(flags);
	}
}

/*
 * ------------------------------------------------------------
 * rs_ioctl() and friends
 * ------------------------------------------------------------
 */

static int zs_get_serial_info(struct mac_serial * info,
			   struct serial_struct * retinfo)
{
	struct serial_struct tmp;
  
	if (!retinfo)
		return -EFAULT;
	memset(&tmp, 0, sizeof(tmp));
	tmp.type = info->type;
	tmp.line = info->line;
	tmp.port = info->port;
	tmp.irq = info->irq;
	tmp.flags = info->flags;
	tmp.baud_base = info->baud_base;
	tmp.close_delay = info->close_delay;
	tmp.closing_wait = info->closing_wait;
	tmp.custom_divisor = info->custom_divisor;
	memcpy_tofs(retinfo,&tmp,sizeof(*retinfo));

	return 0;
}

static int zs_set_serial_info(struct mac_serial * info,
			   struct serial_struct * new_info)
{
	struct serial_struct new_serial;
	struct mac_serial old_info;
	int 			retval = 0;

	if (!new_info)
		return -EFAULT;
	memcpy_fromfs(&new_serial,new_info,sizeof(new_serial));
	old_info = *info;

	if (!suser()) {
		if ((new_serial.baud_base != info->baud_base) ||
		    (new_serial.type != info->type) ||
		    (new_serial.close_delay != info->close_delay) ||
		    ((new_serial.flags & ~ZILOG_USR_MASK) !=
		     (info->flags & ~ZILOG_USR_MASK)))
			return -EPERM;
		info->flags = ((info->flags & ~ZILOG_USR_MASK) |
			       (new_serial.flags & ZILOG_USR_MASK));
		info->custom_divisor = new_serial.custom_divisor;
		goto check_and_exit;
	}

	if (info->count > 1)
		return -EBUSY;

	/*
	 * OK, past this point, all the error checking has been done.
	 * At this point, we start making changes.....
	 */

	info->baud_base = new_serial.baud_base;
	info->flags = ((info->flags & ~ZILOG_FLAGS) |
			(new_serial.flags & ZILOG_FLAGS));
	info->type = new_serial.type;
	info->close_delay = new_serial.close_delay;
	info->closing_wait = new_serial.closing_wait;

check_and_exit:
	retval = zs_startup(info);
	return retval;
}

/*
 * get_lsr_info - get line status register info
 *
 * Purpose: Let user call ioctl() to get info when the UART physically
 * 	    is emptied.  On bus types like RS485, the transmitter must
 * 	    release the bus after transmitting. This must be done when
 * 	    the transmit shift register is empty, not be done when the
 * 	    transmit holding register is empty.  This functionality
 * 	    allows an RS485 driver to be written in user space. 
 */
static int zs_get_lsr_info(struct mac_serial * info, unsigned int *value)
{
	unsigned char status;

	cli();
	status = read_zsreg(info->zs_channel, 0);
	sti();
	put_user(status,value);
	return 0;
}

static int zs_get_modem_info(struct mac_serial *info, unsigned int *value)
{
	unsigned char control, status;
	unsigned int result;

	cli();
	control = info->curregs[5];
	status = read_zsreg(info->zs_channel, 0);
	sti();
	result =  ((control & RTS) ? TIOCM_RTS: 0)
		| ((control & DTR) ? TIOCM_DTR: 0)
		| ((status  & DCD) ? TIOCM_CAR: 0)
		| ((status  & CTS) ? 0: TIOCM_CTS);
	put_user(result,value);
	return 0;
}

static int zs_set_modem_info(struct mac_serial *info, unsigned int cmd,
			  unsigned int *value)
{
	int error;
	unsigned int arg, bits;

	error = verify_area(VERIFY_READ, value, sizeof(int));
	if (error)
		return error;
	arg = get_user(value);
	bits = (arg & TIOCM_RTS? RTS: 0) + (arg & TIOCM_DTR? DTR: 0);
	cli();
	switch (cmd) {
	case TIOCMBIS:
		info->curregs[5] |= bits;
		break;
	case TIOCMBIC:
		info->curregs[5] &= ~bits;
		break;
	case TIOCMSET:
		info->curregs[5] = (info->curregs[5] & ~(DTR | RTS)) | bits;
		break;
	default:
		sti();
		return -EINVAL;
	}
	write_zsreg(info->zs_channel, 5, info->curregs[5]);
	sti();
	return 0;
}

/*
 * This routine sends a break character out the serial port.
 */
static void zs_send_break(struct mac_serial * info, int duration)
{
	if (!info->port)
		return;
	current->state = TASK_INTERRUPTIBLE;
	current->timeout = jiffies + duration;
	cli();
	info->curregs[5] |= SND_BRK;
	write_zsreg(info->zs_channel, 5, info->curregs[5]);
	schedule();
	info->curregs[5] &= ~SND_BRK;
	write_zsreg(info->zs_channel, 5, info->curregs[5]);
	sti();
}

static int zs_ioctl(struct tty_struct *tty, struct file * file,
		    unsigned int cmd, unsigned long arg)
{
	int error;
	struct mac_serial * info = (struct mac_serial *)tty->driver_data;
	int retval;

	if (zs_serial_paranoia_check(info, tty->device, "zs_ioctl"))
		return -ENODEV;

	if ((cmd != TIOCGSERIAL) && (cmd != TIOCSSERIAL) &&
	    (cmd != TIOCSERCONFIG) && (cmd != TIOCSERGWILD)  &&
	    (cmd != TIOCSERSWILD) && (cmd != TIOCSERGSTRUCT)) {
		if (tty->flags & (1 << TTY_IO_ERROR))
		    return -EIO;
	}
	
	switch (cmd) {
		case TCSBRK:	/* SVID version: non-zero arg --> no break */
			retval = tty_check_change(tty);
			if (retval)
				return retval;
			tty_wait_until_sent(tty, 0);
			if (!arg)
				zs_send_break(info, HZ/4);	/* 1/4 second */
			return 0;
		case TCSBRKP:	/* support for POSIX tcsendbreak() */
			retval = tty_check_change(tty);
			if (retval)
				return retval;
			tty_wait_until_sent(tty, 0);
			zs_send_break(info, arg ? arg*(HZ/10) : HZ/4);
			return 0;
		case TIOCGSOFTCAR:
			error = verify_area(VERIFY_WRITE, (void *) arg,sizeof(long));
			if (error)
				return error;
			put_user(C_CLOCAL(tty) ? 1 : 0, (int *) arg);
			return 0;
		case TIOCSSOFTCAR:
			error = verify_area(VERIFY_WRITE, (void *) arg,sizeof(long));
			if (error)
				return error;
			arg = get_user((int *) arg);
			tty->termios->c_cflag =
				((tty->termios->c_cflag & ~CLOCAL) |
				 (arg ? CLOCAL : 0));
			return 0;
		case TIOCMGET:
			error = verify_area(VERIFY_WRITE, (void *) arg,
				sizeof(unsigned int));
			if (error)
				return error;
			return zs_get_modem_info(info, (unsigned int *) arg);
		case TIOCMBIS:
		case TIOCMBIC:
		case TIOCMSET:
			return zs_set_modem_info(info, cmd, (unsigned int *) arg);
		case TIOCGSERIAL:
			error = verify_area(VERIFY_WRITE, (void *) arg,
						sizeof(struct serial_struct));
			if (error)
				return error;
			return zs_get_serial_info(info, (struct serial_struct *) arg);
		case TIOCSSERIAL:
			error = verify_area(VERIFY_READ, (void *) arg,
						sizeof(struct serial_struct));
			if (error)
				return error;
			return zs_set_serial_info(info,
					       (struct serial_struct *) arg);
		case TIOCSERGETLSR: /* Get line status register */
			error = verify_area(VERIFY_WRITE, (void *) arg,
				sizeof(unsigned int));
			if (error)
				return error;
			else
			    return zs_get_lsr_info(info, (unsigned int *) arg);

		case TIOCSERGSTRUCT:
			error = verify_area(VERIFY_WRITE, (void *) arg,
						sizeof(struct mac_serial));
			if (error)
				return error;
			memcpy_tofs((struct mac_serial *) arg,
				    info, sizeof(struct mac_serial));
			return 0;
			
		default:
			return -ENOIOCTLCMD;
		}
	return 0;
}

static void zs_set_termios(struct tty_struct *tty, struct termios *old_termios)
{
	struct mac_serial *info = (struct mac_serial *)tty->driver_data;
	int was_stopped;

	if (tty->termios->c_cflag == old_termios->c_cflag)
		return;
	was_stopped = info->tx_stopped;

	//zs_change_speed(info);

	if ((old_termios->c_cflag & CRTSCTS) &&
	     !(tty->termios->c_cflag & CRTSCTS)) {
		tty->hw_stopped = 0;
		zs_start(tty);
        }
}

/*
 * ------------------------------------------------------------
 * rs_close()
 * 
 * This routine is called when the serial port gets closed.
 * Wait for the last remaining data to be sent.
 * ------------------------------------------------------------
 */
static void zs_close(struct tty_struct *tty, struct file * filp)
{
	struct mac_serial * info = (struct mac_serial *)tty->driver_data;
	unsigned long flags;

	if (!info || zs_serial_paranoia_check(info, tty->device, "zs_close"))
		return;
	
	save_flags(flags); cli();
	
	if (tty_hung_up_p(filp)) {
		restore_flags(flags);
		return;
	}
	
#ifdef SERIAL_DEBUG_OPEN
	printk("zs_close ttys%d, count = %d\n", info->line, info->count);
#endif
	if ((tty->count == 1) && (info->count != 1)) {
		/*
		 * Uh, oh.  tty->count is 1, which means that the tty
		 * structure will be freed.  Info->count should always
		 * be one in these conditions.  If it's greater than
		 * one, we've got real problems, since it means the
		 * serial port won't be zs_shutdown.
		 */
		printk("zs_close: bad serial port count; tty->count is 1, "
		       "info->count is %d\n", info->count);
		info->count = 1;
	}
	if (--info->count < 0) {
		printk("zs_close: bad serial port count for ttys%d: %d\n",
		       info->line, info->count);
		info->count = 0;
	}
	if (info->count) {
		restore_flags(flags);
		return;
	}
	info->flags |= ZILOG_CLOSING;
	/*
	 * Save the termios structure, since this port may have
	 * separate termios for callout and dialin.
	 */
	if (info->flags & ZILOG_NORMAL_ACTIVE)
		info->normal_termios = *tty->termios;
	if (info->flags & ZILOG_CALLOUT_ACTIVE)
		info->callout_termios = *tty->termios;
	/*
	 * Now we wait for the transmit buffer to clear; and we notify 
	 * the line discipline to only process XON/XOFF characters.
	 */
	tty->closing = 1;
	if (info->closing_wait != ZILOG_CLOSING_WAIT_NONE)
		tty_wait_until_sent(tty, info->closing_wait);
	/*
	 * At this point we stop accepting input.  To do this, we
	 * disable the receive line status interrupts, and tell the
	 * interrupt driver to stop checking the data ready bit in the
	 * line status register.
	 */
	/** if (!info->iscons) ... **/
	info->curregs[3] &= ~RxENABLE;
	write_zsreg(info->zs_channel, 3, info->curregs[3]);
	info->curregs[1] &= ~(0x18);	/* disable any rx ints */
	write_zsreg(info->zs_channel, 1, info->curregs[1]);
	ZS_CLEARFIFO(info->zs_channel);

	zs_shutdown(info);
	if (tty->driver.flush_buffer)
		tty->driver.flush_buffer(tty);
	if (tty->ldisc.flush_buffer)
		tty->ldisc.flush_buffer(tty);
	tty->closing = 0;
	info->event = 0;
	info->tty = 0;
	if (tty->ldisc.num != ldiscs[N_TTY].num) {
		if (tty->ldisc.close)
			(tty->ldisc.close)(tty);
		tty->ldisc = ldiscs[N_TTY];
		tty->termios->c_line = N_TTY;
		if (tty->ldisc.open)
			(tty->ldisc.open)(tty);
	}
	if (info->blocked_open) {
		if (info->close_delay) {
			current->state = TASK_INTERRUPTIBLE;
			current->timeout = jiffies + info->close_delay;
			schedule();
		}
		wake_up_interruptible(&info->open_wait);
	}
	info->flags &= ~(ZILOG_NORMAL_ACTIVE|ZILOG_CALLOUT_ACTIVE|
			 ZILOG_CLOSING);
	wake_up_interruptible(&info->close_wait);
#ifdef SERIAL_DEBUG_OPEN
	printk("zs_close tty-%d exiting, count = %d\n", info->line, info->count);
#endif
	restore_flags(flags);
}

/*
 * zs_hangup() --- called by tty_hangup() when a hangup is signaled.
 */
void zs_hangup(struct tty_struct *tty)
{
	struct mac_serial * info = (struct mac_serial *)tty->driver_data;

	if (zs_serial_paranoia_check(info, tty->device, "zs_hangup"))
		return;

	if(info->is_cons)
		return;

#ifdef SERIAL_DEBUG_OPEN
	printk("zs_hangup<%p: tty-%d, count = %d bye\n",
	       __builtin_return_address(0), info->line, info->count);
#endif

	zs_flush_buffer(tty);
	zs_shutdown(info);
	info->event = 0;
	info->count = 0;
	info->flags &= ~(ZILOG_NORMAL_ACTIVE|ZILOG_CALLOUT_ACTIVE);
	info->tty = 0;
	wake_up_interruptible(&info->open_wait);
}

/*
 * ------------------------------------------------------------
 * rs_open() and friends
 * ------------------------------------------------------------
 */
static int zs_block_til_ready(struct tty_struct *tty, struct file * filp,
			      struct mac_serial *info)
{
	struct wait_queue wait = { current, NULL };
	int		retval;
	int		do_clocal = 0;
	unsigned char	r0;

	/*
	 * If the device is in the middle of being closed, then block
	 * until it's done, and then try again.
	 */
	if (info->flags & ZILOG_CLOSING) {
		interruptible_sleep_on(&info->close_wait);
#ifdef SERIAL_DO_RESTART
		if (info->flags & ZILOG_HUP_NOTIFY)
			return -EAGAIN;
		else
			return -ERESTARTSYS;
#else
		return -EAGAIN;
#endif
	}

	/*
	 * If this is a callout device, then just make sure the normal
	 * device isn't being used.
	 */
	if (tty->driver.subtype == SERIAL_TYPE_CALLOUT) {
		if (info->flags & ZILOG_NORMAL_ACTIVE)
			return -EBUSY;
		if ((info->flags & ZILOG_CALLOUT_ACTIVE) &&
		    (info->flags & ZILOG_SESSION_LOCKOUT) &&
		    (info->session != current->session))
		    return -EBUSY;
		if ((info->flags & ZILOG_CALLOUT_ACTIVE) &&
		    (info->flags & ZILOG_PGRP_LOCKOUT) &&
		    (info->pgrp != current->pgrp))
		    return -EBUSY;
		info->flags |= ZILOG_CALLOUT_ACTIVE;
		return 0;
	}
	
	/*
	 * If non-blocking mode is set, or the port is not enabled,
	 * then make the check up front and then exit.
	 */
	if ((filp->f_flags & O_NONBLOCK) ||
	    (tty->flags & (1 << TTY_IO_ERROR))) {
		if (info->flags & ZILOG_CALLOUT_ACTIVE)
			return -EBUSY;
		info->flags |= ZILOG_NORMAL_ACTIVE;
		return 0;
	}

	if (info->flags & ZILOG_CALLOUT_ACTIVE) {
		if (info->normal_termios.c_cflag & CLOCAL)
			do_clocal = 1;
	} else {
		if (tty->termios->c_cflag & CLOCAL)
			do_clocal = 1;
	}
	
	/*
	 * Block waiting for the carrier detect and the line to become
	 * free (i.e., not in use by the callout).  While we are in
	 * this loop, info->count is dropped by one, so that
	 * zs_close() knows when to free things.  We restore it upon
	 * exit, either normal or abnormal.
	 */
	retval = 0;
	add_wait_queue(&info->open_wait, &wait);
#ifdef SERIAL_DEBUG_OPEN
	printk("zs_block_til_ready before block: ttys%d, count = %d\n",
	       info->line, info->count);
#endif
	cli();
	if(!tty_hung_up_p(filp))
		info->count--;
	sti();
	info->blocked_open++;
	while (1) {
		cli();
		if (!(info->flags & ZILOG_CALLOUT_ACTIVE))
			zs_rtsdtr(info, 1);
		sti();
		current->state = TASK_INTERRUPTIBLE;
		if (tty_hung_up_p(filp) ||
		    !(info->flags & ZILOG_INITIALIZED)) {
#ifdef SERIAL_DEBUG_OPEN
			printk("zs_block_til_ready hup-ed: ttys%d, count = %d\n",
			       info->line, info->count);
#endif
#ifdef SERIAL_DO_RESTART
			if (info->flags & ZILOG_HUP_NOTIFY)
				retval = -EAGAIN;
			else
				retval = -ERESTARTSYS;	
#else
			retval = -EAGAIN;
#endif
			break;
		}

		cli();
		r0 = read_zsreg(info->zs_channel, R0);
		sti();
		if (!(info->flags & ZILOG_CALLOUT_ACTIVE) &&
		    !(info->flags & ZILOG_CLOSING) &&
		    (do_clocal || (DCD & r0)))
			break;
		if (current->signal & ~current->blocked) {
			retval = -ERESTARTSYS;
			break;
		}
#ifdef SERIAL_DEBUG_OPEN
		printk("zs_block_til_ready blocking: ttys%d, count = %d\n",
		       info->line, info->count);
#endif
		schedule();
	}
	current->state = TASK_RUNNING;
	remove_wait_queue(&info->open_wait, &wait);
	if (!tty_hung_up_p(filp))
		info->count++;
	info->blocked_open--;
#ifdef SERIAL_DEBUG_OPEN
	printk("zs_block_til_ready after blocking: ttys%d, count = %d\n",
	       info->line, info->count);
#endif
	if (retval)
		return retval;
	info->flags |= ZILOG_NORMAL_ACTIVE;
	return 0;
}

/*
 * This routine is called whenever a serial port is opened.  It
 * enables interrupts for a serial port, linking in its ZILOG structure into
 * the IRQ chain.   It also performs the serial-specific
 * initialization for the tty structure.
 */
int zs_open(struct tty_struct *tty, struct file * filp)
{
	struct mac_serial	*info;
	int 			retval, line;

	line = MINOR(tty->device) - tty->driver.minor_start;
	if ((line < 0) || (line >= zs_channels_found))
		return -ENODEV;
	info = zs_soft + line;

	/* Is the kgdb running over this line? */
	if (info->kgdb_channel)
		return -ENODEV;
	if (zs_serial_paranoia_check(info, tty->device, "zs_open"))
		return -ENODEV;
#ifdef SERIAL_DEBUG_OPEN
	printk("zs_open %s%d, count = %d\n", tty->driver.name, info->line,
	       info->count);
#endif
	if (info->tty != 0 && info->tty != tty) {
		/* Never happen? */
		printk("zs_open %s%d, tty overwrite.\n", tty->driver.name, info->line);
		return -EBUSY;
	}
	info->count++;
	tty->driver_data = info;
	info->tty = tty;

	/*
	 * Start up serial port
	 */
	retval = zs_startup(info);
	if (retval)
		return retval;

	retval = zs_block_til_ready(tty, filp, info);
	if (retval) {
#ifdef SERIAL_DEBUG_OPEN
		printk("zs_open returning after zs_block_til_ready with %d\n",
		       retval);
#endif
		return retval;
	}

	if ((info->count == 1) && (info->flags & ZILOG_SPLIT_TERMIOS)) {
		if (tty->driver.subtype == SERIAL_TYPE_NORMAL)
			*tty->termios = info->normal_termios;
		else 
			*tty->termios = info->callout_termios;
		zs_change_speed(info);
	}

	info->session = current->session;
	info->pgrp = current->pgrp;

#ifdef SERIAL_DEBUG_OPEN
	printk("zs_open ttys%d successful...", info->line);
#endif
	return 0;
}

/* Finally, routines used to initialize the serial driver. */

static void zs_show_serial_version(void)
{
	char *revision = "$Revision: 1.17 $";
	char *version, *p;

	version = strchr(revision, ' ');
	p = strchr(++version, ' ');
	*p = '\0';
	printk("Sparc Zilog8530 serial driver version %s\n", version);
	*p = ' ';
}

/* Ask the PROM how many Z8530s we have and initialize their zs_channels */
/* We only have a single interface; the second interface is not wired. */
static void zs_probe_sccs(void)
{
	struct mac_serial **pp;
	int n;

	n = 0;
	pp = &zs_chain;
	zs_channels[n].control = (volatile unsigned char *) 0xbc800001;
	zs_channels[n].data = (volatile unsigned char *) 0xbc800003;
	zs_soft[n].zs_channel = &zs_channels[n];
	zs_soft[n].irq = 4;
	zs_soft[n].zs_chan_a = &zs_channels[n];

	*pp = &zs_soft[n];
	pp = &zs_soft[n].zs_next;
	n++;
	*pp = 0;
	zs_channels_found = n;
}

static int rs_init_zilog(void)
{
	int channel, i;
	unsigned long flags;
	struct mac_serial *info;

	/* Setup base handler, and timer table. */
	init_bh(SERIAL_BH, zs_do_serial_bh);
	timer_table[RS_TIMER].fn = zs_timer;
	timer_table[RS_TIMER].expires = 0;

	/* Find out how many Z8530 SCCs we have */
	if (zs_chain == 0)
		zs_probe_sccs();

	zs_show_serial_version();

	/* Initialize the tty_driver structure */
	/* Not all of this is exactly right for us. */

	memset(&serial_driver, 0, sizeof(struct tty_driver));
	serial_driver.magic = TTY_DRIVER_MAGIC;
	serial_driver.name = "ttyS";
	serial_driver.major = TTY_MAJOR;
	serial_driver.minor_start = 64;
	serial_driver.num = zs_channels_found;
	serial_driver.type = TTY_DRIVER_TYPE_SERIAL;
	serial_driver.subtype = SERIAL_TYPE_NORMAL;
	serial_driver.init_termios = tty_std_termios;

	serial_driver.init_termios.c_cflag =
		B115200 | CS8 | CREAD | HUPCL | CLOCAL;
	serial_driver.flags = TTY_DRIVER_REAL_RAW;
	serial_driver.refcount = &serial_refcount;
	serial_driver.table = zs_serial_table;
	serial_driver.termios = zs_serial_termios;
	serial_driver.termios_locked = zs_serial_termios_locked;

	serial_driver.open = zs_open;
	serial_driver.close = zs_close;
	serial_driver.write = zs_write;
	serial_driver.flush_chars = zs_flush_chars;
	serial_driver.write_room = zs_write_room;
	serial_driver.chars_in_buffer = zs_chars_in_buffer;
	serial_driver.flush_buffer = zs_flush_buffer;
	serial_driver.ioctl = zs_ioctl;
	serial_driver.throttle = zs_throttle;
	serial_driver.unthrottle = zs_unthrottle;
	serial_driver.set_termios = zs_set_termios;
	serial_driver.stop = zs_stop;
	serial_driver.start = zs_start;
	serial_driver.hangup = zs_hangup;

	/*
	 * The callout device is just like normal device except for
	 * major number and the subtype code.
	 */
	callout_driver = serial_driver;
	callout_driver.name = "cua";
	callout_driver.major = TTYAUX_MAJOR;
	callout_driver.subtype = SERIAL_TYPE_CALLOUT;

	if (tty_register_driver(&serial_driver))
		panic("Couldn't register serial driver\n");
	if (tty_register_driver(&callout_driver))
		panic("Couldn't register callout driver\n");

	save_flags(flags); cli();

	for (channel = 0; channel < zs_channels_found; ++channel) {
		zs_soft[channel].clk_divisor = 16;
		zs_soft[channel].zs_baud = get_zsbaud(&zs_soft[channel]);

		/* If console serial line, then enable interrupts. */
		if (zs_soft[channel].is_cons) {
			write_zsreg(zs_soft[channel].zs_channel, R1,
				    (EXT_INT_ENAB | INT_ALL_Rx | TxINT_ENAB));
			write_zsreg(zs_soft[channel].zs_channel, R9,
				    (NV | MIE));
		}
		/* If this is the kgdb line, enable interrupts because we
		 * now want to receive the 'control-c' character from the
		 * client attached to us asynchronously.
		 */
		if (zs_soft[channel].kgdb_channel)
			zs_kgdb_chaninit(&zs_soft[channel], 1,
				      zs_soft[channel].zs_baud);
	}

	for (info = zs_chain, i = 0; info; info = info->zs_next, i++) {
		info->magic = SERIAL_MAGIC;
		info->port = (int) info->zs_channel->control;
		info->line = i;
		info->tty = 0;
		info->custom_divisor = 16;
		info->close_delay = 50;
		info->closing_wait = 3000;
		info->x_char = 0;
		info->event = 0;
		info->count = 0;
		info->blocked_open = 0;
		info->tqueue.routine = zs_do_softint;
		info->tqueue.data = info;
		info->tqueue_hangup.routine = zs_do_serial_hangup;
		info->tqueue_hangup.data = info;
		info->callout_termios =callout_driver.init_termios;
		info->normal_termios = serial_driver.init_termios;
		info->open_wait = 0;
		info->close_wait = 0;
		printk("tty%02d at 0x%08x (irq = %d)", info->line, 
		       info->port, info->irq);
		printk(" is a Z8530 ESCC\n");
	}

	if (request_irq(4, zs_interrupt, SA_INTERRUPT,"SCC", &zs_soft[0]))
		panic("cobaltserial: can't get irq %d", 4);

	restore_flags(flags);
	return 0;
}

/* ======================== 16550-specific portion of the driver ========================== */

#define SERIAL_PARANOIA_CHECK
#define SERIAL_DO_RESTART

#define RS_STROBE_TIME (10*HZ)
#define RS_ISR_PASS_LIMIT 256

#define IRQ_T(info) SA_INTERRUPT
#define BASE_BAUD ( 18432000 / 16 )
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST )

static struct async_struct *IRQ_ports[16];
static int IRQ_timeout[16];

static void ns16550_autoconfig(struct async_struct *info);
static void ns16550_change_speed(struct async_struct *info);

/* Detected in rs_init_16550() */
static int ns16550_modem_attached = 0;

#define C_P(card,port) (((card)<<6|(port)<<3) + 1)

struct async_struct ns16550_table[] = {
	/*   UART CLK   PORT        IRQ FLAGS        */
	{ 0, BASE_BAUD, 0x1c800000, 7,  STD_COM_FLAGS },	/* ttyS0 */
};
#define NR_PORTS	(sizeof(ns16550_table)/sizeof(struct async_struct))

static struct tty_struct *ns16550_serial_table[NR_PORTS];
static struct termios *ns16550_serial_termios[NR_PORTS];
static struct termios *ns16550_serial_termios_locked[NR_PORTS];

static inline int ns16550_serial_paranoia_check(struct async_struct *info,
						kdev_t device, const char *routine)
{
#ifdef SERIAL_PARANOIA_CHECK
	static const char *badmagic =
		"Warning: bad magic number for serial struct (%s) in %s\n";
	static const char *badinfo =
		"Warning: null async_struct for (%s) in %s\n";

	if (!info) {
		printk(badinfo, kdevname(device), routine);
		return 1;
	}
	if (info->magic != SERIAL_MAGIC) {
		printk(badmagic, kdevname(device), routine);
		return 1;
	}
#endif
	return 0;
}

/*
 * This is used to figure out the divisor speeds and the timeouts
 */
static int ns16550_baud_table[] = {
	0, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800,
	9600, 19200, 38400, 57600, 115200, 0 };

static inline unsigned int serial_in(struct async_struct *info, int offset)
{
	return inb(info->port + offset);
}

static inline unsigned int serial_inp(struct async_struct *info, int offset)
{
	return inb(info->port + offset);
}

static inline void serial_out(struct async_struct *info, int offset, int value)
{
	outb(value, info->port+offset);
}

static inline void serial_outp(struct async_struct *info, int offset,
			       int value)
{
	outb(value, info->port+offset);
}

static void ns16550_stop(struct tty_struct *tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	unsigned long flags;

	if (ns16550_serial_paranoia_check(info, tty->device, "ns16550_stop"))
		return;
	
	save_flags(flags); cli();
	if (info->IER & UART_IER_THRI) {
		info->IER &= ~UART_IER_THRI;
		serial_out(info, UART_IER, info->IER);
	}
	restore_flags(flags);
}

static void ns16550_start(struct tty_struct *tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	unsigned long flags;
	
	if (ns16550_serial_paranoia_check(info, tty->device, "ns16550_start"))
		return;
	
	save_flags(flags); cli();
	if (info->xmit_cnt && info->xmit_buf && !(info->IER & UART_IER_THRI)) {
		info->IER |= UART_IER_THRI;
		serial_out(info, UART_IER, info->IER);
	}
	restore_flags(flags);
}

static _INLINE_ void ns16550_sched_event(struct async_struct *info,
					 int event)
{
	info->event |= 1 << event;
	queue_task_irq_off(&info->tqueue, &tq_serial);
	mark_bh(SERIAL_BH);
}

static _INLINE_ void ns16550_receive_chars(struct async_struct *info,
					   int *status)
{
	struct tty_struct *tty = info->tty;
	unsigned char ch;
	int ignored = 0;

	do {
		ch = serial_inp(info, UART_RX);
		if (*status & info->ignore_status_mask) {
			if (++ignored > 100)
				break;
			goto ignore_char;
		}
		if (tty->flip.count >= TTY_FLIPBUF_SIZE)
			break;
		tty->flip.count++;
		if (*status & (UART_LSR_BI)) {
#ifdef SERIAL_DEBUG_INTR
			printk("handling break....");
#endif
			*tty->flip.flag_buf_ptr++ = TTY_BREAK;
			if (info->flags & ASYNC_SAK)
				do_SAK(tty);
		} else if (*status & UART_LSR_PE)
			*tty->flip.flag_buf_ptr++ = TTY_PARITY;
		else if (*status & UART_LSR_FE)
			*tty->flip.flag_buf_ptr++ = TTY_FRAME;
		else if (*status & UART_LSR_OE) 
			*tty->flip.flag_buf_ptr++ = TTY_OVERRUN;
		else
			*tty->flip.flag_buf_ptr++ = 0;
		*tty->flip.char_buf_ptr++ = ch;
#ifndef CONFIG_COBALT_28
		/* Serial console must wakeup sleepers on keypress_wait. */
		wake_up(&keypress_wait);
#endif
	ignore_char:
		*status = serial_inp(info, UART_LSR) & info->read_status_mask;
	} while (*status & UART_LSR_DR);
	queue_task_irq_off(&tty->flip.tqueue, &tq_timer);
#ifdef SERIAL_DEBUG_INTR
	printk("DR...");
#endif
}

static _INLINE_ void ns16550_transmit_chars(struct async_struct *info, int *intr_done)
{
	int count;
	
	if (info->x_char) {
		serial_outp(info, UART_TX, info->x_char);
		info->x_char = 0;
		if (intr_done)
			*intr_done = 0;
		return;
	}
	if ((info->xmit_cnt <= 0) || info->tty->stopped ||
	    info->tty->hw_stopped) {
		info->IER &= ~UART_IER_THRI;
		serial_out(info, UART_IER, info->IER);
		return;
	}
	
	count = info->xmit_fifo_size;
	do {
		serial_out(info, UART_TX, info->xmit_buf[info->xmit_tail++]);
		info->xmit_tail = info->xmit_tail & (SERIAL_XMIT_SIZE-1);
		if (--info->xmit_cnt <= 0)
			break;
	} while (--count > 0);
	
	if (info->xmit_cnt < WAKEUP_CHARS)
		ns16550_sched_event(info, RS_EVENT_WRITE_WAKEUP);

#ifdef SERIAL_DEBUG_INTR
	printk("THRE...");
#endif
	if (intr_done)
		*intr_done = 0;

	if (info->xmit_cnt <= 0) {
		info->IER &= ~UART_IER_THRI;
		serial_out(info, UART_IER, info->IER);
	}
}

static _INLINE_ void ns16550_check_modem_status(struct async_struct *info)
{
	int	status;
	
	status = serial_in(info, UART_MSR);

	if (status & UART_MSR_ANY_DELTA) {
		/* update input line counters */
		if (status & UART_MSR_TERI)
			info->icount.rng++;
		if (status & UART_MSR_DDSR)
			info->icount.dsr++;
		if (status & UART_MSR_DDCD)
			info->icount.dcd++;
		if (status & UART_MSR_DCTS)
			info->icount.cts++;
		wake_up_interruptible(&info->delta_msr_wait);
	}

	if ((info->flags & ASYNC_CHECK_CD) && (status & UART_MSR_DDCD)) {
#if (defined(SERIAL_DEBUG_OPEN) || defined(SERIAL_DEBUG_INTR))
		printk("ttys%d CD now %s...", info->line,
		       (status & UART_MSR_DCD) ? "on" : "off");
#endif		
		if (status & UART_MSR_DCD)
			wake_up_interruptible(&info->open_wait);
		else if (!((info->flags & ASYNC_CALLOUT_ACTIVE) &&
			   (info->flags & ASYNC_CALLOUT_NOHUP))) {
#ifdef SERIAL_DEBUG_OPEN
			printk("scheduling hangup...");
#endif
			queue_task_irq_off(&info->tqueue_hangup,
					   &tq_scheduler);
		}
	}
	if (info->flags & ASYNC_CTS_FLOW) {
		if (info->tty->hw_stopped) {
			if (status & UART_MSR_CTS) {
#if (defined(SERIAL_DEBUG_INTR) || defined(SERIAL_DEBUG_FLOW))
				printk("CTS tx start...");
#endif
				info->tty->hw_stopped = 0;
				info->IER |= UART_IER_THRI;
				serial_out(info, UART_IER, info->IER);
				ns16550_sched_event(info, RS_EVENT_WRITE_WAKEUP);
				return;
			}
		} else {
			if (!(status & UART_MSR_CTS)) {
#if (defined(SERIAL_DEBUG_INTR) || defined(SERIAL_DEBUG_FLOW))
				printk("CTS tx stop...");
#endif
				info->tty->hw_stopped = 1;
				info->IER &= ~UART_IER_THRI;
				serial_out(info, UART_IER, info->IER);
			}
		}
	}
}

/*
 * This is the 16550 serial driver's generic interrupt routine
 */
static void ns16550_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	int status;
	struct async_struct * info;
	int pass_counter = 0;
	struct async_struct *end_mark = 0;

#ifdef SERIAL_DEBUG_INTR
	printk("ns16550_interrupt(%d)...", irq);
#endif
	
	info = IRQ_ports[irq];
	if (!info)
		return;
	
	do {
		if (!info->tty ||
		    (serial_in(info, UART_IIR) & UART_IIR_NO_INT)) {
			if (!end_mark)
				end_mark = info;
			goto next;
		}
		end_mark = 0;

		info->last_active = jiffies;

		status = serial_inp(info, UART_LSR) & info->read_status_mask;
#ifdef SERIAL_DEBUG_INTR
		printk("status = %x...", status);
#endif
		if (status & UART_LSR_DR)
			ns16550_receive_chars(info, &status);
		ns16550_check_modem_status(info);
		if (status & UART_LSR_THRE)
			ns16550_transmit_chars(info, 0);

	next:
		info = info->next_port;
		if (!info) {
			info = IRQ_ports[irq];
			if (pass_counter++ > RS_ISR_PASS_LIMIT) {
#if 0
				printk("rs loop break\n");
#endif
				break; 	/* Prevent infinite loops */
			}
			continue;
		}
	} while (end_mark != info);
#ifdef SERIAL_DEBUG_INTR
	printk("end.\n");
#endif
}

/*
 * This is the serial driver's interrupt routine for a single port
 */
static void ns16550_interrupt_single(int irq, void *dev_id, struct pt_regs * regs)
{
	int status;
	int pass_counter = 0;
	struct async_struct * info;
	
#ifdef SERIAL_DEBUG_INTR
	printk("ns16550_interrupt_single(%d)...", irq);
#endif
	
	info = IRQ_ports[irq];
	if (!info || !info->tty)
		return;

	do {
		status = serial_inp(info, UART_LSR) & info->read_status_mask;
#ifdef SERIAL_DEBUG_INTR
		printk("status = %x...", status);
#endif
		if (status & UART_LSR_DR)
			ns16550_receive_chars(info, &status);
		ns16550_check_modem_status(info);
		if (status & UART_LSR_THRE)
			ns16550_transmit_chars(info, 0);
		if (pass_counter++ > RS_ISR_PASS_LIMIT) {
#if 0
			printk("rs_single loop break.\n");
#endif
			break;
		}
	} while (!(serial_in(info, UART_IIR) & UART_IIR_NO_INT));
	info->last_active = jiffies;
#ifdef SERIAL_DEBUG_INTR
	printk("end.\n");
#endif
}

static void ns16550_do_serial_bh(void)
{
	run_task_queue(&tq_serial);
}

static void ns16550_do_softint(void *private_)
{
	struct async_struct	*info = (struct async_struct *) private_;
	struct tty_struct	*tty;
	
	tty = info->tty;
	if (!tty)
		return;

	if (clear_bit(RS_EVENT_WRITE_WAKEUP, &info->event)) {
		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
		    tty->ldisc.write_wakeup)
			(tty->ldisc.write_wakeup)(tty);
		wake_up_interruptible(&tty->write_wait);
	}
}

static void ns16550_do_serial_hangup(void *private_)
{
	struct async_struct	*info = (struct async_struct *) private_;
	struct tty_struct	*tty;
	
	tty = info->tty;
	if (!tty)
		return;

	tty_hangup(tty);
}

static void ns16550_timer(void)
{
	static unsigned long last_strobe = 0;
	struct async_struct *info;
	unsigned int	i;

	if ((jiffies - last_strobe) >= RS_STROBE_TIME) {
		for (i=1; i < 16; i++) {
			info = IRQ_ports[i];
			if (!info)
				continue;
			cli();
			ns16550_interrupt_single(i, NULL, NULL);
			sti();
		}
	}
	last_strobe = jiffies;
	timer_table[RS_TIMER].expires = jiffies + RS_STROBE_TIME;
	timer_active |= 1 << RS_TIMER;

	if (IRQ_ports[0]) {
		cli();
		ns16550_interrupt(0, NULL, NULL);
		sti();

		timer_table[RS_TIMER].expires = jiffies + IRQ_timeout[0] - 2;
	}
}

static void ns16550_figure_IRQ_timeout(int irq)
{
	struct	async_struct	*info;
	int	timeout = 60*HZ;	/* 60 seconds === a long time :-) */

	info = IRQ_ports[irq];
	if (!info) {
		IRQ_timeout[irq] = 60*HZ;
		return;
	}
	while (info) {
		if (info->timeout < timeout)
			timeout = info->timeout;
		info = info->next_port;
	}
	if (!irq)
		timeout = timeout / 2;
	IRQ_timeout[irq] = timeout ? timeout : 1;
}

static int ns16550_startup(struct async_struct * info)
{
	unsigned long flags;
	int	retval;
	void (*handler)(int, void *, struct pt_regs *);
	unsigned long page;

	page = get_free_page(GFP_KERNEL);
	if (!page)
		return -ENOMEM;

	save_flags(flags); cli();

	if (info->flags & ASYNC_INITIALIZED) {
		free_page(page);
		restore_flags(flags);
		return 0;
	}

	if (!info->port || !info->type) {
		if (info->tty)
			set_bit(TTY_IO_ERROR, &info->tty->flags);
		free_page(page);
		restore_flags(flags);
		return 0;
	}
	if (info->xmit_buf)
		free_page(page);
	else
		info->xmit_buf = (unsigned char *) page;

#ifdef SERIAL_DEBUG_OPEN
	printk("starting up ttys%d (irq %d)...", info->line, info->irq);
#endif

	/*
	 * Clear the FIFO buffers and disable them
	 * (they will be reenabled in ns16550_change_speed())
	 */
	if (info->type == PORT_16650) {
		serial_outp(info, UART_FCR, (UART_FCR_CLEAR_RCVR |
					     UART_FCR_CLEAR_XMIT));
		info->xmit_fifo_size = 1; /* disabled for now */
	} else if (info->type == PORT_16550A) {
		serial_outp(info, UART_FCR, (UART_FCR_CLEAR_RCVR |
					     UART_FCR_CLEAR_XMIT));
		info->xmit_fifo_size = 16;
	} else
		info->xmit_fifo_size = 1;

	/*
	 * At this point there's no way the LSR could still be 0xFF;
	 * if it is, then bail out, because there's likely no UART
	 * here.
	 */
	if (serial_inp(info, UART_LSR) == 0xff) {
		restore_flags(flags);
		if (suser()) {
			if (info->tty)
				set_bit(TTY_IO_ERROR, &info->tty->flags);
			return 0;
		} else
			return -ENODEV;
	}
	
	/*
	 * Allocate the IRQ if necessary
	 */
	if (info->irq && (!IRQ_ports[info->irq] ||
			  !IRQ_ports[info->irq]->next_port)) {
		handler = ns16550_interrupt_single;
		retval = request_irq(info->irq, handler, IRQ_T(info),
				     "serial", NULL);
		if (retval) {
			restore_flags(flags);
			if (suser()) {
				if (info->tty)
					set_bit(TTY_IO_ERROR,
						&info->tty->flags);
				return 0;
			} else
				return retval;
		}
	}

	/*
	 * Clear the interrupt registers.
	 */
     /* (void) serial_inp(info, UART_LSR); */   /* (see above) */
	(void) serial_inp(info, UART_RX);
	(void) serial_inp(info, UART_IIR);
	(void) serial_inp(info, UART_MSR);

	/*
	 * Now, initialize the UART 
	 */
	serial_outp(info, UART_LCR, UART_LCR_WLEN8);	/* reset DLAB */
	info->MCR = UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2;
	info->MCR_noint = UART_MCR_DTR | UART_MCR_RTS;
#if defined(__alpha__) && !defined(CONFIG_PCI)
	info->MCR |= UART_MCR_OUT1 | UART_MCR_OUT2;
	info->MCR_noint |= UART_MCR_OUT1 | UART_MCR_OUT2;
#endif
	if (info->irq == 0)
		info->MCR = info->MCR_noint;
	serial_outp(info, UART_MCR, info->MCR);
	
	/*
	 * Finally, enable interrupts
	 */
	info->IER = UART_IER_MSI | UART_IER_RLSI | UART_IER_RDI;
	serial_outp(info, UART_IER, info->IER);	/* enable interrupts */
	
	/*
	 * And clear the interrupt registers again for luck.
	 */
	(void)serial_inp(info, UART_LSR);
	(void)serial_inp(info, UART_RX);
	(void)serial_inp(info, UART_IIR);
	(void)serial_inp(info, UART_MSR);

	if (info->tty)
		clear_bit(TTY_IO_ERROR, &info->tty->flags);
	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;

	/*
	 * Insert serial port into IRQ chain.
	 */
	info->prev_port = 0;
	info->next_port = IRQ_ports[info->irq];
	if (info->next_port)
		info->next_port->prev_port = info;
	IRQ_ports[info->irq] = info;
	ns16550_figure_IRQ_timeout(info->irq);

	/*
	 * Set up serial timers...
	 */
	timer_table[RS_TIMER].expires = jiffies + 2*HZ/100;
	timer_active |= 1 << RS_TIMER;

	/*
	 * and set the speed of the serial port
	 */
	ns16550_change_speed(info);

	info->flags |= ASYNC_INITIALIZED;
	restore_flags(flags);
	return 0;
}

static void ns16550_shutdown(struct async_struct * info)
{
	unsigned long	flags;
	int		retval;

	if (!(info->flags & ASYNC_INITIALIZED))
		return;

#ifdef SERIAL_DEBUG_OPEN
	printk("Shutting down serial port %d (irq %d)....", info->line,
	       info->irq);
#endif
	
	save_flags(flags); cli(); /* Disable interrupts */

	/*
	 * clear delta_msr_wait queue to avoid mem leaks: we may free the irq
	 * here so the queue might never be waken up
	 */
	wake_up_interruptible(&info->delta_msr_wait);
	
	/*
	 * First unlink the serial port from the IRQ chain...
	 */
	if (info->next_port)
		info->next_port->prev_port = info->prev_port;
	if (info->prev_port)
		info->prev_port->next_port = info->next_port;
	else
		IRQ_ports[info->irq] = info->next_port;
	ns16550_figure_IRQ_timeout(info->irq);
	
	/*
	 * Free the IRQ, if necessary
	 */
	if (info->irq && (!IRQ_ports[info->irq] ||
			  !IRQ_ports[info->irq]->next_port)) {
		if (IRQ_ports[info->irq]) {
			free_irq(info->irq, NULL);
			retval = request_irq(info->irq, ns16550_interrupt_single,
					     IRQ_T(info), "serial", NULL);
			
			if (retval)
				printk("serial shutdown: request_irq: error %d"
				       "  Couldn't reacquire IRQ.\n", retval);
		} else
			free_irq(info->irq, NULL);
	}

	if (info->xmit_buf) {
		free_page((unsigned long) info->xmit_buf);
		info->xmit_buf = 0;
	}

	info->IER = 0;
	serial_outp(info, UART_IER, 0x00);	/* disable all intrs */
	
	if (!info->tty || (info->tty->termios->c_cflag & HUPCL)) {
		info->MCR &= ~(UART_MCR_DTR|UART_MCR_RTS);
		info->MCR_noint &= ~(UART_MCR_DTR|UART_MCR_RTS);
	}
	serial_outp(info, UART_MCR, info->MCR_noint);

	/* disable FIFO's */	
	serial_outp(info, UART_FCR, (UART_FCR_CLEAR_RCVR |
				     UART_FCR_CLEAR_XMIT));
	(void)serial_in(info, UART_RX);    /* read data port to reset things */
	
	if (info->tty)
		set_bit(TTY_IO_ERROR, &info->tty->flags);
	
	info->flags &= ~ASYNC_INITIALIZED;
	restore_flags(flags);
}

static void ns16550_change_speed(struct async_struct *info)
{
	unsigned int port;
	int	quot = 0;
	unsigned cflag,cval,fcr;
	int	i;
	unsigned long flags;

	if (!info->tty || !info->tty->termios)
		return;
	cflag = info->tty->termios->c_cflag;
	if (!(port = info->port))
		return;
	i = cflag & CBAUD;
	if (i & CBAUDEX) {
		i &= ~CBAUDEX;
		if (i < 1 || i > 2) 
			info->tty->termios->c_cflag &= ~CBAUDEX;
		else
			i += 15;
	}
	if (i == 15) {
		if ((info->flags & ASYNC_SPD_MASK) == ASYNC_SPD_HI)
			i += 1;
		if ((info->flags & ASYNC_SPD_MASK) == ASYNC_SPD_VHI)
			i += 2;
		if ((info->flags & ASYNC_SPD_MASK) == ASYNC_SPD_CUST)
			quot = info->custom_divisor;
	}
	if (quot) {
		info->timeout = ((info->xmit_fifo_size*HZ*15*quot) /
				 info->baud_base) + 2;
	} else if (ns16550_baud_table[i] == 134) {
		quot = (2*info->baud_base / 269);
		info->timeout = (info->xmit_fifo_size*HZ*30/269) + 2;
	} else if (ns16550_baud_table[i]) {
		quot = info->baud_base / ns16550_baud_table[i];
		info->timeout = (info->xmit_fifo_size*HZ*15/ns16550_baud_table[i]) + 2;
	} else {
		quot = 0;
		info->timeout = 0;
	}
	if (quot) {
		info->MCR |= UART_MCR_DTR;
		info->MCR_noint |= UART_MCR_DTR;
		save_flags(flags); cli();
		serial_out(info, UART_MCR, info->MCR);
		restore_flags(flags);
	} else {
		info->MCR &= ~UART_MCR_DTR;
		info->MCR_noint &= ~UART_MCR_DTR;
		save_flags(flags); cli();
		serial_out(info, UART_MCR, info->MCR);
		restore_flags(flags);
		return;
	}
	/* byte size and parity */
	switch (cflag & CSIZE) {
	      case CS5: cval = 0x00; break;
	      case CS6: cval = 0x01; break;
	      case CS7: cval = 0x02; break;
	      case CS8: cval = 0x03; break;
	      default:  cval = 0x00; break;	/* too keep GCC shut... */
	}
	if (cflag & CSTOPB) {
		cval |= 0x04;
	}
	if (cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(cflag & PARODD))
		cval |= UART_LCR_EPAR;
	if (info->type == PORT_16550A) {
		if ((info->baud_base / quot) < 2400)
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
		else
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_8;
	} else if (info->type == PORT_16650) {
		/*
		 * On the 16650, we disable the FIFOs altogether
		 * because of a design bug in how the implement
		 * things.  We could support it by completely changing
		 * how we handle the interrupt driver, but not today....
		 *
		 * N.B.  Because there's no way to set a FIFO trigger
		 * at 1 char, we'd probably disable at speed below
		 * 2400 baud anyway...
		 */
		fcr = 0;
	} else
		fcr = 0;
	
	/* CTS flow control flag and modem status interrupts */
	info->IER &= ~UART_IER_MSI;
	if (cflag & CRTSCTS) {
		info->flags |= ASYNC_CTS_FLOW;
		info->IER |= UART_IER_MSI;
	} else
		info->flags &= ~ASYNC_CTS_FLOW;
	if (cflag & CLOCAL)
		info->flags &= ~ASYNC_CHECK_CD;
	else {
		info->flags |= ASYNC_CHECK_CD;
		info->IER |= UART_IER_MSI;
	}
	serial_out(info, UART_IER, info->IER);

	/*
	 * Set up parity check flag
	 */
#define RELEVANT_IFLAG(iflag) (iflag & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

	info->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (I_INPCK(info->tty))
		info->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (I_BRKINT(info->tty) || I_PARMRK(info->tty))
		info->read_status_mask |= UART_LSR_BI;
	
	info->ignore_status_mask = 0;
#if 0
	/* This should be safe, but for some broken bits of hardware... */
	if (I_IGNPAR(info->tty)) {
		info->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
		info->read_status_mask |= UART_LSR_PE | UART_LSR_FE;
	}
#endif
	if (I_IGNBRK(info->tty)) {
		info->ignore_status_mask |= UART_LSR_BI;
		info->read_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignore parity and break indicators, ignore 
		 * overruns too.  (For real raw support).
		 */
		if (I_IGNPAR(info->tty)) {
			info->ignore_status_mask |= UART_LSR_OE | \
				UART_LSR_PE | UART_LSR_FE;
			info->read_status_mask |= UART_LSR_OE | \
				UART_LSR_PE | UART_LSR_FE;
		}
	}
	save_flags(flags); cli();
	serial_outp(info, UART_LCR, cval | UART_LCR_DLAB);	/* set DLAB */
	serial_outp(info, UART_DLL, quot & 0xff);	/* LS of divisor */
	serial_outp(info, UART_DLM, quot >> 8);		/* MS of divisor */
	serial_outp(info, UART_LCR, cval);		/* reset DLAB */
	serial_outp(info, UART_FCR, fcr); 	/* set fcr */
	restore_flags(flags);
}

static void ns16550_put_char(struct tty_struct *tty, unsigned char ch)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	unsigned long flags;

	if (ns16550_serial_paranoia_check(info, tty->device, "rs_put_char"))
		return;

	if (!tty || !info->xmit_buf)
		return;

	save_flags(flags); cli();
	if (info->xmit_cnt >= SERIAL_XMIT_SIZE - 1) {
		restore_flags(flags);
		return;
	}

	info->xmit_buf[info->xmit_head++] = ch;
	info->xmit_head &= SERIAL_XMIT_SIZE-1;
	info->xmit_cnt++;
	restore_flags(flags);
}

static __inline__ void ns16550_cons_put_char(char ch, unsigned long ioaddr)
{
	char lsr;

	do {	lsr = inb(ioaddr + UART_LSR);
	} while ((lsr & (UART_LSR_TEMT | UART_LSR_THRE)) != (UART_LSR_TEMT | UART_LSR_THRE));
	outb(ch, ioaddr + UART_TX);
}

static void ns16550_console_print(const char *p)
{
	unsigned long port = 0x1c800000;
	char c, lsr, ier;

	/* If modem is attached, don't do anything. */
	if (ns16550_modem_attached != 0)
		return;

	ier = inb(port + UART_IER);
	outb(0x00, port + UART_IER);
	while((c = *(p++)) != 0) {
		if(c == '\n')
			ns16550_cons_put_char('\r', port);
		ns16550_cons_put_char(c, port);
	}

	do {	lsr = inb(port + UART_LSR);
	} while ((lsr & (UART_LSR_TEMT | UART_LSR_THRE)) != (UART_LSR_TEMT | UART_LSR_THRE));
	outb(ier, port + UART_IER);
}

static void ns16550_flush_chars(struct tty_struct *tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	unsigned long flags;
				
	if (ns16550_serial_paranoia_check(info, tty->device, "rs_flush_chars"))
		return;

	if (info->xmit_cnt <= 0 || tty->stopped || tty->hw_stopped ||
	    !info->xmit_buf)
		return;

	save_flags(flags); cli();
	info->IER |= UART_IER_THRI;
	serial_out(info, UART_IER, info->IER);
	restore_flags(flags);
}

static int ns16550_write(struct tty_struct * tty, int from_user,
			 const unsigned char *buf, int count)
{
	int	c, total = 0;
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	unsigned long flags;
				
	if (ns16550_serial_paranoia_check(info, tty->device, "rs_write"))
		return 0;

	if (!tty || !info->xmit_buf)
		return 0;
	    
	if (from_user)
		down(&tmp_buf_sem);
	save_flags(flags);
	while (1) {
		cli();		
		c = MIN(count, MIN(SERIAL_XMIT_SIZE - info->xmit_cnt - 1,
				   SERIAL_XMIT_SIZE - info->xmit_head));
		if (c <= 0)
			break;

		if (from_user) {
			memcpy_fromfs(tmp_buf, buf, c);
			c = MIN(c, MIN(SERIAL_XMIT_SIZE - info->xmit_cnt - 1,
				       SERIAL_XMIT_SIZE - info->xmit_head));
			memcpy(info->xmit_buf + info->xmit_head, tmp_buf, c);
		} else
			memcpy(info->xmit_buf + info->xmit_head, buf, c);
		info->xmit_head = (info->xmit_head + c) & (SERIAL_XMIT_SIZE-1);
		info->xmit_cnt += c;
		restore_flags(flags);
		buf += c;
		count -= c;
		total += c;
	}
	if (from_user)
		up(&tmp_buf_sem);
	if (info->xmit_cnt && !tty->stopped && !tty->hw_stopped &&
	    !(info->IER & UART_IER_THRI)) {
		info->IER |= UART_IER_THRI;
		serial_out(info, UART_IER, info->IER);
	}
	restore_flags(flags);
	return total;
}

static int ns16550_write_room(struct tty_struct *tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	int	ret;
				
	if (ns16550_serial_paranoia_check(info, tty->device, "rs_write_room"))
		return 0;
	ret = SERIAL_XMIT_SIZE - info->xmit_cnt - 1;
	if (ret < 0)
		ret = 0;
	return ret;
}

static int ns16550_chars_in_buffer(struct tty_struct *tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
				
	if (ns16550_serial_paranoia_check(info, tty->device, "rs_chars_in_buffer"))
		return 0;
	return info->xmit_cnt;
}

static void ns16550_flush_buffer(struct tty_struct *tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
				
	if (ns16550_serial_paranoia_check(info, tty->device, "rs_flush_buffer"))
		return;
	cli();
	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;
	sti();
	wake_up_interruptible(&tty->write_wait);
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
	    tty->ldisc.write_wakeup)
		(tty->ldisc.write_wakeup)(tty);
}

static void ns16550_throttle(struct tty_struct * tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
#ifdef SERIAL_DEBUG_THROTTLE
	char	buf[64];
	
	printk("throttle %s: %d....\n", _tty_name(tty, buf),
	       tty->ldisc.chars_in_buffer(tty));
#endif

	if (ns16550_serial_paranoia_check(info, tty->device, "rs_throttle"))
		return;
	
	if (I_IXOFF(tty))
		info->x_char = STOP_CHAR(tty);

	info->MCR &= ~UART_MCR_RTS;
	info->MCR_noint &= ~UART_MCR_RTS;
	cli();
	serial_out(info, UART_MCR, info->MCR);
	sti();
}

static void ns16550_unthrottle(struct tty_struct * tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
#ifdef SERIAL_DEBUG_THROTTLE
	char	buf[64];
	
	printk("unthrottle %s: %d....\n", _tty_name(tty, buf),
	       tty->ldisc.chars_in_buffer(tty));
#endif

	if (ns16550_serial_paranoia_check(info, tty->device, "rs_unthrottle"))
		return;
	
	if (I_IXOFF(tty)) {
		if (info->x_char)
			info->x_char = 0;
		else
			info->x_char = START_CHAR(tty);
	}
	info->MCR |= UART_MCR_RTS;
	info->MCR_noint |= UART_MCR_RTS;
	cli();
	serial_out(info, UART_MCR, info->MCR);
	sti();
}

static int ns16550_get_serial_info(struct async_struct * info,
				   struct serial_struct * retinfo)
{
	struct serial_struct tmp;
  
	if (!retinfo)
		return -EFAULT;
	memset(&tmp, 0, sizeof(tmp));
	tmp.type = info->type;
	tmp.line = info->line;
	tmp.port = info->port;
	tmp.irq = info->irq;
	tmp.flags = info->flags;
	tmp.baud_base = info->baud_base;
	tmp.close_delay = info->close_delay;
	tmp.closing_wait = info->closing_wait;
	tmp.custom_divisor = info->custom_divisor;
	tmp.hub6 = info->hub6;
	memcpy_tofs(retinfo,&tmp,sizeof(*retinfo));
	return 0;
}

static int ns16550_set_serial_info(struct async_struct * info,
				   struct serial_struct * new_info)
{
	struct serial_struct new_serial;
	struct async_struct old_info;
	unsigned int		i,change_irq,change_port;
	int 			retval = 0;

	if (!new_info)
		return -EFAULT;
	memcpy_fromfs(&new_serial,new_info,sizeof(new_serial));
	old_info = *info;

	change_irq = new_serial.irq != info->irq;
	change_port = (new_serial.port != info->port) || (new_serial.hub6 != info->hub6);

	if (!suser()) {
		if (change_irq || change_port ||
		    (new_serial.baud_base != info->baud_base) ||
		    (new_serial.type != info->type) ||
		    (new_serial.close_delay != info->close_delay) ||
		    ((new_serial.flags & ~ASYNC_USR_MASK) !=
		     (info->flags & ~ASYNC_USR_MASK)))
			return -EPERM;
		info->flags = ((info->flags & ~ASYNC_USR_MASK) |
			       (new_serial.flags & ASYNC_USR_MASK));
		info->custom_divisor = new_serial.custom_divisor;
		goto check_and_exit;
	}

	if (new_serial.irq == 2)
		new_serial.irq = 9;

	if ((new_serial.irq > 15) || (new_serial.port > 0xffff) ||
	    (new_serial.type < PORT_UNKNOWN) || (new_serial.type > PORT_MAX)) {
		return -EINVAL;
	}

	/* Make sure address is not already in use */
	if (new_serial.type) {
		for (i = 0 ; i < NR_PORTS; i++)
			if ((info != &ns16550_table[i]) &&
			    (ns16550_table[i].port == new_serial.port) &&
			    ns16550_table[i].type)
				return -EADDRINUSE;
	}

	if ((change_port || change_irq) && (info->count > 1))
		return -EBUSY;

	/*
	 * OK, past this point, all the error checking has been done.
	 * At this point, we start making changes.....
	 */

	info->baud_base = new_serial.baud_base;
	info->flags = ((info->flags & ~ASYNC_FLAGS) |
			(new_serial.flags & ASYNC_FLAGS));
	info->custom_divisor = new_serial.custom_divisor;
	info->type = new_serial.type;
	info->close_delay = new_serial.close_delay * HZ/100;
	info->closing_wait = new_serial.closing_wait * HZ/100;

	if (change_port || change_irq) {
		/*
		 * We need to shutdown the serial port at the old
		 * port/irq combination.
		 */
		ns16550_shutdown(info);
		info->irq = new_serial.irq;
		info->port = new_serial.port;
		info->hub6 = new_serial.hub6;
	}

check_and_exit:
	if (!info->port || !info->type)
		return 0;
	if (info->flags & ASYNC_INITIALIZED) {
		if (((old_info.flags & ASYNC_SPD_MASK) !=
		     (info->flags & ASYNC_SPD_MASK)) ||
		    (old_info.custom_divisor != info->custom_divisor))
			ns16550_change_speed(info);
	} else
		retval = ns16550_startup(info);
	return retval;
}

static int ns16550_get_lsr_info(struct async_struct * info, unsigned int *value)
{
	unsigned char status;
	unsigned int result;

	cli();
	status = serial_in(info, UART_LSR);
	sti();
	result = ((status & UART_LSR_TEMT) ? TIOCSER_TEMT : 0);
	put_user(result,value);
	return 0;
}


static int ns16550_get_modem_info(struct async_struct * info, unsigned int *value)
{
	unsigned char control, status;
	unsigned int result;

	control = info->MCR;
	cli();
	status = serial_in(info, UART_MSR);
	sti();
	result =  ((control & UART_MCR_RTS) ? TIOCM_RTS : 0)
		| ((control & UART_MCR_DTR) ? TIOCM_DTR : 0)
		| ((status  & UART_MSR_DCD) ? TIOCM_CAR : 0)
		| ((status  & UART_MSR_RI) ? TIOCM_RNG : 0)
		| ((status  & UART_MSR_DSR) ? TIOCM_DSR : 0)
		| ((status  & UART_MSR_CTS) ? TIOCM_CTS : 0);
	put_user(result,value);
	return 0;
}

static int ns16550_set_modem_info(struct async_struct * info, unsigned int cmd,
				  unsigned int *value)
{
	int error;
	unsigned int arg;

	error = verify_area(VERIFY_READ, value, sizeof(int));
	if (error)
		return error;
	arg = get_user(value);
	switch (cmd) {
	case TIOCMBIS: 
		if (arg & TIOCM_RTS) {
			info->MCR |= UART_MCR_RTS;
			info->MCR_noint |= UART_MCR_RTS;
		}
		if (arg & TIOCM_DTR) {
			info->MCR |= UART_MCR_DTR;
			info->MCR_noint |= UART_MCR_DTR;
		}
		break;
	case TIOCMBIC:
		if (arg & TIOCM_RTS) {
			info->MCR &= ~UART_MCR_RTS;
			info->MCR_noint &= ~UART_MCR_RTS;
		}
		if (arg & TIOCM_DTR) {
			info->MCR &= ~UART_MCR_DTR;
			info->MCR_noint &= ~UART_MCR_DTR;
		}
		break;
	case TIOCMSET:
		info->MCR = ((info->MCR & ~(UART_MCR_RTS | UART_MCR_DTR))
			     | ((arg & TIOCM_RTS) ? UART_MCR_RTS : 0)
			     | ((arg & TIOCM_DTR) ? UART_MCR_DTR : 0));
		info->MCR_noint = ((info->MCR_noint
				    & ~(UART_MCR_RTS | UART_MCR_DTR))
				   | ((arg & TIOCM_RTS) ? UART_MCR_RTS : 0)
				   | ((arg & TIOCM_DTR) ? UART_MCR_DTR : 0));
		break;
	default:
		return -EINVAL;
	}
	cli();
	serial_out(info, UART_MCR, info->MCR);
	sti();
	return 0;
}

static void ns16550_send_break(struct async_struct * info, int duration)
{
	if (!info->port)
		return;
	current->state = TASK_INTERRUPTIBLE;
	current->timeout = jiffies + duration;
	cli();
	serial_out(info, UART_LCR, serial_inp(info, UART_LCR) | UART_LCR_SBC);
	schedule();
	serial_out(info, UART_LCR, serial_inp(info, UART_LCR) & ~UART_LCR_SBC);
	sti();
}

static int ns16550_ioctl(struct tty_struct *tty, struct file * file,
			 unsigned int cmd, unsigned long arg)
{
	int error;
	struct async_struct * info = (struct async_struct *)tty->driver_data;
	int retval;
	struct async_icount cprev, cnow;	/* kernel counter temps */
	struct serial_icounter_struct *p_cuser;	/* user space */

	if (ns16550_serial_paranoia_check(info, tty->device, "rs_ioctl"))
		return -ENODEV;

	if ((cmd != TIOCGSERIAL) && (cmd != TIOCSSERIAL) &&
	    (cmd != TIOCSERCONFIG) && (cmd != TIOCSERGWILD)  &&
	    (cmd != TIOCSERSWILD) && (cmd != TIOCSERGSTRUCT) &&
	    (cmd != TIOCMIWAIT) && (cmd != TIOCGICOUNT)) {
		if (tty->flags & (1 << TTY_IO_ERROR))
		    return -EIO;
	}
	
	switch (cmd) {
		case TCSBRK:	/* SVID version: non-zero arg --> no break */
			retval = tty_check_change(tty);
			if (retval)
				return retval;
			tty_wait_until_sent(tty, 0);
			if (!arg)
				ns16550_send_break(info, HZ/4);	/* 1/4 second */
			return 0;
		case TCSBRKP:	/* support for POSIX tcsendbreak() */
			retval = tty_check_change(tty);
			if (retval)
				return retval;
			tty_wait_until_sent(tty, 0);
			ns16550_send_break(info, arg ? arg*(HZ/10) : HZ/4);
			return 0;
		case TIOCGSOFTCAR:
			error = verify_area(VERIFY_WRITE, (void *) arg,sizeof(long));
			if (error)
				return error;
			put_fs_long(C_CLOCAL(tty) ? 1 : 0,
				    (unsigned long *) arg);
			return 0;
		case TIOCSSOFTCAR:
			error = verify_area(VERIFY_READ, (void *) arg,sizeof(long));
			if (error)
				return error;
			arg = get_fs_long((unsigned long *) arg);
			tty->termios->c_cflag =
				((tty->termios->c_cflag & ~CLOCAL) |
				 (arg ? CLOCAL : 0));
			return 0;
		case TIOCMGET:
			error = verify_area(VERIFY_WRITE, (void *) arg,
				sizeof(unsigned int));
			if (error)
				return error;
			return ns16550_get_modem_info(info, (unsigned int *) arg);
		case TIOCMBIS:
		case TIOCMBIC:
		case TIOCMSET:
			return ns16550_set_modem_info(info, cmd, (unsigned int *) arg);
		case TIOCGSERIAL:
			error = verify_area(VERIFY_WRITE, (void *) arg,
						sizeof(struct serial_struct));
			if (error)
				return error;
			return ns16550_get_serial_info(info,
						       (struct serial_struct *) arg);
		case TIOCSSERIAL:
			error = verify_area(VERIFY_READ, (void *) arg,
						sizeof(struct serial_struct));
			if (error)
				return error;
			return ns16550_set_serial_info(info,
						       (struct serial_struct *) arg);
		case TIOCSERCONFIG:
#if 1
			return -EINVAL;
#else
			return do_autoconfig(info);
#endif
		case TIOCSERGWILD:
#if 1
			return -EINVAL;
#else
			error = verify_area(VERIFY_WRITE, (void *) arg,
					    sizeof(int));
			if (error)
				return error;
			put_fs_long(rs_wild_int_mask, (unsigned long *) arg);
			return 0;
#endif
		case TIOCSERGETLSR: /* Get line status register */
			error = verify_area(VERIFY_WRITE, (void *) arg,
				sizeof(unsigned int));
			if (error)
				return error;
			else
			    return ns16550_get_lsr_info(info, (unsigned int *) arg);

		case TIOCSERSWILD:
#if 1
			return -EINVAL;
#else
			if (!suser())
				return -EPERM;
			error = verify_area(VERIFY_READ, (void *) arg,sizeof(long));
			if (error)
				return error;
			rs_wild_int_mask = get_fs_long((unsigned long *) arg);
			if (rs_wild_int_mask < 0)
				rs_wild_int_mask = check_wild_interrupts(0);
			return 0;
#endif
		case TIOCSERGSTRUCT:
			error = verify_area(VERIFY_WRITE, (void *) arg,
						sizeof(struct async_struct));
			if (error)
				return error;
			memcpy_tofs((struct async_struct *) arg,
				    info, sizeof(struct async_struct));
			return 0;
			
		case TIOCSERGETMULTI:
#if 1
			return -EINVAL;
#else
			error = verify_area(VERIFY_WRITE, (void *) arg,
				    sizeof(struct serial_multiport_struct));
			if (error)
				return error;
			return get_multiport_struct(info,
				       (struct serial_multiport_struct *) arg);
#endif
		case TIOCSERSETMULTI:
#if 1
			return -EINVAL;
#else
			error = verify_area(VERIFY_READ, (void *) arg,
				    sizeof(struct serial_multiport_struct));
			if (error)
				return error;
			return set_multiport_struct(info,
				       (struct serial_multiport_struct *) arg);
#endif
		/*
		 * Wait for any of the 4 modem inputs (DCD,RI,DSR,CTS) to change
		 * - mask passed in arg for lines of interest
 		 *   (use |'ed TIOCM_RNG/DSR/CD/CTS for masking)
		 * Caller should use TIOCGICOUNT to see which one it was
		 */
		 case TIOCMIWAIT:
			cli();
			cprev = info->icount;	/* note the counters on entry */
			sti();
			while (1) {
				interruptible_sleep_on(&info->delta_msr_wait);
				/* see if a signal did it */
				if (current->signal & ~current->blocked)
					return -ERESTARTSYS;
				cli();
				cnow = info->icount;	/* atomic copy */
				sti();
				if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr && 
				    cnow.dcd == cprev.dcd && cnow.cts == cprev.cts)
					return -EIO; /* no change => error */
				if ( ((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
				     ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
				     ((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
				     ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts)) ) {
					return 0;
				}
				cprev = cnow;
			}
			/* NOTREACHED */

		/* 
		 * Get counter of input serial line interrupts (DCD,RI,DSR,CTS)
		 * Return: write counters to the user passed counter struct
		 * NB: both 1->0 and 0->1 transitions are counted except for
		 *     RI where only 0->1 is counted.
		 */
		case TIOCGICOUNT:
			error = verify_area(VERIFY_WRITE, (void *) arg,
				sizeof(struct serial_icounter_struct));
			if (error)
				return error;
			cli();
			cnow = info->icount;
			sti();
			p_cuser = (struct serial_icounter_struct *) arg;
			put_user(cnow.cts, &p_cuser->cts);
			put_user(cnow.dsr, &p_cuser->dsr);
			put_user(cnow.rng, &p_cuser->rng);
			put_user(cnow.dcd, &p_cuser->dcd);
			return 0;

		default:
			return -ENOIOCTLCMD;
		}
	return 0;
}

static void ns16550_set_termios(struct tty_struct *tty, struct termios *old_termios)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;

	if (   (tty->termios->c_cflag == old_termios->c_cflag)
	    && (   RELEVANT_IFLAG(tty->termios->c_iflag) 
		== RELEVANT_IFLAG(old_termios->c_iflag)))
	  return;

	ns16550_change_speed(info);

	if ((old_termios->c_cflag & CRTSCTS) &&
	    !(tty->termios->c_cflag & CRTSCTS)) {
		tty->hw_stopped = 0;
		ns16550_start(tty);
	}

#if 0
	/*
	 * No need to wake up processes in open wait, since they
	 * sample the CLOCAL flag once, and don't recheck it.
	 * XXX  It's not clear whether the current behavior is correct
	 * or not.  Hence, this may change.....
	 */
	if (!(old_termios->c_cflag & CLOCAL) &&
	    (tty->termios->c_cflag & CLOCAL))
		wake_up_interruptible(&info->open_wait);
#endif
}

static void ns16550_close(struct tty_struct *tty, struct file * filp)
{
	struct async_struct * info = (struct async_struct *)tty->driver_data;
	unsigned long flags;
	unsigned long timeout;

	if (!info || ns16550_serial_paranoia_check(info, tty->device, "rs_close"))
		return;
	
	save_flags(flags); cli();
	
	if (tty_hung_up_p(filp)) {
		restore_flags(flags);
		return;
	}
	
#ifdef SERIAL_DEBUG_OPEN
	printk("rs_close ttys%d, count = %d\n", info->line, info->count);
#endif
	if ((tty->count == 1) && (info->count != 1)) {
		/*
		 * Uh, oh.  tty->count is 1, which means that the tty
		 * structure will be freed.  Info->count should always
		 * be one in these conditions.  If it's greater than
		 * one, we've got real problems, since it means the
		 * serial port won't be shutdown.
		 */
		printk("rs_close: bad serial port count; tty->count is 1, "
		       "info->count is %d\n", info->count);
		info->count = 1;
	}
	if (--info->count < 0) {
		printk("rs_close: bad serial port count for ttys%d: %d\n",
		       info->line, info->count);
		info->count = 0;
	}
	if (info->count) {
		restore_flags(flags);
		return;
	}
	info->flags |= ASYNC_CLOSING;
	/*
	 * Save the termios structure, since this port may have
	 * separate termios for callout and dialin.
	 */
	if (info->flags & ASYNC_NORMAL_ACTIVE)
		info->normal_termios = *tty->termios;
	if (info->flags & ASYNC_CALLOUT_ACTIVE)
		info->callout_termios = *tty->termios;
	/*
	 * Now we wait for the transmit buffer to clear; and we notify 
	 * the line discipline to only process XON/XOFF characters.
	 */
	tty->closing = 1;
	if (info->closing_wait != ASYNC_CLOSING_WAIT_NONE)
		tty_wait_until_sent(tty, info->closing_wait);
	/*
	 * At this point we stop accepting input.  To do this, we
	 * disable the receive line status interrupts, and tell the
	 * interrupt driver to stop checking the data ready bit in the
	 * line status register.
	 */
	info->IER &= ~UART_IER_RLSI;
	info->read_status_mask &= ~UART_LSR_DR;
	if (info->flags & ASYNC_INITIALIZED) {
		serial_out(info, UART_IER, info->IER);
		/*
		 * Before we drop DTR, make sure the UART transmitter
		 * has completely drained; this is especially
		 * important if there is a transmit FIFO!
		 */
		timeout = jiffies+HZ;
		while (!(serial_inp(info, UART_LSR) & UART_LSR_TEMT)) {
			current->state = TASK_INTERRUPTIBLE;
			current->timeout = jiffies + info->timeout;
			schedule();
			if (jiffies > timeout)
				break;
		}
	}
	ns16550_shutdown(info);
	if (tty->driver.flush_buffer)
		tty->driver.flush_buffer(tty);
	if (tty->ldisc.flush_buffer)
		tty->ldisc.flush_buffer(tty);
	tty->closing = 0;
	info->event = 0;
	info->tty = 0;
	if (info->blocked_open) {
		if (info->close_delay) {
			current->state = TASK_INTERRUPTIBLE;
			current->timeout = jiffies + info->close_delay;
			schedule();
		}
		wake_up_interruptible(&info->open_wait);
	}
	info->flags &= ~(ASYNC_NORMAL_ACTIVE|ASYNC_CALLOUT_ACTIVE|
			 ASYNC_CLOSING);
	wake_up_interruptible(&info->close_wait);
	restore_flags(flags);
}

void ns16550_hangup(struct tty_struct *tty)
{
	struct async_struct * info = (struct async_struct *)tty->driver_data;
	
	if (ns16550_serial_paranoia_check(info, tty->device, "rs_hangup"))
		return;
	
#ifndef CONFIG_COBALT_28
	/* When acting as a console, we don't want to shutdown the
	 * line, because that causes userland shutdown to hang for
	 * example.  -DaveM
	 */
	return;
#else
	ns16550_flush_buffer(tty);
	ns16550_shutdown(info);
	info->event = 0;
	info->count = 0;
	info->flags &= ~(ASYNC_NORMAL_ACTIVE|ASYNC_CALLOUT_ACTIVE);
	info->tty = 0;
	wake_up_interruptible(&info->open_wait);
#endif
}

static int ns16550_block_til_ready(struct tty_struct *tty, struct file * filp,
				   struct async_struct *info)
{
	struct wait_queue wait = { current, NULL };
	int		retval;
	int		do_clocal = 0;

	/*
	 * If the device is in the middle of being closed, then block
	 * until it's done, and then try again.
	 */
	if (tty_hung_up_p(filp) ||
	    (info->flags & ASYNC_CLOSING)) {
		if (info->flags & ASYNC_CLOSING)
			interruptible_sleep_on(&info->close_wait);
#ifdef SERIAL_DO_RESTART
		if (info->flags & ASYNC_HUP_NOTIFY)
			return -EAGAIN;
		else
			return -ERESTARTSYS;
#else
		return -EAGAIN;
#endif
	}

	/*
	 * If this is a callout device, then just make sure the normal
	 * device isn't being used.
	 */
	if (tty->driver.subtype == SERIAL_TYPE_CALLOUT) {
		if (info->flags & ASYNC_NORMAL_ACTIVE)
			return -EBUSY;
		if ((info->flags & ASYNC_CALLOUT_ACTIVE) &&
		    (info->flags & ASYNC_SESSION_LOCKOUT) &&
		    (info->session != current->session))
		    return -EBUSY;
		if ((info->flags & ASYNC_CALLOUT_ACTIVE) &&
		    (info->flags & ASYNC_PGRP_LOCKOUT) &&
		    (info->pgrp != current->pgrp))
		    return -EBUSY;
		info->flags |= ASYNC_CALLOUT_ACTIVE;
		return 0;
	}
	
	/*
	 * If non-blocking mode is set, or the port is not enabled,
	 * then make the check up front and then exit.
	 */
	if ((filp->f_flags & O_NONBLOCK) ||
	    (tty->flags & (1 << TTY_IO_ERROR))) {
		if (info->flags & ASYNC_CALLOUT_ACTIVE)
			return -EBUSY;
		info->flags |= ASYNC_NORMAL_ACTIVE;
		return 0;
	}

	if (info->flags & ASYNC_CALLOUT_ACTIVE) {
		if (info->normal_termios.c_cflag & CLOCAL)
			do_clocal = 1;
	} else {
		if (tty->termios->c_cflag & CLOCAL)
			do_clocal = 1;
	}
	
	/*
	 * Block waiting for the carrier detect and the line to become
	 * free (i.e., not in use by the callout).  While we are in
	 * this loop, info->count is dropped by one, so that
	 * rs_close() knows when to free things.  We restore it upon
	 * exit, either normal or abnormal.
	 */
	retval = 0;
	add_wait_queue(&info->open_wait, &wait);
#ifdef SERIAL_DEBUG_OPEN
	printk("block_til_ready before block: ttys%d, count = %d\n",
	       info->line, info->count);
#endif
	cli();
	if (!tty_hung_up_p(filp)) 
		info->count--;
	sti();
	info->blocked_open++;
	while (1) {
		cli();
		if (!(info->flags & ASYNC_CALLOUT_ACTIVE))
			serial_out(info, UART_MCR,
				   serial_inp(info, UART_MCR) |
				   (UART_MCR_DTR | UART_MCR_RTS));
		sti();
		current->state = TASK_INTERRUPTIBLE;
		if (tty_hung_up_p(filp) ||
		    !(info->flags & ASYNC_INITIALIZED)) {
#ifdef SERIAL_DO_RESTART
			if (info->flags & ASYNC_HUP_NOTIFY)
				retval = -EAGAIN;
			else
				retval = -ERESTARTSYS;	
#else
			retval = -EAGAIN;
#endif
			break;
		}
		if (!(info->flags & ASYNC_CALLOUT_ACTIVE) &&
		    !(info->flags & ASYNC_CLOSING) &&
		    (do_clocal || (serial_in(info, UART_MSR) &
				   UART_MSR_DCD)))
			break;
		if (current->signal & ~current->blocked) {
			retval = -ERESTARTSYS;
			break;
		}
#ifdef SERIAL_DEBUG_OPEN
		printk("block_til_ready blocking: ttys%d, count = %d\n",
		       info->line, info->count);
#endif
		schedule();
	}
	current->state = TASK_RUNNING;
	remove_wait_queue(&info->open_wait, &wait);
	if (!tty_hung_up_p(filp))
		info->count++;
	info->blocked_open--;
#ifdef SERIAL_DEBUG_OPEN
	printk("block_til_ready after blocking: ttys%d, count = %d\n",
	       info->line, info->count);
#endif
	if (retval)
		return retval;
	info->flags |= ASYNC_NORMAL_ACTIVE;
	return 0;
}	

int ns16550_open(struct tty_struct *tty, struct file * filp)
{
	struct async_struct	*info;
	int 			retval, line;

	line = MINOR(tty->device) - tty->driver.minor_start;
	if ((line < 0) || (line >= NR_PORTS))
		return -ENODEV;
	info = ns16550_table + line;
	if (ns16550_serial_paranoia_check(info, tty->device, "rs_open"))
		return -ENODEV;

#ifdef SERIAL_DEBUG_OPEN
	printk("rs_open %s%d, count = %d\n", tty->driver.name, info->line,
	       info->count);
#endif
	info->count++;
	tty->driver_data = info;
	info->tty = tty;

	/*
	 * Start up serial port
	 */
	retval = ns16550_startup(info);
	if (retval)
		return retval;

	retval = ns16550_block_til_ready(tty, filp, info);
	if (retval) {
#ifdef SERIAL_DEBUG_OPEN
		printk("rs_open returning after block_til_ready with %d\n",
		       retval);
#endif
		return retval;
	}

	if ((info->count == 1) && (info->flags & ASYNC_SPLIT_TERMIOS)) {
		if (tty->driver.subtype == SERIAL_TYPE_NORMAL)
			*tty->termios = info->normal_termios;
		else 
			*tty->termios = info->callout_termios;
		ns16550_change_speed(info);
	}

	info->session = current->session;
	info->pgrp = current->pgrp;

#ifdef SERIAL_DEBUG_OPEN
	printk("rs_open ttys%d successful...", info->line);
#endif
	return 0;
}

static void ns16550_autoconfig(struct async_struct * info)
{
	unsigned char status1, status2, scratch, scratch2;
	unsigned int port = info->port;
	unsigned long flags;

	info->type = PORT_UNKNOWN;
	
	if (!port)
		return;

	save_flags(flags); cli();
	
	/*
	 * Do a simple existence test first; if we fail this, there's
	 * no point trying anything else.
	 *
	 * 0x80 is used as a nonsense port to prevent against false
	 * positives due to ISA bus float.  The assumption is that
	 * 0x80 is a non-existent port; which should be safe since
	 * include/asm/io.h also makes this assumption.
	 */
	scratch = serial_inp(info, UART_IER);
	serial_outp(info, UART_IER, 0);
#if 0
	outb(0xff, 0x080);
#endif
	scratch2 = serial_inp(info, UART_IER);
	serial_outp(info, UART_IER, scratch);
	if (scratch2) {
		restore_flags(flags);
		return;		/* We failed; there's nothing here */
	}

	/* 
	 * Check to see if a UART is really there.  Certain broken
	 * internal modems based on the Rockwell chipset fail this
	 * test, because they apparently don't implement the loopback
	 * test mode.  So this test is skipped on the COM 1 through
	 * COM 4 ports.  This *should* be safe, since no board
	 * manufacturer would be stupid enough to design a board
	 * that conflicts with COM 1-4 --- we hope!
	 */
	if (!(info->flags & ASYNC_SKIP_TEST)) {
		scratch = serial_inp(info, UART_MCR);
		serial_outp(info, UART_MCR, UART_MCR_LOOP | scratch);
		scratch2 = serial_inp(info, UART_MSR);
		serial_outp(info, UART_MCR, UART_MCR_LOOP | 0x0A);
		status1 = serial_inp(info, UART_MSR) & 0xF0;
		serial_outp(info, UART_MCR, scratch);
		serial_outp(info, UART_MSR, scratch2);
		if (status1 != 0x90) {
			restore_flags(flags);
			return;
		}
	} 
	
	scratch2 = serial_in(info, UART_LCR);
	serial_outp(info, UART_LCR, scratch2 | UART_LCR_DLAB);
	serial_outp(info, UART_EFR, 0);	/* EFR is the same as FCR */
	serial_outp(info, UART_LCR, scratch2);
	serial_outp(info, UART_FCR, UART_FCR_ENABLE_FIFO);
	scratch = serial_in(info, UART_IIR) >> 6;
	info->xmit_fifo_size = 1;
	switch (scratch) {
		case 0:
			info->type = PORT_16450;
			break;
		case 1:
			info->type = PORT_UNKNOWN;
			break;
		case 2:
			info->type = PORT_16550;
			break;
		case 3:
			serial_outp(info, UART_LCR, scratch2 | UART_LCR_DLAB);
			if (serial_in(info, UART_EFR) == 0) {
				info->type = PORT_16650;
				info->xmit_fifo_size = 32;
			} else {
				info->type = PORT_16550A;
				info->xmit_fifo_size = 16;
			}
			serial_outp(info, UART_LCR, scratch2);
			break;
	}
	if (info->type == PORT_16450) {
		scratch = serial_in(info, UART_SCR);
		serial_outp(info, UART_SCR, 0xa5);
		status1 = serial_in(info, UART_SCR);
		serial_outp(info, UART_SCR, 0x5a);
		status2 = serial_in(info, UART_SCR);
		serial_outp(info, UART_SCR, scratch);

		if ((status1 != 0xa5) || (status2 != 0x5a))
			info->type = PORT_8250;
	}

	/*
	 * Reset the UART.
	 */
#if defined(__alpha__) && !defined(CONFIG_PCI)
	/*
	 * I wonder what DEC did to the OUT1 and OUT2 lines?
	 * clearing them results in endless interrupts.
	 */
	serial_outp(info, UART_MCR, 0x0c);
#else
	serial_outp(info, UART_MCR, 0x00);
#endif
	serial_outp(info, UART_FCR, (UART_FCR_CLEAR_RCVR |
				     UART_FCR_CLEAR_XMIT));
	(void)serial_in(info, UART_RX);
	
	restore_flags(flags);
}

static int rs_init_16550(void)
{
	static char *serial_name = "Serial driver";
	static char *serial_version = "4.13";

	struct async_struct * info;
	int i;
	
	init_bh(SERIAL_BH, ns16550_do_serial_bh);
	timer_table[RS_TIMER].fn = ns16550_timer;
	timer_table[RS_TIMER].expires = 0;

	for (i = 0; i < 16; i++) {
		IRQ_ports[i] = 0;
		IRQ_timeout[i] = 0;
	}
	
 	printk(KERN_INFO "%s version %s with", serial_name, serial_version);
	printk(" no serial options enabled\n");

	/* Initialize the tty_driver structure */
	
	memset(&serial_driver, 0, sizeof(struct tty_driver));
	serial_driver.magic = TTY_DRIVER_MAGIC;
	serial_driver.name = "ttyS";
	serial_driver.major = TTY_MAJOR;
	serial_driver.minor_start = 64;
	serial_driver.num = NR_PORTS;
	serial_driver.type = TTY_DRIVER_TYPE_SERIAL;
	serial_driver.subtype = SERIAL_TYPE_NORMAL;
	serial_driver.init_termios = tty_std_termios;
	serial_driver.init_termios.c_cflag =
#ifndef BOOTLOADER
#ifdef DEBUG_LOADER
		/* B9600 | CS8 | CREAD | HUPCL | CLOCAL; */
		B115200 | CS8 | CREAD | HUPCL | CLOCAL;
#else
		B9600 | CS8 | CREAD | HUPCL | CLOCAL;
#endif
#else	/* BOOTLOADER - 115200 always */
		B115200 | CS8 | CREAD | HUPCL | CLOCAL;
#endif
	
	serial_driver.flags = TTY_DRIVER_REAL_RAW;
	serial_driver.refcount = &serial_refcount;
	serial_driver.table = ns16550_serial_table;
	serial_driver.termios = ns16550_serial_termios;
	serial_driver.termios_locked = ns16550_serial_termios_locked;

	serial_driver.open = ns16550_open;
	serial_driver.close = ns16550_close;
	serial_driver.write = ns16550_write;
	serial_driver.put_char = ns16550_put_char;
	serial_driver.flush_chars = ns16550_flush_chars;
	serial_driver.write_room = ns16550_write_room;
	serial_driver.chars_in_buffer = ns16550_chars_in_buffer;
	serial_driver.flush_buffer = ns16550_flush_buffer;
	serial_driver.ioctl = ns16550_ioctl;
	serial_driver.throttle = ns16550_throttle;
	serial_driver.unthrottle = ns16550_unthrottle;
	serial_driver.set_termios = ns16550_set_termios;
	serial_driver.stop = ns16550_stop;
	serial_driver.start = ns16550_start;
	serial_driver.hangup = ns16550_hangup;

	/*
	 * The callout device is just like normal device except for
	 * major number and the subtype code.
	 */
	callout_driver = serial_driver;
	callout_driver.name = "cua";
	callout_driver.major = TTYAUX_MAJOR;
	callout_driver.subtype = SERIAL_TYPE_CALLOUT;

	if (tty_register_driver(&serial_driver))
		panic("Couldn't register serial driver\n");
	if (tty_register_driver(&callout_driver))
		panic("Couldn't register callout driver\n");
	
	for (i = 0, info = ns16550_table; i < NR_PORTS; i++,info++) {
		info->magic = SERIAL_MAGIC;
		info->line = i;
		info->tty = 0;
		info->type = PORT_UNKNOWN;
		info->custom_divisor = 0;
		info->close_delay = 5*HZ/10;
		info->closing_wait = 30*HZ;
		info->x_char = 0;
		info->event = 0;
		info->count = 0;
		info->blocked_open = 0;
		info->tqueue.routine = ns16550_do_softint;
		info->tqueue.data = info;
		info->tqueue_hangup.routine = ns16550_do_serial_hangup;
		info->tqueue_hangup.data = info;
		info->callout_termios =callout_driver.init_termios;
		info->normal_termios = serial_driver.init_termios;
		info->open_wait = 0;
		info->close_wait = 0;
		info->delta_msr_wait = 0;
		info->icount.cts = info->icount.dsr = 
			info->icount.rng = info->icount.dcd = 0;
		info->next_port = 0;
		info->prev_port = 0;
		if (info->type == PORT_UNKNOWN) {
			if (!(info->flags & ASYNC_BOOT_AUTOCONF))
				continue;
			ns16550_autoconfig(info);
			if (info->type == PORT_UNKNOWN)
				continue;
		}
		printk(KERN_INFO "tty%02d at 0x%04x (irq = %d)", info->line, 
		       info->port, info->irq);
		switch (info->type) {
			case PORT_8250:
				printk(" is a 8250\n");
				break;
			case PORT_16450:
				printk(" is a 16450\n");
				break;
			case PORT_16550:
				printk(" is a 16550\n");
				break;
			case PORT_16550A:
				printk(" is a 16550A\n");
				break;
			case PORT_16650:
				printk(" is a 16650\n");
				break;
			default:
				printk("\n");
				break;
		}
	}
	return 0;
}

/* =========================== No serial present driver =================================== */

/* COBALT HACK: Give a dummy serial console so init has somewhere
 *              to stuff it's output and successfully boot the system. -DaveM
 */
static int rs_fake_open(struct tty_struct *tty, struct file *filp)
{
	/* Open always succeeds. */
	return 0;
}

static void rs_fake_close(struct tty_struct *tty, struct file *filp)
{
	/* Close always succeeds. */
	return;
}

static int rs_fake_write(struct tty_struct *tty, int from_user,
		  const unsigned char *buf, int count)
{
	/* We always write everything. */
	return count;
}

static void rs_fake_flush_chars(struct tty_struct *tty)
{
	/* This does nothing... */
	return;
}

static int rs_fake_write_room(struct tty_struct *tty)
{
	/* We always have the max amount of room... */
	return SERIAL_XMIT_SIZE - 1;
}

static int rs_fake_chars_in_buffer(struct tty_struct *tty)
{
	/* Nothings being transmitted, so nothing in our buffer... */
	return 0;
}

static void rs_fake_flush_buffer(struct tty_struct *tty)
{
	/* No buffer to flush... but we must wake people up if necessary. */
	wake_up_interruptible(&tty->write_wait);
	if((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
	   tty->ldisc.write_wakeup)
		(tty->ldisc.write_wakeup)(tty);
}

static int rs_fake_ioctl(struct tty_struct *tty, struct file *file,
		  unsigned int cmd, unsigned long arg)
{
	/* Just fake out that we do something here... */
	if ((cmd != TIOCGSERIAL) && (cmd != TIOCSSERIAL) &&
	    (cmd != TIOCSERCONFIG) && (cmd != TIOCSERGWILD)  &&
	    (cmd != TIOCSERSWILD) && (cmd != TIOCSERGSTRUCT)) {
		if (tty->flags & (1 << TTY_IO_ERROR))
		    return -EIO;
	}

	/* Yeah we did it ;-) */
	return 0;
}

static void rs_fake_throttle(struct tty_struct *tty)
{
	/* another nop */
	return;
}

static void rs_fake_unthrottle(struct tty_struct *tty)
{
	/* yet another nop */
	return;
}

static void rs_fake_set_termios(struct tty_struct *tty, struct termios *old_termios)
{
	if(tty->termios->c_cflag == old_termios->c_cflag)
		return;

	/* We just need to keep track of a few flags... */
	if ((old_termios->c_cflag & CRTSCTS) &&
	     !(tty->termios->c_cflag & CRTSCTS))
		tty->hw_stopped = 0;
}

static void rs_fake_stop(struct tty_struct *tty)
{
	/* nop */
	return;
}

static void rs_fake_start(struct tty_struct *tty)
{
	/* another Stonis special... slash slash */
	return;
}

static void rs_fake_hangup(struct tty_struct *tty)
{
	/* nop... */
	return;
}

static void rs_init_dummy(void)
{
	memset(&serial_driver, 0, sizeof(struct tty_driver));
	serial_driver.magic = TTY_DRIVER_MAGIC;
	serial_driver.name = "ttyS";
	serial_driver.major = TTY_MAJOR;
	serial_driver.minor_start = 64;
	serial_driver.num = 2; /* lie */
	serial_driver.type = TTY_DRIVER_TYPE_SERIAL;
	serial_driver.subtype = SERIAL_TYPE_NORMAL;
	serial_driver.init_termios = tty_std_termios;

	serial_driver.init_termios.c_cflag =
		B115200 | CS8 | CREAD | HUPCL | CLOCAL;
	serial_driver.flags = TTY_DRIVER_REAL_RAW;
	serial_driver.refcount = &serial_refcount;
	serial_driver.table = zs_serial_table;
	serial_driver.termios = zs_serial_termios;
	serial_driver.termios_locked = zs_serial_termios_locked;

	serial_driver.open = rs_fake_open;
	serial_driver.close = rs_fake_close;
	serial_driver.write = rs_fake_write;
	serial_driver.flush_chars = rs_fake_flush_chars;
	serial_driver.write_room = rs_fake_write_room;
	serial_driver.chars_in_buffer = rs_fake_chars_in_buffer;
	serial_driver.flush_buffer = rs_fake_flush_buffer;
	serial_driver.ioctl = rs_fake_ioctl;
	serial_driver.throttle = rs_fake_throttle;
	serial_driver.unthrottle = rs_fake_unthrottle;
	serial_driver.set_termios = rs_fake_set_termios;
	serial_driver.stop = rs_fake_stop;
	serial_driver.start = rs_fake_start;
	serial_driver.hangup = rs_fake_hangup;

	/*
	 * The callout device is just like normal device except for
	 * major number and the subtype code.
	 */
	callout_driver = serial_driver;
	callout_driver.name = "cua";
	callout_driver.major = TTYAUX_MAJOR;
	callout_driver.subtype = SERIAL_TYPE_CALLOUT;

	if (tty_register_driver(&serial_driver))
		panic("Couldn't register serial driver\n");
	if (tty_register_driver(&callout_driver))
		panic("Couldn't register callout driver\n");
}

/* =========================== Boot time probing and setup ============================= */

/* Returns the type of serial controller present, or none.
 * We only perform the probe once, so that:
 * 1) We always return the same value each time.
 * 2) We don't perform the slightly dangerous 16550 port
 *    poking more than once.
 */
static enum cobalt_scc_type cobalt_sccs_present(void)
{
	static int called = 0;  /* Only poke things once. */
	static enum cobalt_scc_type cons_type_cached = COBALT_SCC_TYPE_NONE;

	if(called == 0) {
		extern int cobalt_serial_present, cobalt_serial_type;

		called = 1;
		if (mips_machgroup == MACH_GROUP_ARC &&
		    mips_machtype  == MACH_DESKSTATION_RPC44) {
			cons_type_cached = COBALT_SCC_TYPE_NONE;
		} else if (cobalt_serial_present != 0x0) {
			/* Now we check the ROM type cookie.  We know that:
			 *
			 * 1) For all older ROMS it will be zero.
			 *    This indicates Zilog and works for all
			 *    existing Qubes we have ever built as a result.
			 * 2) For newer ROMS it will explicitly set this
			 *    type cookie to zero for Zilog and one
			 *    for ns16550 chipsets.
			 */
			if(cobalt_serial_type == 0x1) {
				cons_type_cached = COBALT_SCC_TYPE_16550;
			} else if (cobalt_serial_type == 0x0) {
				cons_type_cached = COBALT_SCC_TYPE_ZILOG;
			} else {
				/* Unexpected value, we should blow up.
				 * But play it safe and say we have no serial.
				 */
				cons_type_cached = COBALT_SCC_TYPE_NONE;
			}
		} else {
			cons_type_cached = COBALT_SCC_TYPE_NONE;
		}
	}
	return cons_type_cached;
}

/* rs_init inits the driver */
int rs_init(void)
{
	enum cobalt_scc_type cons_type;

	/* Only initialize on a Qube */
	cons_type = cobalt_sccs_present();
	if (cons_type == COBALT_SCC_TYPE_NONE) {
		/* But provide a fake serial console for init and friends. */
		rs_init_dummy();
		return 0;
	} else if (cons_type == COBALT_SCC_TYPE_16550) {
		/* A 16550, set it up. */
		return rs_init_16550();
	} else {
		/* It is a Zilog, do the needed. */
		return rs_init_zilog();
	}
}

/*
 * register_serial and unregister_serial allows for serial ports to be
 * configured at run-time, to support PCMCIA modems.
 */
/* PowerMac: Unused at this time, just here to make things link. */
int register_serial(struct serial_struct *req)
{
	return -1;
}

void unregister_serial(int line)
{
	return;
}

/*
 * Initialization values for when a channel is used for
 * a serial console.
 */
static unsigned char zs_cons_init_regs[16] = {
	0, 0, 0,		/* write 0, 1, 2 */
	(Rx8 | RxENABLE),	/* write 3 */
	(X16CLK | SB1),		/* write 4 */
	(Tx8 | TxENAB | RTS | DTR),	/* write 5 */
	0, 0, 0,		/* write 6, 7, 8 */
	0,			/* write 9 */
	(NRZ),			/* write 10 */
	(TCBR | RCBR | TRxCBR | TRxCOI),		/* write 11 */
	1, 0,			/* 115200 baud divisor, write 12 + 13 */
	(BRSRC | BRENABL),		/* write 14 */
	0			/* write 15 */
};

extern void register_console(void (*proc)(const char *));

/*
 * Hooks for running a serial console.  con_init() calls this if the
 * console is being run over one of the serial ports.
 * 'channel' is decoded as 1=modem, 2=printer.
 */
static void cons_hook_zilog(int chip, int out, int channel)
{
	int brg;

	if (zs_consinfo != 0) {
		printk("rs_cons_hook called twice?\n");
		return;
	}

	if (zs_chain == 0)
		zs_probe_sccs();
 	--channel;
	if (channel < 0 || channel >= zs_channels_found) {
		printk("rs_cons_hook: channel = %d?\n", channel);
		return;
	}

	zs_cons_chan = channel;
	zs_consinfo = &zs_soft[channel];
	zs_conschan = zs_consinfo->zs_channel;
	zs_consinfo->clk_divisor = 16;
	zs_consinfo->zs_baud = 115200;
	zs_consinfo->is_cons = 1;

	memcpy(zs_consinfo->curregs, zs_cons_init_regs, sizeof(zs_cons_init_regs));
	/* brg = BPS_TO_BRG(zs_consinfo->zs_baud, ZS_CLOCK/16); */
	brg = 0x01;
	zs_consinfo->curregs[R12] = brg;
	zs_consinfo->curregs[R13] = brg >> 8;
	load_zsregs(zs_conschan, zs_consinfo->curregs);

	register_console(zs_console_print);
	printk("zs%d: console I/O\n", channel);
}

static void cons_hook_16550(int chip, int out, int channel)
{
	int comstat, hi, lo;
	unsigned long port = 0x1c800000;

	comstat = inb(port + UART_LCR);
	outb(comstat | UART_LCR_DLAB, port + UART_LCR);
	hi = inb(port + UART_DLM);
	lo = inb(port + UART_DLL);
	outb(comstat, port + UART_LCR);

	outb(0x03, port + UART_LCR);
	outb(0x83, port + UART_LCR);

	outb(0x00, port + UART_DLM);

#ifndef BOOTLOADER
#ifndef DEBUG_LOADER
	/* 9600 baud -- default; for real kernel that boots from disk */
	outb(0x78, port + UART_DLL);
#else
	/* 115200 for debugging.. to match ramcode + rom kernel 
	   debug output speeds */
	outb(0x0a, port + UART_DLL);
#endif
#else
	/* BOOTLOADER - rom kernel, 115200 */
	outb(0x0a, port + UART_DLL);
#endif

	outb(0x03, port + UART_LCR);

	comstat = inb(port + UART_LSR);
	comstat = inb(port + UART_RX);
	outb(0x00, port + UART_IER);

	/* See if a modem is attached and on. */
#if 0
	{
		int tmp = inb(port + UART_MSR);

		/* If we have "Clear To Send" clear, we
		 * have a modem attached and it is on.
		 */
		if ((tmp & UART_MSR_CTS) == 0)
			ns16550_modem_attached = 1;
	}
#endif
#ifdef CONFIG_COBALT_28
	/* Gross, but until I can test this modem detection
	 * code some more, we disable console output entirely.
	 */
	ns16550_modem_attached = 1;
#endif
	register_console(ns16550_console_print);
	printk("ns16550: console I/O\n");
}

#ifdef BOOTLOADER

/*
 * ok, this is all just copied in from rom code.  I don't
 * understand any of this.    -gid
 */

#include "../../../include/diagdefs.h"
#include "../../../include/serial.h"

static void rom_ns_write_byte(unsigned char byte)
{
     while (( Read16550( UART_LSR ) & UART_LSR_TEMT ) == 0 )
       RegisterDelay();

     Write16550( UART_TX, byte );
       RegisterDelay();
}

static void rom_ns16550_console_print(const char *p)
{
        unsigned char c;

        while((c = *(p++)) != 0) {
                if(c == '\n')
                        rom_ns_write_byte('\r');
                rom_ns_write_byte(c);
        }
}

static void rom_cons_hook_16550(int chip, int out, int channel)
{

 // Set the Divisor latch bit
 Write16550( UART_LCR, UART_LCR_WLEN8 | UART_LCR_DLAB );

 // Set the divisor latch high byte...
 Write16550( UART_DLM, 0 );

 // Set the divisor latch low byte...
 Write16550( UART_DLL, 10);

 // Set the Line control (clearing DLB)
 Write16550( UART_LCR, UART_LCR_WLEN8 );

 // Use polled mode...
 Write16550( UART_IER, 0 );

 // Turn on RTS and DTR
 Write16550( UART_MCR, 3 );


 // Turn FIFOs off
 Write16550( UART_FCR, 0 );

 register_console(rom_ns16550_console_print);
 printk("rom_cons_hook_16550: first printk\n");
}

#endif /* BOOTLOADER */


void rs_cons_hook(int chip, int out, int channel)
{
	enum cobalt_scc_type cons_type;

	cons_type = cobalt_sccs_present();
        /* Only initialize on a Qube */
	if (cons_type != COBALT_SCC_TYPE_NONE) {
		if (cons_type == COBALT_SCC_TYPE_16550) {
			/* A 16550 on a RAQ board. */
#ifdef BOOTLOADER
			rom_cons_hook_16550(chip, out, channel);
#else
			cons_hook_16550(chip, out, channel);
#endif
		} else {
			/* A Zilog, do it to it. */
			cons_hook_zilog(chip, out, channel);
		}
	}
}

/* This is called at boot time to prime the kgdb serial debugging
 * serial line.  The 'tty_num' argument is 0 for /dev/ttyS0 and 1
 * for /dev/ttyS1 which is determined in setup_arch() from the
 * boot command line flags.
 */
void
rs_kgdb_hook(int tty_num)
{
	/* Don't do this on anything other than a Zilog. */
	if (cobalt_sccs_present() != COBALT_SCC_TYPE_ZILOG)
		return;
	if (zs_chain == 0)
		zs_probe_sccs();
	zs_kgdbchan = zs_soft[tty_num].zs_channel;
	zs_soft[tty_num].clk_divisor = 16;
	zs_soft[tty_num].zs_baud = get_zsbaud(&zs_soft[tty_num]);
	zs_soft[tty_num].kgdb_channel = 1;     /* This runs kgdb */
	zs_soft[tty_num ^ 1].kgdb_channel = 0; /* This does not */
	/* Turn on transmitter/receiver at 8-bits/char */
	zs_kgdb_chaninit(&zs_soft[tty_num], 0, 9600);
	ZS_CLEARERR(zs_kgdbchan);
	ZS_CLEARFIFO(zs_kgdbchan);
}
