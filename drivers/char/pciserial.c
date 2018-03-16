/*
 *  linux/drivers/char/pciserial.c (munged from serial.c)
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  Extensively rewritten by Theodore Ts'o, 8/16/92 -- 9/14/92.  Now
 *  much more extensible to support other serial cards based on the
 *  16450/16550A UART's.  Added support for the AST FourPort and the
 *  Accent Async board.  
 *
 *  set_serial_info fixed to set the flags, custom divisor, and uart
 * 	type fields.  Fix suggested by Michael K. Johnson 12/12/92.
 *
 *  11/95: TIOCMIWAIT, TIOCGICOUNT by Angelo Haritsis <ah@doc.ic.ac.uk>
 *
 *  03/96: Modularised by Angelo Haritsis <ah@doc.ic.ac.uk>
 *
 *  rs_set_termios fixed to look also for changes of the input
 *      flags INPCK, BRKINT, PARMRK, IGNPAR and IGNBRK.
 *                                            Bernd Anhäupl 05/17/96.
 * 
 * This module exports the following rs232 io functions:
 *
 *	int pci_rs_init(void);
 * 	int pci_rs_open(struct tty_struct * tty, struct file * filp)
 *
 * COBALT LOCAL:
 *
 * Whacked a lot from original serial.c sources to make it work with
 * PCI expansion cards in non-Intel systems.  -DaveM
 *
 * Hacked up even more for it actually work.  PCI now behaves relatively 
 * corrctly, modularization works, and cruft has been/is being removed
 *
 * Since this is basically a Cobalt special driver (we only have one PCI slot)
 * we can cut out stuff for other arch's and make it clean.
 *	-- TPH
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_reg.h>
#include <linux/config.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/bios32.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/segment.h>
#include <asm/bitops.h>

static char *serial_name = "PCI Serial driver";
static char *serial_version = "1.05";

DECLARE_TASK_QUEUE(tq_pciserial);

static struct tty_driver serial_driver, callout_driver;
static int serial_refcount;

/* serial subtype definitions */
#define SERIAL_TYPE_NORMAL	1
#define SERIAL_TYPE_CALLOUT	2

/* number of characters left in xmit buffer before we ask for more */
#define WAKEUP_CHARS 256

/*
 * Serial driver configuration section.  Here are the various options:
 *
 * SERIAL_PARANOIA_CHECK
 * 		Check the magic number for the async_structure where
 * 		ever possible.
 */

#define SERIAL_PARANOIA_CHECK
#define SERIAL_DO_RESTART

#undef SERIAL_DEBUG_INTR
#undef SERIAL_DEBUG_OPEN
#undef SERIAL_DEBUG_FLOW

#define PCI_RS_STROBE_TIME (10*HZ)
#define PCI_RS_ISR_PASS_LIMIT 256

#ifdef CONFIG_COBALT_27
/* COBALT LOCAL: We don't have shared IRQs  -DaveM */
#define IRQ_T(info) (SA_INTERRUPT)
#else
#define IRQ_T(info) ((info->flags & ASYNC_SHARE_IRQ) ? SA_SHIRQ : SA_INTERRUPT)
#endif

#define _INLINE_ inline

#if defined(MODULE) && defined(SERIAL_DEBUG_MCOUNT)
#define DBG_CNT(s) printk("(%s): [%x] refc=%d, serc=%d, ttyc=%d -> %s\n", \
 kdevname(tty->device), (info->flags), serial_refcount,info->count,tty->count,s
#else
#define DBG_CNT(s)
#endif

/*
 * IRQ_timeout		- How long the timeout should be for each IRQ
 * 				should be after the IRQ has been active.
 */

static struct async_struct *IRQ_ports[16];
static struct pci_rs_multiport_struct pci_rs_multiport[16];
static int IRQ_timeout[16];
static int pci_rs_wild_int_mask;

static void autoconfig(struct async_struct * info);
static void change_speed(struct async_struct *info);
	
/*
 * This assumes you have a 1.8432 MHz clock for your UART.
 *
 * It'd be nice if someone built a serial card with a 24.576 MHz
 * clock, since the 16550A is capable of handling a top speed of 1.5
 * megabits/second; but this requires the faster clock.
 */
#define BASE_BAUD ( 1843200 / 16 )

/* PCI COM flags */
#define PCI_COM_FLAGS ASYNC_BOOT_AUTOCONF

static struct async_struct pci_rs_table[] = {
	/* UART CLK   PORT IRQ     FLAGS        */
	{ 0, BASE_BAUD, 0x0, 0, PCI_COM_FLAGS },	/* ttyS4 */
	{ 0, BASE_BAUD, 0x0, 0, PCI_COM_FLAGS },	/* ttyS5 */
	{ 0, BASE_BAUD, 0x0, 0, PCI_COM_FLAGS },	/* ttyS6 */
	{ 0, BASE_BAUD, 0x0, 0, PCI_COM_FLAGS },	/* ttyS7 */

	{ 0, BASE_BAUD, 0x0, 0, PCI_COM_FLAGS }, 	/* ttyS8 */
	{ 0, BASE_BAUD, 0x0, 0, PCI_COM_FLAGS },	/* ttyS9 */
};

#define NR_PORTS	(sizeof(pci_rs_table)/sizeof(struct async_struct))

static struct tty_struct *serial_table[NR_PORTS];
static struct termios *serial_termios[NR_PORTS];
static struct termios *serial_termios_locked[NR_PORTS];

#ifndef MIN
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#endif

/*
 * pci_tmp_buf is used as a temporary buffer by serial_write.  We need to
 * lock it in case the memcpy_fromfs blocks while swapping in a page,
 * and some other program tries to do a serial write at the same time.
 * Since the lock will only come under contention when the system is
 * swapping and available memory is low, it makes sense to share one
 * buffer across all the serial ports, since it significantly saves
 * memory if large numbers of serial ports are open.
 */
static unsigned char *pci_tmp_buf = NULL;
static struct semaphore pci_tmp_buf_sem = MUTEX;

static inline int serial_paranoia_check(struct async_struct *info,
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
static int baud_table[] = {
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

/*
 * ------------------------------------------------------------
 * pci_rs_stop() and pci_rs_start()
 *
 * This routines are called before setting or resetting tty->stopped.
 * They enable or disable transmitter interrupts, as necessary.
 * ------------------------------------------------------------
 */
static void pci_rs_stop(struct tty_struct *tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	unsigned long flags;

	if (serial_paranoia_check(info, tty->device, "pci_rs_stop"))
		return;
	
	save_flags(flags); cli();
	if (info->IER & UART_IER_THRI) {
		info->IER &= ~UART_IER_THRI;
		serial_out(info, UART_IER, info->IER);
	}
	restore_flags(flags);
}

static void pci_rs_start(struct tty_struct *tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	unsigned long flags;
	
	if (serial_paranoia_check(info, tty->device, "pci_rs_start"))
		return;
	
	save_flags(flags); cli();
	if (info->xmit_cnt && info->xmit_buf && !(info->IER & UART_IER_THRI)) {
		info->IER |= UART_IER_THRI;
		serial_out(info, UART_IER, info->IER);
	}
	restore_flags(flags);
}

/*
 * ----------------------------------------------------------------------
 *
 * Here starts the interrupt handling routines.  All of the following
 * subroutines are declared as inline and are folded into
 * pci_rs_interrupt().  They were separated out for readability's sake.
 *
 * Note: pci_rs_interrupt() is a "fast" interrupt, which means that it
 * runs with interrupts turned off.  People who may want to modify
 * pci_rs_interrupt() should try to keep the interrupt handler as fast as
 * possible.  After you are done making modifications, it is not a bad
 * idea to do:
 * 
 * gcc -S -DKERNEL -Wall -Wstrict-prototypes -O6 -fomit-frame-pointer serial.c
 *
 * and look at the resulting assemble code in serial.s.
 *
 * 				- Ted Ts'o (tytso@mit.edu), 7-Mar-93
 * -----------------------------------------------------------------------
 */

/*
 * This routine is used by the interrupt handler to schedule
 * processing in the software interrupt portion of the driver.
 */
static _INLINE_ void pci_rs_sched_event(struct async_struct *info,
					int event)
{
	info->event |= 1 << event;
	queue_task_irq_off(&info->tqueue, &tq_pciserial);
	mark_bh(PCISERIAL_BH);
}

static _INLINE_ void receive_chars(struct async_struct *info,
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
	ignore_char:
		*status = serial_inp(info, UART_LSR) & info->read_status_mask;
	} while (*status & UART_LSR_DR);
	queue_task_irq_off(&tty->flip.tqueue, &tq_timer);
#ifdef SERIAL_DEBUG_INTR
	printk("DR...");
#endif
}

static _INLINE_ void transmit_chars(struct async_struct *info, int *intr_done)
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
		pci_rs_sched_event(info, PCI_RS_EVENT_WRITE_WAKEUP);

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

static _INLINE_ void check_modem_status(struct async_struct *info)
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
				pci_rs_sched_event(info, PCI_RS_EVENT_WRITE_WAKEUP);
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
 * This is the serial driver's generic interrupt routine
 */
static void pci_rs_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	int status;
	struct async_struct * info;
	int pass_counter = 0;
	struct async_struct *end_mark = 0;
	int first_multi = 0;
	struct pci_rs_multiport_struct *multi;

#ifdef SERIAL_DEBUG_INTR
	printk("pci_rs_interrupt(%d)...", irq);
#endif
	
	info = IRQ_ports[irq];
	if (!info)
		return;
	
	multi = &pci_rs_multiport[irq];
	if (multi->port_monitor)
		first_multi = inb(multi->port_monitor);

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
			receive_chars(info, &status);
		check_modem_status(info);
		if (status & UART_LSR_THRE)
			transmit_chars(info, 0);

	next:
		info = info->next_port;
		if (!info) {
			info = IRQ_ports[irq];
			if (pass_counter++ > PCI_RS_ISR_PASS_LIMIT) {
#if 0
				printk("rs loop break\n");
#endif
				break; 	/* Prevent infinite loops */
			}
			continue;
		}
	} while (end_mark != info);
	if (multi->port_monitor)
		printk("rs port monitor (normal) irq %d: 0x%x, 0x%x\n",
		       info->irq, first_multi, inb(multi->port_monitor));
#ifdef SERIAL_DEBUG_INTR
	printk("end.\n");
#endif
}

/*
 * This is the serial driver's interrupt routine for a single port
 */
static void pci_rs_interrupt_single(int irq, void *dev_id, struct pt_regs * regs)
{
	int status;
	int pass_counter = 0;
	int first_multi = 0;
	struct async_struct * info;
	struct pci_rs_multiport_struct *multi;
	
#ifdef SERIAL_DEBUG_INTR
	printk("pci_rs_interrupt_single(%d)...", irq);
#endif
	
	info = IRQ_ports[irq];
	if (!info || !info->tty)
		return;

	multi = &pci_rs_multiport[irq];
	if (multi->port_monitor)
		first_multi = inb(multi->port_monitor);

	do {
		status = serial_inp(info, UART_LSR) & info->read_status_mask;
#ifdef SERIAL_DEBUG_INTR
		printk("status = %x...", status);
#endif
		if (status & UART_LSR_DR)
			receive_chars(info, &status);
		check_modem_status(info);
		if (status & UART_LSR_THRE)
			transmit_chars(info, 0);
		if (pass_counter++ > PCI_RS_ISR_PASS_LIMIT) {
#if 0
			printk("pci_rs_single loop break.\n");
#endif
			break;
		}
	} while (!(serial_in(info, UART_IIR) & UART_IIR_NO_INT));
	info->last_active = jiffies;
	if (multi->port_monitor)
		printk("rs port monitor (single) irq %d: 0x%x, 0x%x\n",
		       info->irq, first_multi, inb(multi->port_monitor));
#ifdef SERIAL_DEBUG_INTR
	printk("end.\n");
#endif
}

/*
 * This is the serial driver's for multiport boards
 */
static void pci_rs_interrupt_multi(int irq, void *dev_id, struct pt_regs * regs)
{
	int status;
	struct async_struct * info;
	int pass_counter = 0;
	int first_multi= 0;
	struct pci_rs_multiport_struct *multi;

#ifdef SERIAL_DEBUG_INTR
	printk("pci_rs_interrupt_multi(%d)...", irq);
#endif
	
	info = IRQ_ports[irq];
	if (!info)
		return;
	multi = &pci_rs_multiport[irq];
	if (!multi->port1) {
		/* Should never happen */
		printk("pci_rs_interrupt_multi: NULL port1!\n");
		return;
	}
	if (multi->port_monitor)
		first_multi = inb(multi->port_monitor);
	
	while (1) {
		if (!info->tty ||
		    (serial_in(info, UART_IIR) & UART_IIR_NO_INT))
			goto next;

		info->last_active = jiffies;

		status = serial_inp(info, UART_LSR) & info->read_status_mask;
#ifdef SERIAL_DEBUG_INTR
		printk("status = %x...", status);
#endif
		if (status & UART_LSR_DR)
			receive_chars(info, &status);
		check_modem_status(info);
		if (status & UART_LSR_THRE)
			transmit_chars(info, 0);

	next:
		info = info->next_port;
		if (info)
			continue;

		info = IRQ_ports[irq];
		if (pass_counter++ > PCI_RS_ISR_PASS_LIMIT) {
#if 1
			printk("pci_rs_multi loop break\n");
#endif
			break; 	/* Prevent infinite loops */
		}
		if (multi->port_monitor)
			printk("rs port monitor irq %d: 0x%x, 0x%x\n",
			       info->irq, first_multi,
			       inb(multi->port_monitor));
		if ((inb(multi->port1) & multi->mask1) != multi->match1)
			continue;
		if (!multi->port2)
			break;
		if ((inb(multi->port2) & multi->mask2) != multi->match2)
			continue;
		if (!multi->port3)
			break;
		if ((inb(multi->port3) & multi->mask3) != multi->match3)
			continue;
		if (!multi->port4)
			break;
		if ((inb(multi->port4) & multi->mask4) == multi->match4)
			continue;
		break;
	} 
#ifdef SERIAL_DEBUG_INTR
	printk("end.\n");
#endif
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
 * pci_rs_interrupt() has returned, BUT WITH INTERRUPTS TURNED ON.  This
 * is where time-consuming activities which can not be done in the
 * interrupt driver proper are done; the interrupt driver schedules
 * them using pci_rs_sched_event(), and they get done here.
 */
static void do_serial_bh(void)
{
	run_task_queue(&tq_pciserial);
}

static void do_softint(void *private_)
{
	struct async_struct	*info = (struct async_struct *) private_;
	struct tty_struct	*tty;
	
	tty = info->tty;
	if (!tty)
		return;

	if (clear_bit(PCI_RS_EVENT_WRITE_WAKEUP, &info->event)) {
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
 * 	do_serial_hangup() -> tty->hangup() -> pci_rs_hangup()
 * 
 */
static void do_serial_hangup(void *private_)
{
	struct async_struct	*info = (struct async_struct *) private_;
	struct tty_struct	*tty;
	
	tty = info->tty;
	if (!tty)
		return;

	tty_hangup(tty);
}


/*
 * This subroutine is called when the PCI_RS_TIMER goes off.  It is used
 * by the serial driver to handle ports that do not have an interrupt
 * (irq=0).  This doesn't work very well for 16450's, but gives barely
 * passable results for a 16550A.  (Although at the expense of much
 * CPU overhead).
 */
static void pci_rs_timer(void)
{
	static unsigned long last_strobe = 0;
	struct async_struct *info;
	unsigned int	i;
	unsigned long flags;

	if ((jiffies - last_strobe) >= PCI_RS_STROBE_TIME) {
		for (i=1; i < 16; i++) {
			info = IRQ_ports[i];
			if (!info)
				continue;
			save_flags(flags); cli();
			if (info->next_port) {
				do {
					serial_out(info, UART_IER, 0);
					info->IER |= UART_IER_THRI;
					serial_out(info, UART_IER, info->IER);
					info = info->next_port;
				} while (info);
				if (pci_rs_multiport[i].port1)
					pci_rs_interrupt_multi(i, NULL, NULL);
				else
					pci_rs_interrupt(i, NULL, NULL);
			} else
				pci_rs_interrupt_single(i, NULL, NULL);
			restore_flags(flags);
		}
	}
	last_strobe = jiffies;
	timer_table[PCI_RS_TIMER].expires = jiffies + PCI_RS_STROBE_TIME;
	timer_active |= 1 << PCI_RS_TIMER;

	if (IRQ_ports[0]) {
		save_flags(flags); cli();
		pci_rs_interrupt(0, NULL, NULL);
		restore_flags(flags);

		timer_table[PCI_RS_TIMER].expires = jiffies + IRQ_timeout[0] - 2;
	}
}

/*
 * ---------------------------------------------------------------
 * Low level utility subroutines for the serial driver:  routines to
 * figure out the appropriate timeout for an interrupt chain, routines
 * to initialize and startup a serial port, and routines to shutdown a
 * serial port.  Useful stuff like that.
 * ---------------------------------------------------------------
 */

/*
 * This routine figures out the correct timeout for a particular IRQ.
 * It uses the smallest timeout of all of the serial ports in a
 * particular interrupt chain.  Now only used for IRQ 0....
 */
static void figure_IRQ_timeout(int irq)
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

static int startup(struct async_struct * info)
{
	unsigned short ICP;
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
	 * (they will be reenabled in change_speed())
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
		if (IRQ_ports[info->irq]) {
			free_irq(info->irq, NULL);
			if (pci_rs_multiport[info->irq].port1)
				handler = pci_rs_interrupt_multi;
			else
				handler = pci_rs_interrupt;
		} else 
			handler = pci_rs_interrupt_single;

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
	if (info->flags & ASYNC_FOURPORT) {
		info->MCR = UART_MCR_DTR | UART_MCR_RTS;
		info->MCR_noint = UART_MCR_DTR | UART_MCR_OUT1;
	} else {
		info->MCR = UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2;
		info->MCR_noint = UART_MCR_DTR | UART_MCR_RTS;
	}
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
	
	if (info->flags & ASYNC_FOURPORT) {
		/* Enable interrupts on the AST Fourport board */
		ICP = (info->port & 0xFE0) | 0x01F;
		outb_p(0x80, ICP);
		(void) inb_p(ICP);
	}

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
	figure_IRQ_timeout(info->irq);

	/*
	 * Set up serial timers...
	 */
	timer_table[PCI_RS_TIMER].expires = jiffies + 2*HZ/100;
	timer_active |= 1 << PCI_RS_TIMER;

	/*
	 * and set the speed of the serial port
	 */
	change_speed(info);

	info->flags |= ASYNC_INITIALIZED;
	restore_flags(flags);
	return 0;
}

/*
 * This routine will shutdown a serial port; interrupts are disabled, and
 * DTR is dropped if the hangup on close termio flag is on.
 */
static void shutdown(struct async_struct * info)
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
	figure_IRQ_timeout(info->irq);
	
	/*
	 * Free the IRQ, if necessary
	 */
	if (info->irq && (!IRQ_ports[info->irq] ||
			  !IRQ_ports[info->irq]->next_port)) {
		if (IRQ_ports[info->irq]) {
			free_irq(info->irq, NULL);
			retval = request_irq(info->irq, pci_rs_interrupt_single,
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
	if (info->flags & ASYNC_FOURPORT) {
		/* reset interrupts on the AST Fourport board */
		(void) inb((info->port & 0xFE0) | 0x01F);
	}
	
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

/*
 * This routine is called to set the UART divisor registers to match
 * the specified baud rate for a serial port.
 */
static void change_speed(struct async_struct *info)
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
	} else if (baud_table[i] == 134) {
		quot = (2*info->baud_base / 269);
		info->timeout = (info->xmit_fifo_size*HZ*30/269) + 2;
	} else if (baud_table[i]) {
		quot = info->baud_base / baud_table[i];
		info->timeout = (info->xmit_fifo_size*HZ*15/baud_table[i]) + 2;
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
	if (fcr & UART_FCR_ENABLE_FIFO) {
		/* DSP emulated UARTs (Lucent Venus 167x) need two steps */
		serial_outp(info, UART_FCR, UART_FCR_ENABLE_FIFO);
	}
	serial_outp(info, UART_FCR, fcr); 	/* set fcr */
	restore_flags(flags);
}

static void pci_rs_put_char(struct tty_struct *tty, unsigned char ch)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	unsigned long flags;

	if (serial_paranoia_check(info, tty->device, "pci_rs_put_char"))
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

static void pci_rs_flush_chars(struct tty_struct *tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	unsigned long flags;
				
	if (serial_paranoia_check(info, tty->device, "pci_rs_flush_chars"))
		return;

	if (info->xmit_cnt <= 0 || tty->stopped || tty->hw_stopped ||
	    !info->xmit_buf)
		return;

	save_flags(flags); cli();
	info->IER |= UART_IER_THRI;
	serial_out(info, UART_IER, info->IER);
	restore_flags(flags);
}

static int pci_rs_write(struct tty_struct * tty, int from_user,
			const unsigned char *buf, int count)
{
	int	c, total = 0;
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	unsigned long flags;
				
	if (serial_paranoia_check(info, tty->device, "pci_rs_write"))
		return 0;

	if (!tty || !info->xmit_buf || !pci_tmp_buf)
		return 0;
	    
	if (from_user)
		down(&pci_tmp_buf_sem);
	save_flags(flags);
	while (1) {
		cli();		
		c = MIN(count, MIN(SERIAL_XMIT_SIZE - info->xmit_cnt - 1,
				   SERIAL_XMIT_SIZE - info->xmit_head));
		if (c <= 0)
			break;

		if (from_user) {
			memcpy_fromfs(pci_tmp_buf, buf, c);
			c = MIN(c, MIN(SERIAL_XMIT_SIZE - info->xmit_cnt - 1,
				       SERIAL_XMIT_SIZE - info->xmit_head));
			memcpy(info->xmit_buf+info->xmit_head, pci_tmp_buf, c);
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
		up(&pci_tmp_buf_sem);
	if (info->xmit_cnt && !tty->stopped && !tty->hw_stopped &&
	    !(info->IER & UART_IER_THRI)) {
		info->IER |= UART_IER_THRI;
		serial_out(info, UART_IER, info->IER);
	}
	restore_flags(flags);
	return total;
}

static int pci_rs_write_room(struct tty_struct *tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	int	ret;
				
	if (serial_paranoia_check(info, tty->device, "pci_rs_write_room"))
		return 0;
	ret = SERIAL_XMIT_SIZE - info->xmit_cnt - 1;
	if (ret < 0)
		ret = 0;
	return ret;
}

static int pci_rs_chars_in_buffer(struct tty_struct *tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
				
	if (serial_paranoia_check(info, tty->device, "pci_rs_chars_in_buffer"))
		return 0;
	return info->xmit_cnt;
}

static void pci_rs_flush_buffer(struct tty_struct *tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	unsigned long flags;
				
	if (serial_paranoia_check(info, tty->device, "pci_rs_flush_buffer"))
		return;
	save_flags(flags); cli();
	info->xmit_cnt = info->xmit_head = info->xmit_tail = 0;
	restore_flags(flags);
	wake_up_interruptible(&tty->write_wait);
	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
	    tty->ldisc.write_wakeup)
		(tty->ldisc.write_wakeup)(tty);
}

/*
 * ------------------------------------------------------------
 * pci_rs_throttle()
 * 
 * This routine is called by the upper-layer tty layer to signal that
 * incoming characters should be throttled.
 * ------------------------------------------------------------
 */
static void pci_rs_throttle(struct tty_struct * tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	unsigned long flags;
#ifdef SERIAL_DEBUG_THROTTLE
	char	buf[64];
	
	printk("throttle %s: %d....\n", _tty_name(tty, buf),
	       tty->ldisc.chars_in_buffer(tty));
#endif

	if (serial_paranoia_check(info, tty->device, "pci_rs_throttle"))
		return;
	
	if (I_IXOFF(tty))
		info->x_char = STOP_CHAR(tty);

	info->MCR &= ~UART_MCR_RTS;
	info->MCR_noint &= ~UART_MCR_RTS;
	save_flags(flags); cli();
	serial_out(info, UART_MCR, info->MCR);
	restore_flags(flags);
}

static void pci_rs_unthrottle(struct tty_struct * tty)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;
	unsigned long flags;
#ifdef SERIAL_DEBUG_THROTTLE
	char	buf[64];
	
	printk("unthrottle %s: %d....\n", _tty_name(tty, buf),
	       tty->ldisc.chars_in_buffer(tty));
#endif

	if (serial_paranoia_check(info, tty->device, "pci_rs_unthrottle"))
		return;
	
	if (I_IXOFF(tty)) {
		if (info->x_char)
			info->x_char = 0;
		else
			info->x_char = START_CHAR(tty);
	}
	info->MCR |= UART_MCR_RTS;
	info->MCR_noint |= UART_MCR_RTS;
	save_flags(flags); cli();
	serial_out(info, UART_MCR, info->MCR);
	restore_flags(flags);
}

/*
 * ------------------------------------------------------------
 * pci_rs_ioctl() and friends
 * ------------------------------------------------------------
 */

static int get_serial_info(struct async_struct * info,
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

static int set_serial_info(struct async_struct * info,
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
			if ((info != &pci_rs_table[i]) &&
			    (pci_rs_table[i].port == new_serial.port) &&
			    pci_rs_table[i].type)
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

	release_region(info->port,8);
	if (change_port || change_irq) {
		/*
		 * We need to shutdown the serial port at the old
		 * port/irq combination.
		 */
		shutdown(info);
		info->irq = new_serial.irq;
		info->port = new_serial.port;
		info->hub6 = new_serial.hub6;
	}
	if(info->type != PORT_UNKNOWN)
		request_region(info->port,8,"serial(set)");

	
check_and_exit:
	if (!info->port || !info->type)
		return 0;
	if (info->flags & ASYNC_INITIALIZED) {
		if (((old_info.flags & ASYNC_SPD_MASK) !=
		     (info->flags & ASYNC_SPD_MASK)) ||
		    (old_info.custom_divisor != info->custom_divisor))
			change_speed(info);
	} else
		retval = startup(info);
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
static int get_lsr_info(struct async_struct * info, unsigned int *value)
{
	unsigned char status;
	unsigned int result;
	unsigned long flags;

	save_flags(flags); cli();
	status = serial_in(info, UART_LSR);
	restore_flags(flags);
	result = ((status & UART_LSR_TEMT) ? TIOCSER_TEMT : 0);
	put_user(result,value);
	return 0;
}


static int get_modem_info(struct async_struct * info, unsigned int *value)
{
	unsigned char control, status;
	unsigned int result;
	unsigned long flags;

	control = info->MCR;
	save_flags(flags); cli();
	status = serial_in(info, UART_MSR);
	restore_flags(flags);
	result =  ((control & UART_MCR_RTS) ? TIOCM_RTS : 0)
		| ((control & UART_MCR_DTR) ? TIOCM_DTR : 0)
		| ((status  & UART_MSR_DCD) ? TIOCM_CAR : 0)
		| ((status  & UART_MSR_RI) ? TIOCM_RNG : 0)
		| ((status  & UART_MSR_DSR) ? TIOCM_DSR : 0)
		| ((status  & UART_MSR_CTS) ? TIOCM_CTS : 0);
	put_user(result,value);
	return 0;
}

static int set_modem_info(struct async_struct * info, unsigned int cmd,
			  unsigned int *value)
{
	int error;
	unsigned int arg;
	unsigned long flags;

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
	save_flags(flags); cli();
	serial_out(info, UART_MCR, info->MCR);
	restore_flags(flags);
	return 0;
}

static int do_autoconfig(struct async_struct * info)
{
	int			retval;
	unsigned long flags;
	
	if (!suser())
		return -EPERM;
	
	if (info->count > 1)
		return -EBUSY;
	
	shutdown(info);

	save_flags(flags); cli();
	autoconfig(info);
	restore_flags(flags);

	retval = startup(info);
	if (retval)
		return retval;
	return 0;
}


/*
 * This routine sends a break character out the serial port.
 */
static void send_break(	struct async_struct * info, int duration)
{
	unsigned long flags;

	if (!info->port)
		return;
	current->state = TASK_INTERRUPTIBLE;
	current->timeout = jiffies + duration;
	save_flags(flags); cli();
	serial_out(info, UART_LCR, serial_inp(info, UART_LCR) | UART_LCR_SBC);
	schedule();
	serial_out(info, UART_LCR, serial_inp(info, UART_LCR) & ~UART_LCR_SBC);
	restore_flags(flags);
}

static int get_multiport_struct(struct async_struct * info,
				struct serial_multiport_struct *retinfo)
{
	struct serial_multiport_struct ret;
	struct pci_rs_multiport_struct *multi;
	
	multi = &pci_rs_multiport[info->irq];

	ret.port_monitor = multi->port_monitor;
	
	ret.port1 = multi->port1;
	ret.mask1 = multi->mask1;
	ret.match1 = multi->match1;
	
	ret.port2 = multi->port2;
	ret.mask2 = multi->mask2;
	ret.match2 = multi->match2;
	
	ret.port3 = multi->port3;
	ret.mask3 = multi->mask3;
	ret.match3 = multi->match3;
	
	ret.port4 = multi->port4;
	ret.mask4 = multi->mask4;
	ret.match4 = multi->match4;

	ret.irq = info->irq;

	memcpy_tofs(retinfo,&ret,sizeof(*retinfo));
	return 0;
	
}

static int set_multiport_struct(struct async_struct * info,
				struct serial_multiport_struct *in_multi)
{
	struct serial_multiport_struct new_multi;
	struct pci_rs_multiport_struct *multi;
	int	was_multi, now_multi;
	int	retval;
	void (*handler)(int, void *, struct pt_regs *);

	if (!suser())
		return -EPERM;
	if (!in_multi)
		return -EFAULT;
	memcpy_fromfs(&new_multi, in_multi,
		      sizeof(struct serial_multiport_struct));

	if (new_multi.irq != info->irq || info->irq == 0 ||
	    !IRQ_ports[info->irq])
		return -EINVAL;

	multi = &pci_rs_multiport[info->irq];
	was_multi = (multi->port1 != 0);
	
	multi->port_monitor = new_multi.port_monitor;
	
	if (multi->port1)
		release_region(multi->port1,1);
	multi->port1 = new_multi.port1;
	multi->mask1 = new_multi.mask1;
	multi->match1 = new_multi.match1;
	if (multi->port1)
		request_region(multi->port1,1,"serial(multiport1)");

	if (multi->port2)
		release_region(multi->port2,1);
	multi->port2 = new_multi.port2;
	multi->mask2 = new_multi.mask2;
	multi->match2 = new_multi.match2;
	if (multi->port2)
		request_region(multi->port2,1,"serial(multiport2)");

	if (multi->port3)
		release_region(multi->port3,1);
	multi->port3 = new_multi.port3;
	multi->mask3 = new_multi.mask3;
	multi->match3 = new_multi.match3;
	if (multi->port3)
		request_region(multi->port3,1,"serial(multiport3)");

	if (multi->port4)
		release_region(multi->port4,1);
	multi->port4 = new_multi.port4;
	multi->mask4 = new_multi.mask4;
	multi->match4 = new_multi.match4;
	if (multi->port4)
		request_region(multi->port4,1,"serial(multiport4)");

	now_multi = (multi->port1 != 0);
	
	if (IRQ_ports[info->irq]->next_port &&
	    (was_multi != now_multi)) {
		free_irq(info->irq, NULL);
		if (now_multi)
			handler = pci_rs_interrupt_multi;
		else
			handler = pci_rs_interrupt;

		retval = request_irq(info->irq, handler, IRQ_T(info),
				     "serial", NULL);
		if (retval) {
			printk("Couldn't reallocate serial interrupt "
			       "driver!!\n");
		}
	}

	return 0;
}

static int pci_rs_ioctl(struct tty_struct *tty, struct file * file,
			unsigned int cmd, unsigned long arg)
{
	int error;
	struct async_struct * info = (struct async_struct *)tty->driver_data;
	int retval;
	unsigned long flags;
	struct async_icount cprev, cnow;	/* kernel counter temps */
	struct serial_icounter_struct *p_cuser;	/* user space */

	if (serial_paranoia_check(info, tty->device, "pci_rs_ioctl"))
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
				send_break(info, HZ/4);	/* 1/4 second */
			return 0;
		case TCSBRKP:	/* support for POSIX tcsendbreak() */
			retval = tty_check_change(tty);
			if (retval)
				return retval;
			tty_wait_until_sent(tty, 0);
			send_break(info, arg ? arg*(HZ/10) : HZ/4);
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
			return get_modem_info(info, (unsigned int *) arg);
		case TIOCMBIS:
		case TIOCMBIC:
		case TIOCMSET:
			return set_modem_info(info, cmd, (unsigned int *) arg);
		case TIOCGSERIAL:
			error = verify_area(VERIFY_WRITE, (void *) arg,
						sizeof(struct serial_struct));
			if (error)
				return error;
			return get_serial_info(info,
					       (struct serial_struct *) arg);
		case TIOCSSERIAL:
			error = verify_area(VERIFY_READ, (void *) arg,
						sizeof(struct serial_struct));
			if (error)
				return error;
			return set_serial_info(info,
					       (struct serial_struct *) arg);
		case TIOCSERCONFIG:
			return do_autoconfig(info);

		case TIOCSERGWILD:
			error = verify_area(VERIFY_WRITE, (void *) arg,
					    sizeof(int));
			if (error)
				return error;
			put_fs_long(pci_rs_wild_int_mask, (unsigned long *) arg);
			return 0;

		case TIOCSERGETLSR: /* Get line status register */
			error = verify_area(VERIFY_WRITE, (void *) arg,
				sizeof(unsigned int));
			if (error)
				return error;
			else
			    return get_lsr_info(info, (unsigned int *) arg);

		case TIOCSERGSTRUCT:
			error = verify_area(VERIFY_WRITE, (void *) arg,
						sizeof(struct async_struct));
			if (error)
				return error;
			memcpy_tofs((struct async_struct *) arg,
				    info, sizeof(struct async_struct));
			return 0;
			
		case TIOCSERGETMULTI:
			error = verify_area(VERIFY_WRITE, (void *) arg,
				    sizeof(struct serial_multiport_struct));
			if (error)
				return error;
			return get_multiport_struct(info,
				       (struct serial_multiport_struct *) arg);
		case TIOCSERSETMULTI:
			error = verify_area(VERIFY_READ, (void *) arg,
				    sizeof(struct serial_multiport_struct));
			if (error)
				return error;
			return set_multiport_struct(info,
				       (struct serial_multiport_struct *) arg);
		/*
		 * Wait for any of the 4 modem inputs (DCD,RI,DSR,CTS) to change
		 * - mask passed in arg for lines of interest
 		 *   (use |'ed TIOCM_RNG/DSR/CD/CTS for masking)
		 * Caller should use TIOCGICOUNT to see which one it was
		 */
		 case TIOCMIWAIT:
			save_flags(flags); cli();
			cprev = info->icount;	/* note the counters on entry */
			restore_flags(flags);
			while (1) {
				interruptible_sleep_on(&info->delta_msr_wait);
				/* see if a signal did it */
				if (current->signal & ~current->blocked)
					return -ERESTARTSYS;
				save_flags(flags); cli();
				cnow = info->icount;	/* atomic copy */
				restore_flags(flags);
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
			save_flags(flags); cli();
			cnow = info->icount;
			restore_flags(flags);
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

static void pci_rs_set_termios(struct tty_struct *tty, struct termios *old_termios)
{
	struct async_struct *info = (struct async_struct *)tty->driver_data;

	if (   (tty->termios->c_cflag == old_termios->c_cflag)
	    && (   RELEVANT_IFLAG(tty->termios->c_iflag) 
		== RELEVANT_IFLAG(old_termios->c_iflag)))
	  return;

	change_speed(info);

	if ((old_termios->c_cflag & CRTSCTS) &&
	    !(tty->termios->c_cflag & CRTSCTS)) {
		tty->hw_stopped = 0;
		pci_rs_start(tty);
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

/*
 * ------------------------------------------------------------
 * pci_rs_close()
 * 
 * This routine is called when the serial port gets closed.  First, we
 * wait for the last remaining data to be sent.  Then, we unlink its
 * async structure from the interrupt chain if necessary, and we free
 * that IRQ if nothing is left in the chain.
 * ------------------------------------------------------------
 */
static void pci_rs_close(struct tty_struct *tty, struct file * filp)
{
	struct async_struct * info = (struct async_struct *)tty->driver_data;
	unsigned long flags;
	unsigned long timeout;

	if (!info || serial_paranoia_check(info, tty->device, "pci_rs_close"))
		return;
	
	save_flags(flags); cli();
	
	if (tty_hung_up_p(filp)) {
		DBG_CNT("before DEC-hung");
		MOD_DEC_USE_COUNT;
		restore_flags(flags);
		return;
	}
	
#ifdef SERIAL_DEBUG_OPEN
	printk("pci_rs_close ttys%d, count = %d\n", info->line, info->count);
#endif
	if ((tty->count == 1) && (info->count != 1)) {
		/*
		 * Uh, oh.  tty->count is 1, which means that the tty
		 * structure will be freed.  Info->count should always
		 * be one in these conditions.  If it's greater than
		 * one, we've got real problems, since it means the
		 * serial port won't be shutdown.
		 */
		printk("pci_rs_close: bad serial port count; tty->count is 1, "
		       "info->count is %d\n", info->count);
		info->count = 1;
	}
	if (--info->count < 0) {
		printk("pci_rs_close: bad serial port count for ttys%d: %d\n",
		       info->line, info->count);
		info->count = 0;
	}
	if (info->count) {
		DBG_CNT("before DEC-2");
		MOD_DEC_USE_COUNT;
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
	shutdown(info);
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
	MOD_DEC_USE_COUNT;
	restore_flags(flags);
}

/*
 * pci_rs_hangup() --- called by tty_hangup() when a hangup is signaled.
 */
static void pci_rs_hangup(struct tty_struct *tty)
{
	struct async_struct * info = (struct async_struct *)tty->driver_data;
	
	if (serial_paranoia_check(info, tty->device, "pci_rs_hangup"))
		return;
	
	pci_rs_flush_buffer(tty);
	shutdown(info);
	info->event = 0;
	info->count = 0;
	info->flags &= ~(ASYNC_NORMAL_ACTIVE|ASYNC_CALLOUT_ACTIVE);
	info->tty = 0;
	wake_up_interruptible(&info->open_wait);
}

/*
 * ------------------------------------------------------------
 * pci_rs_open() and friends
 * ------------------------------------------------------------
 */
static int block_til_ready(struct tty_struct *tty, struct file * filp,
			   struct async_struct *info)
{
	struct wait_queue wait = { current, NULL };
	int		retval;
	int		do_clocal = 0;
	unsigned long	flags;

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
	 * pci_rs_close() knows when to free things.  We restore it upon
	 * exit, either normal or abnormal.
	 */
	retval = 0;
	add_wait_queue(&info->open_wait, &wait);
#ifdef SERIAL_DEBUG_OPEN
	printk("block_til_ready before block: ttys%d, count = %d\n",
	       info->line, info->count);
#endif
	save_flags(flags); cli();
	if (!tty_hung_up_p(filp)) 
		info->count--;
	restore_flags(flags);
	info->blocked_open++;
	while (1) {
		save_flags(flags); cli();
		if (!(info->flags & ASYNC_CALLOUT_ACTIVE))
			serial_out(info, UART_MCR,
				   serial_inp(info, UART_MCR) |
				   (UART_MCR_DTR | UART_MCR_RTS));
		restore_flags(flags);
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

/*
 * This routine is called whenever a serial port is opened.  It
 * enables interrupts for a serial port, linking in its async structure into
 * the IRQ chain.   It also performs the serial-specific
 * initialization for the tty structure.
 */
static int pci_rs_open(struct tty_struct *tty, struct file * filp)
{
	struct async_struct	*info;
	int 			retval, line;
	unsigned long		page;

	line = MINOR(tty->device) - tty->driver.minor_start;
	if ((line < 0) || (line >= NR_PORTS))
		return -ENODEV;
	info = pci_rs_table + line;
	if (serial_paranoia_check(info, tty->device, "pci_rs_open"))
		return -ENODEV;

#ifdef SERIAL_DEBUG_OPEN
	printk("pci_rs_open %s%d, count = %d\n", tty->driver.name, info->line,
	       info->count);
#endif
	info->count++;
	tty->driver_data = info;
	info->tty = tty;

	if (!pci_tmp_buf) {
		page = get_free_page(GFP_KERNEL);
		if (!page)
			return -ENOMEM;
		if (pci_tmp_buf)
			free_page(page);
		else
			pci_tmp_buf = (unsigned char *) page;
	}
	
	/*
	 * Start up serial port
	 */
	retval = startup(info);
	if (retval)
		return retval;

	MOD_INC_USE_COUNT;
	retval = block_til_ready(tty, filp, info);
	if (retval) {
#ifdef SERIAL_DEBUG_OPEN
		printk("pci_rs_open returning after block_til_ready with %d\n",
		       retval);
#endif
		return retval;
	}

	if ((info->count == 1) && (info->flags & ASYNC_SPLIT_TERMIOS)) {
		if (tty->driver.subtype == SERIAL_TYPE_NORMAL)
			*tty->termios = info->normal_termios;
		else 
			*tty->termios = info->callout_termios;
		change_speed(info);
	}

	info->session = current->session;
	info->pgrp = current->pgrp;

#ifdef SERIAL_DEBUG_OPEN
	printk("pci_rs_open ttys%d successful...", info->line);
#endif
	return 0;
}

/*
 * ---------------------------------------------------------------------
 * pci_rs_init() and friends
 *
 * pci_rs_init() is called at boot-time to initialize the serial driver.
 * ---------------------------------------------------------------------
 */

/*
 * This routine prints out the appropriate serial driver version
 * number, and identifies which options were configured into this
 * driver.
 */
static void show_serial_version(void)
{
 	printk(KERN_INFO "%s version %s\n", serial_name, serial_version);
}

/*
 * This routine is called by pci_rs_init() to initialize a specific serial
 * port.  It determines what type of UART chip this serial port is
 * using: 8250, 16450, 16550, 16550A.  The important question is
 * whether or not this UART is a 16550A or not, since this will
 * determine whether or not we can use its FIFO features or not.
 */
static void autoconfig(struct async_struct * info)
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
		/* this is needed for slow-to-respond PCI modems */
		udelay(10000);
		status1 = serial_inp(info, UART_MSR);
		status1 &= 0xF0;
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
	/* this is needed for slow-to-respond PCI modems */
	udelay(10000);
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
	request_region(info->port,8,"serial(auto)");

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

/*
 * The serial driver boot-time initialization code!
 */
int pci_rs_init(void)
{
	struct async_struct * info;
	unsigned char bus, devfn, irq;
	unsigned int ioaddrs[6];
	int i, num_ioaddrs = 0;
	
	i = pcibios_find_class(PCI_CLASS_COMMUNICATION_SERIAL << 8, 0,
			       &bus, &devfn);
	if (i != PCIBIOS_SUCCESSFUL)
		/* some PCI modems ID themselves as COMMUNICATION_OTHER */
		i = pcibios_find_class(PCI_CLASS_COMMUNICATION_OTHER << 8, 0,
			       &bus, &devfn);
	if(i != PCIBIOS_SUCCESSFUL)
		return 0;

	for(i = 0; i < NR_PORTS; i++)
		ioaddrs[i] = 0;
	/* find all I/O base addresses for this driver */
	for(i = PCI_BASE_ADDRESS_0; i <= PCI_BASE_ADDRESS_5; i += 4) {
		pcibios_read_config_dword(bus, devfn, i, &ioaddrs[num_ioaddrs]);
		if(!ioaddrs[num_ioaddrs])
			continue;
		if (ioaddrs[num_ioaddrs] & PCI_BASE_ADDRESS_SPACE_IO)
			ioaddrs[num_ioaddrs++] &= PCI_BASE_ADDRESS_IO_MASK;
	}
	if(!ioaddrs[0])
		return 0;
	pcibios_read_config_byte(bus, devfn, PCI_INTERRUPT_LINE, &irq);
	printk("pciserial: Found PCI serial controller at bus(%x) devfn(%x) "
	       "IO(%x) irq(%d)\n", bus, devfn, ioaddrs[0], irq);

	init_bh(PCISERIAL_BH, do_serial_bh);
	timer_table[PCI_RS_TIMER].fn = pci_rs_timer;
	timer_table[PCI_RS_TIMER].expires = 0;

	for (i = 0; i < 16; i++) {
		IRQ_ports[i] = 0;
		IRQ_timeout[i] = 0;
		memset(&pci_rs_multiport[i], 0, sizeof(struct pci_rs_multiport_struct));
	}
	
	show_serial_version();

	/* Initialize the tty_driver structure */
	
	memset(&serial_driver, 0, sizeof(struct tty_driver));
	serial_driver.magic = TTY_DRIVER_MAGIC;
	serial_driver.name = "ttyS";
	serial_driver.major = TTY_MAJOR;
	serial_driver.minor_start = 64 + 4;
	serial_driver.num = NR_PORTS;
	serial_driver.type = TTY_DRIVER_TYPE_SERIAL;
	serial_driver.subtype = SERIAL_TYPE_NORMAL;
	serial_driver.init_termios = tty_std_termios;
	serial_driver.init_termios.c_cflag =
		B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	serial_driver.flags = TTY_DRIVER_REAL_RAW;
	serial_driver.refcount = &serial_refcount;
	serial_driver.table = serial_table;
	serial_driver.termios = serial_termios;
	serial_driver.termios_locked = serial_termios_locked;

	serial_driver.open = pci_rs_open;
	serial_driver.close = pci_rs_close;
	serial_driver.write = pci_rs_write;
	serial_driver.put_char = pci_rs_put_char;
	serial_driver.flush_chars = pci_rs_flush_chars;
	serial_driver.write_room = pci_rs_write_room;
	serial_driver.chars_in_buffer = pci_rs_chars_in_buffer;
	serial_driver.flush_buffer = pci_rs_flush_buffer;
	serial_driver.ioctl = pci_rs_ioctl;
	serial_driver.throttle = pci_rs_throttle;
	serial_driver.unthrottle = pci_rs_unthrottle;
	serial_driver.set_termios = pci_rs_set_termios;
	serial_driver.stop = pci_rs_stop;
	serial_driver.start = pci_rs_start;
	serial_driver.hangup = pci_rs_hangup;

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
	
	for (i = 0, info = pci_rs_table; i < NR_PORTS; i++,info++) {
		int j;
		/* check each IO address (used ones get zeroed below) */
		for (j = 0; j < num_ioaddrs; j++) {
		  if (!ioaddrs[j])
			continue;

		  info->magic = SERIAL_MAGIC;
		  info->line = i+4;
		  info->tty = 0;
		  info->type = PORT_UNKNOWN;
		  info->custom_divisor = 0;
		  info->close_delay = 5*HZ/10;
		  info->closing_wait = 30*HZ;
		  info->x_char = 0;
		  info->event = 0;
		  info->count = 0;
		  info->blocked_open = 0;
		  info->tqueue.routine = do_softint;
		  info->tqueue.data = info;
		  info->tqueue_hangup.routine = do_serial_hangup;
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
#ifdef CONFIG_COBALT_27
		  /* COBALT: We probed these from the PCI config space. */
		  info->irq = irq;
		  if(i < NR_PORTS)
			info->port = ioaddrs[j];
		  else
			info->port = 0;
#else
		  if (info->irq == 2)
			info->irq = 9;
#endif
		  if (info->type == PORT_UNKNOWN) {
			if (!(info->flags & ASYNC_BOOT_AUTOCONF))
				continue;
			autoconfig(info);
			if (info->type == PORT_UNKNOWN)
				continue;
		  }
		  printk(KERN_INFO "tty%02d%s at 0x%04x (irq = %d)", 
			info->line, 
			(info->flags & ASYNC_FOURPORT) ? " FourPort" : "",
		       info->port, info->irq);

		  /* flag it as used */
		  ioaddrs[j] = 0;

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
		  break;
		}
	}
	return 0;
}

#ifdef MODULE
int init_module(void)
{
	return pci_rs_init();
}

void cleanup_module(void)
{
	unsigned long flags;
	int e1, e2;
	int i;

	printk(KERN_INFO "Unloading %s\n", serial_name);
	save_flags(flags); cli();
	timer_active &= ~(1 << PCI_RS_TIMER);
	timer_table[PCI_RS_TIMER].fn = NULL;
	timer_table[PCI_RS_TIMER].expires = 0;
	if ((e1 = tty_unregister_driver(&serial_driver)))
		printk("PCISERIAL: failed to unregister serial driver (%d)\n",
			e1);
	if ((e2 = tty_unregister_driver(&callout_driver)))
		printk("PCISERIAL: failed to unregister callout driver (%d)\n",
			e2);
	restore_flags(flags);

	for (i = 0; i < NR_PORTS; i++) {
		if (pci_rs_table[i].type != PORT_UNKNOWN)
			release_region(pci_rs_table[i].port, 8);
	}
	if (pci_tmp_buf) {
		free_page((unsigned long) pci_tmp_buf);
		pci_tmp_buf = NULL;
	}
}
#endif /* MODULE */
