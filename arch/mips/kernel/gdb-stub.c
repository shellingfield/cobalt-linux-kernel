/*
 *  arch/mips/kernel/gdb-stub.c
 *
 *  Originally written by Glenn Engel, Lake Stevens Instrument Division
 *
 *  Contributed by HP Systems
 *
 *  Modified for SPARC by Stu Grossman, Cygnus Support.
 *
 *  Modified for Linux/MIPS (and MIPS in general) by Andreas Busse
 *  Send complaints, suggestions etc. to <andy@waldorf-gmbh.de>
 *
 *  Copyright (C) 1995 Andreas Busse
 *
 * $Id: gdb-stub.c,v 1.4 1999/06/16 03:24:28 cjohnson Exp $
 */

/*
 *  To enable debugger support, two things need to happen.  One, a
 *  call to set_debug_traps() is necessary in order to allow any breakpoints
 *  or error conditions to be properly intercepted and reported to gdb.
 *  Two, a breakpoint needs to be generated to begin communication.  This
 *  is most easily accomplished by a call to breakpoint().  Breakpoint()
 *  simulates a breakpoint by executing a BREAK instruction.
 *
 *
 *    The following gdb commands are supported:
 *
 * command          function                               Return value
 *
 *    g             return the value of the CPU registers  hex data or ENN
 *    G             set the value of the CPU registers     OK or ENN
 *
 *    mAA..AA,LLLL  Read LLLL bytes at address AA..AA      hex data or ENN
 *    MAA..AA,LLLL: Write LLLL bytes at address AA.AA      OK or ENN
 *
 *    c             Resume at current address              SNN   ( signal NN)
 *    cAA..AA       Continue at address AA..AA             SNN
 *
 *    s             Step one instruction                   SNN
 *    sAA..AA       Step one instruction from AA..AA       SNN
 *
 *    k             kill
 *
 *    ?             What was the last sigval ?             SNN   (signal NN)
 *
 *    bBB..BB	    Set baud rate to BB..BB		   OK or BNN, then sets
 *							   baud rate
 *
 * All commands and responses are sent with a packet which includes a
 * checksum.  A packet consists of
 *
 * $<packet info>#<checksum>.
 *
 * where
 * <packet info> :: <characters representing the command or response>
 * <checksum>    :: < two hex digits computed as modulo 256 sum of <packetinfo>>
 *
 * When a packet is received, it is first acknowledged with either '+' or '-'.
 * '+' indicates a successful transfer.  '-' indicates a failed transfer.
 *
 * Example:
 *
 * Host:                  Reply:
 * $m0,10#2a               +$00010203040506070809101112131415#42
 *
 */

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/mm.h>

#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/gdb-stub.h>

/*
 * external low-level support routines
 */

extern int putDebugChar(char c);    /* write a single character      */
extern char getDebugChar(void);     /* read and return a single char */
extern void trap_low(void);

/*
 * breakpoint and test functions
 */
extern void breakpoint(void);
extern void breakinst(void);
extern void adel(void);

/*
 * local prototypes
 */

static void getpacket(char *buffer);
static void putpacket(char *buffer);
static int computeSignal(int tt);
static int hex(unsigned char ch);
static int hexToInt(char **ptr, int *intValue);
static unsigned char *mem2hex(char *mem, char *buf, int count, int may_fault);
void handle_exception(struct gdb_regs *regs);

/*
 * BUFMAX defines the maximum number of characters in inbound/outbound buffers
 * at least NUMREGBYTES*2 are needed for register packets
 */
#define BUFMAX 2048

static char input_buffer[BUFMAX];
static char output_buffer[BUFMAX];
static int initialized = 0;	/* !0 means we've been initialized */
static const char hexchars[]="0123456789abcdef";

/*
 * This "works" because printk uses a different backend than putDebugChar.
 * putDebugChar() sets the high bit, which is stripped by gdb then used
 * for its own protocol.  Gdb forwards output without the high bit on
 * straight to the user.  This hack give kernel printk and gdb together.
 */
#ifdef WAY_TOO_MUCH_DEBUG
#define dbg_out printk
#else
/* 
 * The compiler ought to know to zap the following
 */
#define dbg_out if (0) printk
#endif


/*
 * Convert ch from a hex digit to an int
 */
static int hex(unsigned char ch)
{
	if (ch >= 'a' && ch <= 'f')
		return ch-'a'+10;
	if (ch >= '0' && ch <= '9')
		return ch-'0';
	if (ch >= 'A' && ch <= 'F')
		return ch-'A'+10;
	return -1;
}

/*
 * scan for the sequence $<data>#<checksum>
 */
static void getpacket(char *buffer)
{
	unsigned char checksum;
	unsigned char xmitcsum;
	int i;
	int count;
	unsigned char ch;

	dbg_out(" kgetpacket\n");
	do {
		/*
		 * wait around for the start character,
		 * ignore all other characters
		 */
		while ((ch = (getDebugChar() & 0x7f)) != '$') {
			dbg_out(" kgtp skip %c\n", ch);
			;
		}
		dbg_out("got start\n");

		checksum = 0;
		xmitcsum = -1;
		count = 0;
	
		/*
		 * now, read until a # or end of buffer is found
		 */
		while (count < BUFMAX) {
			ch = getDebugChar() & 0x7f;
			dbg_out("kgtp keep %c\n", ch);
			if (ch == '#')
				break;
			checksum = checksum + ch;
			buffer[count] = ch;
			count = count + 1;
		}

		if (count >= BUFMAX)
			continue;

		buffer[count] = 0;

		if (ch == '#') {
			xmitcsum = hex(getDebugChar() & 0x7f) << 4;
			xmitcsum |= hex(getDebugChar() & 0x7f);

			if (checksum != xmitcsum) {
				dbg_out(" kgtp nak\n");
				putDebugChar('-');	/* failed checksum */
			}
			else {
				dbg_out(" kgtp ack\n");
				putDebugChar('+'); /* successful transfer */

				/*
				 * if a sequence char is present,
				 * reply the sequence ID
				 */
				if (buffer[2] == ':') {
					putDebugChar(buffer[0]);
					putDebugChar(buffer[1]);

					/*
					 * remove sequence chars from buffer
					 */
					count = strlen(buffer);
					for (i=3; i <= count; i++)
						buffer[i-3] = buffer[i];
				}
			}
		}
	}
	while (checksum != xmitcsum);

	dbg_out("kgetpacket read \"%s\"\n", buffer);
}

/*
 * send the packet in buffer.
 */
static void putpacket(char *buffer)
{
	unsigned char checksum;
	int count;
	unsigned char ch;

	/*
	 * $<packet info>#<checksum>.
	 */

	dbg_out(" ksend \"%s\"\n", buffer);
	while (1) {
		putDebugChar('$');
		checksum = 0;
		count = 0;

		while ((ch = buffer[count]) != 0) {
			if (!(putDebugChar(ch)))
				return;
			checksum += ch;
			count += 1;
		}

		putDebugChar('#');
		putDebugChar(hexchars[checksum >> 4]);
		putDebugChar(hexchars[checksum & 0xf]);

		ch = getDebugChar() & 0x7f;

		if (ch == '+') {
		    dbg_out("kptpk ack\n");
		    break;
		}
		else if (ch == '-')
		    dbg_out("kptpk nak\n");
		else dbg_out("kptpk bad ack/nak '%c'\n", ch);
	}
}


/*
 * Handshake with mips/mm/fault.c:do_page_fault()
 * to deal with bad memory addresses.
 */

volatile int debugmem_got_flt;
int debugmem_flt_set;

/*
 * Convert the memory pointed to by mem into hex, placing result in buf.
 * Return a pointer to the last char put in buf (null), in case of mem fault,
 * return 0.
 * If MAY_FAULT is non-zero, then we will handle memory faults by returning
 * a 0, else treat a fault like any other fault in the stub.
 */
static unsigned char *mem2hex(char *mem, char *buf, int count, int may_fault)
{
	unsigned char ch;

	debugmem_got_flt = 0;
	debugmem_flt_set = 1;

	while (count-- > 0 && ! debugmem_got_flt) {
		ch = *(mem++);
		*buf++ = hexchars[ch >> 4];
		*buf++ = hexchars[ch & 0xf];
	}

	if (debugmem_got_flt)
	    buf = 0;
	else *buf = 0;

	debugmem_got_flt = 0;
	debugmem_flt_set = 0;

	return buf;
}

/*
 * convert the hex array pointed to by buf into binary to be placed in mem
 * return a pointer to the character AFTER the last byte written
 */
static char *hex2mem(char *buf, char *mem, int count, int may_fault)
{
	int i;
	unsigned char ch;

	debugmem_got_flt = 0;
	debugmem_flt_set = 1;

	for (i=0; i<count && ! debugmem_got_flt; i++)
	{
		ch = hex(*buf++) << 4;
		ch |= hex(*buf++);
		*(mem++) = ch;
	}

	if (debugmem_got_flt)
	    mem = 0;

	debugmem_got_flt = 0;
	debugmem_flt_set = 0;

	return mem;
}

/*
 * This table contains the mapping between MIPS hardware trap types, and
 * signals, which are primarily what GDB understands.  It also indicates
 * which hardware traps we need to commandeer when initializing the stub.
 *
 * XXXX - note that we don't deal with address errors.  These actually
 *        happen in the kernel, and the instruction emulator cleans them
 *	  up.  If the kernel trap interface was clean, we could use them,
 *	  but it isn't.  Also, we use the wrong regs struct.  Bletch.
 *        cj*
 */
static struct hard_trap_info
{
	unsigned char tt;		/* Trap type code for MIPS R3xxx and R4xxx */
	unsigned char signo;		/* Signal that we map this trap into */
} hard_trap_info[] = {
	/* { 4, SIGBUS }, */			/* address error (load) */
	/* { 5, SIGBUS }, */			/* address error (store) */
	{ 6, SIGBUS },			/* instruction bus error */
	{ 7, SIGBUS },			/* data bus error */
	{ 9, SIGTRAP },			/* break */
	{ 10, SIGILL },			/* reserved instruction */
	{ 11, SIGILL },			/* cpu unusable */
	{ 12, SIGFPE },			/* overflow */
	{ 13, SIGTRAP },		/* trap */
	{ 14, SIGSEGV },		/* virtual instruction cache coherency */
	{ 15, SIGFPE },			/* floating point exception */
	{ 23, SIGSEGV },		/* watch */
	{ 31, SIGSEGV },		/* virtual data cache coherency */
	{ 0, 0}				/* Must be last */
};


/*
 * Set up exception handlers for tracing and breakpoints
 */
void set_debug_traps(void)
{
	struct hard_trap_info *ht;
	unsigned long flags;

	if ( ! initialized) {
	    save_and_cli(flags);
	    for (ht = hard_trap_info; ht->tt && ht->signo; ht++)
		    set_except_vector(ht->tt, trap_low);

	    initialized = 1;
	    restore_flags(flags);
	}
}

void sync_debug_inp(void)
{
	unsigned char c;
	/*
	 * In case GDB is started before us, ack any packets
	 * (presumably "$?#xx") sitting there.
	 *
	 * XXX - this should timeout, getDebugChar() blocks.
	 */
	while((c = getDebugChar()) != '$')
		;
	while((c = getDebugChar()) != '#')
		;
	c = getDebugChar(); /* eat first csum byte */
	c = getDebugChar(); /* eat second csum byte */
	putDebugChar('+'); /* ack it */
}


/*
 * Convert the MIPS hardware trap type code to a unix signal number.
 */
static int computeSignal(int tt)
{
	struct hard_trap_info *ht;

	for (ht = hard_trap_info; ht->tt && ht->signo; ht++)
		if (ht->tt == tt)
			return ht->signo;

	return SIGHUP;		/* default for things we don't know about */
}

/*
 * While we find nice hex chars, build an int.
 * Return number of chars processed.
 */
static int hexToInt(char **ptr, int *intValue)
{
	int numChars = 0;
	int hexValue;

	*intValue = 0;

	while (**ptr)
	{
		hexValue = hex(**ptr);
		if (hexValue < 0)
			break;

		*intValue = (*intValue << 4) | hexValue;
		numChars ++;

		(*ptr)++;
	}

	return (numChars);
}


#if 1
/*
 * Print registers (on target console)
 * Used only to debug the stub...
 */
void show_gdbregs(struct gdb_regs * regs)
{
	/*
	 * Saved main processor registers
	 */
	dbg_out("$0 : %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
	       regs->reg0, regs->reg1, regs->reg2, regs->reg3,
               regs->reg4, regs->reg5, regs->reg6, regs->reg7);
	dbg_out("$8 : %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
	       regs->reg8, regs->reg9, regs->reg10, regs->reg11,
               regs->reg12, regs->reg13, regs->reg14, regs->reg15);
	dbg_out("$16: %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
	       regs->reg16, regs->reg17, regs->reg18, regs->reg19,
               regs->reg20, regs->reg21, regs->reg22, regs->reg23);
	dbg_out("$24: %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
	       regs->reg24, regs->reg25, regs->reg26, regs->reg27,
	       regs->reg28, regs->reg29, regs->reg30, regs->reg31);

	/*
	 * Saved cp0 registers
	 */
	dbg_out("epc  : %08lx\nStatus: %08lx\nCause : %08lx\n",
	       regs->cp0_epc, regs->cp0_status, regs->cp0_cause);
}
#endif /* dead code */

/*
 * This function does all command processing for interfacing to gdb.  It
 * returns 1 if you should skip the instruction at the trap address, 0
 * otherwise.
 */
asmlinkage void handle_exception (struct gdb_regs *regs)
{
	int trap;			/* Trap type */
	int sigval;
	int addr;
	int length;
	char *ptr;
	unsigned long *stack;
	static int init = 0;

#if 1
	dbg_out("in handle_exception()\n");
	show_gdbregs(regs);
#endif
	
	/*
	 * First check trap type. If this is CPU_UNUSABLE and CPU_ID is 1,
	 * the simply switch the FPU on and return since this is no error
	 * condition. kernel/traps.c does the same.
	 * FIXME: This doesn't work yet, so we don't catch CPU_UNUSABLE
	 * traps for now.
	 */
	trap = (regs->cp0_cause & 0x7c) >> 2;
	dbg_out("trap=%d\n",trap);
	if (trap == 11) {
		if (((regs->cp0_cause >> CAUSEB_CE) & 3) == 1) {
			memset(&current->tss.fpu, 0,
					sizeof(union mips_fpu_union));
			regs->cp0_status |= ST0_CU1;
			return;
		}
	}

	if ( ! init) {
		sync_debug_inp();
		init = 1;
	}

	/*
	 * If we're in breakpoint() increment the PC
	 */
	if (trap == 9 && regs->cp0_epc == (unsigned long)breakinst)		
		regs->cp0_epc += 4;

	stack = (long *)regs->reg29;			/* stack ptr */
	sigval = computeSignal(trap);

	/*
	 * reply to host that an exception has occurred
	 */
	ptr = output_buffer;

	/*
	 * Send trap type (converted to signal)
	 */
	*ptr++ = 'T';
	*ptr++ = hexchars[sigval >> 4];
	*ptr++ = hexchars[sigval & 0xf];

	/*
	 * Send Error PC
	 */
	*ptr++ = hexchars[REG_EPC >> 4];
	*ptr++ = hexchars[REG_EPC & 0xf];
	*ptr++ = ':';
	ptr = mem2hex((char *)&regs->cp0_epc, ptr, 4, 0);
	*ptr++ = ';';

	/*
	 * Send frame pointer
	 */
	*ptr++ = hexchars[REG_FP >> 4];
	*ptr++ = hexchars[REG_FP & 0xf];
	*ptr++ = ':';
	ptr = mem2hex((char *)&regs->reg30, ptr, 4, 0);
	*ptr++ = ';';

	/*
	 * Send stack pointer
	 */
	*ptr++ = hexchars[REG_SP >> 4];
	*ptr++ = hexchars[REG_SP & 0xf];
	*ptr++ = ':';
	ptr = mem2hex((char *)&regs->reg29, ptr, 4, 0);
	*ptr++ = ';';

	*ptr++ = 0;
	putpacket(output_buffer);	/* send it off... */

	/*
	 * Wait for input from remote GDB
	 */
	while (1) {
		output_buffer[0] = 0;
		getpacket(input_buffer);

		switch (input_buffer[0])
		{
		case '?':
			output_buffer[0] = 'S';
			output_buffer[1] = hexchars[sigval >> 4];
			output_buffer[2] = hexchars[sigval & 0xf];
			output_buffer[3] = 0;
			break;

		case 'd':
			/* toggle debug flag */
			break;

		/*
		 * Return the value of the CPU registers
		 */
		case 'g':
			(void) mem2hex(KGDB_USERREGS_START(regs), output_buffer,
				    KGDB_USERREGS_BYTES, 0);
			break;
	  
		/*
		 * set the value of the CPU registers - return OK
		 * FIXME: Needs to be written - maybe not (cj)
		 */
		case 'G':
		{
			struct gdb_regs newregs;

			ptr = hex2mem(&input_buffer[1],
				KGDB_USERREGS_START(&newregs),
				    KGDB_USERREGS_BYTES, 0);
			if (ptr) {
			    memcpy(KGDB_USERREGS_START(regs),
				    KGDB_USERREGS_START(&newregs),
				    KGDB_USERREGS_BYTES);
			    strcpy(output_buffer,"OK");
			}
			else strcpy(output_buffer,"ENN");
		 }
		break;

		/*
		 * mAA..AA,LLLL  Read LLLL bytes at address AA..AA
		 */
		case 'm':
			ptr = &input_buffer[1];

			if (hexToInt(&ptr, &addr)
				&& *ptr++ == ','
				&& hexToInt(&ptr, &length)) {
				if (mem2hex((char *)addr, output_buffer, length, 1))
					break;
				strcpy (output_buffer, "E03");
			} else
				strcpy(output_buffer,"E01");
			break;

		/*
		 * MAA..AA,LLLL: Write LLLL bytes at address AA.AA return OK
		 */
		case 'M': 
			ptr = &input_buffer[1];

			if (hexToInt(&ptr, &addr)
				&& *ptr++ == ','
				&& hexToInt(&ptr, &length)
				&& *ptr++ == ':') {
				if (hex2mem(ptr, (char *)addr, length, 1))
					strcpy(output_buffer, "OK");
				else
					strcpy(output_buffer, "E03");
			}
			else
				strcpy(output_buffer, "E02");
			break;

		/*
		 * cAA..AA    Continue at address AA..AA(optional)
		 */
		case 'c':    
			/* try to read optional parameter, pc unchanged if no parm */

			ptr = &input_buffer[1];
			if (hexToInt(&ptr, &addr))
				regs->cp0_epc = addr;
	  
			/*
			 * Need to flush the instruction cache here, as we may
			 * have deposited a breakpoint, and the icache probably
			 * has no way of knowing that a data ref to some location
			 * may have changed something that is in the instruction
			 * cache.
			 * NB: We flush both caches, just to be sure...
			 */

			flush_cache_all();
			return;
			/* NOTREACHED */
			break;


		/*
		 * kill the program or detach
		 */
		case 'k' :
		case 'D' :
			return;
			break;		/* do nothing */


		/*
		 * Reset the whole machine (FIXME: system dependent)
		 */
		case 'r':
			break;


		/*
		 * Step to next instruction
		 * FIXME: Needs to be written
		 * XXXX - gdb should write a bkpt instr, and continue.
		 *        why does the kernel need a single step? (cj)
		 */
		case 's':
			strcpy (output_buffer, "S01");
			break;

		/*
		 * Set baud rate (bBB)
		 * FIXME: Needs to be written
		 */
		case 'b':
		{
#if 0				
			int baudrate;
			extern void set_timer_3();

			ptr = &input_buffer[1];
			if (!hexToInt(&ptr, &baudrate))
			{
				strcpy(output_buffer,"B01");
				break;
			}

			/* Convert baud rate to uart clock divider */

			switch (baudrate)
			{
				case 38400:
					baudrate = 16;
					break;
				case 19200:
					baudrate = 33;
					break;
				case 9600:
					baudrate = 65;
					break;
				default:
					baudrate = 0;
					strcpy(output_buffer,"B02");
					goto x1;
			}

			if (baudrate) {
				putpacket("OK");	/* Ack before changing speed */
				set_timer_3(baudrate); /* Set it */
			}
#endif
		}
		break;
		}			/* switch */

		/*
		 * reply to the request
		 */

		putpacket(output_buffer);

	} /* while */
}

/*
 * This function will generate a breakpoint exception.  It is used at the
 * beginning of a program to sync up with a debugger and can be used
 * otherwise as a quick means to stop program execution and "break" into
 * the debugger.
 */
void breakpoint(void)
{
	if (!initialized)
		return;

	__asm__ __volatile__("
			.globl	breakinst
			.set	noreorder
			nop
breakinst:		break
			nop
			.set	reorder
	");
}

void adel(void)
{
	__asm__ __volatile__("
			.globl	adel
			la	$8,0x80000001
			lw	$9,0($8)
	");
}
