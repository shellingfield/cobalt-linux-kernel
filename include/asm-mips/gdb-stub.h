/*
 *  include/asm-mips/gdb-stub.h
 *
 *  Copyright (C) 1995 Andreas Busse
 */

#ifndef __ASM_MIPS_GDB_STUB_H
#define __ASM_MIPS_GDB_STUB_H


/*
 * important register numbers
 */

#define REG_EPC			GDB_FR_EPC
#define REG_FP			GDB_FR_FRP
#define REG_SP			GDB_FR_REG29

/*
 * Stack layout for the GDB exception handler
 * Derived from the stack layout described in asm-mips/stackframe.h
 *
 * The first PTRSIZE*5 bytes are argument save space for C subroutines.
 */
#define NUMREGS			90

#define GDB_FR_REG0		(PTRSIZE*5)			/* 0 */
#define GDB_FR_REG1		((GDB_FR_REG0) + 4)		/* 1 */
#define GDB_FR_REG2		((GDB_FR_REG1) + 4)		/* 2 */
#define GDB_FR_REG3		((GDB_FR_REG2) + 4)		/* 3 */
#define GDB_FR_REG4		((GDB_FR_REG3) + 4)		/* 4 */
#define GDB_FR_REG5		((GDB_FR_REG4) + 4)		/* 5 */
#define GDB_FR_REG6		((GDB_FR_REG5) + 4)		/* 6 */
#define GDB_FR_REG7		((GDB_FR_REG6) + 4)		/* 7 */
#define GDB_FR_REG8		((GDB_FR_REG7) + 4)		/* 8 */
#define GDB_FR_REG9	        ((GDB_FR_REG8) + 4)		/* 9 */
#define GDB_FR_REG10		((GDB_FR_REG9) + 4)		/* 10 */
#define GDB_FR_REG11		((GDB_FR_REG10) + 4)		/* 11 */
#define GDB_FR_REG12		((GDB_FR_REG11) + 4)		/* 12 */
#define GDB_FR_REG13		((GDB_FR_REG12) + 4)		/* 13 */
#define GDB_FR_REG14		((GDB_FR_REG13) + 4)		/* 14 */
#define GDB_FR_REG15		((GDB_FR_REG14) + 4)		/* 15 */
#define GDB_FR_REG16		((GDB_FR_REG15) + 4)		/* 16 */
#define GDB_FR_REG17		((GDB_FR_REG16) + 4)		/* 17 */
#define GDB_FR_REG18		((GDB_FR_REG17) + 4)		/* 18 */
#define GDB_FR_REG19		((GDB_FR_REG18) + 4)		/* 19 */
#define GDB_FR_REG20		((GDB_FR_REG19) + 4)		/* 20 */
#define GDB_FR_REG21		((GDB_FR_REG20) + 4)		/* 21 */
#define GDB_FR_REG22		((GDB_FR_REG21) + 4)		/* 22 */
#define GDB_FR_REG23		((GDB_FR_REG22) + 4)		/* 23 */
#define GDB_FR_REG24		((GDB_FR_REG23) + 4)		/* 24 */
#define GDB_FR_REG25		((GDB_FR_REG24) + 4)		/* 25 */
#define GDB_FR_REG26		((GDB_FR_REG25) + 4)		/* 26 */
#define GDB_FR_REG27		((GDB_FR_REG26) + 4)		/* 27 */
#define GDB_FR_REG28		((GDB_FR_REG27) + 4)		/* 28 */
#define GDB_FR_REG29		((GDB_FR_REG28) + 4)		/* 29 */
#define GDB_FR_REG30		((GDB_FR_REG29) + 4)		/* 30 */
#define GDB_FR_REG31		((GDB_FR_REG30) + 4)		/* 31 */

/*
 * Saved floating point registers
 */
#define GDB_FR_FPR0		((GDB_FR_REG31) + 4)		/* 32 */
#define GDB_FR_FPR1		((GDB_FR_FPR0) + 4)		/* 33 */
#define GDB_FR_FPR2		((GDB_FR_FPR1) + 4)		/* 34 */
#define GDB_FR_FPR3		((GDB_FR_FPR2) + 4)		/* 35 */
#define GDB_FR_FPR4		((GDB_FR_FPR3) + 4)		/* 36 */
#define GDB_FR_FPR5		((GDB_FR_FPR4) + 4)		/* 37 */
#define GDB_FR_FPR6		((GDB_FR_FPR5) + 4)		/* 38 */
#define GDB_FR_FPR7		((GDB_FR_FPR6) + 4)		/* 39 */
#define GDB_FR_FPR8		((GDB_FR_FPR7) + 4)		/* 40 */
#define GDB_FR_FPR9		((GDB_FR_FPR8) + 4)		/* 41 */
#define GDB_FR_FPR10		((GDB_FR_FPR9) + 4)		/* 42 */
#define GDB_FR_FPR11		((GDB_FR_FPR10) + 4)		/* 43 */
#define GDB_FR_FPR12		((GDB_FR_FPR11) + 4)		/* 44 */
#define GDB_FR_FPR13		((GDB_FR_FPR12) + 4)		/* 45 */
#define GDB_FR_FPR14		((GDB_FR_FPR13) + 4)		/* 46 */
#define GDB_FR_FPR15		((GDB_FR_FPR14) + 4)		/* 47 */
#define GDB_FR_FPR16		((GDB_FR_FPR15) + 4)		/* 48 */
#define GDB_FR_FPR17		((GDB_FR_FPR16) + 4)		/* 49 */
#define GDB_FR_FPR18		((GDB_FR_FPR17) + 4)		/* 50 */
#define GDB_FR_FPR19		((GDB_FR_FPR18) + 4)		/* 51 */
#define GDB_FR_FPR20		((GDB_FR_FPR19) + 4)		/* 52 */
#define GDB_FR_FPR21		((GDB_FR_FPR20) + 4)		/* 53 */
#define GDB_FR_FPR22		((GDB_FR_FPR21) + 4)		/* 54 */
#define GDB_FR_FPR23		((GDB_FR_FPR22) + 4)		/* 55 */
#define GDB_FR_FPR24		((GDB_FR_FPR23) + 4)		/* 56 */
#define GDB_FR_FPR25		((GDB_FR_FPR24) + 4)		/* 57 */
#define GDB_FR_FPR26		((GDB_FR_FPR25) + 4)		/* 58 */
#define GDB_FR_FPR27		((GDB_FR_FPR26) + 4)		/* 59 */
#define GDB_FR_FPR28		((GDB_FR_FPR27) + 4)		/* 60 */
#define GDB_FR_FPR29		((GDB_FR_FPR28) + 4)		/* 61 */
#define GDB_FR_FPR30		((GDB_FR_FPR29) + 4)		/* 62 */
#define GDB_FR_FPR31		((GDB_FR_FPR30) + 4)		/* 63 */

/*
 * Saved special registers
 */
#define GDB_FR_EPC		((GDB_FR_FPR31) + 4)		/* 64 */
#define GDB_FR_CAUSE		((GDB_FR_EPC) + 4)		/* 65 */
#define GDB_FR_BADVADDR		((GDB_FR_CAUSE) + 4)		/* 66 */
#define GDB_FR_LO		((GDB_FR_BADVADDR) + 4)		/* 67 */
#define GDB_FR_HI		((GDB_FR_LO) + 4)		/* 68 */

#define GDB_FR_FSR		((GDB_FR_HI) + 4)		/* 69 */
#define GDB_FR_FIR		((GDB_FR_FSR) + 4)		/* 70 */

/*
 * the following are unknown to gdb, but are still saved/restored
 * by mips/kernel/gdb-low.S - perhaps the kernel debugger can be
 * taught about special purpose registers?
 */
#define GDB_FR_FRP		((GDB_FR_FIR) + 4)		/* 71 */
#define GDB_FR_STATUS		((GDB_FR_FRP) + 4)		/* 72 */

/*
 * Again, CP0 registers - not all of these are saved/restored.
 */
#define GDB_FR_CP0_INDEX	((GDB_FR_STATUS) + 4)		/* 73 */
#define GDB_FR_CP0_RANDOM	((GDB_FR_CP0_INDEX) + 4)	/* 74 */
#define GDB_FR_CP0_ENTRYLO0	((GDB_FR_CP0_RANDOM) + 4)	/* 75 */
#define GDB_FR_CP0_ENTRYLO1	((GDB_FR_CP0_ENTRYLO0) + 4)	/* 76 */
#define GDB_FR_CP0_REG4		((GDB_FR_CP0_ENTRYLO1) + 4)	/* 77 */
#define GDB_FR_CP0_PAGEMASK	((GDB_FR_CP0_REG4) + 4)		/* 78 */
#define GDB_FR_CP0_WIRED	((GDB_FR_CP0_PAGEMASK) + 4)	/* 79 */
#define GDB_FR_CP0_REG7		((GDB_FR_CP0_WIRED) + 4)	/* 80 */
#define GDB_FR_CP0_REG8		((GDB_FR_CP0_REG7) + 4)		/* 81 */
#define GDB_FR_CP0_REG9		((GDB_FR_CP0_REG8) + 4)		/* 82 */
#define GDB_FR_CP0_ENTRYHI	((GDB_FR_CP0_REG9) + 4)		/* 83 */
#define GDB_FR_CP0_REG11	((GDB_FR_CP0_ENTRYHI) + 4)	/* 84 */
#define GDB_FR_CP0_REG12	((GDB_FR_CP0_REG11) + 4)	/* 85 */
#define GDB_FR_CP0_REG13	((GDB_FR_CP0_REG12) + 4)	/* 86 */
#define GDB_FR_CP0_REG14	((GDB_FR_CP0_REG13) + 4)	/* 87 */
#define GDB_FR_CP0_PRID		((GDB_FR_CP0_REG14) + 4)	/* 88 */

#define GDB_FR_SIZE		((((GDB_FR_CP0_PRID) + 4) + (PTRSIZE-1)) & ~(PTRSIZE-1))

#ifndef __LANGUAGE_ASSEMBLY__

/*
 * This is the same as above, but for the high-level
 * part of the GDB stub.
 */

struct gdb_regs {
	/*
	 * Pad bytes for argument save space on the stack
	 * 20/40 Bytes for 32/64 bit code
	 */
	unsigned long pad0[5];

	/*
	 * saved main processor registers
	 */
	long	 reg0,  reg1,  reg2,  reg3,  reg4,  reg5,  reg6,  reg7;
	long	 reg8,  reg9, reg10, reg11, reg12, reg13, reg14, reg15;
	long	reg16, reg17, reg18, reg19, reg20, reg21, reg22, reg23;
	long	reg24, reg25, reg26, reg27, reg28, reg29, reg30, reg31;

	/*
	 * Saved floating point registers
	 */
	long	fpr0,  fpr1,  fpr2,  fpr3,  fpr4,  fpr5,  fpr6,  fpr7;
	long	fpr8,  fpr9,  fpr10, fpr11, fpr12, fpr13, fpr14, fpr15;
	long	fpr16, fpr17, fpr18, fpr19, fpr20, fpr21, fpr22, fpr23;
	long	fpr24, fpr25, fpr26, fpr27, fpr28, fpr29, fpr30, fpr31;

	/*
	 * Saved special registers
	 */
	long	cp0_epc;
	long	cp0_cause;
	long	cp0_badvaddr;
	long	lo;
	long	hi;

	long	cp1_fsr;
	long	cp1_fir;

	/*
	 * Frame pointer
	 */
	long	frame_ptr;
	long	cp0_status;
	
	/*
	 * saved cp0 registers
	 */
	long	cp0_index;
	long	cp0_random;
	long	cp0_entrylo0;
	long	cp0_entrylo1;
	long	cp0_reg4;
	long	cp0_pagemask;
	long	cp0_wired;
	long	cp0_reg7;
	long	cp0_reg8;
	long	cp0_reg9;
	long	cp0_entryhi;
	long	cp0_reg11;
	long	cp0_reg12;
	long	cp0_reg13;
	long	cp0_reg14;
	long	cp0_prid;
};

/*
 * We only pass part of the above structure to kernel GDB in
 * response to register operations.  Don't know why, but the
 * the following macros at least make the code _look_ clean.
 */
#define KGDB_USERREGS_START(r)	((char *) (&(r)->reg0))
#define KGDB_USERREGS_BYTES					\
	((char *) &(((struct gdb_regs *) 0)->frame_ptr)		\
		- ((char *) &(((struct gdb_regs *) 0)->reg0)))

/*
 * Prototypes
 */

void set_debug_traps(void);

#endif /* __LANGUAGE_ASSEMBLY */
#endif /* __ASM_MIPS_GDB_STUB_H */
