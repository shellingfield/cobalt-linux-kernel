#
# Makefile for the Linux/MIPS-specific parts of the memory manager.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# Note 2! The CFLAGS definition is now in the main makefile...
#
# $Id: Makefile,v 1.3 1997/11/24 23:10:13 davem Exp $
#

# Grrr, we should not even need this... -DaveM
.S.s:
	$(CPP) $(CFLAGS) $< -o $*.s
.S.o:
	$(CC) $(CFLAGS) -c $< -o $*.o

O_TARGET := mm.o
O_OBJS	 := init.o fault.o r4ktlb.o r4kcache.o r4xx0.o r2300.o r6000.o \
	    tfp.o andes.o loadmmu.o

include $(TOPDIR)/Rules.make
