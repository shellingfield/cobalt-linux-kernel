#
# Makefile for Linux arch/m68k/amiga source directory
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# Note 2! The CFLAGS definitions are now in the main makefile...

O_TARGET := amiga.o
O_OBJS	 := config.o amikeyb.o amiints.o \
	   chipram.o amisound.o amifb.o zorro.o
OX_OBJS = ksyms.o

ifdef CONFIG_FB_CYBER
O_OBJS := $(O_OBJS) cyberfb.o
endif

include $(TOPDIR)/Rules.make
