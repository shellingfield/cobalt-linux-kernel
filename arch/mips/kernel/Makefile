#
# Makefile for the Linux/MIPS kernel.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# $Id: Makefile,v 1.4 1998/02/28 00:57:04 davem Exp $
#

.S.o:
	$(CC) $(CFLAGS) -c $< -o $*.o

all:	kernel.o head.o
EXTRA_ASFLAGS = -mips3 -mcpu=r4000
O_TARGET := kernel.o
O_OBJS	:= branch.o process.o signal.o entry.o traps.o ptrace.o vm86.o \
	   ioport.o ksyms.o pci.o reset.o scall_o32.o setup.o syscall.o \
	   sysmips.o ipc.o r4k_switch.o r4k_misc.o r4k_fpu.o r2300_switch.o \
	   r2300_misc.o r2300_fpu.o r6000_fpu.o unaligned.o

#
# SGI's have very different interrupt/timer hardware.
#
ifndef CONFIG_SGI
O_OBJS += irq.o time.o
endif

#
# Do we want to be able to execute IRIX elf binaries?
#
ifdef CONFIG_BINFMT_IRIX
O_OBJS += irixelf.o irixioctl.o irixsig.o sysirix.o
endif

#
# Kernel debugging
#
ifdef CONFIG_REMOTE_DEBUG
O_OBJS += gdb-low.o gdb-stub.o 
endif

#
# Depending from some other kernel option
#
ifdef CONFIG_PROC_FS
O_OBJS += proc.o
endif

#
# Since we add the same object files to O_OBJS for different configurations.
# O_OBJS might contain duplicate files.  We correct this by filtering out
# duplicate files.  Just to avoid users having to know about all the
# compatibility stuff between various boards and boards.
#
O_OBJS := $(sort $(O_OBJS))

all: kernel.o head.o

entry.o: entry.S

head.o: head.S

clean:

include $(TOPDIR)/Rules.make
