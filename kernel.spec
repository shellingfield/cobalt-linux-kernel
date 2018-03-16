Summary: Cobalt MIPS kernel 
Name: kernel
Version: 2.0.34C51_SK
%define version %{PACKAGE_VERSION}
Release: 1
Copyright: GPL/Cobalt
Group: Base/Kernel
ExclusiveOS: Linux
Source: linux-2.0.34-cobalt.tar.gz
BuildRoot: /var/tmp/kernel

%changelog
* Thu Sep 30 1999 Chris Johnson <cjohnson@cobaltnet.com>
  - Fix to mmap to align to 16k boundary to deal with newly
    discovered virtual cache tag bits in Q.E.D. processor.
  - C51

* Tue Sep 21 1999  Tim Hockin <thockin@cobaltnet.com>
  - Adding multi-LUN support
  - Added 'make rpm' to Makefile
  - C50 

* Wed Sep 08 1999  Tim Hockin <thockin@cobaltnet.com>
  - Fix for socket (af_unix at least) bug crashing system
  - At some point asum checked in quota fixes

* Mon Aug 09 1999  Tim Hockin <thockin@cobaltnet.com>
  - PCI serial now assigns ttys and probes ioports correctly
  - PCI serial now detects UART types correctly for PCI modems

* Mon Aug 09 1999  Tim Hockin <thockin@cobaltnet.com>
  - PCI serial now builds as a module in the default config
  - PCI serial fixes: 
	doesn't crash on module reload
	autoprobes for PCI serial cards
	doesn't probe base mem address for UART
	loads UARTs as ttyS4 correctly
	adds a small (1ms) delay during UART probing
	removed HUB6 support (not needed)
  - PCI device ID's updated for serial card/modem
  - Added -f flag to gzip in 'make cobalt'
  - Added vmlinux.gz to 'make clean'
  - Fixed Galileo PCI ID
  - Added chown to specfile to make sure files are owned by root in the RPM
	
* Tue Aug 03 1999  Tim Hockin <thockin@cobaltnet.com>
  - Fixed PCI serial driver to work as a module
  - Added kernel.spec to CVS tree

* Wed Jul 21 1999  Tim Hockin <thockin@cobaltnet.com>
  - Fixed IPC errors - IPC structs now use correct typedefs for fields

* Mon Jul 19 1999  Tim Hockin <thockin@cobaltnet.com>
  - Fixed %version mismatch for EXTRAVERSION kernels

* Fri Jul 16 1999  Tim Hockin <thockin@cobaltnet.com>
  - Added EXTRAVERSION patch

  - Added FP align fix from Chris Johnson

  - Added warnings for misaligned stack pointers when creating new processes

  - This release is paired with a new glibc, fixing all known data-killing 
    floating point bugs

* Tue Jul 06 1999  Tim Hockin <thockin@cobaltnet.com>
  - Make menuconfig now works

  - Using config-sk now builds just about everything as modules
    This should make a small enough kernel to use for ROM

  - /lib/modules/%{version} is now included by this package

  - .config is now included in this package

  - Added $(MODROOT) for make modules_install

  - ISDN4Linux tree pulled from 2.0.36

  - Added PCI IDs for ISDN cards (Fritz Elfert)

  - Added strstr symbol export

  - Added isdnlog patch from Fritz Elfert

  - config-sk now builds ISDN modules by default

  - Changed /tmp/kernel to /var/tmp/kernel for BuildRoot

  - Added %clean section to specfile

* Tue Jun 15 1999  Chris Johnson <cjohnson@cobaltnet.com>
  - Mods to make kernel modules work cleanly.  (Tim Hockin's first
    chance to break the kernel.)

  - Mods to improve kernel debugger interface and clean up some
    missing initialization.  If CONFIG_REMOTE_DEBUG is set, panic
    will forward the fault to the debugger, and fault will try to
    recover from stray memory references by the debugger.

  - New and improved tulip driver.  Supports 2800, 2700, and PCI
    tulip in 2700 Cache qube.

  - Backed out earlier change in locore that turned out to be
    unnecessary.  Network drivers can interrupt the protocol
    stack safely.  This improves network performance slightly.

* Mon May 31 1999  Chris Johnson <cjohnson@cobaltnet.com> 
  - Fix for quota to check effective user/group ID instead of real.

  - A few mods inside the 'ifdef BOOTLOADER' code.

* Wed May 5 1999  Chris Johnson <cjohnson@cobaltnet.com> 
  - Added suppport for Rev 2 Galileo chip (just initialize as rev 1).

* Wed Apr 28 1999  Chris Johnson <cjohnson@cobaltnet.com>
  - Turned on CONFIG_SYN_COOKIES to protect vs. TCP SYN flood attacks.

  - Added memory reserve interface, and use it to ensure memory is
    available for net/core/dev.c message queue.

* Tue Apr 20 1999  Chris Johnson <cjohnson@cobaltnet.com>
  - Added CDROM Filesystem (iso9660) and NLS support to default kernel

  - Have spec file %pre and %post rules create a .doug if it is missing.
    Use the previous kernel if available (%pre), or the current kernel.

* Wed Apr 7 1999  Chris Johnson <cjohnson@cobaltnet.com>
  - Support kernel debugging
  
  - added compile time selectable kernel malloc leak detector.

* Thu Mar 25 1999  Chris Johnson <cjohnson@cobaltnet.com>
  - Minor string contents fixes

  - More serial line baud rate fixes - deal with 0x1 since that is
    the value used by the ROM.  Pass "CONSOLE=/dev/null" to init in
    the env to disable output from init its children.
 
* Fri Mar 19 1999  Chris Johnson <cjohnson@cobaltnet.com>

  - Merged in 2.0.36 TCP to dodge tacky security hole

  - Fixed cobaltide.c to support Raq1

  - Fixed several bugs in cobaltserial.c related to processing
    interrupts.  Bugs caused system hangs in some situations.

  - Added support for CMOS byte to define console baud rate
    and whether to send kernel printk's to the serial line.

  - Added config-sk for single kernel build universe.

  - Also fixed header install (below) to do_the_right_thing()
    during 'rpm -U'.

* Thu Feb 25 1999  Chris Johnson <cjohnson@cobaltnet.com>

  - Multiple fixes for system hangs and misc. weird errors under
    heavy 100baseT load.  Problems were caused by interrupt stack
    overflow due to timer and ethernet interrupts ping-ponging
    without ever unwinding stack.

  - arch/mips/cobalt/int-handler.S
    Fixed interrupt vectors to reflect actual hardware.
    This is right for Qube-2 and Raq-2, and may work for
    Raq-1 (untested).  Qube-1 will need a generic fix,
    or live forever in special case land.

    Also, we now mask all current interrupts for the
    duration of the current processing.  This eliminates
    re-entrant interrupts.

    Finally, during ethernet interrupts mask out the
    other ethernet device.  They seem to colide at the
    higher layers otherwise.

  - arch/mips/kernel/entry.S
    Mask network device interrupts while the software
    interrupt handler is running.

  - arch/mips/kernel/irq.c
    Detect interrupt stack overflows during the interrupt,
    rather than waiting with corrupted memory until the
    process exit()'s.  Added code to dump stack information
    and halt.  With above changes, this should "never happen".

  - include/asm-mips/stackframe.h
    Modified the SAVE_ALL macro that saves the processor state
    on interrupts and traps.  SAVE_ALL now sets the `EXL' bit,
    the exception level, in the saved status.  This is cleared
    automatically during the 'eret' that completes traps and
    interrupts.  Without this change, as soon as the old status
    register is loaded a new interrupt can be delivered, which
    can result in multiple register frames on the stack.

  - include/asm-mips/bootinfo.h
    Fix for bug #2590 gratuitously bundled into bigger fix.
    Change /proc/cpuinfo "system type" to "Cobalt Networks"
    instead of the previous "Cobalt Microserver 27".

* Thu Jan 28 1999  David S. Miller  <davem@redhat.com>

  -Rewrite Tulip driver interrupt scheme to avoid lost interrupts
   and crashes in out of memory and high traffic situations.
  -Set disconnect/retry to 64/32 respectively for new
   Galileo.

* Wed Jan 20 1999  David S. Miller  <davem@dm.cobaltmicro.com>

  -Set Galileo SyncMode in it's command register to binary 01
  -Flush setup frame before transmission in Tulip driver.

* Sun Jan 17 1999  David S. Miller  <davem@dm.cobaltmicro.com>

  -Access only descriptors from non-cached space for Tulip.
  -Enable IP_MASQ and transparent proxy.

* Wed Jan 13 1999  David S. Miller  <davem@dm.cobaltmicro.com>

  -Update to latest PPP driver
  -Fix ntohl/htonl declarations.

* Mon Jan 11 1999  David S. Miller  <davem@dm.cobaltmicro.com>

  -Fix PCI deadlocks caused by VIA.

* Sat Dec 19 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Did I say this was the final batch of 2800/Tulip fixes?

* Tue Dec 15 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Hopefully the final batch of 2800/Tulip fixes.

* Wed Dec  2 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -More Tulip fixes.

* Thu Nov 19 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Fix 100baseT Tulip problems.
  -Full new style link check implementation.
  -Working PCI memory space and Adaptec driver tweaked
   to use it.
  -Serial driver hacks to not output anything for console
   to prevent bootup hangs when modem is attached.

* Tue Nov 17 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Enable multicast for gated's sake.
  -Kernel can now work on both old and new 2800 hardware.

* Fri Nov 13 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Fix Tulip breakage on 250Mhz systems.

* Wed Nov  4 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Add Adapted driver, remove ncr53c8xx driver.

* Fri Sep 18 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Now 2800 test kernels are RAQ hardware.
  -Add SLIP and PPP drivers.

* Mon Sep 14 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Integrated signal SA_* mask fixes for restartable syscalls
   etc.

* Wed Sep  9 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Integrated GDB fixes.

* Fri Jul 17 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Fix netgear bootup hang bug.

* Wed Jul  7 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Merge generic kernel to 2.0.34 for package update release.

* Wed Jun 10 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Quick hack for cache qube kernel RPM build
  -Include Ingo's memcpy optimizations
  -Include latest bug fixed PCI slot ethernet drivers

* Fri Apr 24 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Quick hacked up build of FW/MASQ support kernel.

* Fri Apr 21 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Add support for 3c59x and IntelEEPRO100 cards in expansion slot.

* Fri Apr  3 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Tulip tx timeout fixes, hopefully this kills it for good.

* Tue Mar 10 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Hardcode timer register value for 150MHZ cpu speed.
  -Remove bogus initscripts requires from this specs file.

* Fri Feb 27 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Kernel works with no serial console
  -LCD driver updates, return button value for lcd_read

* Wed Feb 25 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Backout sys_brk() optimizations
  -More Tulip tx timeout fixes.

* Thu Feb 12 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Boot message cleanup.
  -Tulip lockup fixes.

* Wed Feb 11 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Integrate swapping and vm bug fixes.
  -Small TCP fixes from Alan Cox.
  -Memory leak during page table allocation fixes.

* Fri Feb  5 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Put loopback block device back into config as temporary
   hack to fix ide bootup problems...

* Fri Jan 27 1998  David S. Miller  <davem@dm.cobaltmicro.com>

  -Final ethernet collision fixes.

* Fri Jan 23 1998 Timothy Stonis <tim@cobaltmicro.com>

  -Updated with Dave's collision fix.
  -Added IP aliasing support by default

* Thu Jan 15 1998 Timothy Stonis <tim@cobaltmicro.com>

 -Added Andrew's new LCD driver.
 -Removed loop back block device

* Mon Dec 22 1997  David S. Miller  <davem@dm.cobaltmicro.com>

  Added in sys_brk() bug fixes.

* Mon Dec 22 1997 Timothy Stonis <tim@cobaltmicro.com>
 
  Updated with Dave's strace fixes.

* Thu Dec 18 1997 Timothy Stonis <tim@cobaltmicro.com>

  Updated with Boseman's LINK stuff.

* Wed Dec 17 1997 Timothy Stonis <tim@cobaltmicro.com>

  Initial package creation of 2.0.33 based from 2.0.32

%package source
Requires: kernel-headers = %{version}
Summary: Kernel source tree
Group: Base/Kernel

%package headers
Summary: Header files for the Linux kernel.
Group: Base/Kernel

%description
This package contains the Linux kernel and modules that are used to boot 
and run your system. The kernel contains a few device drivers for specific 
hardware, but most hardware is instead supported by modules loaded after 
booting.

%description source
This is the source code for the Linux kernel. It is required to build
most C programs as they depend on constants defined in here. You can
also build a custom kernel that is better tuned to your particular
hardware.

%description headers
These are the C header files for the Linux kernel, which define structures
and constants that are needed when building most standard programs under
Linux, as well as to rebuild the kernel.

%prep
if [ "X" != "${RPM_BUILD_ROOT}X" ]; then
    rm -rf $RPM_BUILD_ROOT
fi

%setup -c -q
mkdir -p $RPM_BUILD_ROOT
mkdir -p $RPM_BUILD_ROOT/usr/src/
mkdir -p $RPM_BUILD_ROOT/lib/modules/%{version}
cd $RPM_BUILD_ROOT/usr/src
zcat $RPM_SOURCE_DIR/linux-2.0.34-cobalt.tar.gz | tar -xf-
chown -R root.root ./linux

find linux -name "*.orig" -print | xargs rm -f
mv linux linux-%{version}
ln -sfn linux-%{version} linux

%build
cd linux
cp config-sk arch/mips/defconfig
make oldconfig
make dep clean
make cobalt
make modules

cd $RPM_BUILD_ROOT/usr/src/linux-%{version}
cp config-sk arch/mips/defconfig
make oldconfig
make include/linux/version.h

%install
cd linux

mkdir -p $RPM_BUILD_ROOT/boot
cp vmlinux.gz $RPM_BUILD_ROOT/boot
cp System.map $RPM_BUILD_ROOT/boot
make modules_install MODROOT=$RPM_BUILD_ROOT

mkdir -p $RPM_BUILD_ROOT/usr/include
ln -s ../src/linux/include/asm $RPM_BUILD_ROOT/usr/include/asm
ln -s ../src/linux/include/linux $RPM_BUILD_ROOT/usr/include/linux
ln -s ../src/linux/include/scsi $RPM_BUILD_ROOT/usr/include/scsi

%clean
rm -rf $RPM_BUILD_ROOT

%pre
if [ ! -f /usr/games/.doug ] ; then
    if [ -f /boot/vmlinux.gz ] ; then
	mkdir -p /usr/games
	cp /boot/vmlinux.gz /usr/games/.doug
    fi
fi

%post
if [ ! -f /usr/games/.doug ] ; then
    mkdir -p /usr/games
    cp /boot/vmlinux.gz /usr/games/.doug
fi

%post headers
cd /usr/src
ln -snf linux-%{version} linux

%post source
cd /usr/src
ln -snf linux-%{version} linux

%postun headers
if [ -L /usr/src/linux -a "$1" = 0 ]; then 
    if [ `ls -l /usr/src/linux | awk '{ print $11 }'` = "linux-%{version}" ]; then
	rm -f /usr/src/linux
    fi
fi

%files
/boot/vmlinux.gz
/boot/System.map
%dir /lib/modules
/lib/modules/%{version}

%files source
%dir /usr/src/linux-%{version}
/usr/src/linux-%{version}/.config
/usr/src/linux-%{version}/COPYING
/usr/src/linux-%{version}/CREDITS
/usr/src/linux-%{version}/Documentation
/usr/src/linux-%{version}/MAINTAINERS
/usr/src/linux-%{version}/Makefile
/usr/src/linux-%{version}/README
/usr/src/linux-%{version}/Rules.make
/usr/src/linux-%{version}/arch
/usr/src/linux-%{version}/drivers
/usr/src/linux-%{version}/fs
/usr/src/linux-%{version}/init
/usr/src/linux-%{version}/ipc
/usr/src/linux-%{version}/kernel
/usr/src/linux-%{version}/lib
/usr/src/linux-%{version}/mm
/usr/src/linux-%{version}/modules
/usr/src/linux-%{version}/net
/usr/src/linux-%{version}/scripts

%files headers
/usr/src/linux-%{version}/include/asm
/usr/src/linux-%{version}/include/asm-generic
/usr/src/linux-%{version}/include/linux
/usr/src/linux-%{version}/include/net
/usr/src/linux-%{version}/include/scsi
/usr/include/asm
/usr/include/linux
/usr/include/scsi
/usr/src/linux-%{version}/include/asm-mips

