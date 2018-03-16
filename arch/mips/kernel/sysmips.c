/*
 * MIPS specific syscalls
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1995, 1996, 1997 by Ralf Baechle
 *
 * $Id: sysmips.c,v 1.5 1998/11/23 11:31:19 gid Exp $
 */
#include <linux/config.h>
#include <linux/errno.h>
#include <linux/linkage.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/smp_lock.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/utsname.h>

#include <asm/cachectl.h>
#include <asm/pgtable.h>
#include <asm/sysmips.h>
#include <asm/segment.h>

#ifdef CONFIG_COBALT_MICRO_SERVER
#include <asm/mc146818rtc.h>
#endif

static inline size_t
strnlen_user(const char *s, size_t count)
{
	return strnlen(s, count);
}

/*
 * How long a hostname can we get from user space?
 *  -EFAULT if invalid area or too long
 *  0 if ok
 *  >0 EFAULT after xx bytes
 */
static inline int
get_max_hostname(unsigned long address)
{
	struct vm_area_struct * vma;

	vma = find_vma(current->mm, address);
	if (!vma || vma->vm_start > address || !(vma->vm_flags & VM_READ))
		return -EFAULT;
	address = vma->vm_end - address;
	if (address > PAGE_SIZE)
		return 0;
	if (vma->vm_next && vma->vm_next->vm_start == vma->vm_end &&
	   (vma->vm_next->vm_flags & VM_READ))
		return 0;
	return address;
}

asmlinkage int
sys_sysmips(int cmd, int arg1, int arg2, int arg3)
{
	int	*p;
	char	*name;
	int	flags, tmp, len, retval;

	switch(cmd)
	{
	case SETNAME:
		if (!suser())
			return -EPERM;
		name = (char *) arg1;
		retval = get_max_hostname((unsigned long)name);
		if (retval < 0)
			return len;
		len = strnlen_user(name, retval);
		if (len == 0 || len > __NEW_UTS_LEN)
			return -EINVAL;
		memcpy_fromfs(system_utsname.nodename, name, len);
		system_utsname.nodename[len] = '\0';
		return 0;

	case MIPS_ATOMIC_SET:
		/* This is broken in case of page faults and SMP ...
		   Risc/OS fauls after maximum 20 tries with EAGAIN.  */
		p = (int *) arg1;
		retval = verify_area(VERIFY_WRITE, p, sizeof(*p));
		if (retval)
			goto out;
		save_and_cli(flags);
		retval = *p;
		*p = arg2;
		restore_flags(flags);
		goto out;

	case MIPS_FIXADE:
		tmp = current->tss.mflags & ~3;
		current->tss.mflags = tmp | (arg1 & 3);
		retval = 0;
		goto out;

	case FLUSH_CACHE:
		flush_cache_all();
		retval = 0;
		goto out;

	case MIPS_RDNVRAM:
		retval = -EIO;
		goto out;

#ifdef CONFIG_COBALT_MICRO_SERVER
	case MIPS_READ_CMOS:
		if (!suser())
			return -EPERM;
		retval = CMOS_READ(arg1);
		goto out;

	case MIPS_WRITE_CMOS:
		if (!suser())
			return -EPERM;

		/*
		 * CMOS_WRITE expects the cmos location as the *second* arg
		 * and the value as the first.  this is way counter-intuitive
		 * given that CMOS_READ passes the location as the first (sole)
		 * arg.  (add new args at the end of existing arg lists, not at
		 * the beginning.  e.g. strcpy vs strncpy)
		 * 
		 * CMOS_WRITE is used in a bunch of places around the kernel
		 * so I'm not going to change its interface, only this sysmips
		 * interface, which is depracated anyway so I can do what I want
		 * with it.  -gid
		 */

		CMOS_WRITE(arg2, arg1);
		retval = 0;
		goto out;
#endif

	default:
		retval = -EINVAL;
		goto out;
	}

out:
	return retval;
}

/*
 * No implemented yet ...
 */
asmlinkage int
sys_cachectl(char *addr, int nbytes, int op)
{
	return -ENOSYS;
}

#ifdef CONFIG_BINFMT_IRIX
/*
 * For emulation of various binary types, and their shared libs,
 * we need this.
 */
extern int do_open_namei(const char *pathname, int flag, int mode,
			 struct inode **res_inode, struct inode *base);

/* Only one at this time. */
#define IRIX32_EMUL "/usr/gnemul/irix"

int open_namei(const char *pathname, int flag, int mode,
	       struct inode **res_inode, struct inode *base)
{
	if(!base && (current->personality == PER_IRIX32) &&
	   *pathname == '/') {
		struct inode *emul_ino;
		const char *p = pathname;
		char *emul_path = IRIX32_EMUL;
		int v;

		while (*p == '/')
			p++;

		if(do_open_namei (emul_path, flag, mode, &emul_ino, NULL) >= 0 &&
		   emul_ino) {
			v = do_open_namei (p, flag, mode, res_inode, emul_ino);
			if(v >= 0)
				return v;
		}
	}
	return do_open_namei (pathname, flag, mode, res_inode, base);
}

#endif /* CONFIG_BINFMT_IRIX */
