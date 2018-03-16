/*
 * SMP- and interrupt-safe semaphores..
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * (C) Copyright 1996  Linus Torvalds, Ralf Baechle
 *
 * $Id: semaphore.h,v 1.1 1997/10/27 23:26:49 davem Exp $
 */
#ifndef __ASM_MIPS_SEMAPHORE_H
#define __ASM_MIPS_SEMAPHORE_H

#include <asm/atomic.h>

struct semaphore {
	int count;
	int waking;
	int lock;			/* to make waking testing atomic */
	struct wait_queue * wait;
};

#define MUTEX ((struct semaphore) { 1, 0, 0, NULL })
#define MUTEX_LOCKED ((struct semaphore) { 0, 0, 0, NULL })

extern void __down(struct semaphore * sem);
extern int  __down_interruptible(struct semaphore * sem);
extern void __up(struct semaphore * sem);

#define sema_init(sem, val)	atomic_set(&((sem)->count), val)

extern inline void down(struct semaphore * sem)
{
	if (atomic_dec_return(&sem->count) < 0)
		__down(sem);
}

/*
 * Primitives to spin on a lock.  Needed only for SMP version.
 */
extern inline void get_buzz_lock(int *lock_ptr)
{
#ifdef __SMP__
	while (xchg(lock_ptr,1) != 0) ;
#endif
} /* get_buzz_lock */

extern inline void give_buzz_lock(int *lock_ptr)
{
#ifdef __SMP__
	*lock_ptr = 0 ;
#endif
} /* give_buzz_lock */

extern inline int down_interruptible(struct semaphore * sem)
{
	int ret = 0;
	if (atomic_dec_return(&sem->count) < 0)
		ret = __down_interruptible(sem);
	return ret;
}

/*
 * Note! This is subtle. We jump to wake people up only if
 * the semaphore was negative (== somebody was waiting on it).
 */
extern inline void up(struct semaphore * sem)
{
	if (atomic_inc_return(&sem->count) <= 0)
		__up(sem);
}

#endif /* __ASM_MIPS_SEMAPHORE_H */
