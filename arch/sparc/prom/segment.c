/* $Id: segment.c,v 1.1.1.1 1997/10/27 22:44:04 davem Exp $
 * segment.c:  Prom routine to map segments in other contexts before
 *             a standalone is completely mapped.  This is for sun4 and
 *             sun4c architectures only.
 *
 * Copyright (C) 1995 David S. Miller (davem@caip.rutgers.edu)
 */

#include <asm/openprom.h>
#include <asm/oplib.h>

/* Set physical segment 'segment' at virtual address 'vaddr' in
 * context 'ctx'.
 */
void
prom_putsegment(int ctx, unsigned long vaddr, int segment)
{
	(*(romvec->pv_setctxt))(ctx, (char *) vaddr, segment);
	return;
}
