/* $Id: env.c,v 1.1 1997/10/27 23:26:13 davem Exp $
 * env.c: ARCS environment variable routines.
 *
 * Copyright (C) 1996 David S. Miller (dm@engr.sgi.com)
 */

#include <linux/kernel.h>
#include <linux/string.h>

#include <asm/sgialib.h>

char *prom_getenv(char *name)
{
	return romvec->get_evar(name);
}

long prom_setenv(char *name, char *value)
{
	return romvec->set_evar(name, value);
}
