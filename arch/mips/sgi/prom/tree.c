/* $Id: tree.c,v 1.1 1997/10/27 23:26:14 davem Exp $
 * tree.c: PROM component device tree code.
 *
 * Copyright (C) 1996 David S. Miller (dm@engr.sgi.com)
 */

#include <asm/sgialib.h>

#define DEBUG_PROM_TREE

pcomponent *prom_getsibling(pcomponent *this)
{
	if(this == PROM_NULL_COMPONENT)
		return PROM_NULL_COMPONENT;
	return romvec->next_component(this);
}

pcomponent *prom_getchild(pcomponent *this)
{
	return romvec->child_component(this);
}

pcomponent *prom_getparent(pcomponent *child)
{
	if(child == PROM_NULL_COMPONENT)
		return PROM_NULL_COMPONENT;
	return romvec->parent_component(child);
}

long prom_getcdata(void *buffer, pcomponent *this)
{
	return romvec->component_data(buffer, this);
}

pcomponent *prom_childadd(pcomponent *this, pcomponent *tmp, void *data)
{
	return romvec->child_add(this, tmp, data);
}

long prom_delcomponent(pcomponent *this)
{
	return romvec->comp_del(this);
}

pcomponent *prom_componentbypath(char *path)
{
	return romvec->component_by_path(path);
}

#ifdef DEBUG_PROM_TREE
static char *classes[] = {
	"system", "processor", "cache", "adapter", "controller", "peripheral",
	"memory"
};

static char *types[] = {
	"arc", "cpu", "fpu", "picache", "pdcache", "sicache", "sdcache", "sccache",
	"memdev", "eisa adapter", "tc adapter", "scsi adapter", "dti adapter",
	"multi-func adapter", "disk controller", "tp controller",
	"cdrom controller", "worm controller", "serial controller",
	"net controller", "display controller", "parallel controller",
	"pointer controller", "keyboard controller", "audio controller",
	"misc controller", "disk peripheral", "floppy peripheral",
	"tp peripheral", "modem peripheral", "monitor peripheral",
	"printer peripheral", "pointer peripheral", "keyboard peripheral",
	"terminal peripheral", "line peripheral", "net peripheral",
	"misc peripheral", "anonymous"
};

static char *iflags[] = {
	"bogus", "read only", "removable", "console in", "console out",
	"input", "output"
};

static void dump_component(pcomponent *p)
{
	prom_printf("[%p]:class<%s>type<%s>flags<%s>ver<%d>rev<%d>",
		    p, classes[p->class], types[p->type],
		    iflags[p->iflags], p->vers, p->rev);
	prom_printf("key<%08lx>\n\tamask<%08lx>cdsize<%d>ilen<%d>iname<%s>\n",
		    p->key, p->amask, (int)p->cdsize, (int)p->ilen, p->iname);
}

static void traverse(pcomponent *p, int op)
{
	dump_component(p);
	if(prom_getchild(p))
		traverse(prom_getchild(p), 1);
	if(prom_getsibling(p) && op)
		traverse(prom_getsibling(p), 1);
}

void prom_testtree(void)
{
	pcomponent *p;

	p = prom_getchild(PROM_NULL_COMPONENT);
	dump_component(p);
	p = prom_getchild(p);
	while(p) {
		dump_component(p);
		p = prom_getsibling(p);
	}
	prom_printf("press a key\n");
	prom_getchar();
}
#endif
