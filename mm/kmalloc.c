/*
 *  linux/mm/kmalloc.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds & Roger Wolff.
 *
 *  Written by R.E. Wolff Sept/Oct '93.
 *
 */

/*
 * Modified by Alex Bligh (alex@cconcepts.co.uk) 4 Apr 1994 to use multiple
 * pages. So for 'page' throughout, read 'area'.
 *
 * Largely rewritten.. Linus
 */

#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <asm/system.h>
#include <asm/dma.h>

/* Define this if you want to try to catch 'use after free' bugs */
#undef SADISTIC_KMALLOC

/* Private flags. */

#define MF_USED 0xffaa0055
#define MF_DMA  0xff00aa55
#define MF_FREE 0x0055ffaa

/*
 * Define this is you think you are loosing your memory
 * - call mem_leak_dump() to see who has what.
 * Only tested on mips.
 */
#undef MEM_LEAK_CHK

#if defined(MEM_LEAK_CHK) /* { */

void mem_leak_insert(void *va, void *pc, int len);
void mem_leak_delete(void *va, int len);

#ifdef mips /* { */
		/* Tacky, but builtin_return_address is broken */
#define MEM_LEAK_GET_RA 	\
		void *ra_save; __asm__("sw $31,%0": "=m" (ra_save) : )
#define MEM_LEAK_RA ra_save
#else	/* } untested below { */
#define MEM_LEAK_GET_RA
#define MEM_LEAK_RA builtin_return_address(1)
#endif /* } */

#define MEM_LEAK_INSERT(va, pc, len)	mem_leak_insert(va, pc, len)
#define MEM_LEAK_DELETE(va, len)	mem_leak_delete(va, len)
#else /* } MEM_LEAK_CHK { */
#define MEM_LEAK_GET_RA
#define MEM_LEAK_INSERT(va, pc, len)
#define MEM_LEAK_DELETE(va, len)
#endif /* } */


/*
 * Much care has gone into making these routines in this file reentrant.
 *
 * The fancy bookkeeping of nbytesmalloced and the like are only used to
 * report them to the user (oooohhhhh, aaaaahhhhh....) are not
 * protected by cli(). (If that goes wrong. So what?)
 *
 * These routines restore the interrupt status to allow calling with ints
 * off.
 */

/*
 * A block header. This is in front of every malloc-block, whether free or not.
 */
struct block_header {
	unsigned long bh_flags;
	union {
		unsigned long ubh_length;
		struct block_header *fbh_next;
	} vp;
};


#define bh_length vp.ubh_length
#define bh_next   vp.fbh_next
#define BH(p) ((struct block_header *)(p))


/*
 * The page descriptor is at the front of every page that malloc has in use.
 */
struct page_descriptor {
	struct page_descriptor *next;
	struct block_header *firstfree;
	int order;
	int nfree;
};


#define PAGE_DESC(p) ((struct page_descriptor *)(((unsigned long)(p)) & PAGE_MASK))


/*
 * A size descriptor describes a specific class of malloc sizes.
 * Each class of sizes has its own freelist.
 */
struct size_descriptor {
	struct page_descriptor *firstfree;
	struct page_descriptor *dmafree;	/* DMA-able memory */
	int nblocks;

	int nmallocs;
	int nfrees;
	int nbytesmalloced;
	int npages;
	unsigned long gfporder;	/* number of pages in the area required */
};

/*
 * For now it is unsafe to allocate bucket sizes between n and
 * n-sizeof(page_descriptor) where n is PAGE_SIZE * any power of two
 *
 * The blocksize and sizes arrays _must_ match!
 */
#if PAGE_SIZE == 4096
static const unsigned int blocksize[] = {
	32,
	64,
	128,
	252,
	508,
	1020,
	2040,
	4096 - 16,
	8192 - 16,
	16384 - 16,
	32768 - 16,
	65536 - 16,
	131072 - 16,
	0
};

static struct size_descriptor sizes[] =
{
	{NULL, NULL, 127, 0, 0, 0, 0, 0},
	{NULL, NULL, 63, 0, 0, 0, 0, 0},
	{NULL, NULL, 31, 0, 0, 0, 0, 0},
	{NULL, NULL, 16, 0, 0, 0, 0, 0},
	{NULL, NULL, 8, 0, 0, 0, 0, 0},
	{NULL, NULL, 4, 0, 0, 0, 0, 0},
	{NULL, NULL, 2, 0, 0, 0, 0, 0},
	{NULL, NULL, 1, 0, 0, 0, 0, 0},
	{NULL, NULL, 1, 0, 0, 0, 0, 1},
	{NULL, NULL, 1, 0, 0, 0, 0, 2},
	{NULL, NULL, 1, 0, 0, 0, 0, 3},
	{NULL, NULL, 1, 0, 0, 0, 0, 4},
	{NULL, NULL, 1, 0, 0, 0, 0, 5},
	{NULL, NULL, 0, 0, 0, 0, 0, 0}
};
#elif PAGE_SIZE == 8192
static const unsigned int blocksize[] = {
	64,
	128,
	248,
	504,
	1016,
	2040,
	4080,
	8192 - 32,
	16384 - 32,
	32768 - 32,
	65536 - 32,
	131072 - 32,
	262144 - 32,
	0
};

struct size_descriptor sizes[] =
{
	{NULL, NULL, 127, 0, 0, 0, 0, 0},
	{NULL, NULL, 63, 0, 0, 0, 0, 0},
	{NULL, NULL, 31, 0, 0, 0, 0, 0},
	{NULL, NULL, 16, 0, 0, 0, 0, 0},
	{NULL, NULL, 8, 0, 0, 0, 0, 0},
	{NULL, NULL, 4, 0, 0, 0, 0, 0},
	{NULL, NULL, 2, 0, 0, 0, 0, 0},
	{NULL, NULL, 1, 0, 0, 0, 0, 0},
	{NULL, NULL, 1, 0, 0, 0, 0, 1},
	{NULL, NULL, 1, 0, 0, 0, 0, 2},
	{NULL, NULL, 1, 0, 0, 0, 0, 3},
	{NULL, NULL, 1, 0, 0, 0, 0, 4},
	{NULL, NULL, 1, 0, 0, 0, 0, 5},
	{NULL, NULL, 0, 0, 0, 0, 0, 0}
};
#else
#error you need to make a version for your pagesize
#endif

#define NBLOCKS(order)          (sizes[order].nblocks)
#define BLOCKSIZE(order)        (blocksize[order])
#define AREASIZE(order)		(PAGE_SIZE<<(sizes[order].gfporder))

/*
 * Create a small cache of page allocations: this helps a bit with
 * those pesky 8kB+ allocations for NFS when we're temporarily
 * out of memory..
 *
 * This is a _truly_ small cache, we just cache one single page
 * order (for orders 0, 1 and 2, that is  4, 8 and 16kB on x86).
 */
#define MAX_CACHE_ORDER 3
struct page_descriptor * kmalloc_cache[MAX_CACHE_ORDER];

static inline struct page_descriptor * get_kmalloc_pages(unsigned long priority,
	unsigned long order, int dma)
{
	return (struct page_descriptor *) __get_free_pages(priority, order, dma);
}

static inline void free_kmalloc_pages(struct page_descriptor * page,
	unsigned long order, int dma)
{
	if (!dma && order < MAX_CACHE_ORDER) {
		page = xchg(kmalloc_cache+order, page);
		if (!page)
			return;
	}
	free_pages((unsigned long) page, order);
}

long kmalloc_init(long start_mem, long end_mem)
{
	int order;

/*
 * Check the static info array. Things will blow up terribly if it's
 * incorrect. This is a late "compile time" check.....
 */
	for (order = 0; BLOCKSIZE(order); order++) {
		if ((NBLOCKS(order) * BLOCKSIZE(order) + sizeof(struct page_descriptor)) >
		    AREASIZE(order)) {
			printk("Cannot use %d bytes out of %d in order = %d block mallocs\n",
			       (int) (NBLOCKS(order) * BLOCKSIZE(order) +
				      sizeof(struct page_descriptor)),
			        (int) AREASIZE(order),
			       BLOCKSIZE(order));
			panic("This only happens if someone messes with kmalloc");
		}
	}
	return start_mem;
}


/*
 * Ugh, this is ugly, but we want the default case to run
 * straight through, which is why we have the ugly goto's
 */
void *kmalloc(size_t size, int priority)
{
	unsigned long flags;
	unsigned long type;
	int order, dma;
	struct block_header *p;
	struct page_descriptor *page, **pg;
	struct size_descriptor *bucket = sizes;
	MEM_LEAK_GET_RA;	/* Must be last */

	/* Get order */
	order = 0;
	{
		unsigned int realsize = size + sizeof(struct block_header);
		for (;;) {
			int ordersize = BLOCKSIZE(order);
			if (realsize <= ordersize)
				break;
			order++;
			bucket++;
			if (ordersize)
				continue;
			printk("kmalloc of too large a block (%d bytes).\n", (int) size);
			return NULL;
		}
	}

	dma = 0;
	type = MF_USED;
	pg = &bucket->firstfree;
	if (priority & GFP_DMA) {
		dma = 1;
		type = MF_DMA;
		pg = &bucket->dmafree;
	}

	priority &= GFP_LEVEL_MASK;

/* Sanity check... */
	if (intr_count && priority != GFP_ATOMIC) {
		static int count = 0;
		if (++count < 5) {
			printk("kmalloc called nonatomically from interrupt %p\n",
			       __builtin_return_address(0));
			priority = GFP_ATOMIC;
		}
	}

	save_flags(flags);
	cli();
	page = *pg;
	if (!page)
		goto no_bucket_page;

	p = page->firstfree;
	if (p->bh_flags != MF_FREE)
		goto not_free_on_freelist;

found_it:
	page->firstfree = p->bh_next;
	page->nfree--;
	if (!page->nfree)
		*pg = page->next;
	restore_flags(flags);
	bucket->nmallocs++;
	bucket->nbytesmalloced += size;
	p->bh_flags = type;	/* As of now this block is officially in use */
	p->bh_length = size;
#ifdef SADISTIC_KMALLOC
	memset(p+1, 0xf0, size);
#endif
	MEM_LEAK_INSERT(p + 1, MEM_LEAK_RA, size);
	return p + 1;		/* Pointer arithmetic: increments past header */


no_bucket_page:
	/*
	 * If we didn't find a page already allocated for this
	 * bucket size, we need to get one..
	 *
	 * This can be done with ints on: it is private to this invocation
	 */
	restore_flags(flags);

	{
		int i, sz;
		
		/* sz is the size of the blocks we're dealing with */
		sz = BLOCKSIZE(order);

		page = get_kmalloc_pages(priority, bucket->gfporder, dma);
		if (!page)
			goto no_free_page;
found_cached_page:

		bucket->npages++;

		page->order = order;
		/* Loop for all but last block: */
		i = (page->nfree = bucket->nblocks) - 1;
		p = BH(page + 1);
		while (i > 0) {
			i--;
			p->bh_flags = MF_FREE;
			p->bh_next = BH(((long) p) + sz);
			p = p->bh_next;
		}
		/* Last block: */
		p->bh_flags = MF_FREE;
		p->bh_next = NULL;

		p = BH(page+1);
	}

	/*
	 * Now we're going to muck with the "global" freelist
	 * for this size: this should be uninterruptible
	 */
	cli();
	page->next = *pg;
	*pg = page;
	goto found_it;


no_free_page:
	/*
	 * No free pages, check the kmalloc cache of
	 * pages to see if maybe we have something available
	 */
	if (!dma && order < MAX_CACHE_ORDER) {
		page = xchg(kmalloc_cache+order, page);
		if (page)
			goto found_cached_page;
	}
	{
		static unsigned long last = 0;
		if (priority != GFP_BUFFER && priority != GFP_IO &&
		    (last + 10 * HZ < jiffies)) {
			last = jiffies;
			printk("Couldn't get a free page.....\n");
		}
		return NULL;
	}

not_free_on_freelist:
	restore_flags(flags);
	printk("Problem: block on freelist at %08lx isn't free.\n", (long) p);
	return NULL;
}

void kfree(void *__ptr)
{
	int dma;
	unsigned long flags;
	unsigned int order;
	struct page_descriptor *page, **pg;
	struct size_descriptor *bucket;

	if (!__ptr)
		goto null_kfree;

	MEM_LEAK_DELETE(__ptr, 0);

#define ptr ((struct block_header *) __ptr)
	page = PAGE_DESC(ptr);
	__ptr = ptr - 1;
	if (~PAGE_MASK & (unsigned long)page->next)
		goto bad_order;
	order = page->order;
	if (order >= sizeof(sizes) / sizeof(sizes[0]))
		goto bad_order;
	bucket = sizes + order;
	dma = 0;
	pg = &bucket->firstfree;
	if (ptr->bh_flags == MF_DMA) {
		dma = 1;
		ptr->bh_flags = MF_USED;
		pg = &bucket->dmafree;
	}
	if (ptr->bh_flags != MF_USED)
		goto bad_order;

	ptr->bh_flags = MF_FREE;	/* As of now this block is officially free */
#ifdef SADISTIC_KMALLOC
	memset(ptr+1, 0xe0, ptr->bh_length);
#endif
	save_flags(flags);
	cli();

	bucket->nfrees++;
	bucket->nbytesmalloced -= ptr->bh_length;

	ptr->bh_next = page->firstfree;
	page->firstfree = ptr;
	if (!page->nfree++) {
/* Page went from full to one free block: put it on the freelist. */
		if (bucket->nblocks == 1)
			goto free_page;
		page->next = *pg;
		*pg = page;
	}
/* If page is completely free, free it */
	if (page->nfree == bucket->nblocks) {
		for (;;) {
			struct page_descriptor *tmp = *pg;
			if (!tmp)
				goto not_on_freelist;
			if (tmp == page)
				break;
			pg = &tmp->next;
		}
		*pg = page->next;
free_page:
		bucket->npages--;
		free_kmalloc_pages(page, bucket->gfporder, dma);
	}
	restore_flags(flags);
null_kfree:
	return;

bad_order:
	printk("kfree of non-kmalloced memory: %p, next= %p, order=%d\n",
	       ptr+1, page->next, page->order);
	return;

not_on_freelist:
	restore_flags(flags);
	printk("Ooops. page %p doesn't show on freelist.\n", page);
}


#ifdef MEM_LEAK_CHK /* { */
struct mem_hash {
    void		*va;		/* vaddr of hash */
    struct leak_pc_data	*data;
    struct mem_hash	*next;
    void		*pad;		/* to simplify alignment */
};

/* 
 * These numbers are derived by inspection from the
 * "blocksize" arrays above.  It should be dynamic.
 */
#define LEAK_NLENS 14 /* kernel vm number */
/* #define LEAK_NLENS 8 user mode test number */
#define MIN_ALLOC_SHFT 6
#define MAX_ALLOC_SHFT (MIN_ALLOC_SHFT + LEAK_NLENS)

struct leak_pc_data {
    void	*pc;
    int		tot_allocs;
    int		len[LEAK_NLENS];	/* active allocs per len */
};

/*
 * These hashlists grows for every PC that does does an alloc, and we
 * track an entry in the va hash for every active alloc
 */

#define LEAK_NHASH	4096

struct mem_hash *mem_leak_pc[LEAK_NHASH];
struct mem_hash *mem_leak_va[LEAK_NHASH];

char *leak_cur_alloc;	/* ptr to page for next leak struct alloc */
char *leak_cur_end;	/* ptr to end of region for next leak alloc */
int meta_alloc;

int leak_nmallocs;
int leak_nfrees;

#define LEAK_VA_ALGN	2
#define LEAK_HASH(va)	(((int) (va) >> LEAK_VA_ALGN) & (LEAK_NHASH - 1))

struct mem_hash  *vahash_list;	/* we recycle VA mem_hash structures */

#define dprint if (0) printk

void *
mem_leak_alloc(int size)
{
    void *cur;
    /*
     * Do we have spare mem_hash's lying around?
     */
    if (size == sizeof(struct mem_hash) && vahash_list) {
	cur = vahash_list;
	vahash_list = vahash_list->next;
	return cur;
    }

    /*
     * No recycled structs, so see if we have space in the current
     * block of free space.  If no, do the alloc thing...
     */
    if (leak_cur_alloc + size > leak_cur_end) {
	leak_cur_alloc = (char *) get_kmalloc_pages(GFP_ATOMIC, 1, 0);
	if ( ! leak_cur_alloc) {
	    leak_cur_end = NULL;
	    printk("mem_leak_alloc: malloc failed\n");
	    return NULL;
	}
	leak_cur_end = leak_cur_alloc + PAGE_SIZE;
	meta_alloc += PAGE_SIZE;
    }
    cur = leak_cur_alloc;
    leak_cur_alloc += size;

    return (void *) cur;
}

void
mem_leak_free(void *va, int size)
{
    struct mem_hash *vam;

    if (size != sizeof(struct mem_hash)) {
	printk("mem_leak_free: unexpected size %d\n", size);
	return;
    }

    vam = (struct mem_hash *) va;

    vam->next = vahash_list;
    vahash_list = vam;
}

void
mem_leak_insert(void *va, void *pc, int len)
{
    struct mem_hash	**pchp;
    struct mem_hash	**vahp;
    struct leak_pc_data	*lpcd;
    int			bckt;
    unsigned long flags;

    save_flags(flags);
    cli();

    dprint("mem_leak_insert: add va 0x%x, pc 0x%x, len 0x%x\n",
	    (int) va, (int) pc, len);
    /*
     * Look up the pc, create if necessary, then
     * lookup the va and point it at the pc.
     *
     * complain if va already allocated
     */
    pchp = &mem_leak_pc[LEAK_HASH(pc)];

    while (*pchp && (*pchp)->va != pc)
	pchp = &(*pchp)->next;

    /*
     * Do we need alloc?
     */
    if ( ! *pchp) {
	*pchp = mem_leak_alloc(sizeof(**pchp));
	if ( ! *pchp) {
	    restore_flags(flags);
	    return;
	}
	memset(*pchp, 0, sizeof(**pchp));
	(*pchp)->va = pc;
	(*pchp)->data = (struct leak_pc_data *)
			mem_leak_alloc(sizeof(struct leak_pc_data));

	if ( ! (*pchp)->data) {
	    mem_leak_free(*pchp, sizeof(*pchp));
	    *pchp = 0;
	    restore_flags(flags);
	    return;
	}
	lpcd = (*pchp)->data;
	memset(lpcd, 0, sizeof(struct leak_pc_data));
	lpcd->pc = pc;
    }
    else {
	lpcd = (*pchp)->data;
	if (lpcd->pc != pc) {
	    printk("mem_leak_insert: pc mismatch have 0x%x, found 0x%x\n",
		    (int) pc, (int) lpcd->pc);
	}
    }

    dprint("\tleak_pc_data at 0x%x\n", (int) lpcd);

    for (bckt = 0; 1 << (bckt + MIN_ALLOC_SHFT) < len
    			&& bckt < LEAK_NLENS - 1; bckt++)
	    ;

    vahp = &mem_leak_va[LEAK_HASH(va)];

    while (*vahp && (*vahp)->va != va)
	vahp = &(*vahp)->next;

    if ( ! *vahp) {
	*vahp = mem_leak_alloc(sizeof(**vahp));

	if ( ! (*vahp)) {
	    restore_flags(flags);
	    return;
	}
	memset(*vahp, 0, sizeof(**vahp));
	(*vahp)->va = va;
    }

    lpcd->tot_allocs++;
    lpcd->len[bckt]++;

    if ((*vahp)->data) {
	printk("mem_leak_insert: va 0x%x not free - old pc 0x%x, new pc 0x%x\n",
		(int) va, (int) (*vahp)->data->pc, (int) lpcd->pc);
    }

    (*vahp)->data = lpcd;
    (*vahp)->pad = (void *) len;
    leak_nmallocs++;
    restore_flags(flags);
}

/*
 *    Delete a VA and decrement the PC histogram.
 */
void
mem_leak_delete(void *va, int len)
{
    struct mem_hash	**vahp, *ovap;
    struct leak_pc_data	*lpcd;
    int			bckt;
    unsigned long flags;

    dprint("mem_leak_delete: del va 0x%x ", (int) va);

    save_flags(flags);
    cli();

    vahp = &mem_leak_va[LEAK_HASH(va)];

    while (*vahp && (*vahp)->va != va)
	vahp = &(*vahp)->next;

    if ( ! *vahp) {
	printk("mem_leak_delete: va 0x%x never allocated\n", (int) va);
	restore_flags(flags);
	return;
    }
    ovap = *vahp;

    lpcd = ovap->data;

    if ( ! lpcd) {
	printk("mem_leak_delete: va 0x%x currently free\n", (int) va);
	restore_flags(flags);
	return;
    }

    if ( ! len)
	len = (int) ovap->pad;

    dprint("pc 0x%x, len 0x%x\n", (int) lpcd->pc, len);
    dprint("\tleak_pc_data at 0x%x\n", (int) lpcd);

    for (bckt = 0; 1 << (bckt + MIN_ALLOC_SHFT) < len
    			&& bckt < LEAK_NLENS - 1; bckt++)
	    ;

    if (--lpcd->tot_allocs < 0) {
	printk("mem_leak_delete: total underflow pc 0x%x\n", (int) lpcd->pc);
    }
    if (--lpcd->len[bckt] < 0) {
	printk("mem_leak_delete: underflow pc 0x%x, bucket %d\n",
			(int) lpcd->pc, 1 << (bckt + MIN_ALLOC_SHFT));
    }
    *vahp = ovap->next;
    mem_leak_free(ovap, sizeof(*ovap));
    leak_nfrees++;
    restore_flags(flags);
}

void
mem_leak_dump(void)
{
    struct mem_hash *pc;
    struct leak_pc_data *lpcd;
    int i, j;
    int do_warn;
    int tot_sum, sum;
    int outcnt;
    unsigned long flags;

    save_flags(flags);
    cli(); 

    printk("%d total mallocs, %d active, %d pages meta data\n",
	    leak_nmallocs, leak_nmallocs - leak_nfrees,
	    (int) (meta_alloc / PAGE_SIZE));
    tot_sum = 0;
    for (i = 0; i < LEAK_NHASH; i++) {
	for (pc = mem_leak_pc[i]; pc; pc = pc->next) {
	    lpcd = pc->data;
	    do_warn = 1;
	    sum = 0;
	    tot_sum += lpcd->tot_allocs;
	    outcnt = 0;

	    if (lpcd->pc != pc->va) {
		printk("pc mismatch hash 0x%x, histogram 0x%x\n",
			(int) pc->va, (int) lpcd->pc);
	    }
	    if (lpcd->tot_allocs) 
		printk("0x%08x : %-6d - ", (int) pc->va, lpcd->tot_allocs);

	    for (j = 0; j < LEAK_NLENS; j++) {
		if (lpcd->len[j]) {
		    sum += lpcd->len[j];
		    if ( ! lpcd->tot_allocs && do_warn) {
			printk("0x%x : zero         - ", (int) pc->va);
			do_warn = 0;
		    }
		    outcnt++;
		    if (outcnt && (outcnt % 4) == 0)
			printk("\n                      ");
		    printk("%5d : %-5d ",
		    		1 << (MIN_ALLOC_SHFT + j), lpcd->len[j]);
		}
	    }
	    if (lpcd->tot_allocs || do_warn == 0)
		printk("\n");
	    if (sum != lpcd->tot_allocs)
		printk("WARNING: total allocs != sum of allocs\n");
	}
    }
    if (tot_sum != leak_nmallocs - leak_nfrees) {
	printk("total allocs %d not balanced with mallocs/frees\n", tot_sum);
    }
    printk("\n");
    restore_flags(flags);
}
#endif /* } MEM_LEAK_CHK */
