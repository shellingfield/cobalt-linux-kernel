#ifndef __ASM_MIPS_PGTABLE_H
#define __ASM_MIPS_PGTABLE_H

#include <asm/addrspace.h>
#include <asm/mipsconfig.h>

#ifndef __LANGUAGE_ASSEMBLY__

#include <linux/linkage.h>
#include <asm/cachectl.h>
#include <asm/mmu_context.h>

/* Basically we have the same two-level (which is the logical three level
 * Linux page table layout folded) page tables as the i386.  Some day
 * when we have proper page coloring support we can have a 1% quicker
 * tlb refill handling mechanism, but for now it is a bit slower but
 * works even with the cache aliasing problem the R4k and above have.
 */

#endif /* !defined (__LANGUAGE_ASSEMBLY__) */

/* PMD_SHIFT determines the size of the area a second-level page table can map */
#define PMD_SHIFT	22
#define PMD_SIZE	(1UL << PMD_SHIFT)
#define PMD_MASK	(~(PMD_SIZE-1))

/* PGDIR_SHIFT determines what a third-level page table entry can map */
#define PGDIR_SHIFT	22
#define PGDIR_SIZE	(1UL << PGDIR_SHIFT)
#define PGDIR_MASK	(~(PGDIR_SIZE-1))

/* Entries per page directory level: we use two-level, so
 * we don't really have any PMD directory physically.
 */
#define PTRS_PER_PTE	1024
#define PTRS_PER_PMD	1
#define PTRS_PER_PGD	1024

#define VMALLOC_START     KSEG2
#define VMALLOC_VMADDR(x) ((unsigned long)(x))

/* Note that we shift the lower 32bits of each EntryLo[01] entry
 * 6 bits to the left. That way we can convert the PFN into the
 * physical address by a single 'and' operation and gain 6 additional
 * bits for storing information which isn't present in a normal
 * MIPS page table.
 *
 * Similar to the Alpha port, we need to keep track of the ref
 * and mod bits in software.  We have a software "yeah you can read
 * from this page" bit, and a hardware one which actually lets the
 * process read from the page.  On the same token we have a software
 * writable bit and the real hardware one which actually lets the
 * process write to the page, this keeps a mod bit via the hardware
 * dirty bit.
 *
 * Certain revisions of the R4000 and R5000 have a bug where if a
 * certain sequence occurs in the last 3 instructions of an executable
 * page, and the following page is not mapped, the cpu can do
 * unpredictable things.  The code (when it is written) to deal with
 * this problem will be in the update_mmu_cache() code for the r4k.
 */
#define _PAGE_PRESENT               (1<<0)  /* implemented in software */
#define _PAGE_READ                  (1<<1)  /* implemented in software */
#define _PAGE_WRITE                 (1<<2)  /* implemented in software */
#define _PAGE_ACCESSED              (1<<3)  /* implemented in software */
#define _PAGE_MODIFIED              (1<<4)  /* implemented in software */
#define _PAGE_R4KBUG                (1<<5)  /* workaround for r4k bug  */
#define _PAGE_GLOBAL                (1<<6)
#define _PAGE_VALID                 (1<<7)
#define _PAGE_SILENT_READ           (1<<7)  /* synonym                 */
#define _PAGE_DIRTY                 (1<<8)  /* The MIPS dirty bit      */
#define _PAGE_SILENT_WRITE          (1<<8)
#define _CACHE_CACHABLE_NO_WA       (0<<9)  /* R4600 only              */
#define _CACHE_CACHABLE_WA          (1<<9)  /* R4600 only              */
#define _CACHE_UNCACHED             (2<<9)  /* R4[0246]00              */
#define _CACHE_CACHABLE_NONCOHERENT (3<<9)  /* R4[0246]00              */
#define _CACHE_CACHABLE_CE          (4<<9)  /* R4[04]00 only           */
#define _CACHE_CACHABLE_COW         (5<<9)  /* R4[04]00 only           */
#define _CACHE_CACHABLE_CUW         (6<<9)  /* R4[04]00 only           */
#define _CACHE_CACHABLE_ACCELERATED (7<<9)  /* R10000 only             */
#define _CACHE_MASK                 (7<<9)

#define __READABLE	(_PAGE_READ | _PAGE_SILENT_READ | _PAGE_ACCESSED)
#define __WRITEABLE	(_PAGE_WRITE | _PAGE_SILENT_WRITE | _PAGE_MODIFIED)

#define _PAGE_CHG_MASK  (PAGE_MASK | __READABLE | __WRITEABLE | _CACHE_MASK)

#define PAGE_NONE	__pgprot(_PAGE_PRESENT | _PAGE_ACCESSED | \
                        _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_SHARED     __pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_WRITE | \
			_CACHE_CACHABLE_NONCOHERENT)
#define PAGE_COPY       __pgprot(_PAGE_PRESENT | _PAGE_READ | \
			_CACHE_CACHABLE_NONCOHERENT)
#define PAGE_READONLY   __pgprot(_PAGE_PRESENT | _PAGE_READ | \
			_CACHE_CACHABLE_NONCOHERENT)
#define PAGE_KERNEL	__pgprot(_PAGE_PRESENT | __READABLE | __WRITEABLE | \
			_CACHE_CACHABLE_NONCOHERENT)

/*
 * MIPS can't do page protection for execute, and considers that the same like
 * read. Also, write permissions imply read permissions. This is the closest
 * we can get by reasonable means..
 */
#define __P000	PAGE_NONE
#define __P001	PAGE_READONLY
#define __P010	PAGE_COPY
#define __P011	PAGE_COPY
#define __P100	PAGE_READONLY
#define __P101	PAGE_READONLY
#define __P110	PAGE_COPY
#define __P111	PAGE_COPY

#define __S000	PAGE_NONE
#define __S001	PAGE_READONLY
#define __S010	PAGE_SHARED
#define __S011	PAGE_SHARED
#define __S100	PAGE_READONLY
#define __S101	PAGE_READONLY
#define __S110	PAGE_SHARED
#define __S111	PAGE_SHARED

#if !defined (__LANGUAGE_ASSEMBLY__)

/* Cache flushing:
 *
 *  - flush_cache_all() flushes entire cache
 *  - flush_cache_mm(mm) flushes the specified mm context's cache lines
 *  - flush_cache_page(vma, vmaddr) flushes a single page
 *  - flush_cache_range(mm, start, end) flushes a range of pages
 *  - flush_page_to_ram(page) write back kernel page to ram
 */
#define flush_cache_sigtramp(__addr)		  \
do {	unsigned long __line = (__addr) & ~31;	  \
	__asm__ __volatile__(			  \
	".set	mips3\n\t"			  \
	".set	noreorder\n\t"			  \
	"cache	0x15, 0x00(%0)\n\t"		  \
	"cache	0x15, 0x20(%0)\n\t"		  \
	"cache	0x10, 0x00(%0)\n\t"		  \
	"cache	0x10, 0x20(%0)\n\t"		  \
	".set	reorder\n\t"			  \
	".set	mips0"				  \
	: /* no outputs */			  \
	: "r" (__line));			  \
} while(0)

extern void flush_cache_all(void);
#define flush_icache_range(start, end) flush_cache_all()

#define flush_cache_mm(mmptr)	       \
do {	if((mmptr)->context != 0)      \
		flush_cache_all();     \
} while(0)

#define flush_cache_range(mmptr, __start, __end)	\
do {	if((mmptr)->context != 0)			\
		flush_cache_all();			\
} while(0)

extern void __flush_cache_page_current(unsigned long page, int text);
extern void __flush_cache_page_other(unsigned long page, int text);
extern __inline__ void flush_cache_page(struct vm_area_struct *vma,
					unsigned long page, pte_t pte)
{
	struct mm_struct *mm = vma->vm_mm;

	if((mm->context != 0) && (pte_val(pte) & _PAGE_PRESENT)) {
		int text = vma->vm_flags & VM_EXEC;

		if((mm == current->mm) && (pte_val(pte) & _PAGE_VALID)) {
			__flush_cache_page_current(page, text);
		} else {
			extern int dcache_size;

			page = (KSEG0 + (page & (dcache_size - 1)));
			__flush_cache_page_other(page, text);
		}
	}
}

/* If the addresses passed to these routines are valid, they are
 * either:
 *
 * 1) In KSEG0, so we can do a direct flush of the page.
 * 2) In KSEG2, and since every process can translate those
 *    addresses all the time in kernel mode we can do a direct
 *    flush.
 * 3) In KSEG1, no flush necessary.
 */
#define flush_page_to_ram(__pg)						\
do {	if(((__pg) >= KSEG0 && (__pg) < KSEG1) || ((__pg) >= KSEG2))	\
		__flush_cache_page_current((__pg), 0);			\
} while(0)

/*
 * Prototype of the DMA related cacheflushing stuff.
 */
extern void flush_cache_pre_dma_out(unsigned long start, unsigned long size);
extern void flush_cache_post_dma_in(unsigned long start, unsigned long size);

/* TLB flushing:
 *
 *  - flush_tlb_all() flushes all processes TLB entries
 *  - flush_tlb_mm(mm) flushes the specified mm context TLB entries
 *  - flush_tlb_page(mm, vmaddr) flushes a single page
 *  - flush_tlb_range(mm, start, end) flushes a range of pages
 */
extern void flush_tlb_all(void);

extern void __flush_tlb_range(unsigned long ctx, unsigned long start, unsigned long end);
#define flush_tlb_range(mmptr, first_page, last_page)			\
do {	unsigned long __ctx = (mmptr)->context;				\
	if(__ctx != 0) {						\
	       unsigned long __fsize;					\
	       __fsize = ((last_page) - (first_page) + (PAGE_SIZE - 1));\
	       __fsize >>= PAGE_SHIFT;					\
	       __fsize = (__fsize + 1) >> 1;				\
	       if(__fsize <= 48) {					\
			__flush_tlb_range(__ctx, first_page, last_page);\
	       } else {							\
			unsigned long __flags;				\
			save_and_cli(__flags);				\
	       		get_new_mmu_context((mmptr), asid_cache);	\
			if((mmptr) == current->mm)			\
				   set_entryhi((mmptr)->context & 0xff);\
			restore_flags(__flags);				\
	       }							\
	}								\
} while(0)

#define flush_tlb_mm(mmptr)					\
do {	if((mmptr)->context != 0) {				\
		unsigned long __flags;				\
		save_and_cli(__flags);				\
		get_new_mmu_context((mmptr), asid_cache);	\
		if((mmptr) == current->mm)			\
			set_entryhi((mmptr)->context & 0xff);	\
		restore_flags(__flags);				\
	}							\
} while(0)

extern void __flush_tlb_page(unsigned long ctx, unsigned long page);
#define flush_tlb_page(vma, page)		\
do {	struct mm_struct *__mm = (vma)->vm_mm;	\
	unsigned long __ctx = __mm->context;	\
	if(__ctx != 0)				\
		__flush_tlb_page(__ctx, (page));\
} while(0)

/*
 * BAD_PAGETABLE is used when we need a bogus page-table, while
 * BAD_PAGE is used for a bogus page.
 *
 * ZERO_PAGE is a global shared page that is always zero: used
 * for zero-mapped memory areas etc..
 */
extern pte_t __bad_page(void);
extern pte_t *__bad_pagetable(void);

extern unsigned long __zero_page(void);

extern unsigned long empty_zero_page[1024];

#define BAD_PAGETABLE __bad_pagetable()
#define BAD_PAGE __bad_page()
#define ZERO_PAGE ((unsigned long)empty_zero_page)

/* number of bits that fit into a memory pointer */
#define BITS_PER_PTR			(8*sizeof(unsigned long))

/* to align the pointer to a pointer address */
#define PTR_MASK			(~(sizeof(void*)-1))

/*
 * sizeof(void*) == (1 << SIZEOF_PTR_LOG2)
 */
#define SIZEOF_PTR_LOG2			2

/* to find an entry in a page-table */
#define PAGE_PTR(address) \
((unsigned long)(address)>>(PAGE_SHIFT-SIZEOF_PTR_LOG2)&PTR_MASK&~PAGE_MASK)

/* to set the page-dir */
#define SET_PAGE_DIR(__tsk, __pgdir)			\
do {	if(__tsk == current) {				\
		__asm__ __volatile__("mtc0	%0, $28"\
				     : /* no outputs */	\
				     : "r" (__pgdir));	\
	}						\
} while(0)

extern pmd_t invalid_pte_table[PAGE_SIZE/sizeof(pmd_t)];
#define PMD_MAGIC		0x40

/*
 * Conversion functions: convert a page and protection to a page entry,
 * and a page entry and page directory to the page they refer to.
 */
extern inline unsigned long pte_page(pte_t pte)
{
	return PAGE_OFFSET + (pte_val(pte) & PAGE_MASK);
}

extern inline unsigned long pmd_page(pmd_t pmd)
{
	return pmd_val(pmd);
}

extern inline void pmd_set(pmd_t * pmdp, pte_t * ptep)
{
	pmd_val(*pmdp) = ((unsigned long) ptep);
}

extern inline int pte_none(pte_t pte)    { return !pte_val(pte); }
extern inline int pte_present(pte_t pte) { return pte_val(pte) & _PAGE_PRESENT; }

/* Certain architectures need to do special things when pte's
 * within a page table are directly modified.  Thus, the following
 * hook is made available.
 */
#define set_pte(ptep, pteval) (*(ptep) = (pteval))

extern inline void pte_clear(pte_t *ptep)
{
	set_pte(ptep, __pte(0));
}

/*
 * Empty pgd/pmd entries point to the invalid_pte_table.
 */
extern inline int pmd_none(pmd_t pmd)
{
	return (pmd_val(pmd) & PMD_MAGIC);
}

extern inline int pmd_bad(pmd_t pmd)
{
	return 0;
}

extern inline void pmd_clear(pmd_t *pmdp)
{
	pmd_val(*pmdp) = ((unsigned long) invalid_pte_table);
}

/*
 * The "pgd_xxx()" functions here are trivial for a folded two-level
 * setup: the pgd is never bad, and a pmd always exists (as it's folded
 * into the pgd entry)
 */
extern inline int pgd_none(pgd_t pgd)		{ return 0; }
extern inline int pgd_bad(pgd_t pgd)		{ return 0; }
extern inline int pgd_present(pgd_t pgd)	{ return 1; }
extern inline void pgd_clear(pgd_t *pgdp)	{ }

/*
 * The following only work if pte_present() is true.
 * Undefined behaviour if not..
 */
extern inline int pte_read(pte_t pte)	{ return pte_val(pte) & _PAGE_READ; }
extern inline int pte_write(pte_t pte)	{ return pte_val(pte) & _PAGE_WRITE; }
extern inline int pte_dirty(pte_t pte)	{ return pte_val(pte) & _PAGE_MODIFIED; }
extern inline int pte_young(pte_t pte)	{ return pte_val(pte) & _PAGE_ACCESSED; }

extern inline pte_t pte_wrprotect(pte_t pte)
{
	pte_val(pte) &= ~(_PAGE_WRITE | _PAGE_SILENT_WRITE);
	return pte;
}

extern inline pte_t pte_rdprotect(pte_t pte)
{
	pte_val(pte) &= ~(_PAGE_READ | _PAGE_SILENT_READ);
	return pte;
}

extern inline pte_t pte_mkclean(pte_t pte)
{
	pte_val(pte) &= ~(_PAGE_MODIFIED|_PAGE_SILENT_WRITE);
	return pte;
}

extern inline pte_t pte_mkold(pte_t pte)
{
	pte_val(pte) &= ~(_PAGE_ACCESSED|_PAGE_SILENT_READ);
	return pte;
}

extern inline pte_t pte_mkwrite(pte_t pte)
{
	pte_val(pte) |= _PAGE_WRITE;
	if (pte_val(pte) & _PAGE_MODIFIED)
		pte_val(pte) |= _PAGE_SILENT_WRITE;
	return pte;
}

extern inline pte_t pte_mkread(pte_t pte)
{
	pte_val(pte) |= _PAGE_READ;
	if (pte_val(pte) & _PAGE_ACCESSED)
		pte_val(pte) |= _PAGE_SILENT_READ;
	return pte;
}

extern inline pte_t pte_mkdirty(pte_t pte)
{
	pte_val(pte) |= _PAGE_MODIFIED;
	if (pte_val(pte) & _PAGE_WRITE)
		pte_val(pte) |= _PAGE_SILENT_WRITE;
	return pte;
}

extern inline pte_t pte_mkyoung(pte_t pte)
{
	pte_val(pte) |= _PAGE_ACCESSED;
	if (pte_val(pte) & _PAGE_READ)
		pte_val(pte) |= _PAGE_SILENT_READ;
	return pte;
}

/*
 * Conversion functions: convert a page and protection to a page entry,
 * and a page entry and page directory to the page they refer to.
 */
extern inline pte_t mk_pte(unsigned long page, pgprot_t pgprot)
{
	return __pte(((page & PAGE_MASK) - PAGE_OFFSET) | pgprot_val(pgprot));
}

extern inline pte_t mk_pte_phys(unsigned long physpage, pgprot_t pgprot)
{
	return __pte((physpage - PAGE_OFFSET) | pgprot_val(pgprot));
}

extern inline pte_t pte_modify(pte_t pte, pgprot_t newprot)
{
	return __pte((pte_val(pte) & _PAGE_CHG_MASK) | pgprot_val(newprot));
}

/* to find an entry in a page-table-directory */
extern inline pgd_t *pgd_offset(struct mm_struct *mm, unsigned long address)
{
	return mm->pgd + (address >> PGDIR_SHIFT);
}

/* Find an entry in the second-level page table.. */
extern inline pmd_t *pmd_offset(pgd_t *dir, unsigned long address)
{
	return (pmd_t *) dir;
}

/* Find an entry in the third-level page table.. */ 
extern inline pte_t *pte_offset(pmd_t * dir, unsigned long address)
{
	return (pte_t *) (pmd_page(*dir)) +
	       ((address >> PAGE_SHIFT) & (PTRS_PER_PTE - 1));
}

/*
 * Allocate and free page tables. The xxx_kernel() versions are
 * used to allocate a kernel page table - this turns on ASN bits
 * if any.
 */

extern unsigned long *pgd_quicklist;
extern unsigned long *pte_quicklist;
extern unsigned long pgtable_cache_size;

extern pgd_t *get_pgd_slow(void);
extern __inline__ pgd_t *get_pgd_fast(void)
{
	pgd_t *ret;

	if((ret = (pgd_t *)pgd_quicklist) != NULL) {
		pgd_quicklist = (unsigned long *)pgd_val(*ret);
		pgd_val(ret[0]) = pgd_val(ret[1]);
		(pgtable_cache_size)--;
	} else
		ret = get_pgd_slow();
	return ret;
}

extern __inline__ void free_pgd_fast(pgd_t *pgd)
{
	pgd_val(*pgd) = (unsigned long) pgd_quicklist;
	pgd_quicklist = (unsigned long *) pgd;
	(pgtable_cache_size)++;
}

extern pte_t *get_pte_slow(pmd_t *pmd, unsigned long address_preadjusted);
extern __inline__ pte_t *get_pte_fast(void)
{
	pte_t *ret;

	if((ret = (pte_t *)pte_quicklist) != NULL) {
		pte_quicklist = (unsigned long *)pte_val(*ret);
		pte_val(ret[0]) = pte_val(ret[1]);
		(pgtable_cache_size)--;
	}
	return ret;
}

extern __inline__ void free_pte_fast(pte_t *pte)
{
	pte_val(*pte) = (unsigned long) pte_quicklist;
	pte_quicklist = (unsigned long *) pte;
	(pgtable_cache_size)++;
}

#define pte_free_kernel(pte)	free_pte_fast(pte)
#define pte_free(pte)		free_pte_fast(pte)
#define pgd_free(pgd)		free_pgd_fast(pgd)
#define pgd_alloc()		get_pgd_fast()

extern inline void pmd_free_kernel(pmd_t *pmd)
{
	pmd_val(*pmd) = ((unsigned long) invalid_pte_table);
}

extern inline pmd_t *pmd_alloc_kernel(pgd_t *pgd, unsigned long address)
{
	return (pmd_t *) pgd;
}

extern inline pte_t *pte_alloc(pmd_t *pmd, unsigned long address)
{
	address = (address >> PAGE_SHIFT) & (PTRS_PER_PTE - 1);
	if (pmd_none(*pmd)) {
		pte_t *page = get_pte_fast();

		if(!page)
			return get_pte_slow(pmd, address);
		pmd_set(pmd, page);
		return page + address;
	}
	return (pte_t *) pmd_page(*pmd) + address;
}

#define pte_alloc_kernel(pmd, addr)	pte_alloc(pmd, addr)

/*
 * allocating and freeing a pmd is trivial: the 1-entry pmd is
 * inside the pgd, so has no extra memory associated with it.
 */
extern inline void pmd_free(pmd_t *pmd)
{
	pmd_val(*pmd) = ((unsigned long) invalid_pte_table);
}

extern inline pmd_t *pmd_alloc(pgd_t *pgd, unsigned long address)
{
	return (pmd_t *) pgd;
}

/*
 * Initialize new page directory with pointers to invalid ptes
 */
extern void pgd_init(unsigned long page);

extern pgd_t swapper_pg_dir[1024];

/*
 * Kernel with 32 bit address space
 */
#define SWP_TYPE(entry) (((entry) >> 8) & 0x7f)
#define SWP_OFFSET(entry) ((entry) >> 15)
#define SWP_ENTRY(type,offset) (((type) << 8) | ((offset) << 15))

/* TLB operations. */
extern inline void tlb_probe(void)
{
	__asm__ __volatile__(
		".set noreorder\n\t"
		"tlbp\n\t"
		".set reorder");
}

extern inline void tlb_read(void)
{
	__asm__ __volatile__(
		".set noreorder\n\t"
		"tlbr\n\t"
		".set reorder");
}

extern inline void tlb_write_indexed(void)
{
	__asm__ __volatile__(
		".set noreorder\n\t"
		"tlbwi\n\t"
		".set reorder");
}

extern inline void tlb_write_random(void)
{
	__asm__ __volatile__(
		".set noreorder\n\t"
		"tlbwr\n\t"
		".set reorder");
}

/* Dealing with various CP0 mmu/cache related registers. */

/* CP0_PAGEMASK register */
extern inline unsigned long get_pagemask(void)
{
	unsigned long val;

	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mfc0 %0, $5\n\t"
		".set mips0\n\t"
		".set reorder"
		: "=r" (val));
	return val;
}

extern inline void set_pagemask(unsigned long val)
{
	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mtc0 %0, $5\n\t"
		".set mips0\n\t"
		".set reorder"
		: : "r" (val));
}

/* CP0_ENTRYLO0 and CP0_ENTRYLO1 registers */
extern inline unsigned long get_entrylo0(void)
{
	unsigned long val;

	__asm__ __volatile__(	
		".set noreorder\n\t"
		".set mips3\n\t"
		"mfc0 %0, $2\n\t"
		".set mips0\n\t"
		".set reorder"
		: "=r" (val));
	return val;
}

extern inline void set_entrylo0(unsigned long val)
{
	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mtc0 %0, $2\n\t"
		".set mips0\n\t"
		".set reorder"
		: : "r" (val));
}

extern inline unsigned long get_entrylo1(void)
{
	unsigned long val;

	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mfc0 %0, $3\n\t"
		".set mips0\n\t"
		".set reorder" : "=r" (val));

	return val;
}

extern inline void set_entrylo1(unsigned long val)
{
	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mtc0 %0, $3\n\t"
		".set mips0\n\t"
		".set reorder"
		: : "r" (val));
}

/* CP0_ENTRYHI register */
extern inline unsigned long get_entryhi(void)
{
	unsigned long val;

	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mfc0 %0, $10\n\t"
		".set mips0\n\t"
		".set reorder"
		: "=r" (val));

	return val;
}

extern inline void set_entryhi(unsigned long val)
{
	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mtc0 %0, $10\n\t"
		".set mips0\n\t"
		".set reorder"
		: : "r" (val));
}

/* CP0_INDEX register */
extern inline unsigned long get_index(void)
{
	unsigned long val;

	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mfc0 %0, $0\n\t"
		".set mips0\n\t"
		".set reorder"
		: "=r" (val));
	return val;
}

extern inline void set_index(unsigned long val)
{
	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mtc0 %0, $0\n\t"
		".set mips0\n\t"
		".set reorder\n\t"
		: : "r" (val));
}

/* CP0_WIRED register */
extern inline unsigned long get_wired(void)
{
	unsigned long val;

	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mfc0 %0, $6\n\t"
		".set mips0\n\t"
		".set reorder\n\t"
		: "=r" (val));
	return val;
}

extern inline void set_wired(unsigned long val)
{
	__asm__ __volatile__(
		"\n\t.set noreorder\n\t"
		".set mips3\n\t"
		"mtc0 %0, $6\n\t"
		".set mips0\n\t"
		".set reorder"
		: : "r" (val));
}

/* CP0_TAGLO and CP0_TAGHI registers */
extern inline unsigned long get_taglo(void)
{
	unsigned long val;

	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mfc0 %0, $28\n\t"
		".set mips0\n\t"
		".set reorder"
		: "=r" (val));
	return val;
}

extern inline void set_taglo(unsigned long val)
{
	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mtc0 %0, $28\n\t"
		".set mips0\n\t"
		".set reorder"
		: : "r" (val));
}

extern inline unsigned long get_taghi(void)
{
	unsigned long val;

	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mfc0 %0, $29\n\t"
		".set mips0\n\t"
		".set reorder"
		: "=r" (val));
	return val;
}

extern inline void set_taghi(unsigned long val)
{
	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mtc0 %0, $29\n\t"
		".set mips0\n\t"
		".set reorder"
		: : "r" (val));
}

/* CP0_CONTEXT register */
extern inline unsigned long get_context(void)
{
	unsigned long val;

	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mfc0 %0, $4\n\t"
		".set mips0\n\t"
		".set reorder"
		: "=r" (val));

	return val;
}

extern inline void set_context(unsigned long val)
{
	__asm__ __volatile__(
		".set noreorder\n\t"
		".set mips3\n\t"
		"mtc0 %0, $4\n\t"
		".set mips0\n\t"
		".set reorder"
		: : "r" (val));
}

extern __inline__ void update_mmu_cache(struct vm_area_struct *vma,
					unsigned long address,
					pte_t *pte)
{
	unsigned long flags;
	int idx, pid;

	pte = (pte_t *) (((unsigned long)pte) & ~(0x4UL));

	pid = (get_entryhi() & 0xff);
	save_and_cli(flags);
	address &= (PAGE_MASK << 1);
	set_entryhi(address | pid);
	__asm__ __volatile__("nop; nop; nop;");
	tlb_probe();
	idx = get_index();
	set_entrylo0(pte_val(*pte++) >> 6);
	set_entrylo1(pte_val(*pte) >> 6);
	set_entryhi(address | pid);
	if(idx < 0)
		tlb_write_random();
	else
		tlb_write_indexed();
	__asm__ __volatile__("nop; nop; nop;");
	set_entryhi(pid);
	restore_flags(flags);
}

/* Kernel stack allocation/freeing. */
extern inline unsigned long alloc_kernel_stack(void)
{
	return __get_free_page(GFP_KERNEL);
}

extern inline void free_kernel_stack(unsigned long stack)
{
	free_page(stack);
}

#endif /* !defined (__LANGUAGE_ASSEMBLY__) */

#endif /* __ASM_MIPS_PGTABLE_H */
