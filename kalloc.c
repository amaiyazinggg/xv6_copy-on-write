// Physical memory allocator, intended to allocate
// memory for user processes, kernel stacks, page table pages,
// and pipe buffers. Allocates 4096-byte pages.

#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "spinlock.h"

void freerange(void *vstart, void *vend);
extern char end[]; // first address after kernel loaded from ELF file
                   // defined by the kernel linker script in kernel.ld

struct run
{
  struct run *next;
};

struct
{
  struct spinlock lock;
  int use_lock;
  uint num_free_pages; // store number of free pages
  struct run *freelist;
  int rmap[PHYSTOP >> PTXSHIFT];
  pte_t *shared_ptes[PHYSTOP >> PTXSHIFT][NPROC];
} kmem;

// Initialization happens in two phases.
// 1. main() calls kinit1() while still using entrypgdir to place just
// the pages mapped by entrypgdir on free list.
// 2. main() calls kinit2() with the rest of the physical pages
// after installing a full page table that maps them on all cores.
void kinit1(void *vstart, void *vend)
{
  initlock(&kmem.lock, "kmem");
  kmem.use_lock = 0;
  kmem.num_free_pages = 0;
  freerange(vstart, vend);
}

void kinit2(void *vstart, void *vend)
{
  freerange(vstart, vend);
  kmem.use_lock = 1;
}

void set_rmap_value_no_lock(uint pa, int value)
{
  if (pa >= PHYSTOP || pa < (uint)V2P(end))
    panic("rmap_value_no_lock");
  kmem.rmap[pa >> PTXSHIFT] = value;
  return;
}

void freerange(void *vstart, void *vend)
{
  char *p;
  p = (char *)PGROUNDUP((uint)vstart);
  for (; p + PGSIZE <= (char *)vend; p += PGSIZE)
  {
    // initialising rmap with value 0
    set_rmap_value_no_lock(V2P(p), 0);
    kfree(p);
  }
}

// PAGEBREAK: 21
//  Free the page of physical memory pointed at by v,
//  which normally should have been returned by a
//  call to kalloc().  (The exception is when
//  initializing the allocator; see kinit above.)
void kfree(char *v)
{
  struct run *r;

  if ((uint)v % PGSIZE || v < end || V2P(v) >= PHYSTOP)
    panic("kfree");

  // Fill with junk to catch dangling refs.
  // memset(v, 1, PGSIZE);

  if (kmem.use_lock)
    acquire(&kmem.lock);

  r = (struct run *)v;

  if (kmem.rmap[V2P(v) >> PTXSHIFT] > 0)
    kmem.rmap[V2P(v) >> PTXSHIFT] -= 1;

  if (kmem.rmap[V2P(v) >> PTXSHIFT] == 0)
  {
    // Fill with junk to catch dangling refs.
    memset(v, 1, PGSIZE);
    r->next = kmem.freelist;
    kmem.num_free_pages += 1;
    kmem.freelist = r;
  }

  if (kmem.use_lock)
    release(&kmem.lock);
}

pte_t *get_memshared_pte(uint pa, int i)
{
  if (pa >= PHYSTOP || pa < (uint)V2P(end))
    panic("get_memshared_pte");

  acquire(&kmem.lock);
  pte_t *pte = kmem.shared_ptes[pa >> PTXSHIFT][i];
  release(&kmem.lock);

  return pte;
}

uint get_rmap_value(uint pa)
{
  if (pa >= PHYSTOP || pa < (uint)V2P(end))
    panic("rmap_value");
  uint rfc;
  acquire(&kmem.lock);
  rfc = kmem.rmap[pa >> PTXSHIFT];
  release(&kmem.lock);
  return rfc;
}

void set_rmap_value(uint pa, int value)
{
  if (pa >= PHYSTOP || pa < (uint)V2P(end))
    panic("rmap_value");
  acquire(&kmem.lock);
  kmem.rmap[pa >> PTXSHIFT] = value;
  release(&kmem.lock);
  return;
}

void inc_rmap_value(uint pa)
{
  if (pa >= PHYSTOP || pa < (uint)V2P(end))
    panic("inc_rmap_value");

  acquire(&kmem.lock);
  kmem.rmap[pa >> PTXSHIFT] = kmem.rmap[pa >> PTXSHIFT] + 1;
  release(&kmem.lock);
  return;
}

void add_memshared_pte(uint pa, int index, pte_t *pte)
{
  if (pa >= PHYSTOP || pa < (uint)V2P(end))
    panic("dec_rmap_value");

  acquire(&kmem.lock);
  kmem.shared_ptes[pa >> PTXSHIFT][index] = pte;
  release(&kmem.lock);
  return;
}

pte_t *set_pte_mem(pte_t *pte, int index, uint pa)
{
  acquire(&kmem.lock);
  pte_t *newpte = kmem.shared_ptes[pa >> PTXSHIFT][index];
  release(&kmem.lock);
  if (newpte == 0)
    return newpte;
  *newpte = *pte;
  return newpte;
}

void set_all_kmem_zero(uint pa)
{
  for (int i = 0; i < NPROC; i++)
    kmem.shared_ptes[pa >> 12][i] = 0;
  return;
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
char *
kalloc(void)
{
  struct run *r;

  // locking kmem so 2 processes never enter at once
  if (kmem.use_lock)
    acquire(&kmem.lock);
  r = kmem.freelist;
  if (r)
  {
    kmem.freelist = r->next;
    uint pa = V2P((char *)r);
    set_rmap_value_no_lock(pa, 1);
    kmem.num_free_pages -= 1;
    set_all_kmem_zero(pa);
  }

  if (kmem.use_lock)
    release(&kmem.lock);

  if (r)
    return (char *)r;

  swap_out();
  return kalloc();
}

void mem_remove_entry(uint pa, pte_t *pte)
{
  acquire(&kmem.lock);
  pte_t** i;
  pte_t** shared_ptes_pa = kmem.shared_ptes[pa >> PTXSHIFT];
  for (i = shared_ptes_pa; i < kmem.shared_ptes[pa >> PTXSHIFT] + NPROC; i++)
    if (*i == pte)
      *i = 0;
  *pte = 0;
  release(&kmem.lock);
}

void add_pte_mem(uint pa, pte_t *pte)
{
  int found = 0;
  pte_t **i;
  pte_t **kmem_pa = kmem.shared_ptes[pa >> PTXSHIFT];
  pte_t **found_entry;
  for (i = kmem_pa; i < kmem_pa + NPROC; i++)
  {
      if (!found)
          if (*i == 0)
          {
              found = 1;
              found_entry = i;
          }
      if (*i == pte)
          return;
  }
  if (found) *found_entry = pte;
  return;
}

uint num_of_FreePages(void)
{
  acquire(&kmem.lock);

  uint num_free_pages = kmem.num_free_pages;

  release(&kmem.lock);

  return num_free_pages;
}
