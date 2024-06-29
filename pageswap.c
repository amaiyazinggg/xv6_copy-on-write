#include "types.h"
#include "param.h"
#include "defs.h"
#include "mmu.h"
#include "memlayout.h"
#include "x86.h"
#include "traps.h"
#include "proc.h"
#include "fs.h"
#include "spinlock.h"

struct swap_slot
{
    struct spinlock lock;
    int page_perm; // stores permission of a swapped memory page
    int free_int;  // denotes availability of swap slot
    pte_t *shared_ptes[NPROC];
    int refcount;
};

struct spinlock swaplock;

struct swap_slot swapblock[SWAPBLOCKS / 8];

void swapinit()
{
    initlock(&swaplock, "global lock");
    for (int i = 0; i < SWAPBLOCKS / 8; i++)
    {
        swapblock[i].free_int = 1;
        swapblock[i].page_perm = 0;
        swapblock[i].refcount = 0;
        initlock(&swapblock[i].lock, "swapblock");
    }
}

static pte_t *
walkpgdir(pde_t *pgdir, const void *va, int alloc)
{
    pde_t *pde;
    pte_t *pgtab;

    pde = &pgdir[PDX(va)];
    if (*pde & PTE_P)
    {
        pgtab = (pte_t *)P2V(PTE_ADDR(*pde));
    }
    else
    {
        if (!alloc || (pgtab = (pte_t *)kalloc()) == 0)
            return 0;
        // Make sure all those PTE_P bits are zero.
        memset(pgtab, 0, PGSIZE);
        // The permissions here are overly generous, but they can
        // be further restricted by the permissions in the page table
        // entries, if necessary.
        *pde = V2P(pgtab) | PTE_P | PTE_W | PTE_U;
    }
    return &pgtab[PTX(va)];
}

pte_t *get_page(struct proc *victim_proc)
{
    pte_t *victim_page = get_victim_page(victim_proc);

    // victim page is 0 if not found
    if (!victim_page)
    {
        clear_proc_access(victim_proc);
        victim_page = get_victim_page(victim_proc);
    }

    if (!victim_page)
    {
        panic("SWAP OUT NOTHING FOUND");
    }

    return victim_page;
}

void swap_out()
{
    struct proc *victim_proc = get_victim_proc();
    victim_proc->rss -= 4096;
    // cprintf("GOT VICTIM PROC %d \n", victim_proc->pid);
    pte_t *victim_page = get_page(victim_proc);
    // cprintf("INSIDE SWAP OUT : GOT PAGE %x\n", *victim_page);

    acquire(&swaplock);

    int i;
    for (i = 0; i < SWAPBLOCKS / 8; i++)
    {
        if (swapblock[i].free_int)
        {
            break;
        }
    }

    if (i == SWAPBLOCKS / 8)
    {
        release(&swaplock);
        panic("NO FREE SLOTS FOUND");
    }

    int blockno = i;

    swapblock[blockno].free_int = 0;
    swapblock[blockno].page_perm = PTE_FLAGS(*victim_page);
    uint p_addr = PTE_ADDR(*victim_page);

    release(&swaplock);

    page_to_disk(ROOTDEV, (char *)P2V(p_addr), 2 + 8 * blockno);

    *victim_page = (2 + 8 * blockno) << PTXSHIFT;
    *victim_page = *victim_page & ~PTE_P;
    *victim_page = *victim_page | 0x008;

    swapblock[blockno].refcount = get_rmap_value(p_addr);
    pte_t *stored_ptes[NPROC];

    int j = 0;
    while (j < NPROC)
    {
        pte_t *newpte = set_pte_mem(victim_page, j, p_addr);
        stored_ptes[j] = newpte;
        j++;
    }

    j = 0;
    while (j < NPROC)
    {
        swapblock[blockno].shared_ptes[j] = stored_ptes[j];
        j++;
    }

    set_rmap_value(p_addr, 0);

    kfree((char *)P2V(p_addr));
    return;
}

void handle_page_write_off()
{
    uint cr2 = rcr2();
    uint pa, flags;
    pte_t *pte = walkpgdir(myproc()->pgdir, (void *)cr2, 0);
    // now I have the page table entry

    if (*pte & PTE_P)
    {
        if (PTE_W & *pte)
        {
            panic("Inside PGFLT: Write Bit is already ON");
        }
        // the case where write bit was off

        pa = PTE_ADDR(*pte);
        flags = PTE_FLAGS(*pte);

        uint rfc;
        rfc = get_rmap_value(pa);

        if (rfc < 1)
        {
            panic("Incorrect Reference Count");
            return;
        }
        if (rfc == 1)
        {
            *pte = *pte | PTE_W;
            lcr3(V2P(myproc()->pgdir));
            return;
        }
        char *mem = kalloc();
        if (mem == 0)
            panic("HANDLE PAGE WRITE OFF MEM = 0");

        memmove(mem, (char *)P2V(pa), PGSIZE);
        mem_remove_entry(pa, pte);
        kfree(P2V(pa));
        *pte = V2P(mem) | flags | PTE_W;
        add_pte_mem(V2P(mem), pte);
        lcr3(V2P(myproc()->pgdir));
        // cprintf("Exiting Page Write Off\n");
        return;
    }
    else
    {
        // cprintf("pgflt not present\n");

        if (!(*pte & 0x008))
            panic("pgflt no swap out bit");

        // the case where the page is in disk
        myproc()->rss += 4096;
        uint blockid = PTE_ADDR(*pte) >> PTXSHIFT;

        char *mem = kalloc();
        disk_to_page(ROOTDEV, mem, blockid);

        uint pa = V2P(mem);
        int blockno = (blockid - 2) / 8;
        *pte = PTE_ADDR(pa) | swapblock[blockno].page_perm | PTE_P;
        *pte &= ~0x008;

        set_rmap_value(pa, swapblock[blockno].refcount);
        pte_t **i;
        pte_t **swapblock_blockno = swapblock[blockno].shared_ptes;
        int counter = 0;
        for (i = swapblock_blockno; i < swapblock_blockno + NPROC; i++)
        {
            if (*i != 0)
            {
                **i = *pte;
                add_memshared_pte(pa, counter, *i);
            }
            counter++;
        }
        swapblock[blockno].free_int = 1;

        // cprintf("out pgflt not present");
        return;
    }
}

void set_block_non_writeable(int blockid)
{
    int blockno = (blockid - 2) / 8;
    swapblock[blockno].page_perm = swapblock[blockno].page_perm & ~PTE_W;
}

void diskremove_entry(pte_t *pte)
{
    uint blockid = *pte >> PTXSHIFT;
    int blockno = (blockid - 2) / 8;
    pte_t **i;
    pte_t **swapblock_blockno = swapblock[blockno].shared_ptes;
    for (i = swapblock_blockno; i < swapblock_blockno + NPROC; i++)
        if (*i == pte)
            *i = 0;
    *pte = 0;
}

void inc_block_refcount(int blockid)
{
    int blockno = (blockid - 2) / 8;
    swapblock[blockno].refcount = swapblock[blockno].refcount + 1;
    set_block_non_writeable(blockno);
}

void add_pte_block(int blockid, pte_t *pte)
{
    int blockno = (blockid - 2) / 8;
    int found = 0;
    pte_t **i;
    pte_t **swapblock_blockno = swapblock[blockno].shared_ptes;
    pte_t **found_entry;
    for (i = swapblock_blockno; i < swapblock_blockno + NPROC; i++)
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
    if (found)
        *found_entry = pte;
    return;
}

void dec_block_refcount(int blockid)
{
    int blockno = (blockid - 2) / 8;
    swapblock[blockno].refcount -= 1;
}

void block_free(int blockno)
{
    blockno = (blockno - 2) / 8;
    swapblock[blockno].free_int = 1;
}

void set_block_free(int blockid)
{
    int blockno = (blockid - 2) / 8;
    if (swapblock[blockno].refcount == 0)
    {
        swapblock[blockno].free_int = 1;
    }
}