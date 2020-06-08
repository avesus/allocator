#include <allocator/allocator.h>

#define MM_DEFAULT_LENGTH    ((1UL) << 30)
#define MM_DEFAULT_BASE_ADDR 0

// -2 is because we are skipping PAGE_SIZE and PAGE_SIZE / 2
#define MM_N_SMALL_REGIONS       ((PAGE_SIZE / MM_ALIGNMENT) - 2)
#define MM_BOOK_KEEP_REGION_SIZE (PAGE_SIZE * PAGE_SIZE)

#define MM_MIN_SIZE     (16)
#define MM_LOG_MIN_SIZE (4)

#define MM_SR_IDX_TO_N_VECS(X) (PAGE_SIZE >> ((X) + MM_LOG_MIN_SIZE))
#define MM_N_L1_VECS           ((MM_DEFAULT_LENGTH / ((64) * PAGE_SIZE)) / 64)

#define MM_N_L2_VECS (MM_DEFAULT_LENGTH / ((64) * PAGE_SIZE))

#define MM_L1_VEC_FULL (~(0UL))
#define MM_L2_VEC_FULL (0UL)

typedef struct region_head {
    uint64_t l0_vec;
    uint64_t l1_vecs[MM_N_L1_VECS];
    // bit vec of free pages
    uint64_t l2_vecs[MM_N_L2_VECS];

    // amount of pages for each page start idx
    uint8_t page_start_sizes[MM_DEFAULT_LENGTH / PAGE_SIZE];

    // this is total of 254 * 2 * 4 bytes = 2032 bytes
    uint32_t ll_heads[MM_N_SMALL_REGIONS];
    uint32_t ll_tails[MM_N_SMALL_REGIONS];

} region_head_t;


typedef struct small_region {
    uint32_t region_size;
    uint32_t next_region_page;
    uint64_t avail_vecs[0];
} small_region_t;


static region_head_t * addr_start = NULL;

//////////////////////////////////////////////////////////////////////
// Initialization

static void
init() {
    addr_start =
        (region_head_t *)mymmap(MM_DEFAULT_BASE_ADDR,
                                MM_DEFAULT_LENGTH,
                                PROT_READ | PROT_WRITE,
                                MAP_ANONYMOUS | MAP_NORESERVE | MAP_PRIVATE,
                                (-1),
                                0);

    memset(addr_start->l2_vecs, ~(0), sizeof(uint64_t) * MM_N_L2_VECS);
    addr_start->l0_vec = ~(0UL);
}
//////////////////////////////////////////////////////////////////////
static uint64_t
try_place(uint64_t         l1_vec,
          uint64_t * const l2_vecs,
          const uint32_t   npages,
          const uint32_t   max_npage_bit) {

    uint64_t l2_idx, ret_idx;

    while (l1_vec) {
        ff1_asm_tz(l1_vec, l2_idx);
        PRINT(LOW_VERBOSE,
              "l1_iter -> %16lX -> %lu\n\t-> %16lX\n",
              l1_vec,
              l2_idx,
              l2_vecs[l2_idx]);
        l1_vec ^= ((1UL) << l2_idx);

        assert(l2_vecs[l2_idx] != MM_L2_VEC_FULL);

        PRINT(MED_VERBOSE,
              "Internal doing l2_idx (%lu) -> %16lX & %16lX >> %d\n",
              l2_idx,
              l2_vecs[l2_idx],
              l2_vecs[l2_idx],
              npages);
        uint64_t temp_l2_vec = l2_vecs[l2_idx] & (l2_vecs[l2_idx] >> npages);
        PRINT(MED_VERBOSE, "Page Shifted: %16lX\n", temp_l2_vec);

        uint32_t max_npage_bit_shift = max_npage_bit / 2;

        while (max_npage_bit_shift > 0 && temp_l2_vec) {
            PRINT(MED_VERBOSE,
                  "\tLoop %16lX &= %16lX >> %d = %16lX & %16lX = %16lX\n",
                  temp_l2_vec,
                  temp_l2_vec,
                  max_npage_bit_shift,
                  temp_l2_vec,
                  temp_l2_vec >> max_npage_bit_shift,
                  temp_l2_vec & (temp_l2_vec >> max_npage_bit_shift));
            temp_l2_vec &= (temp_l2_vec >> max_npage_bit_shift);
            max_npage_bit_shift /= 2;
        }
        if (temp_l2_vec) {
            ff1_asm_tz(temp_l2_vec, ret_idx);
            return (l2_idx << 32) | ret_idx;
        }
    }
    return ~(0UL);
}


static uint64_t
find_contig_region(const uint32_t npages, const uint32_t max_npage_bit) {

    uint64_t         l0_vec  = addr_start->l0_vec;
    uint64_t * const l1_vecs = addr_start->l1_vecs;
    uint64_t * const l2_vecs = addr_start->l2_vecs;

    const uint64_t erase_mask =
        ((1UL) << (npages | ((1UL) << max_npage_bit))) - 1;

#ifdef SEGLISTS
    const uint32_t offset = npages | ((1UL) << max_npage_bit);
    l0_vec                = (l0_vec >> offset) | (l0_vec << (64 - offset));
#endif

    uint64_t i;
    while (l0_vec) {
        ff1_asm_tz(l0_vec, i);
        l0_vec ^= ((1UL) << i);
#ifdef SEGLISTS
        i = (i + offset) & 63;
#endif
        PRINT(LOW_VERBOSE, "l1_vec(%lu) -> %16lX\n", i, l1_vecs[i]);
        if (l1_vecs[i] != MM_L1_VEC_FULL) {
            PRINT(LOW_VERBOSE, "\tTrying to Place l2_vec(%lu)\n", i);
            const uint64_t ret = try_place(~l1_vecs[i],
                                           l2_vecs + (64 * i),
                                           npages,
                                           max_npage_bit);
            if (ret != (~(0UL))) {
                const uint32_t l1_idx = ret >> 32;
                const uint32_t l2_idx = ret;
                PRINT(LOW_VERBOSE,
                      "Got Ret(%d, %d) -> %16lX\n",
                      l1_idx,
                      l2_idx,
                      l2_vecs[64 * i + l1_idx]);
                PRINT(LOW_VERBOSE,
                      "Erasing %16lX ^ %16lX ->\n",
                      (erase_mask << l2_idx),
                      l2_vecs[64 * i + l1_idx]);
                assert((l2_vecs[64 * i + l1_idx] ^ (erase_mask << l2_idx)) ==
                       (l2_vecs[64 * i + l1_idx] & (~(erase_mask << l2_idx))));
                l2_vecs[64 * i + l1_idx] ^= (erase_mask << l2_idx);
                PRINT(LOW_VERBOSE, "\t%16lX\n", l2_vecs[64 * i + l1_idx]);
                addr_start
                    ->page_start_sizes[64 * 64 * i + 64 * l1_idx + l2_idx] =
                    npages | (1 << max_npage_bit);
                if (!l2_vecs[64 * i + l1_idx]) {
                    l1_vecs[i] ^= ((1UL) << l1_idx);
                    if (l1_vecs[i] == MM_L1_VEC_FULL) {
                        addr_start->l0_vec ^= ((1UL) << i);
                    }
                }

                return ((uint64_t)(addr_start + 1)) +
                       PAGE_SIZE * (64 * i + l1_idx);
            }
        }
    }
    DBG_ASSERT(0, "Out Of Memory!")
}
void
dealloc(void * addr) {
    return;
    if (!addr) {
        return;
    }
    const uint64_t page_start = (uint64_t)(addr_start + 1);
    const uint64_t v_idx      = (((uint64_t)addr) - page_start) / PAGE_SIZE;

    const uint32_t npages               = addr_start->page_start_sizes[v_idx];
    addr_start->page_start_sizes[v_idx] = 0;
    const uint64_t erase                = (((1UL) << npages) - 1) << v_idx;

    const uint32_t l0_idx = (v_idx >> 6) & (64 - 1);
    const uint32_t l1_idx = (v_idx >> 6) & (64 - 1);
    const uint32_t l2_idx = v_idx & (64 - 1);


    assert(npages + l2_idx < 64);
    if (!addr_start->l2_vecs[64 * 64 * l0_idx + 64 * l1_idx]) {
        if (!addr_start->l1_vecs[l0_idx]) {
            addr_start->l0_vec ^= ((1UL) << l0_idx);
        }
        addr_start->l1_vecs[l0_idx] ^= ((1UL) << l1_idx);
    }
    addr_start->l2_vecs[64 * l0_idx + l1_idx] ^= (erase << l2_idx);
}

void *
alloc(const uint64_t size) {
    if (addr_start == NULL) {
        init();
    }
    if (!size) {
        return NULL;
    }
    if (size > (64 * PAGE_SIZE)) {
        assert(0);
    }

    uint32_t npages = (size + PAGE_SIZE - 1) / PAGE_SIZE;
    uint32_t max_npage_bit;
    fl1_asm_lz(npages, max_npage_bit);
    max_npage_bit = 31 - max_npage_bit;
    npages ^= (1 << max_npage_bit);
    PRINT(LOW_VERBOSE, "size(%lu) -> %d | %d\n", size, npages, max_npage_bit);
    return (void *)find_contig_region(npages, max_npage_bit);
}
