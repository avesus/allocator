#include <allocator/allocator.h>

#define U64_MAX (~(0UL))

typedef __int128          int128_t;
typedef unsigned __int128 uint128_t;

typedef union cast128 {
    uint128_t u128;
    uint64_t  u64[2];
    uint32_t  u32[4];
} cast128_t;

typedef cast128_t word_t;
#define WBITS (128)


const uint128_t SHIFT_HELPER = 1;
#define MM_VEC_FULL 0


//#define MM_DEBUG
#define WORD_FORMAT   "0x%016lX %016lX"
#define WORD_PRINT(X) (X).u64[0], (X).u64[1]
#ifdef MM_DEBUG
#define MM_DBG_CHECK                    check_heap(__FILE__, __FUNCTION__, __LINE__)
#define MM_DBG_VASSERT(X, msg, args...) DBG_ASSERT(X, msg, ##args)
#define MM_DBG_ASSERT(X)                assert(X)
#else
#define MM_DBG_CHECK
#define MM_DBG_VASSERT(X, msg, args...)
#define MM_DBG_ASSERT(X)
#endif

const uint64_t MM_CONTINUE = (~(0UL));
const uint32_t MM_MIN_SIZE = 16;

const uint32_t MM_MED_REGION_N_VECS      = 4;
const uint32_t MM_MED_REGION_TOTAL_SLOTS = MM_MED_REGION_N_VECS * WBITS - 1;
const uint32_t MM_MED_REGION_CHUNK_SIZE  = 256;
const uint64_t MM_MED_REGION_SIZE =
    MM_MED_REGION_N_VECS * WBITS * MM_MED_REGION_CHUNK_SIZE;


const uint32_t MM_SMALL_REGION_BOUND = (128);
const uint32_t MM_MED_REGION_BOUND = (PAGE_SIZE - MM_MED_REGION_CHUNK_SIZE) - 1;

const uint32_t MM_N_SMALL_REGIONS = MM_SMALL_REGION_BOUND / MM_MIN_SIZE;

const uint32_t MM_N_L1_VECS = WBITS;
const uint32_t MM_N_L2_VECS = WBITS * WBITS;
const uint32_t MM_N_MED_REGION_VECS =
    (WBITS * WBITS) / (MM_MED_REGION_SIZE / PAGE_SIZE);


// for chunk size allignment in med_region struct
const uint32_t MED_REGION_PADDING_SIZE =
    MM_MED_REGION_CHUNK_SIZE - ((sizeof(word_t) * (2 * MM_MED_REGION_N_VECS) +
                                 sizeof(void *) * 2 + sizeof(uint64_t)) %
                                MM_MED_REGION_CHUNK_SIZE);


typedef struct small_region {
    //[0 - 7]   size
    //[8 - 31]  prev ptr
    //[32 - 64] next ptr
    uint64_t info;
    uint64_t vecs[1];
} small_region_t;

typedef struct med_region {
    word_t              vecs[MM_MED_REGION_N_VECS];
    word_t              size_vecs[MM_MED_REGION_N_VECS];
    struct med_region * next;
    struct med_region * prev;
    uint64_t            avail_bits;
    uint8_t             padding[MED_REGION_PADDING_SIZE];
} med_region_t;


typedef struct region_head {
    //////////////////////////////////////////////////////////////////////
    // memory starting here is initialized as 1s
    word_t l0_vec;                 // vec of free L1_VECS
    word_t l1_vecs[MM_N_L1_VECS];  // vec of free L2_VECS
    word_t l2_vecs[MM_N_L2_VECS];  // vec of free pages

    //////////////////////////////////////////////////////////////////////
    // memory after this point is initialized as 0s

    word_t l2_size_vecs[MM_N_L2_VECS];             // vec of allocation sizes
    word_t med_region_vecs[MM_N_MED_REGION_VECS];  // vec of medium regions


    med_region_t *   ll_mr_head;
    small_region_t * ll_sr_heads[MM_N_SMALL_REGIONS];
} region_head_t;

static uint64_t        heap_start = 0;
static region_head_t * heap       = NULL;


//////////////////////////////////////////////////////////////////////
// Global Helper Macros / Inline Functions
#define MM_IS_SMALL_REGION_ADDR(X) (((uint64_t)(X)) % PAGE_SIZE)

#define MM_NPI_MAX_BIT(X) ((X) >> 16)
#define MM_NPI_N_LOWER(X) ((X)&0xffff)
#define MM_NPI_N_TOTAL(X) (MM_NPI_MAX_BIT(X) | MM_NPI_N_LOWER(X))

#define MM_SMALL_ROUND(X)                                                      \
    ((((X) + MM_MIN_SIZE - 1) / MM_MIN_SIZE) * MM_MIN_SIZE)

#define MM_GET_NPAGES(X) (((X) + PAGE_SIZE - 1) / PAGE_SIZE)

//////////////////////////////////////////////////////////////////////
// Small Region Info Macros

#define MM_SR_GET_N_VECS(X) ((((PAGE_SIZE - 1) / (X)) / 64) + 1)
#define MM_SR_ADDR_TO_IDX(W, X, Y, Z)                                          \
    ((((uint64_t)(W)) -                                                        \
      (((uint64_t)(X)) + sizeof(uint64_t) * (((Y) | 0x1) + 1))) /              \
     (Z))

#define SIZE_SHIFT 0
#define PREV_SHIFT 8
#define NEXT_SHIFT 32

#define SIZE_MASK 0xff
#define PREV_MASK (0xffffffUL << 8)
#define NEXT_MASK (0xffffffffUL << 32)

#define MM_SR_GET_NEXT_BITS(X) (((X)&NEXT_MASK) >> NEXT_SHIFT)
#define MM_SR_GET_PREV_BITS(X) (((X)&PREV_MASK) >> PREV_SHIFT)

#define MM_SR_SET_NEXT_BITS(X, Y)                                              \
    ((X) = (((X) & (~NEXT_MASK)) | (Y) << NEXT_SHIFT))
#define MM_SR_SET_PREV_BITS(X, Y)                                              \
    ((X) = (((X) & (~PREV_MASK)) | (Y) << PREV_SHIFT))

#define MM_SR_GET_SIZE(X) (MM_MIN_SIZE * (((X)&SIZE_MASK) >> SIZE_SHIFT))
#define MM_SR_GET_NEXT(X)                                                      \
    ((small_region_t *)(MM_SR_GET_NEXT_BITS(X)                                 \
                            ? (PAGE_SIZE * MM_SR_GET_NEXT_BITS(X) +            \
                               ((uint64_t)heap))                               \
                            : 0))

#define MM_SR_GET_PREV(X)                                                      \
    ((small_region_t *)(MM_SR_GET_PREV_BITS(X)                                 \
                            ? (PAGE_SIZE * MM_SR_GET_PREV_BITS(X) +            \
                               ((uint64_t)heap))                               \
                            : 0))

#define MM_SR_SET_SIZE(X, Y)                                                   \
    ((X) = (((X) & (~SIZE_MASK)) | ((Y) / MM_MIN_SIZE) << SIZE_SHIFT))

#define MM_SR_SET_NEXT(X, Y)                                                   \
    MM_SR_SET_NEXT_BITS(                                                       \
        X,                                                                     \
        (Y) ? ((((uint64_t)(Y)) - ((uint64_t)heap)) / PAGE_SIZE) : 0)

#define MM_SR_SET_PREV(X, Y)                                                   \
    MM_SR_SET_PREV_BITS(                                                       \
        X,                                                                     \
        (Y) ? ((((uint64_t)(Y)) - ((uint64_t)heap)) / PAGE_SIZE) : 0)


#define MM_SMALL_VEC_LAST_AVAILS(X, Y)                                         \
    (((1UL) << (((PAGE_SIZE - sizeof(uint64_t) * ((Y) + 1)) / (X)) % 64)) - 1)


//////////////////////////////////////////////////////////////////////
// Inline Helper(s)


static inline uint32_t
get_size(const word_t l2_size_vec, const uint32_t l2_v_pos) {
    uint64_t size;
    if (l2_v_pos < 64) {
        ff0_asm_tz(l2_size_vec.u64[0] >> l2_v_pos, size);
        if (size == (64 - l2_v_pos)) {
            ff0_asm_tz(l2_size_vec.u64[1], size);
            size += (64 - l2_v_pos);
        }
    }
    else {
        ff0_asm_tz(l2_size_vec.u64[1] >> (l2_v_pos - 64), size);
    }
    return size + 1;
}

static uint32_t
dbg_get_addr_size(const uint64_t addr) {
    const uint64_t v_idx    = (((uint64_t)addr) - heap_start) / PAGE_SIZE;
    const uint32_t l0_idx   = (v_idx / (WBITS * WBITS)) & (WBITS - 1);
    const uint32_t l1_v_pos = (v_idx / WBITS) & (WBITS - 1);
    const uint32_t l2_v_pos = v_idx & (WBITS - 1);

    return get_size(heap->l2_size_vecs[WBITS * l0_idx + l1_v_pos], l2_v_pos);
}

static inline void
set_mr(const uint64_t mr) {
    const uint64_t mr_v_idx = (mr - heap_start) / MM_MED_REGION_SIZE;
    heap->med_region_vecs[mr_v_idx / WBITS].u128 |=
        (SHIFT_HELPER << (mr_v_idx % WBITS));
}

static inline void
unset_mr(const uint64_t mr) {
    const uint64_t mr_v_idx = (mr - heap_start) / MM_MED_REGION_SIZE;
    heap->med_region_vecs[mr_v_idx / WBITS].u128 ^=
        (SHIFT_HELPER << (mr_v_idx % WBITS));
}

static inline uint32_t
is_med_region(const uint64_t mr) {
    const uint64_t mr_v_idx = (mr - heap_start) / MM_MED_REGION_SIZE;
    return !!(heap->med_region_vecs[mr_v_idx / WBITS].u128 &
              (SHIFT_HELPER << (mr_v_idx % WBITS)));
}

static inline uint32_t
BITCOUNT(const word_t w) {
    return bitcount_64(w.u64[0]) + bitcount_64(w.u64[1]);
}

//////////////////////////////////////////////////////////////////////

const uint64_t MM_DEFAULT_LENGTH =
    (WBITS * WBITS * WBITS * PAGE_SIZE) + sizeof(region_head_t);


static void init_heap();

static uint64_t try_place(word_t               l1_vec,
                          const word_t * const l2_vecs,
                          const uint64_t       npage_info);
static uint64_t find_contig_region(const uint32_t npage_info);
static uint64_t find_contig_region_aligned(const uint32_t alignment,
                                           const uint32_t true_npage);

static void * alloc_small(const uint32_t rounded_size);
static void   dealloc_small(const uint64_t addr);
static void   remove_small_region(small_region_t ** const      head,
                                  const small_region_t * const sr);


static void
init_heap() {
    heap = (region_head_t *)mymmap(0,
                                   MM_DEFAULT_LENGTH,
                                   PROT_READ | PROT_WRITE,
                                   MAP_ANONYMOUS | MAP_NORESERVE | MAP_PRIVATE,
                                   (-1),
                                   0);
    
    memset(heap, (~0), sizeof(word_t) * (1 + MM_N_L1_VECS + MM_N_L2_VECS));
    heap_start =
        ((((uint64_t)(heap + 1))) + MM_MED_REGION_SIZE - 1) &
        (~(MM_MED_REGION_SIZE - 1));

    MM_DBG_CHECK;
}


static uint64_t
try_place(word_t               l1_vec,
          const word_t * const l2_vecs,
          const uint64_t       npage_info) {

    uint64_t l2_idx, ret;
    for (uint32_t i = 0; i < 2; i++) {
        while (l1_vec.u64[i]) {
            ff1_asm_tz(l1_vec.u64[i], l2_idx);
            l1_vec.u64[i] ^= (1UL) << l2_idx;

            // each u64 represents 128 total slots (i == 1 is 64 offset)
            l2_idx += 64 * i;

            word_t temp_l2_vec = { .u128 = l2_vecs[l2_idx].u128 &
                                           (l2_vecs[l2_idx].u128 >>
                                            MM_NPI_N_LOWER(npage_info)) };

            uint32_t max_npage_bit_shift = (MM_NPI_MAX_BIT(npage_info)) / 2;

            // group together a set of contiguous 1s that are big enough for req
            // number of pages
            while (max_npage_bit_shift > 0 && temp_l2_vec.u128) {
                temp_l2_vec.u128 &= (temp_l2_vec.u128 >> max_npage_bit_shift);
                max_npage_bit_shift /= 2;
            }
            if (temp_l2_vec.u64[0]) {
                ff1_asm_tz(temp_l2_vec.u64[0], ret);
                return (l2_idx << 32) | ret;
            }
            else if (temp_l2_vec.u64[1]) {
                ff1_asm_tz(temp_l2_vec.u64[1], ret);
                return (l2_idx << 32) | (ret + 64);
            }
        }
    }
    return MM_CONTINUE;
}

static void
remove_small_region(small_region_t ** const head, const uint64_t sr_info) {

    if (MM_SR_GET_NEXT(sr_info) == 0 && MM_SR_GET_PREV(sr_info) == 0) {

        (*head) = 0;
    }
    else if (MM_SR_GET_NEXT(sr_info) != 0 && MM_SR_GET_PREV(sr_info) == 0) {

        (*head) = MM_SR_GET_NEXT(sr_info);
        MM_SR_SET_PREV((*head)->info, 0);
    }
    else if (MM_SR_GET_NEXT(sr_info) == 0 && MM_SR_GET_PREV(sr_info) != 0) {

        MM_SR_SET_NEXT(MM_SR_GET_PREV(sr_info)->info, 0);
    }
    else {

        MM_SR_SET_NEXT(MM_SR_GET_PREV(sr_info)->info, MM_SR_GET_NEXT(sr_info));
        MM_SR_SET_PREV(MM_SR_GET_NEXT(sr_info)->info, MM_SR_GET_PREV(sr_info));
    }
}

static void
dealloc_small(const uint64_t addr) {

    MM_DBG_CHECK;
    small_region_t * const sr = (small_region_t *)(addr & (~(PAGE_SIZE - 1)));

    const uint32_t size        = MM_SR_GET_SIZE(sr->info);
    const uint32_t nvecs       = MM_SR_GET_N_VECS(size);
    const uint32_t list_idx    = (size / MM_MIN_SIZE) - 1;
    const uint64_t last_avails = MM_SMALL_VEC_LAST_AVAILS(size, nvecs | 0x1);

    const uint32_t vec_idx = MM_SR_ADDR_TO_IDX(addr, sr, nvecs, size);
    uint32_t       i;

    small_region_t ** const head = heap->ll_sr_heads + list_idx;

    sr->vecs[vec_idx / 64] ^= ((1UL) << (vec_idx % 64));

    for (i = 0;
         (i < nvecs) && ((i == (nvecs - 1)) ? (sr->vecs[i] == last_avails)
                                            : (sr->vecs[i] == (~(0UL))));
         i++) {
    }
    if (i == nvecs) {
        remove_small_region(head, sr->info);
        MM_DBG_CHECK;
        return dealloc((void *)sr);
    }

    else if ((*head) == 0) {
        (*head) = sr;
        MM_DBG_CHECK;
        return;
    }

    // both ptrs are NULL and head != sr (i.e its not in list)
    else if ((sr->info & (~SIZE_MASK)) == 0 && (*head) != sr) {
        MM_SR_SET_NEXT(sr->info, (*head));
        MM_SR_SET_PREV((*head)->info, sr);
        (*head) = sr;
    }


    MM_DBG_CHECK;
}


static void *
alloc_small(const size_t rounded_size) {
    MM_DBG_CHECK;

    const uint32_t nvecs = MM_SR_GET_N_VECS(rounded_size);
    const uint64_t last_avails =
        MM_SMALL_VEC_LAST_AVAILS(rounded_size, nvecs | 0x1);

    const uint32_t list_idx = (rounded_size / MM_MIN_SIZE) - 1;

    small_region_t ** const head = heap->ll_sr_heads + list_idx;

    // no blocks for this size class
    if ((*head) == 0) {
        small_region_t * const sr =
            (small_region_t * const)find_contig_region(1 << 16);
        (*head) = sr;

        // init info and vectors
        sr->info = rounded_size / MM_MIN_SIZE;
        for (uint32_t i = 0; i < nvecs - 1; i++) {
            sr->vecs[i] = (~(0UL));
        }
        sr->vecs[nvecs - 1] = last_avails;
        sr->vecs[0] ^= 0x1;


        MM_DBG_CHECK;

        return (void *)(((uint64_t)sr) +
                        sizeof(uint64_t) * ((nvecs | 0x1) + 1));
    }

    else {

        small_region_t * const sr = (small_region_t *)((*head));
        uint64_t               vec_idx;
        uint32_t               i;

        for (i = 0; (i < nvecs) && (sr->vecs[i] == 0); i++) {
        }


        ff1_asm_tz(sr->vecs[i], vec_idx);
        sr->vecs[i] ^= ((1UL) << vec_idx);

        // remove from linked list if filled up
        if (i == (nvecs - 1) && sr->vecs[i] == 0) {
            remove_small_region(head, sr->info);

            // set prev/next to 0 (necessary for is in small list check)
            sr->info &= SIZE_MASK;
        }


        MM_DBG_CHECK;

        return (void *)(((uint64_t)sr) +
                        sizeof(uint64_t) * ((nvecs | 0x1) + 1) +
                        (i * 64 + vec_idx) * rounded_size);
    }
}

static void
remove_med_region(med_region_t ** const head, const med_region_t * const mr) {
    if (mr->next == NULL && mr->prev == NULL) {
        (*head) = NULL;
    }
    else if (mr->next != NULL && mr->prev == NULL) {
        (*head)       = mr->next;
        (*head)->prev = NULL;
    }
    else if (mr->next == NULL && mr->prev != NULL) {
        mr->prev->next = NULL;
    }
    else {
        mr->prev->next = mr->next;
        mr->next->prev = mr->prev;
    }
}

static void
dealloc_med(const uint64_t addr) {
    MM_DBG_CHECK;
    med_region_t * const mr =
        (med_region_t * const)(addr & (~(MM_MED_REGION_SIZE - 1)));

    const uint32_t v_pos =
        ((addr % MM_MED_REGION_SIZE) / MM_MED_REGION_CHUNK_SIZE) - 1;

    const uint32_t nchunks =
        get_size(mr->size_vecs[v_pos / WBITS], v_pos % WBITS);

    const uint128_t erase_mask = (SHIFT_HELPER << nchunks) - 1;

    mr->vecs[v_pos / WBITS].u128 ^= (erase_mask << (v_pos % WBITS));


    mr->avail_bits += nchunks;
    if (mr->avail_bits == nchunks) {
        mr->prev = NULL;
        mr->next = heap->ll_mr_head;
        if (heap->ll_mr_head) {
            heap->ll_mr_head->prev = mr;
        }
        heap->ll_mr_head = mr;
    }
    else if (mr->avail_bits == MM_MED_REGION_TOTAL_SLOTS) {
        remove_med_region(&heap->ll_mr_head, mr);
        unset_mr((const uint64_t)mr);
        MM_DBG_CHECK;
        return dealloc((void *)mr);
    }

    MM_DBG_CHECK;
}

static void *
alloc_new_med(med_region_t ** const head, const uint32_t nchunks) {
    const uint128_t erase_mask = (SHIFT_HELPER << nchunks) - 1;

#ifdef MM_DEBUG
    uint32_t start_count = MM_DBG_CHECK;
#endif

    // 2 * size - 1 so we can make it an alligned alloc
    med_region_t * const mr = (med_region_t * const)find_contig_region_aligned(
        MM_MED_REGION_SIZE / PAGE_SIZE,
        MM_MED_REGION_SIZE / PAGE_SIZE);

#ifdef MM_DEBUG
    MM_DBG_ASSERT(MM_DBG_CHECK ==
                  start_count + (MM_MED_REGION_SIZE / PAGE_SIZE));
    MM_DBG_ASSERT((((uint64_t)mr) % MM_MED_REGION_SIZE) == 0);
#endif

    memset(mr->vecs + 1, (~0), sizeof(word_t) * (MM_MED_REGION_N_VECS - 2));

    mr->vecs[0].u128 = ~(erase_mask);
    mr->size_vecs[0].u128 &= (~erase_mask);
    mr->size_vecs[0].u128 |= (erase_mask >> 1);

    // we cant use 1 slot in last vec because 1 chunk is designated for meta
    // info on med_region.
    mr->vecs[MM_MED_REGION_N_VECS - 1].u128 = ~(SHIFT_HELPER << (WBITS - 1));
    mr->prev                                = NULL;
    mr->next                                = (*head);
    mr->avail_bits = MM_MED_REGION_TOTAL_SLOTS - nchunks;

    MM_DBG_CHECK;

    set_mr((const uint64_t)mr);

    if (*head) {
        (*head)->prev = mr;
    }
    *head = mr;

    MM_DBG_CHECK;
    return (void *)(mr + 1);
}

static void *
alloc_med(const size_t size) {
    uint32_t nchunks =
        (size + MM_MED_REGION_CHUNK_SIZE - 1) / MM_MED_REGION_CHUNK_SIZE;
    uint32_t max_chunk_bit;

    MM_DBG_CHECK;
    const uint128_t erase_mask = (SHIFT_HELPER << nchunks) - 1;

    fl1_asm_lz(nchunks, max_chunk_bit);
    max_chunk_bit = (1 << (31 - max_chunk_bit));

    med_region_t ** const head = &(heap->ll_mr_head);
    med_region_t *        mr   = *head;

    const uint32_t r = rand() % MM_MED_REGION_N_VECS;

    while (mr) {
        if (mr->avail_bits < nchunks) {
            mr = mr->next;
            continue;
        }

        const word_t * const vecs = (const word_t * const)mr->vecs;
        // randomize start vec to avoid chunking. This will cause more page
        // usage but speed > memory efficiency imo

        for (uint32_t i = r; i < (MM_MED_REGION_N_VECS + r); i++) {
            const uint32_t _i = i % MM_MED_REGION_N_VECS;

            uint32_t max_chunk_bit_shift = max_chunk_bit / 2;
            word_t   temp_vec            = { .u128 = vecs[_i].u128 &
                                        (vecs[_i].u128 >>
                                         (nchunks ^ max_chunk_bit)) };


            while (max_chunk_bit_shift > 0 && temp_vec.u128) {
                temp_vec.u128 &= (temp_vec.u128 >> max_chunk_bit_shift);
                max_chunk_bit_shift /= 2;
            }
            if (temp_vec.u128) {
                uint64_t v_pos;
                if (temp_vec.u64[0]) {
                    ff1_asm_tz(temp_vec.u64[0], v_pos);
                }
                else {

                    ff1_asm_tz(temp_vec.u64[1], v_pos);
                    v_pos += 64;
                }
                mr->vecs[_i].u128 ^= (erase_mask << v_pos);
                mr->size_vecs[_i].u128 &= (~(erase_mask << v_pos));
                mr->size_vecs[_i].u128 |= ((erase_mask >> 1) << v_pos);

                mr->avail_bits -= nchunks;

                if (mr->avail_bits == 0) {
                    remove_med_region(head, mr);
                }
                MM_DBG_CHECK;
                return (void *)(((uint64_t)(mr + 1)) +
                                MM_MED_REGION_CHUNK_SIZE *
                                    (WBITS * _i + v_pos));
            }
        }
        mr = mr->next;
    }
    MM_DBG_CHECK;
    return alloc_new_med(head, nchunks);
}


static uint64_t
find_contig_region_aligned(const uint32_t alignment,
                           const uint32_t true_npage) {

    word_t         l0_vec       = heap->l0_vec;
    word_t * const l1_vecs      = heap->l1_vecs;
    word_t * const l2_vecs      = heap->l2_vecs;
    word_t * const l2_size_vecs = heap->l2_size_vecs;

    const uint128_t erase_mask = (SHIFT_HELPER << (true_npage)) - 1;

    const uint32_t r = rand() % 64;
    uint64_t       l1_idx;
    for (uint32_t i = 0; i < 2; i++) {
        const uint32_t _i = i;  //((npage_info & 0x1) - i) & 0x1;
        uint64_t       reshape_l0 =
            (l0_vec.u64[_i] >> r) | (l0_vec.u64[_i] >> (64 - r));
        while (reshape_l0) {
            ff1_asm_tz(reshape_l0, l1_idx);
            reshape_l0 ^= ((1UL) << l1_idx);

            l1_idx += 64 * _i;
            if (l1_idx >= r) {
                l1_idx -= r;
            }
            else {
                l1_idx += r;
            }

            if (l1_vecs[l1_idx].u128 != MM_VEC_FULL) {
                const uint64_t ret =
                    try_place(l1_vecs[l1_idx],
                              l2_vecs + (WBITS * l1_idx),
                              (alignment << 16) | (alignment - 1));

                if (ret != MM_CONTINUE) {
                    const uint32_t l1_v_pos = ret >> 32;
                    const uint32_t l2_v_pos =
                        (ret + (alignment - 1)) & (~(alignment - 1));


                    l2_vecs[WBITS * l1_idx + l1_v_pos].u128 ^=
                        (erase_mask << l2_v_pos);


                    l2_size_vecs[WBITS * l1_idx + l1_v_pos].u128 &=
                        (~(erase_mask << l2_v_pos));
                    l2_size_vecs[WBITS * l1_idx + l1_v_pos].u128 |=
                        ((erase_mask >> 1) << l2_v_pos);


                    if (l2_vecs[WBITS * l1_idx + l1_v_pos].u128 ==
                        MM_VEC_FULL) {
                        l1_vecs[l1_idx].u128 ^= (SHIFT_HELPER) << l1_v_pos;
                        if (l1_vecs[l1_idx].u128 == MM_VEC_FULL) {
                            heap->l0_vec.u128 ^= ((SHIFT_HELPER) << l1_idx);
                        }
                    }

                    return heap_start +
                           PAGE_SIZE * (WBITS * WBITS * l1_idx +
                                        WBITS * l1_v_pos + l2_v_pos);
                }
            }
        }
    }
    DBG_ASSERT(0, "Out Of Memory!")
}

static uint64_t
find_contig_region(const uint32_t npage_info) {
    word_t         l0_vec       = heap->l0_vec;
    word_t * const l1_vecs      = heap->l1_vecs;
    word_t * const l2_vecs      = heap->l2_vecs;
    word_t * const l2_size_vecs = heap->l2_size_vecs;

    const uint128_t erase_mask =
        (SHIFT_HELPER << (MM_NPI_N_TOTAL(npage_info))) - 1;

    const uint32_t r = rand() % 64;
    uint64_t       l1_idx;
    for (uint32_t i = 0; i < 2; i++) {
        const uint32_t _i = i;  //((npage_info & 0x1) - i) & 0x1;
        uint64_t       reshape_l0 =
            (l0_vec.u64[_i] >> r) | (l0_vec.u64[_i] >> (64 - r));
        while (reshape_l0) {
            ff1_asm_tz(reshape_l0, l1_idx);
            reshape_l0 ^= ((1UL) << l1_idx);

            l1_idx += 64 * _i;
            if (l1_idx >= r) {
                l1_idx -= r;
            }
            else {
                l1_idx += r;
            }

            if (l1_vecs[l1_idx].u128 != MM_VEC_FULL) {
                const uint64_t ret = try_place(l1_vecs[l1_idx],
                                               l2_vecs + (WBITS * l1_idx),
                                               npage_info);

                if (ret != MM_CONTINUE) {
                    const uint32_t l1_v_pos = ret >> 32;
                    const uint32_t l2_v_pos = ret;

                    l2_vecs[WBITS * l1_idx + l1_v_pos].u128 ^=
                        (erase_mask << l2_v_pos);


                    l2_size_vecs[WBITS * l1_idx + l1_v_pos].u128 &=
                        (~(erase_mask << l2_v_pos));
                    l2_size_vecs[WBITS * l1_idx + l1_v_pos].u128 |=
                        ((erase_mask >> 1) << l2_v_pos);


                    if (l2_vecs[WBITS * l1_idx + l1_v_pos].u128 ==
                        MM_VEC_FULL) {
                        l1_vecs[l1_idx].u128 ^= (SHIFT_HELPER) << l1_v_pos;
                        if (l1_vecs[l1_idx].u128 == MM_VEC_FULL) {
                            heap->l0_vec.u128 ^= ((SHIFT_HELPER) << l1_idx);
                        }
                    }

                    return heap_start +
                           PAGE_SIZE * (WBITS * WBITS * l1_idx +
                                        WBITS * l1_v_pos + l2_v_pos);
                }
            }
        }
    }
    DBG_ASSERT(0, "Out Of Memory!")
}

void
dealloc(void * const addr) {
    if (!addr) {
        return;
    }
    MM_DBG_CHECK;
    if (is_med_region((const uint64_t)addr)) {
        dealloc_med((const uint64_t)addr);
        MM_DBG_CHECK;
        return;
    }
    else if (MM_IS_SMALL_REGION_ADDR(addr)) {
        dealloc_small((const uint64_t)addr);
        MM_DBG_CHECK;
        return;
    }

#ifdef MM_DEBUG
    uint32_t start_count = MM_DBG_CHECK;
#endif
    
    const uint64_t v_idx = (((uint64_t)addr) - heap_start) / PAGE_SIZE;

    const uint32_t l0_idx   = (v_idx / (WBITS * WBITS)) & (WBITS - 1);
    const uint32_t l1_v_pos = (v_idx / WBITS) & (WBITS - 1);
    const uint32_t l2_v_pos = v_idx & (WBITS - 1);

    const uint32_t npages =
        get_size(heap->l2_size_vecs[WBITS * l0_idx + l1_v_pos], l2_v_pos);

    const uint128_t erase_mask = (((SHIFT_HELPER) << npages) - 1);
    
    if (heap->l2_vecs[WBITS * l0_idx + l1_v_pos].u128 == MM_VEC_FULL) {
        if (heap->l1_vecs[l0_idx].u128 == MM_VEC_FULL) {
            heap->l0_vec.u128 ^= ((SHIFT_HELPER) << l0_idx);
        }
        heap->l1_vecs[l0_idx].u128 ^= ((SHIFT_HELPER) << l1_v_pos);
    }
    MM_DBG_CHECK;
    heap->l2_vecs[WBITS * l0_idx + l1_v_pos].u128 ^= (erase_mask << l2_v_pos);
    MM_DBG_CHECK;

#ifdef MM_DEBUG
    uint32_t after_count = MM_DBG_CHECK;
    MM_DBG_ASSERT(start_count == after_count + npages);
#endif
}


void *
alloc(const size_t size) {
    if (!size) {
        return NULL;
    }
    if (heap == NULL) {
        init_heap();
    }
    if (size <= MM_SMALL_REGION_BOUND) {
        return alloc_small(MM_SMALL_ROUND(size));
    }
    if (size <= MM_MED_REGION_BOUND) {
        MM_DBG_CHECK;
        return alloc_med(size);
    }

#ifdef MM_DEBUG
    uint32_t start_count = MM_DBG_CHECK;
#endif

    uint32_t npage_info = MM_GET_NPAGES(size);
    uint32_t max_npage_bit;

    // find highest order bit
    fl1_asm_lz(npage_info, max_npage_bit);
    max_npage_bit = (1 << (31 - max_npage_bit));


    // unset highest order bit and
    // set so can be accessed more
    // easily later on
    npage_info ^= (max_npage_bit);
    npage_info |= (max_npage_bit << 16);

    void * const ret = (void *)find_contig_region(npage_info);
#ifdef MM_DEBUG
    uint32_t after_count = MM_DBG_CHECK;
    MM_DBG_ASSERT(start_count + MM_GET_NPAGES(size) == after_count);
#endif

    return ret;
}

uint32_t
check_heap(const char * const f, const char * const fn, const uint32_t ln) {
#ifdef MM_DEBUG
    uint32_t active_small_pages = 0;
    uint32_t med_pages          = 0;
    uint32_t active_med_pages   = 0;
    uint32_t total_pages        = 0;

    uint32_t active_small_allocs = 0;
    uint32_t med_allocs          = 0;
    uint32_t active_med_allocs   = 0;


    const word_t   l0_vec  = heap->l0_vec;
    word_t * const l1_vecs = heap->l1_vecs;
    word_t * const l2_vecs = heap->l2_vecs;

    // small region check
    for (uint32_t i = 0; i < MM_N_SMALL_REGIONS; i++) {
        if (heap->ll_sr_heads[i] == 0) {
            continue;
        }
        small_region_t * temp = heap->ll_sr_heads[i];

        MM_DBG_ASSERT(MM_SR_GET_PREV(temp->info) == 0);


        const uint32_t size     = MM_SR_GET_SIZE(temp->info);
        const uint32_t list_idx = (size / MM_MIN_SIZE) - 1;

        MM_DBG_ASSERT(size == ((i + 1) * 16));
        MM_DBG_ASSERT(list_idx == i);


        small_region_t * prev_ptrs[32];
        uint32_t         prev_idx = 0;

        PRINT(LOW_VERBOSE,
              "%s:%s:%d -> %d == %d\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              list_idx,
              i);
        MM_DBG_ASSERT(list_idx == i);
        while (temp) {
            active_small_pages++;

            for (uint32_t _i = 0; _i < prev_idx; _i++) {
                MM_DBG_ASSERT(temp != prev_ptrs[i]);
            }
            prev_ptrs[prev_idx++] = temp;

            MM_DBG_ASSERT(MM_SR_GET_SIZE(temp->info) == size);
            if (MM_SR_GET_NEXT(temp->info) != 0) {
                MM_DBG_ASSERT(
                    MM_SR_GET_PREV(MM_SR_GET_NEXT(temp->info)->info) == temp);
            }
            if (MM_SR_GET_PREV(temp->info) == 0) {
                MM_DBG_ASSERT(heap->ll_sr_heads[i] == temp);
            }
            else {
                MM_DBG_ASSERT(
                    MM_SR_GET_NEXT(MM_SR_GET_PREV(temp->info)->info) == temp);
            }
            for (uint32_t _i = 0; _i < MM_SR_GET_N_VECS(size); _i++) {
                active_small_allocs += bitcount_64(temp->vecs[_i]);
            }

            temp = MM_SR_GET_NEXT(temp->info);
        }
    }

    // med region check
    med_region_t * active_mr[128];
    uint32_t       active_mr_idx = 0;

    med_region_t * temp = heap->ll_mr_head;
    while (temp) {
        assert((uint64_t)temp >= (heap_start));
        active_mr[active_mr_idx++] = temp;
        uint32_t manual_count      = 0;
        for (uint32_t _i = 0; _i < MM_MED_REGION_N_VECS; _i++) {
            manual_count += BITCOUNT(temp->vecs[_i]);
        }
        MM_DBG_VASSERT(manual_count == temp->avail_bits,
                       "%d != %d ->\n"
                       "\t" WORD_FORMAT
                       "\n"
                       "\t" WORD_FORMAT
                       "\n"
                       "\t" WORD_FORMAT
                       "\n"
                       "\t" WORD_FORMAT "\n",
                       manual_count,
                       temp->avail_bits,
                       WORD_PRINT(temp->vecs[0]),
                       WORD_PRINT(temp->vecs[1]),
                       WORD_PRINT(temp->vecs[2]),
                       WORD_PRINT(temp->vecs[3]));
        active_med_allocs += MM_MED_REGION_TOTAL_SLOTS - manual_count;
        active_med_pages += MM_MED_REGION_SIZE / PAGE_SIZE;

        if (temp->next) {
            MM_DBG_ASSERT(temp->next->prev == temp);
        }
        if (temp->prev) {
            MM_DBG_ASSERT(heap->ll_mr_head != temp);
            MM_DBG_ASSERT(temp->prev->next == temp);
        }
        else {
            MM_DBG_ASSERT(heap->ll_mr_head == temp);
        }
        MM_DBG_ASSERT(is_med_region((const uint64_t)temp));
        temp = temp->next;
    }

    for (uint32_t i = 0; i < MM_N_MED_REGION_VECS; i++) {
        for (uint32_t _i = 0; _i < WBITS; _i++) {
            if (heap->med_region_vecs[i].u128 & (SHIFT_HELPER << _i)) {
                med_pages += MM_MED_REGION_SIZE / PAGE_SIZE;
                med_region_t * temp_mr =
                    (med_region_t *)(heap_start +
                                     (i * WBITS + _i) * MM_MED_REGION_SIZE);

                assert(is_med_region((uint64_t)temp_mr));
                const uint64_t v_idx =
                    (((uint64_t)temp_mr) - heap_start) / PAGE_SIZE;
                const uint32_t l0_idx = (v_idx / (WBITS * WBITS)) & (WBITS - 1);
                const uint32_t l1_v_pos = (v_idx / WBITS) & (WBITS - 1);
                const uint32_t l2_v_pos = v_idx & (WBITS - 1);


                MM_DBG_VASSERT(
                    get_size(heap->l2_size_vecs[WBITS * l0_idx + l1_v_pos],
                             l2_v_pos) == (MM_MED_REGION_SIZE / PAGE_SIZE),
                    "%lu != %lu\n",
                    get_size(heap->l2_size_vecs[WBITS * l0_idx + l1_v_pos],
                             l2_v_pos),
                    (MM_MED_REGION_SIZE / PAGE_SIZE));

                assert(!(heap->l2_vecs[WBITS * l0_idx + l1_v_pos].u128 &
                         (SHIFT_HELPER << l2_v_pos)));


                MM_DBG_ASSERT(temp_mr->avail_bits >= 0);
                MM_DBG_ASSERT(temp_mr->avail_bits < MM_MED_REGION_TOTAL_SLOTS);

                uint32_t manual_count = 0;
                for (uint32_t j = 0; j < MM_MED_REGION_N_VECS; j++) {
                    manual_count += BITCOUNT(temp_mr->vecs[j]);
                }
                MM_DBG_ASSERT(manual_count == temp_mr->avail_bits);
                med_allocs += MM_MED_REGION_TOTAL_SLOTS - manual_count;

                MM_DBG_ASSERT(temp_mr->avail_bits != MM_MED_REGION_TOTAL_SLOTS);
                if (temp_mr->avail_bits == 0) {
                    for (uint32_t j = 0; j < active_mr_idx; j++) {
                        MM_DBG_ASSERT(temp_mr != active_mr[j]);
                    }
                }
                else {
                    for (uint32_t j = 0; j < active_mr_idx; j++) {
                        if (temp_mr == active_mr[j]) {
                            break;
                        }
                        MM_DBG_VASSERT(j != (active_mr_idx - 1),
                                       "Error from: %d\n",
                                       ln);
                    }
                }
            }
        }
    }

    for (uint32_t i = 0; i < MM_N_L2_VECS; i++) {
        if (l2_vecs[i].u128 == MM_VEC_FULL) {
            MM_DBG_VASSERT(!(l1_vecs[(i / WBITS)].u128 &
                             (SHIFT_HELPER << (i & (WBITS - 1)))),
                           "Error: Invalid L1 "
                           "for a full L2: "
                           "%d\n",
                           ln);
        }
        else {
            MM_DBG_VASSERT((l1_vecs[(i / WBITS)].u128 &
                            (SHIFT_HELPER << (i & (WBITS - 1)))),
                           "Error: Invalid L1 "
                           "for a partial L2: "
                           "%d\n",
                           ln);
        }
        if (l1_vecs[(i / WBITS)].u128 == MM_VEC_FULL) {
            MM_DBG_VASSERT(!(l0_vec.u128 & (SHIFT_HELPER << (i / WBITS))),
                           "Error: Invalid L0 "
                           "for a full L1: "
                           "%d\n",
                           ln);
        }
        else {
            MM_DBG_VASSERT((l0_vec.u128 & (SHIFT_HELPER << (i / WBITS))),
                           "Error: Invalid L0 "
                           "for a partial L1: "
                           "%d\n"
                           "\tl0_vec           "
                           " : " WORD_FORMAT
                           "\n"
                           "\tl1_vec           "
                           " : " WORD_FORMAT
                           "\n"
                           "\ti                "
                           " : %d\n"
                           "\ti_l1             "
                           " : %d\n"
                           "\ti_l0             "
                           " : %d\n\n",
                           ln,
                           WORD_PRINT(l0_vec),
                           WORD_PRINT(l1_vecs[i / WBITS]),
                           i,
                           i / WBITS,
                           i / WBITS);
        }
        total_pages += (WBITS - BITCOUNT(l2_vecs[i]));
    }
    return total_pages;
#else
    return 0;
#endif
}
