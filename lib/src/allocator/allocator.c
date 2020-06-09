#include <allocator/allocator.h>

typedef __int128          int128_t;
typedef unsigned __int128 uint128_t;

typedef union cast128 {
    uint128_t u128;
    struct {
        uint64_t u64[2];
    };
    struct {
        uint32_t u32[4];
    };

} cast128_t;

typedef uint128_t word_t;
#define WBITS (128)

#if WBITS == 128

#define TO_VAL(X)   (X).u128
#define WORD_FORMAT "0x%016lX %016lX"
static uint64_t
get_uint64(uint128_t input, uint32_t w_idx) {
    assert(w_idx == 0 || w_idx == 1);
    cast128_t temp = { .u128 = input };
    return temp.u64[w_idx];
}
#define WORD_PRINT(X) get_uint64(X, 0), get_uint64(X, 1)

static inline uint32_t
BITCOUNT(uint128_t x) {
    const cast128_t temp = { .u128 = x };
    return bitcount_64(temp.u64[0]) + bitcount_64(temp.u64[1]);
}

const word_t SHIFT_HELPER = 1;
#define X_TEMP MACRO_COMBINE(_X, ln)
#define Y_TEMP MACRO_COMBINE(_Y, ln)

#define mm_ff0_asm_tz(X, Y) _mm_ff0_asm_tz(X, Y, __LINE__)
#define mm_ff1_asm_tz(X, Y) _mm_ff1_asm_tz(X, Y, __LINE__)
#define mm_fl1_asm_lz(X, Y) fl1_asm_lz(X, Y)

#define _mm_ff0_asm_tz(X, Y, ln)                                               \
    {                                                                          \
        const cast128_t X_TEMP = { .u128 = (X) };                              \
        if (X_TEMP.u64[0]) {                                                   \
            ff0_asm_tz(X_TEMP.u64[0], (Y));                                    \
            if ((Y) == 64) {                                                   \
                ff0_asm_tz(X_TEMP.u64[0], (Y));                                \
                (Y) += 64;                                                     \
            }                                                                  \
        }                                                                      \
        else {                                                                 \
            (Y) = 0;                                                           \
        }                                                                      \
    }

#define _mm_ff1_asm_tz(X, Y, ln)                                               \
    {                                                                          \
        const cast128_t X_TEMP = { .u128 = (X) };                              \
        if (X_TEMP.u64[0]) {                                                   \
            ff1_asm_tz(X_TEMP.u64[0], (Y));                                    \
        }                                                                      \
        else {                                                                 \
            ff1_asm_tz(X_TEMP.u64[1], (Y));                                    \
            (Y) += 64;                                                         \
        }                                                                      \
    }

#else
#define WORD_FORMAT   "0x%016lX"
#define WORD_PRINT(X) (X)

#define SHIFT_HELPER (1UL)
#define TO_VAL(X)    (X)

#define BITCOUNT(X) bitcount_64(X)

#define mm_ff0_asm_tz(X, Y) ff0_asm_tz(X, Y)
#define mm_ff1_asm_tz(X, Y) ff1_asm_tz(X, Y)
#define mm_fl1_asm_lz(X, Y) fl1_asm_lz(X, Y)
#endif


#define MM_DEBUG
#ifdef MM_DEBUG
#define MM_DBG_CHECK                    dbg_count_active_pages(__LINE__)
#define MM_DBG_VASSERT(X, msg, args...) DBG_ASSERT(X, msg, ##args)
#define MM_DBG_ASSERT(X)                assert(X)
#else
#define MM_DBG_CHECK 0
#define MM_DBG_VASSERT(X, msg, args...)
#define MM_DBG_ASSERT(X)
#endif

// if uint128 is used then 2 ^ 33 ~ 8GB if uint64 is used 2 ^ 30 ~ 1 GB
#define MM_DEFAULT_LENGTH                                                      \
    ((WBITS * WBITS * WBITS * PAGE_SIZE) + sizeof(region_head_t))
#define MM_DEFAULT_BASE_ADDR 0

// -2 is because we are skipping PAGE_SIZE and PAGE_SIZE / 2
#define MM_N_SMALL_REGIONS  ((((PAGE_SIZE / 2)) / MM_ALIGNMENT) - 1)
#define MM_SMALL_REGION_MIN ((PAGE_SIZE / 2) - MM_MIN_SIZE)

#define MM_MIN_SIZE     (16)
#define MM_LOG_MIN_SIZE (4)

#define MM_N_L1_VECS (WBITS)
#define MM_N_L2_VECS (WBITS * WBITS)


#if WBITS == 128
const uint128_t l1_full = (-1);
const uint128_t l2_full = 0;

#define MM_L1_VEC_FULL l1_full
#define MM_L2_VEC_FULL l2_full
#else
#define MM_L1_VEC_FULL (~(0UL))
#define MM_L2_VEC_FULL (0UL)
#endif

#define META_DATA_SIZE                                                         \
    ((sizeof(word_t) + MM_N_L1_VECS * sizeof(word_t) +                         \
      2 * MM_N_L2_VECS * sizeof(word_t) +                                      \
      1 * MM_N_SMALL_REGIONS * sizeof(struct small_region *)) %                \
     PAGE_SIZE)

typedef struct region_head {
    word_t l0_vec;
    word_t l1_vecs[MM_N_L1_VECS];
    // bit vec of free pages
    word_t l2_vecs[MM_N_L2_VECS];
    word_t l2_size_vecs[MM_N_L2_VECS];

    struct small_region * ll_heads[MM_N_SMALL_REGIONS];

    uint8_t padding[PAGE_SIZE - META_DATA_SIZE];
} region_head_t;

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
                               ((uint64_t)addr_start))                         \
                            : 0))

#define MM_SR_GET_PREV(X)                                                      \
    ((small_region_t *)(MM_SR_GET_PREV_BITS(X)                                 \
                            ? (PAGE_SIZE * MM_SR_GET_PREV_BITS(X) +            \
                               ((uint64_t)addr_start))                         \
                            : 0))

#define MM_SR_SET_SIZE(X, Y)                                                   \
    ((X) = (((X) & (~SIZE_MASK)) | ((Y) / MM_MIN_SIZE) << SIZE_SHIFT))

#define MM_SR_SET_NEXT(X, Y)                                                   \
    MM_SR_SET_NEXT_BITS(                                                       \
        X,                                                                     \
        (Y) ? ((((uint64_t)(Y)) - ((uint64_t)addr_start)) / PAGE_SIZE) : 0)

#define MM_SR_SET_PREV(X, Y)                                                   \
    MM_SR_SET_PREV_BITS(                                                       \
        X,                                                                     \
        (Y) ? ((((uint64_t)(Y)) - ((uint64_t)addr_start)) / PAGE_SIZE) : 0)


#define MM_SMALL_VEC_LAST_AVAILS(X, Y)                                         \
    (((1UL) << (((PAGE_SIZE - sizeof(uint64_t) * ((Y) + 1)) / (X)) % 64)) - 1)

typedef struct small_region {
    //[0 - 7]   size
    //[8 - 31]  prev ptr
    //[32 - 64] next ptr
    uint64_t region_size_and_ptrs;
    uint64_t vecs[1];
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

    memset(addr_start->l2_vecs, ~(0), sizeof(word_t) * MM_N_L2_VECS);
    addr_start->l0_vec = MM_L1_VEC_FULL;
    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Init New Address Heap\n"
          "\tStart              : %p\n"
          "\tEnd                : %p\n"
          "\tSize               : %lu\n"
          "\tl0_vec             : " WORD_FORMAT "\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          addr_start,
          ((uint8_t *)addr_start) + MM_DEFAULT_LENGTH,
          MM_DEFAULT_LENGTH,
          WORD_PRINT(addr_start->l0_vec));
    MM_DBG_ASSERT(MM_DBG_CHECK == 0);
}
//////////////////////////////////////////////////////////////////////
static inline uint32_t
get_size(const word_t l2_size_vec, const uint32_t l2_idx) {

    uint64_t size;
    mm_ff0_asm_tz(l2_size_vec >> l2_idx, size);

    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Getting Size\n"
          "\tVec                : " WORD_FORMAT
          "\n"
          "\tIdx                : %d\n"
          "\tSize               : %lu\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          WORD_PRINT(l2_size_vec),
          l2_idx,
          size + 1);

    return size + 1;
}

static uint64_t
try_place(word_t         l1_vec,
          word_t * const l2_vecs,
          const uint32_t npages,
          const uint32_t max_npage_bit) {

    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Trying Place:\n"
          "\tl1_vec             : " WORD_FORMAT
          "\n"
          "\tl2_vecs_addr       : %p\n"
          "\tnpages             : %d\n"
          "\tmax_npage_bit      : %d\n"
          "\tTotal Pages        : %d\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          WORD_PRINT(l1_vec),
          l2_vecs,
          npages,
          max_npage_bit,
          npages | (max_npage_bit));


    uint64_t l2_idx, ret_idx;

    while (l1_vec) {
        mm_ff1_asm_tz(l1_vec, l2_idx);
        l1_vec ^= ((SHIFT_HELPER) << l2_idx);


        MM_DBG_ASSERT(l2_vecs[l2_idx] != MM_L2_VEC_FULL);

        word_t temp_l2_vec = l2_vecs[l2_idx] & (l2_vecs[l2_idx] >> npages);

        PRINT(HIGH_VERBOSE,
              "%s:%s:%d -> L1 Loop\n"
              "\tCur l1_vec         : " WORD_FORMAT
              "\n"
              "\tl2_idx             : %lu\n"
              "\tl2_vec             : " WORD_FORMAT "\n\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              WORD_PRINT(l1_vec),
              l2_idx,
              WORD_PRINT(temp_l2_vec));

        uint32_t max_npage_bit_shift = (max_npage_bit) / 2;

        while (max_npage_bit_shift > 0 && temp_l2_vec) {

            PRINT(HIGH_VERBOSE,
                  "%s:%s:%d -> L2 Loop\n"
                  "\tl2_vec             : " WORD_FORMAT
                  "\n"
                  "\tdownshift          : %d\n\n",
                  __FILE__,
                  __FUNCTION__,
                  __LINE__,
                  WORD_PRINT(temp_l2_vec),
                  max_npage_bit_shift);


            temp_l2_vec &= (temp_l2_vec >> max_npage_bit_shift);
            max_npage_bit_shift /= 2;
        }
        if (temp_l2_vec) {
            mm_ff1_asm_tz(temp_l2_vec, ret_idx);

            PRINT(HIGH_VERBOSE,
                  "%s:%s:%d -> Return Found\n"
                  "\tl2_idx             : %lu\n"
                  "\tret_idx            : %lu\n"
                  "\tret                : 0x%016lX\n\n",
                  __FILE__,
                  __FUNCTION__,
                  __LINE__,
                  l2_idx,
                  ret_idx,
                  (l2_idx << 32) | ret_idx);

            return (l2_idx << 32) | ret_idx;
        }
    }
    PRINT(HIGH_VERBOSE,
          "%s:%s:%d -> Return NOT Found\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__);

    return ~(0UL);
}


static word_t
find_contig_region(const uint32_t npages, const uint32_t max_npage_bit) {

    word_t         l0_vec       = addr_start->l0_vec;
    word_t * const l1_vecs      = addr_start->l1_vecs;
    word_t * const l2_vecs      = addr_start->l2_vecs;
    word_t * const l2_size_vecs = addr_start->l2_size_vecs;

    const word_t erase_mask = (SHIFT_HELPER << (npages | (max_npage_bit))) - 1;
    const word_t size_mask  = (erase_mask) >> 1;

    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Find Contig\n"
          "\terase_mask         : " WORD_FORMAT
          "\n"
          "\tsize_mask          : " WORD_FORMAT
          "\n"
          "\tnpages             : %d\n"
          "\tmax_npage_bit      : %d\n"
          "\tTotal Pages        : %d\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          WORD_PRINT(erase_mask),
          WORD_PRINT(size_mask),
          npages,
          max_npage_bit,
          npages | (max_npage_bit));

    uint64_t i;
    while (l0_vec) {
        mm_ff1_asm_tz(l0_vec, i);
        l0_vec ^= ((SHIFT_HELPER) << i);

        PRINT(MED_VERBOSE,
              "%s:%s:%d -> L0 Loop\n"
              "\tl0_vec             : " WORD_FORMAT
              "\n"
              "\ti                  : %lu\n"
              "\tl1_vec             : " WORD_FORMAT "\n\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              WORD_PRINT(l0_vec),
              i,
              WORD_PRINT(l1_vecs[i]));

        if (l1_vecs[i] != MM_L1_VEC_FULL) {
            const word_t ret = try_place(~l1_vecs[i],
                                         l2_vecs + (WBITS * i),
                                         npages,
                                         max_npage_bit);
            if (ret != (~(0UL))) {
                const uint32_t l1_idx = ret >> 32;
                const uint32_t l2_idx = ret;

                PRINT(MED_VERBOSE,
                      "%s:%s:%d -> Got Return Pre Process\n"
                      "\tl1_idx             : %d\n"
                      "\tl2_idx             : %d\n"
                      "\tCompute Idx        : %lu\n"
                      "\tl0_vec             : " WORD_FORMAT
                      "\n"
                      "\tl1_vec             : " WORD_FORMAT
                      "\n"
                      "\tl2_vec             : " WORD_FORMAT
                      "\n"
                      "\tl2_size            : " WORD_FORMAT "\n\n",
                      __FILE__,
                      __FUNCTION__,
                      __LINE__,
                      l1_idx,
                      l2_idx,
                      WBITS * i + l1_idx,
                      WORD_PRINT(l0_vec),
                      WORD_PRINT(l1_vecs[i]),
                      WORD_PRINT(l2_vecs[WBITS * i + l1_idx]),
                      WORD_PRINT(l2_size_vecs[WBITS * i + l1_idx]));

                MM_DBG_ASSERT(BITCOUNT((erase_mask << l2_idx)) ==
                              BITCOUNT(erase_mask));

                MM_DBG_ASSERT(
                    (l2_vecs[WBITS * i + l1_idx] ^ (erase_mask << l2_idx)) ==
                    (l2_vecs[WBITS * i + l1_idx] & (~(erase_mask << l2_idx))));

                l2_vecs[WBITS * i + l1_idx] ^= (erase_mask << l2_idx);
                l2_size_vecs[WBITS * i + l1_idx] &=
                    (((~erase_mask) << l2_idx) |
                     ((SHIFT_HELPER << l2_idx) - 1));


                l2_size_vecs[WBITS * i + l1_idx] |= (size_mask) << l2_idx;

                MM_DBG_ASSERT(get_size(l2_size_vecs[WBITS * i + l1_idx],
                                       l2_idx) == (npages | (max_npage_bit)));


                if (l2_vecs[WBITS * i + l1_idx] == MM_L2_VEC_FULL) {
                    PRINT(MED_VERBOSE,
                          "%s:%s:%d -> L2 Full\n"
                          "\ti              : %lu\n"
                          "\tl1_idx         : %d\n"
                          "\tl2_idx         : %lu\n"
                          "\tl2_vec         : " WORD_FORMAT
                          "\n"
                          "\tl1_vec         : " WORD_FORMAT
                          "\n"
                          "\tl0_vec         : " WORD_FORMAT "\n\n",
                          __FILE__,
                          __FUNCTION__,
                          __LINE__,
                          i,
                          l1_idx,
                          WBITS * i + l1_idx,
                          WORD_PRINT(addr_start->l1_vecs[WBITS * i + l1_idx]),
                          WORD_PRINT(addr_start->l1_vecs[i]),
                          WORD_PRINT(addr_start->l0_vec));

                    l1_vecs[i] ^= ((SHIFT_HELPER) << l1_idx);
                    if (l1_vecs[i] == MM_L1_VEC_FULL) {
                        PRINT(
                            MED_VERBOSE,
                            "%s:%s:%d -> L1 Full\n"
                            "\ti              : %lu\n"
                            "\tl1_idx         : %d\n"
                            "\tl2_idx         : %lu\n"
                            "\tl2_vec         : " WORD_FORMAT
                            "\n"
                            "\tl1_vec         : " WORD_FORMAT
                            "\n"
                            "\tl0_vec         : " WORD_FORMAT "\n\n",
                            __FILE__,
                            __FUNCTION__,
                            __LINE__,
                            i,
                            l1_idx,
                            WBITS * i + l1_idx,
                            WORD_PRINT(addr_start->l1_vecs[WBITS * i + l1_idx]),
                            WORD_PRINT(addr_start->l1_vecs[i]),
                            WORD_PRINT(addr_start->l0_vec));

                        addr_start->l0_vec ^= ((SHIFT_HELPER) << i);
                    }
                }

                PRINT(MED_VERBOSE,
                      "%s:%s:%d -> Got Return Post Process\n"
                      "\tl1_idx             : %d\n"
                      "\tl2_idx             : %d\n"
                      "\tCompute Idx        : %lu\n"
                      "\tl0_vec             : " WORD_FORMAT
                      "\n"
                      "\tl1_vec             : " WORD_FORMAT
                      "\n"
                      "\tl2_vec             : " WORD_FORMAT
                      "\n"
                      "\tl2_size            : " WORD_FORMAT "\n\n",
                      __FILE__,
                      __FUNCTION__,
                      __LINE__,
                      l1_idx,
                      l2_idx,
                      WBITS * i + l1_idx,
                      WORD_PRINT(l0_vec),
                      WORD_PRINT(l1_vecs[i]),
                      WORD_PRINT(l2_vecs[WBITS * i + l1_idx]),
                      WORD_PRINT(l2_size_vecs[WBITS * i + l1_idx]));

                void * const test_addr =
                    (void *)(((uint64_t)(addr_start + 1)) +
                             PAGE_SIZE *
                                 (WBITS * WBITS * i + WBITS * l1_idx + l2_idx));
                const uint64_t page_start = (word_t)(addr_start + 1);
                const uint64_t v_idx =
                    (((word_t)test_addr) - page_start) / PAGE_SIZE;
                const uint32_t _l0_idx =
                    (v_idx / (WBITS * WBITS)) & (WBITS - 1);
                const uint32_t _l1_idx = (v_idx / WBITS) & (WBITS - 1);
                const uint32_t _l2_idx = v_idx & (WBITS - 1);

                MM_DBG_ASSERT(_l0_idx == i);
                MM_DBG_ASSERT(_l1_idx == l1_idx);
                MM_DBG_ASSERT(_l2_idx == l2_idx);


                MM_DBG_ASSERT((((uint64_t)(addr_start + 1)) +
                               PAGE_SIZE * (WBITS * WBITS * i + WBITS * l1_idx +
                                            l2_idx)) %
                                  PAGE_SIZE ==
                              0);

                return ((uint64_t)(addr_start + 1)) +
                       PAGE_SIZE *
                           (WBITS * WBITS * i + WBITS * l1_idx + l2_idx);
            }
        }
    }
    DBG_ASSERT(0, "Out Of Memory!")
}

static void
remove_ll(small_region_t ** const head, small_region_t * const sr) {

    if (MM_SR_GET_NEXT(sr->region_size_and_ptrs) == 0 &&
        MM_SR_GET_PREV(sr->region_size_and_ptrs) == 0) {

        PRINT(LOW_VERBOSE,
              "%s:%s:%d -> Empty block - 1\n",
              __FILE__,
              __FUNCTION__,
              __LINE__);

        MM_DBG_ASSERT((*head) == sr);
        (*head) = 0;
    }
    else if (MM_SR_GET_NEXT(sr->region_size_and_ptrs) != 0 &&
             MM_SR_GET_PREV(sr->region_size_and_ptrs) == 0) {

        MM_DBG_ASSERT((*head) != 0);
        PRINT(LOW_VERBOSE,
              "%s:%s:%d -> Empty block - 2\n",
              __FILE__,
              __FUNCTION__,
              __LINE__);

        MM_DBG_ASSERT((*head) == sr);
        (*head) = MM_SR_GET_NEXT(sr->region_size_and_ptrs);
        MM_SR_SET_PREV((*head)->region_size_and_ptrs, 0);
    }
    else if (MM_SR_GET_NEXT(sr->region_size_and_ptrs) == 0 &&
             MM_SR_GET_PREV(sr->region_size_and_ptrs) != 0) {

        MM_DBG_ASSERT((*head) != 0);
        PRINT(LOW_VERBOSE,
              "%s:%s:%d -> Empty block - 3\n",
              __FILE__,
              __FUNCTION__,
              __LINE__);

        MM_DBG_ASSERT((*head) != sr);
        MM_SR_SET_NEXT(
            MM_SR_GET_PREV(sr->region_size_and_ptrs)->region_size_and_ptrs,
            0);
    }
    else {
        MM_DBG_ASSERT((*head) != 0);
        MM_DBG_ASSERT((*head) != sr);

        PRINT(LOW_VERBOSE,
              "%s:%s:%d -> Empty block - 4\n",
              __FILE__,
              __FUNCTION__,
              __LINE__);

        MM_SR_SET_NEXT(
            MM_SR_GET_PREV(sr->region_size_and_ptrs)->region_size_and_ptrs,
            MM_SR_GET_NEXT(sr->region_size_and_ptrs));
        MM_SR_SET_PREV(
            MM_SR_GET_NEXT(sr->region_size_and_ptrs)->region_size_and_ptrs,
            MM_SR_GET_PREV(sr->region_size_and_ptrs));
    }
}

static void
dealloc_small(void * addr) {
    MM_DBG_CHECK;
    small_region_t * const sr =
        (small_region_t *)(((uint64_t)addr) & (~(PAGE_SIZE - 1)));
    MM_DBG_ASSERT(((uint64_t)sr) % PAGE_SIZE == 0);
    const uint32_t size        = MM_SR_GET_SIZE(sr->region_size_and_ptrs);
    const uint32_t nvecs       = MM_SR_GET_N_VECS(size);
    const uint32_t list_idx    = (size / MM_MIN_SIZE) - 1;
    const uint32_t vec_idx     = MM_SR_ADDR_TO_IDX(addr, sr, nvecs, size);
    const uint64_t last_avails = MM_SMALL_VEC_LAST_AVAILS(size, nvecs | 0x1);
    uint32_t       i;

    small_region_t ** const head = addr_start->ll_heads + list_idx;

    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Dealloc Small\n"
          "\tSR             : %p\n"
          "\tAddr           : %p\n"
          "\tSize           : %d\n"
          "\tnvecs          : %d\n"
          "\tlist_idx       : %d\n"
          "\tvec_idx        : %d\n"
          "\tlast_avails    : 0x%016lX\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          sr,
          addr,
          size,
          nvecs,
          list_idx,
          vec_idx,
          last_avails);

    MM_DBG_ASSERT((vec_idx / 64) < nvecs);
    if ((vec_idx / 64) == nvecs - 1) {
        MM_DBG_ASSERT((vec_idx % 64) < last_avails);
    }

    sr->vecs[vec_idx / 64] ^= ((1UL) << (vec_idx % 64));


    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Partial Block\n",
          __FILE__,
          __FUNCTION__,
          __LINE__);

    // list empty
    if ((*head) == 0) {

        PRINT(LOW_VERBOSE,
              "%s:%s:%d -> Partial Block - 1\n",
              __FILE__,
              __FUNCTION__,
              __LINE__);

        MM_DBG_ASSERT(MM_SR_GET_NEXT(sr->region_size_and_ptrs) == 0);
        MM_DBG_ASSERT(MM_SR_GET_PREV(sr->region_size_and_ptrs) == 0);

        (*head) = sr;
        return;
    }

    for (i = 0;
         (i < nvecs) && ((i == (nvecs - 1)) ? (sr->vecs[i] == last_avails)
                                            : (sr->vecs[i] == (~(0UL))));
         i++) {
    }
    // check to see if small regin is empty
    if (i == nvecs) {
        PRINT(LOW_VERBOSE,
              "%s:%s:%d -> Empty block\n",
              __FILE__,
              __FUNCTION__,
              __LINE__);

        // entire region is empty (might want to put this only in case where
        // there are other blocks of in size class in list)
        MM_DBG_ASSERT((*head) != 0);
        remove_ll(head, sr);

        return dealloc((void *)sr);
    }


    // not in list
    else if ((sr->region_size_and_ptrs & (~SIZE_MASK)) == 0 && (*head) != sr) {

        PRINT(LOW_VERBOSE,
              "%s:%s:%d -> Partial Block - 2\n",
              __FILE__,
              __FUNCTION__,
              __LINE__);

        MM_DBG_ASSERT(MM_SR_GET_PREV(sr->region_size_and_ptrs) == 0);
        MM_DBG_ASSERT(MM_SR_GET_NEXT(sr->region_size_and_ptrs) == 0);

        MM_DBG_ASSERT(size == MM_SR_GET_SIZE((*head)->region_size_and_ptrs));

        MM_SR_SET_NEXT(sr->region_size_and_ptrs, (*head));
        MM_SR_SET_PREV((*head)->region_size_and_ptrs, sr);
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

    small_region_t ** const head = addr_start->ll_heads + list_idx;


    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Pre Alloc Small\n"
          "\tRounded Size       : %lu\n"
          "\tnvecs              : %d\n"
          "\tlist_idx           : %d\n"
          "\tlast_avails        : 0x%016lX\n"
          "\tHead               : %p\n" __FILE__,
          __FUNCTION__,
          __LINE__,
          rounded_size,
          nvecs,
          list_idx,
          last_avails,
          (*head));

    small_region_t * sr = NULL;

    // no blocks for this size class
    if ((*head) == 0) {

        PRINT(LOW_VERBOSE,
              "%s:%s:%d -> New Block\n"
              "\tsize               : %lu\n"
              "\tlist_idx           : %d\n\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              rounded_size,
              list_idx);


        sr = (small_region_t *)find_contig_region(0, 1);
        MM_DBG_ASSERT(sr);
        sr->region_size_and_ptrs = rounded_size / MM_MIN_SIZE;
        (*head)                  = sr;

        for (uint32_t i = 0; i < nvecs - 1; i++) {
            sr->vecs[i] = (~(0UL));
        }
        sr->vecs[nvecs - 1] = last_avails;
    }
    else {
        sr = (small_region_t *)((*head));
        PRINT(LOW_VERBOSE,
              "%s:%s:%d -> Got Block\n\n",
              __FILE__,
              __FUNCTION__,
              __LINE__);
    }


    uint64_t vec_idx;
    uint32_t i;

    PRINT(MED_VERBOSE,
          "%s:%s:%d -> Pre Process Small Regions\n"
          "\tSmall Region Ptr   : %p\n"
          "\tMeta Info          : 0x%016lX\n"
          "\tSize               : %lu\n"
          "\tlist idx           : %d\n"
          "\tNext Small Region  : %p\n"
          "\tPrev Small Region  : %p\n"
          "\tLast Avail Vec     : 0x%016lX\n"
          "\tVectors(%d) -> [\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          sr,
          sr->region_size_and_ptrs,
          MM_SR_GET_SIZE(sr->region_size_and_ptrs),
          list_idx,
          MM_SR_GET_NEXT(sr->region_size_and_ptrs),
          MM_SR_GET_PREV(sr->region_size_and_ptrs),
          last_avails,
          nvecs);

    for (uint32_t _i = 0; _i < nvecs; _i++) {
        PRINT(MED_VERBOSE, "\t\t0x%016lX\n", sr->vecs[_i]);
    }
    PRINT(MED_VERBOSE, "\t]\n\n");

    for (i = 0; (i < nvecs) && (sr->vecs[i] == 0); i++) {
    }

    MM_DBG_ASSERT(i != nvecs);

    ff1_asm_tz(sr->vecs[i], vec_idx);
    sr->vecs[i] ^= ((1UL) << vec_idx);

    // remove from linked list if filled up
    if (i == (nvecs - 1) && sr->vecs[i] == 0) {
        PRINT(MED_VERBOSE,
              "%s:%s:%d -> Small Region Filled\n\n",
              __FILE__,
              __FUNCTION__,
              __LINE__);

        remove_ll(head, sr);
    }


    PRINT(MED_VERBOSE,
          "%s:%s:%d -> Post Process Small Regions\n"
          "\tSmall Region Ptr   : %p\n"
          "\tMeta Info          : 0x%016lX\n"
          "\tSize               : %lu\n"
          "\tlist idx           : %d\n"
          "\tNext Small Region  : %p\n"
          "\tPrev Small Region  : %p\n"
          "\tLast Avail Vec     : 0x%016lX\n"
          "\ti                  : %d\n"
          "\tvec_idx            : %lu\n"
          "\tVectors(%d) -> [\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          sr,
          sr->region_size_and_ptrs,
          MM_SR_GET_SIZE(sr->region_size_and_ptrs),
          list_idx,
          MM_SR_GET_NEXT(sr->region_size_and_ptrs),
          MM_SR_GET_PREV(sr->region_size_and_ptrs),
          last_avails,
          i,
          vec_idx,
          nvecs);

    for (uint32_t _i = 0; _i < nvecs; _i++) {
        PRINT(MED_VERBOSE, "\t\t0x%016lX\n", sr->vecs[_i]);
    }
    PRINT(MED_VERBOSE, "\t]\n\n");
    MM_DBG_CHECK;


    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Alloc Return Addr\n"
          "\tAddress        : %p\n"
          "\tSR             : %p\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          (void *)(((uint64_t)sr) + sizeof(uint64_t) * ((nvecs | 0x1) + 1) +
                   (i * 64 + vec_idx) * rounded_size),
          sr);

    MM_DBG_ASSERT(((uint64_t)sr) % PAGE_SIZE == 0);
    MM_DBG_ASSERT((MM_SR_GET_SIZE(sr->region_size_and_ptrs)) < 0xffff &&
                  ((MM_SR_GET_SIZE(sr->region_size_and_ptrs)) % MM_MIN_SIZE) ==
                      0 &&
                  MM_SR_GET_SIZE(sr->region_size_and_ptrs) > 0);

    return (void *)(((uint64_t)sr) + sizeof(uint64_t) * ((nvecs | 0x1) + 1) +
                    (i * 64 + vec_idx) * rounded_size);
}

void
dealloc(void * addr) {
    if (!addr) {
        return;
    }

    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Dealloc Addr\n"
          "\tAddress           : %p\n"
          "\tstart             : %p\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          addr,
          addr_start);
    if ((((uint64_t)addr) % PAGE_SIZE) != 0) {
        return dealloc_small(addr);
    }

    uint32_t start_count = MM_DBG_CHECK;

    const uint64_t page_start = (word_t)(addr_start + 1);
    const uint64_t v_idx      = (((word_t)addr) - page_start) / PAGE_SIZE;

    const uint32_t l0_idx = (v_idx / (WBITS * WBITS)) & (WBITS - 1);
    const uint32_t l1_idx = (v_idx / WBITS) & (WBITS - 1);
    const uint32_t l2_idx = v_idx & (WBITS - 1);


    const uint32_t npages =
        get_size(addr_start->l2_size_vecs[WBITS * l0_idx + l1_idx], l2_idx);
    const word_t erase = (((SHIFT_HELPER) << npages) - 1);


    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Dealloc Pre Process\n"
          "\taddress            : %p\n"
          "\tnpages             : %d\n"
          "\terase              : " WORD_FORMAT
          "\n"
          "\tv_idx              : %lu\n"
          "\tl0_idx             : %d\n"
          "\tl1_idx             : %d\n"
          "\tl2_idx             : %d\n"
          "\tl0_vec             : " WORD_FORMAT
          "\n"
          "\tl1_vec             : " WORD_FORMAT
          "\n"
          "\tl2_vec             : " WORD_FORMAT
          "\n"
          "\tl2_vec_size        : " WORD_FORMAT "\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          addr,
          npages,
          WORD_PRINT(erase),
          v_idx,
          l0_idx,
          l1_idx,
          l2_idx,
          WORD_PRINT(addr_start->l0_vec),
          WORD_PRINT(addr_start->l1_vecs[l0_idx]),
          WORD_PRINT(addr_start->l2_vecs[WBITS * l0_idx + l1_idx]),
          WORD_PRINT(addr_start->l2_size_vecs[WBITS * l0_idx + l1_idx]));


    MM_DBG_ASSERT(npages + l2_idx <= WBITS);
    if (addr_start->l2_vecs[WBITS * l0_idx + l1_idx] == MM_L2_VEC_FULL) {
        if (addr_start->l1_vecs[l0_idx] == MM_L1_VEC_FULL) {
            MM_DBG_ASSERT(!(addr_start->l0_vec & ((SHIFT_HELPER) << l0_idx)));
            addr_start->l0_vec ^= ((SHIFT_HELPER) << l0_idx);
        }
        MM_DBG_ASSERT(addr_start->l1_vecs[l0_idx] & ((SHIFT_HELPER) << l1_idx));
        addr_start->l1_vecs[l0_idx] ^= ((SHIFT_HELPER) << l1_idx);
    }
    addr_start->l2_vecs[WBITS * l0_idx + l1_idx] ^= (erase << l2_idx);

    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Dealloc Post Process\n"
          "\taddress            : %p\n"
          "\tnpages             : %d\n"
          "\terase              : " WORD_FORMAT
          "\n"
          "\tv_idx              : %lu\n"
          "\tl0_idx             : %d\n"
          "\tl1_idx             : %d\n"
          "\tl2_idx             : %d\n"
          "\tl0_vec             : " WORD_FORMAT
          "\n"
          "\tl1_vec             : " WORD_FORMAT
          "\n"
          "\tl2_vec             : " WORD_FORMAT
          "\n"
          "\tl2_vec_size        : " WORD_FORMAT "\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          addr,
          npages,
          WORD_PRINT(erase),
          v_idx,
          l0_idx,
          l1_idx,
          l2_idx,
          WORD_PRINT(addr_start->l0_vec),
          WORD_PRINT(addr_start->l1_vecs[l0_idx]),
          WORD_PRINT(addr_start->l2_vecs[WBITS * l0_idx + l1_idx]),
          WORD_PRINT(addr_start->l2_size_vecs[WBITS * l0_idx + l1_idx]));

    uint32_t after_count = MM_DBG_CHECK;
    MM_DBG_VASSERT(start_count - npages == after_count,
                   "Invalid Count on Allocation\n"
                   "\tstart         : %d\n"
                   "\tpages         : %d\n"
                   "\tafter         : %d\n",
                   start_count,
                   npages,
                   after_count);
}

void *
alloc(const size_t size) {
    if (addr_start == NULL) {
        init();
    }
    if (!size) {
        return NULL;
    }
    if (size > (WBITS * PAGE_SIZE)) {
        assert(0);
    }
    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Alloc(%lu)\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          size);


    if (size <= MM_SMALL_REGION_MIN) {
        return alloc_small(((size + MM_MIN_SIZE - 1) / MM_MIN_SIZE) *
                           MM_MIN_SIZE);
    }
    uint32_t start_count = MM_DBG_CHECK;

    uint32_t npages = (size + PAGE_SIZE - 1) / PAGE_SIZE;
    uint32_t max_npage_bit;
    mm_fl1_asm_lz(npages, max_npage_bit);
    max_npage_bit = (1 << (31 - max_npage_bit));
    npages ^= (max_npage_bit);

    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Page Allocate\n"
          "\tsize               : %lu\n"
          "\tnpages             : %d\n"
          "\tmax_npage_bit      : %d\n"
          "\tTotal Pages        : %d\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          size,
          npages,
          max_npage_bit,
          npages | (max_npage_bit));

    void * const ret = (void *)find_contig_region(npages, max_npage_bit);

    uint32_t after_count = MM_DBG_CHECK;
    MM_DBG_VASSERT(
        (start_count + (size + PAGE_SIZE - 1) / PAGE_SIZE) == after_count,
        "Invalid Count on Allocation\n"
        "\tstart            : %d\n"
        "\tsize             : %lu\n"
        "\tpages            : %lu\n"
        "\tafter            : %d\n",
        start_count,
        size,
        (size + PAGE_SIZE - 1) / PAGE_SIZE,
        after_count);

    return ret;
}

uint32_t
dbg_count_active_pages(uint32_t ln) {

    uint32_t       total_pages = 0;
    const word_t   l0_vec      = addr_start->l0_vec;
    word_t * const l1_vecs     = addr_start->l1_vecs;
    word_t * const l2_vecs     = addr_start->l2_vecs;

    for (uint32_t i = 0; i < MM_N_SMALL_REGIONS; i++) {
        if (addr_start->ll_heads[i] == 0) {
            continue;
        }
        small_region_t * temp = addr_start->ll_heads[i];
        MM_DBG_VASSERT(temp,
                       "Head and Tail Misaligned: %d\n"
                       "\ti                 : %d\n",
                       ln,
                       i);


        MM_DBG_ASSERT(MM_SR_GET_PREV(temp->region_size_and_ptrs) == 0);

        const uint32_t size     = MM_SR_GET_SIZE(temp->region_size_and_ptrs);
        const uint32_t list_idx = (size / MM_MIN_SIZE) - 1;

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

            for (uint32_t _i = 0; _i < prev_idx; _i++) {
                MM_DBG_ASSERT(temp != prev_ptrs[_i]);
            }
            prev_ptrs[prev_idx++] = temp;

            MM_DBG_ASSERT(MM_SR_GET_SIZE(temp->region_size_and_ptrs) == size);
            if (MM_SR_GET_NEXT(temp->region_size_and_ptrs) != 0) {
                MM_DBG_ASSERT(
                    MM_SR_GET_PREV(MM_SR_GET_NEXT(temp->region_size_and_ptrs)
                                       ->region_size_and_ptrs) == temp);
            }
            if (MM_SR_GET_PREV(temp->region_size_and_ptrs) == 0) {
                MM_DBG_ASSERT(addr_start->ll_heads[i] == temp);
            }
            else {
                MM_DBG_ASSERT(
                    MM_SR_GET_NEXT(MM_SR_GET_PREV(temp->region_size_and_ptrs)
                                       ->region_size_and_ptrs) == temp);
            }
            temp = MM_SR_GET_NEXT(temp->region_size_and_ptrs);
        }
    }

    for (uint32_t i = 0; i < MM_N_L2_VECS; i++) {
        if (l2_vecs[i] == MM_L2_VEC_FULL) {
            MM_DBG_VASSERT(
                l1_vecs[(i / WBITS)] & (SHIFT_HELPER << (i & (WBITS - 1))),
                "Error: Invalid L1 for a full L2: %d\n",
                ln);
        }
        else {
            MM_DBG_VASSERT(
                !(l1_vecs[(i / WBITS)] & (SHIFT_HELPER << (i & (WBITS - 1)))),
                "Error: Invalid L1 for a partial L2: %d\n",
                ln);
        }
        if (l1_vecs[(i / WBITS)] == MM_L1_VEC_FULL) {
            MM_DBG_VASSERT(!(l0_vec & (SHIFT_HELPER << (i / WBITS))),
                           "Error: Invalid L0 for a full L1: %d\n",
                           ln);
        }
        else {
            MM_DBG_VASSERT((l0_vec & (SHIFT_HELPER << (i / WBITS))),
                           "Error: Invalid L0 for a partial L1: %d\n"
                           "\tl0_vec            : " WORD_FORMAT
                           "\n"
                           "\tl1_vec            : " WORD_FORMAT
                           "\n"
                           "\ti                 : %d\n"
                           "\ti_l1              : %d\n"
                           "\ti_l0              : %d\n\n",
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
}
