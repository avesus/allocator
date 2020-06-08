#include <allocator/allocator.h>

typedef __int128          int128_t;
typedef unsigned __int128 uint128_t;

typedef union cast_128 {
    uint128_t u128;
    struct {
        uint64_t u64[2];
    };
    struct {
        uint32_t u32[4];
    };

} cast_128_t;

typedef uint128_t word_t;
#define WBITS (128)

#if WBITS == 128
#define WORD_FORMAT   "0x%016lX %016lX"

uint64_t get_uint64(uint128_t input, uint32_t w_idx) {
    assert(w_idx == 0 || w_idx == 1);
    cast_128_t temp = {.u128 = input};
    return temp.u64[w_idx];
}
#define WORD_PRINT(X) get_uint64(X, 0), get_uint64(X, 1)

const word_t SHIFT_HELPER = 1;
#define X_TEMP MACRO_COMBINE(_X, ln)
#define Y_TEMP MACRO_COMBINE(_Y, ln)

#define mm_ff0_asm_tz(X, Y) _mm_ff0_asm_tz(X, Y, __LINE__)
#define mm_ff1_asm_tz(X, Y) _mm_ff1_asm_tz(X, Y, __LINE__)
#define mm_fl1_asm_lz(X, Y) fl1_asm_lz(X, Y)

#define _mm_ff0_asm_tz(X, Y, ln)                                               \
    {                                                                          \
        const cast_128 X_TEMP = { .u128 = (X) };                               \
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
        const cast_128 X_TEMP = { .u128 = (X) };                               \
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

#define mm_ff0_asm_tz(X, Y) ff0_asm_tz(X, Y)
#define mm_ff1_asm_tz(X, Y) ff1_asm_tz(X, Y)
#define mm_fl1_asm_lz(X, Y) fl1_asm_lz(X, Y)
#endif


#define MM_DEFAULT_LENGTH    ((1UL) << 30)
#define MM_DEFAULT_BASE_ADDR 0

// -2 is because we are skipping PAGE_SIZE and PAGE_SIZE / 2
#define MM_N_SMALL_REGIONS       ((PAGE_SIZE / MM_ALIGNMENT) - 2)
#define MM_BOOK_KEEP_REGION_SIZE (PAGE_SIZE * PAGE_SIZE)

#define MM_MIN_SIZE     (16)
#define MM_LOG_MIN_SIZE (4)

#define MM_N_L1_VECS ((MM_DEFAULT_LENGTH / ((WBITS)*PAGE_SIZE)) / WBITS)

#define MM_N_L2_VECS (MM_DEFAULT_LENGTH / ((WBITS)*PAGE_SIZE))


#if WBITS == 128
const uint128_t l1_full = (-1);
const uint128_t l2_full = 0;

#define MM_L1_VEC_FULL l1_full
#define MM_L2_VEC_FULL l2_full
#else
#define MM_L1_VEC_FULL (~(0UL))
#define MM_L2_VEC_FULL (0UL)
#endif


typedef struct region_head {
    word_t l0_vec;
    word_t l1_vecs[MM_N_L1_VECS];
    // bit vec of free pages
    word_t l2_vecs[MM_N_L2_VECS];
    word_t l2_size_vecs[MM_N_L2_VECS];

    uint32_t ll_heads[MM_N_SMALL_REGIONS];
    uint32_t ll_tails[MM_N_SMALL_REGIONS];

} region_head_t;


typedef struct small_region {
    uint32_t region_size;
    uint32_t next_region_page;
    word_t   avail_vecs[0];
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
    addr_start->l0_vec = ~(0UL);
    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Init New Address Heap\n"
          "\tStart  : %p\n"
          "\tEnd    : %p\n"
          "\tSize   : %lu\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          addr_start,
          ((uint8_t *)addr_start) + MM_DEFAULT_LENGTH,
          MM_DEFAULT_LENGTH);
}
//////////////////////////////////////////////////////////////////////
static inline uint32_t
get_size(const word_t l2_size_vec, const uint32_t l2_idx) {

    uint64_t size;
    mm_ff0_asm_tz(l2_size_vec >> l2_idx, size);

    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Getting Size\n"
          "\tVec            : " WORD_FORMAT
          "\n"
          "\tIdx            : %d\n"
          "\tSize           : %lu\n\n",
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
          "\tl1_vec         : " WORD_FORMAT
          "\n"
          "\tl2_vecs_addr   : %p\n"
          "\tnpages         : %d\n"
          "\tmax_npage_bit  : %d\n"
          "\tTotal Pages    : %d\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          WORD_PRINT(l1_vec),
          l2_vecs,
          npages,
          max_npage_bit,
          npages | (1 << max_npage_bit));


    uint64_t l2_idx, ret_idx;

    while (l1_vec) {
        mm_ff1_asm_tz(l1_vec, l2_idx);
        l1_vec ^= ((SHIFT_HELPER) << l2_idx);


        assert(l2_vecs[l2_idx] != MM_L2_VEC_FULL);

        word_t temp_l2_vec = l2_vecs[l2_idx] & (l2_vecs[l2_idx] >> npages);

        PRINT(HIGH_VERBOSE,
              "%s:%s:%d -> L1 Loop\n"
              "\tCur l1_vec     : " WORD_FORMAT
              "\n"
              "\tl2_idx         : %lu\n"
              "\tl2_vec         : " WORD_FORMAT "\n\n",
              __FILE__,
              __FUNCTION__,
              __LINE__,
              WORD_PRINT(l1_vec),
              l2_idx,
              WORD_PRINT(temp_l2_vec));

        uint32_t max_npage_bit_shift = (1 << (max_npage_bit)) / 2;

        while (max_npage_bit_shift > 0 && temp_l2_vec) {

            PRINT(HIGH_VERBOSE,
                  "%s:%s:%d -> L2 Loop\n"
                  "\tl2_vec         : " WORD_FORMAT
                  "\n"
                  "\tdownshift      : %d\n\n",
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
                  "\tl2_idx         : %lu\n"
                  "\tret_idx        : %lu\n"
                  "\tret            : %16lX\n\n",
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

    const word_t erase_mask =
        (SHIFT_HELPER << (npages | (SHIFT_HELPER << max_npage_bit))) - 1;
    const word_t size_mask = (erase_mask) >> 1;

    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Find Contig\n"
          "\terase_mask     : " WORD_FORMAT
          "\n"
          "\tsize_mask      : " WORD_FORMAT
          "\n"
          "\tnpages         : %d\n"
          "\tmax_npage_bit  : %d\n"
          "\tTotal Pages    : %d\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          WORD_PRINT(erase_mask),
          WORD_PRINT(size_mask),
          npages,
          max_npage_bit,
          npages | (1 << max_npage_bit));

    uint64_t i;
    while (l0_vec) {
        mm_ff1_asm_tz(l0_vec, i);
        l0_vec ^= ((SHIFT_HELPER) << i);

        PRINT(MED_VERBOSE,
              "%s:%s:%d -> L0 Loop\n"
              "\tl0_vec         : " WORD_FORMAT
              "\n"
              "\ti              : %lu\n"
              "\tl1_vec         : " WORD_FORMAT "\n\n",
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
                      "\tl1_idx         : %d\n"
                      "\tl2_idx         : %d\n"
                      "\tCompute Idx    : %lu\n"
                      "\tl0_vec         : " WORD_FORMAT
                      "\n"
                      "\tl1_vec         : " WORD_FORMAT
                      "\n"
                      "\tl2_vec         : " WORD_FORMAT
                      "\n"
                      "\tl2_size        : " WORD_FORMAT "\n\n",
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

#if WBITS == 64
                assert(bitcount_64((erase_mask << l2_idx)) ==
                       bitcount_64(erase_mask));
#endif
                assert(
                    (l2_vecs[WBITS * i + l1_idx] ^ (erase_mask << l2_idx)) ==
                    (l2_vecs[WBITS * i + l1_idx] & (~(erase_mask << l2_idx))));

                l2_vecs[WBITS * i + l1_idx] ^= (erase_mask << l2_idx);
                l2_size_vecs[WBITS * i + l1_idx] &=
                    (((~erase_mask) << l2_idx) |
                     ((SHIFT_HELPER << l2_idx) - 1));


                l2_size_vecs[WBITS * i + l1_idx] |= (size_mask) << l2_idx;

                assert(get_size(l2_size_vecs[WBITS * i + l1_idx], l2_idx) ==
                       (npages | (SHIFT_HELPER << max_npage_bit)));


                if (!l2_vecs[WBITS * i + l1_idx]) {
                    l1_vecs[i] ^= ((SHIFT_HELPER) << l1_idx);
                    if (l1_vecs[i] == MM_L1_VEC_FULL) {
                        addr_start->l0_vec ^= ((SHIFT_HELPER) << i);
                    }
                }

                PRINT(MED_VERBOSE,
                      "%s:%s:%d -> Got Return Post Process\n"
                      "\tl1_idx         : %d\n"
                      "\tl2_idx         : %d\n"
                      "\tCompute Idx    : %lu\n"
                      "\tl0_vec         : " WORD_FORMAT
                      "\n"
                      "\tl1_vec         : " WORD_FORMAT
                      "\n"
                      "\tl2_vec         : " WORD_FORMAT
                      "\n"
                      "\tl2_size        : " WORD_FORMAT "\n\n",
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
                const uint32_t _l0_idx = (v_idx / (WBITS * WBITS)) & (WBITS - 1);
                const uint32_t _l1_idx = (v_idx / WBITS) & (WBITS - 1);
                const uint32_t _l2_idx = v_idx & (WBITS - 1);

                assert(_l0_idx == i);
                assert(_l1_idx == l1_idx);
                assert(_l2_idx == l2_idx);


                return ((uint64_t)(addr_start + 1)) +
                       PAGE_SIZE *
                           (WBITS * WBITS * i + WBITS * l1_idx + l2_idx);
            }
        }
    }
    DBG_ASSERT(0, "Out Of Memory!")
}
void
dealloc(void * addr) {
    if (!addr) {
        return;
    }
    const uint64_t page_start = (word_t)(addr_start + 1);
    const uint64_t v_idx      = (((word_t)addr) - page_start) / PAGE_SIZE;

    const uint32_t l0_idx = (v_idx / (WBITS * WBITS)) & (WBITS - 1);
    const uint32_t l1_idx = (v_idx / WBITS) & (WBITS - 1);
    const uint32_t l2_idx = v_idx & (WBITS - 1);


    const uint32_t npages = get_size(
        addr_start->l2_size_vecs[WBITS * l0_idx + l1_idx],
        l2_idx);
    const word_t erase = (((SHIFT_HELPER) << npages) - 1) << v_idx;


    PRINT(
        LOW_VERBOSE,
        "%s:%s:%d -> Dealloc Pre Process\n"
        "\taddress        : %p\n"
        "\tv_idx          : %lu\n"
        "\tl0_idx         : %d\n"
        "\tl1_idx         : %d\n"
        "\tl2_idx         : %d\n"
        "\tl0_vec         : " WORD_FORMAT
        "\n"
        "\tl1_vec         : " WORD_FORMAT
        "\n"
        "\tl2_vec         : " WORD_FORMAT
        "\n"
        "\tl2_vec_size    : " WORD_FORMAT "\n\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        addr,
        v_idx,
        l0_idx,
        l1_idx,
        l2_idx,
        WORD_PRINT(addr_start->l0_vec),
        WORD_PRINT(addr_start->l1_vecs[l0_idx]),
        WORD_PRINT(
            addr_start->l2_vecs[WBITS * l0_idx + l1_idx]),
        WORD_PRINT(
            addr_start->l2_size_vecs[WBITS * l0_idx + l1_idx]));


    assert(npages + l2_idx <= WBITS);
    if (!addr_start->l2_vecs[WBITS * l0_idx + l1_idx]) {
        if (!addr_start->l1_vecs[l0_idx]) {
            addr_start->l0_vec ^= ((SHIFT_HELPER) << l0_idx);
        }
        addr_start->l1_vecs[l0_idx] ^= ((SHIFT_HELPER) << l1_idx);
    }
    addr_start->l2_vecs[WBITS * l0_idx + l1_idx] ^= (erase << l2_idx);

    PRINT(
        LOW_VERBOSE,
        "%s:%s:%d -> Dealloc Post Process\n"
        "\taddress        : %p\n"
        "\tv_idx          : %lu\n"
        "\tl0_idx         : %d\n"
        "\tl1_idx         : %d\n"
        "\tl2_idx         : %d\n"
        "\tl0_vec         : " WORD_FORMAT
        "\n"
        "\tl1_vec         : " WORD_FORMAT
        "\n"
        "\tl2_vec         : " WORD_FORMAT
        "\n"
        "\tl2_vec_size    : " WORD_FORMAT "\n\n",
        __FILE__,
        __FUNCTION__,
        __LINE__,
        addr,
        v_idx,
        l0_idx,
        l1_idx,
        l2_idx,
        WORD_PRINT(addr_start->l0_vec),
        WORD_PRINT(addr_start->l1_vecs[l0_idx]),
        WORD_PRINT(
            addr_start->l2_vecs[WBITS * l0_idx + l1_idx]),
        WORD_PRINT(
            addr_start->l2_size_vecs[WBITS * l0_idx + l1_idx]));
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


    uint32_t npages = (size + PAGE_SIZE - 1) / PAGE_SIZE;
    uint32_t max_npage_bit;
    mm_fl1_asm_lz(npages, max_npage_bit);
    max_npage_bit = 31 - max_npage_bit;
    npages ^= (1 << max_npage_bit);

    PRINT(LOW_VERBOSE,
          "%s:%s:%d -> Allocate\n"
          "\tsize           : %lu\n"
          "\tnpages         : %d\n"
          "\tmax_npage_bit  : %d\n"
          "\tTotal Pages    : %d\n\n",
          __FILE__,
          __FUNCTION__,
          __LINE__,
          size,
          npages,
          max_npage_bit,
          npages | (1 << max_npage_bit));

    return (void *)find_contig_region(npages, max_npage_bit);
}
