#ifndef _ALLOCATOR_H_
#define _ALLOCATOR_H_

#include <helpers/opt.h>
#include <helpers/util.h>

#define MM_ALIGNMENT (16)

void   dealloc(void * const addr);
void * alloc(const size_t size);

uint32_t addr_get_region_size(void * const addr);
uint32_t check_heap(const char * const f,
                    const char * const fn,
                    const uint32_t     ln);

#endif
