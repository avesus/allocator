#ifndef _ALLOCATOR_H_
#define _ALLOCATOR_H_

#include <helpers/opt.h>
#include <helpers/util.h>

#define MM_ALIGNMENT (16)

void * alloc(const size_t size);
void   dealloc(void * addr);

uint32_t dbg_count_active_pages(uint32_t ln);
#endif
