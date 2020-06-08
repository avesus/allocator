#ifndef _ALLOCATOR_H_
#define _ALLOCATOR_H_

#include <helpers/opt.h>
#include <helpers/util.h>

#define MM_ALIGNMENT (16)

void * alloc(const uint64_t size);
void   dealloc(void * addr);
#endif
