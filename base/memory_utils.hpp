#pragma once 

#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>

#include "basetypes.hpp"

namespace base {

/* malloc extreme lagge memory */
void *map_anon_memory(unsigned long size, bool mlocked, const char *operation, bool zero = false);



} //namespace base

