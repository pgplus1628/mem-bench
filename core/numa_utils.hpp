#include <numa.h>
#include <sched.h>

#include "base/all.hpp"

using namespace base;

void * numa_alloc_node(size_t bytes, int node) {
  void *p = numa_alloc_onnode(bytes, node);
  verify(p != NULL);

  // force to alloc physical memory on this region
  memset(p, 0, bytes);
  return p;
}

void * numa_alloc_cpu(size_t bytes, int cpu) {
  int node = numa_node_of_cpu(cpu);
  return numa_alloc_node(bytes, node);
}


void pin_to_node(int cpu) {
  int node = numa_node_of_cpu(cpu);
  int ret = numa_run_on_node(node);
  verify(ret == 0);

  ret = sched_yield();
  verify(ret == 0);
}


