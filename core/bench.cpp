#include <string.h>
#include "base/all.hpp"
#include "numa_utils.hpp"

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/barrier.hpp>

#define MIN_BUFFER_SIZE (1024 * 1024 * 1024)
#define BUFFER_SIZE (1024 * 1024 * 1024 + 4096)
unsigned long buffer_items = 1;
#define LCG_NEXT(_n, mask) ((1103515245*(_n) + 12345) & mask)

using namespace base;

static double convert(double mbps) {
  double factor = (1 << 20) / ((double)1000000.0);
  return factor * mbps;
}

enum AccessPattern{
  SEQ,
  RAND,
};


class mem_speed {
  int cpu;
  //pthread_barrier_t *thread_sync;
  boost::barrier *thread_sync;
  unsigned char *local_buffer;
  unsigned char *remote_buffer;
  unsigned long chunk_size;
  enum AccessPattern ap;

  public : 
  
  mem_speed(int cpu, unsigned char *local_buffer,
            unsigned char *remote_buffer, unsigned long chunk_size,
            boost::barrier *thread_sync, enum AccessPattern ap) :
    cpu(cpu), local_buffer(local_buffer), 
    remote_buffer(remote_buffer), chunk_size(chunk_size),
    thread_sync(thread_sync), ap(ap)
  {
    pin_to_node(cpu);
  }


  void do_seq_work() {
    unsigned char tmp[chunk_size];
    // seq  local read
    //int rc = pthread_barrier_wait(thread_sync);
    //verify(rc != 0 && rc != PTHREAD_BARRIER_SERIAL_THREAD) ;
  
    thread_sync->wait();

    Timer timer;
    timer.start();
    for (unsigned long i = 0;i < BUFFER_SIZE;i += chunk_size) {
      memcpy(tmp, local_buffer + i, chunk_size);
    }
    timer.stop();
    double mbps = (double)(BUFFER_SIZE) / timer.elapsed_usec();
    LOG_INFO << "MEM_LOCAL_READ_SPPED " << convert(mbps) << "MB/s" ;

    // seq remote read
    timer.reset();
    timer.start();
    for (unsigned long i = 0;i < BUFFER_SIZE;i += chunk_size) {
      memcpy(tmp, remote_buffer + i, chunk_size);
    }
    timer.stop();
    mbps = (double)(BUFFER_SIZE) / timer.elapsed_usec();
    LOG_INFO << "MEM_REMOTE_READ_SPPED " << convert(mbps) << "MB/s" ;

    // seq local write
    timer.reset();
    timer.start();
    for (unsigned long i = 0;i < BUFFER_SIZE;i += chunk_size) {
      memcpy(local_buffer + i, tmp, chunk_size);
    }
    timer.stop();
    mbps = (double)(BUFFER_SIZE) / timer.elapsed_usec();
    LOG_INFO << "MEM_LOCAL_WRITE_SPPED " << convert(mbps) << "MB/s" ;

    // seq remote write
    timer.reset();
    timer.start();
    for (unsigned long i = 0;i < BUFFER_SIZE;i += chunk_size) {
      memcpy(remote_buffer + i, tmp, chunk_size);
    }
    timer.stop();
    mbps = (double)(BUFFER_SIZE) / timer.elapsed_usec();
    LOG_INFO << "MEM_REMOTE_WRITE_SPPED " << convert(mbps) << "MB/s" ;
  }


  void do_rand_work() {
    unsigned char tmp[chunk_size];
    const unsigned long mask = buffer_items - 1;

    //int rc = pthread_barrier_wait(thread_sync);
    //verify(rc != 0 && rc != PTHREAD_BARRIER_SERIAL_THREAD) ;
    thread_sync->wait();

    Timer timer;
    // rand local read
    timer.start();
    for (unsigned long i = 0, j = 0;i < buffer_items; i++) {
      memcpy(tmp, local_buffer + j * chunk_size, chunk_size);
      j = LCG_NEXT(j, mask);
    }
    timer.stop();
    double mbps = ((double)buffer_items * chunk_size) / timer.elapsed_usec();
    LOG_INFO << "MEM_LOCAL_READ_SPEED " << convert(mbps) << "MB/s";

    // rand remote read
    timer.reset();
    timer.start();
    for (unsigned long i = 0, j = 0;i < buffer_items; i++) {
      memcpy(tmp, remote_buffer + j * chunk_size, chunk_size);
      j = LCG_NEXT(j, mask);
    }
    timer.stop();
    mbps = ((double)buffer_items * chunk_size) / timer.elapsed_usec();
    LOG_INFO << "MEM_REMOTE_READ_SPEED " << convert(mbps) << "MB/s";
    
    // rand local write
    timer.reset();
    timer.start();
    for (unsigned long i = 0, j = 0;i < buffer_items; i++) {
      memcpy(local_buffer + j * chunk_size, tmp, chunk_size);
      j = LCG_NEXT(j, mask);
    }
    timer.stop();
    mbps = ((double)buffer_items * chunk_size) / timer.elapsed_usec();
    LOG_INFO << "MEM_LOCAL_WRITE_SPEED " << convert(mbps) << "MB/s";

    // rand remote write
    timer.reset();
    timer.start();
    for (unsigned long i = 0, j = 0;i < buffer_items; i++) {
      memcpy(remote_buffer + j * chunk_size, tmp, chunk_size);
      j = LCG_NEXT(j, mask);
    }
    timer.stop();
    mbps = ((double)buffer_items * chunk_size) / timer.elapsed_usec();
    LOG_INFO << "MEM_REMOTE_WRITE_SPEED " << convert(mbps) << "MB/s";
  }
  

  void operator() () {
    if (ap == SEQ) {
      do_seq_work();
    } else if (ap == RAND) {
      do_rand_work();
    }
  }
};





int main(int argc, char **argv) {

  // check numa
  if (numa_available() < 0) {
    std::cerr << "numa not available" << std::endl;
    return 1;
  }

  numa_set_strict(1);
  int ncpus = numa_num_task_cpus();
  verify(ncpus >= 1);
  LOG_INFO << "ncpus : " << ncpus ;
  int nnodes = numa_num_configured_nodes();
  verify(nnodes >= 1);
  LOG_INFO << "nnodes : " << nnodes;


  int local_node = 0, remote_node = 1;
  int local_cpu = 1;

  unsigned long threads, chunk_size;
  enum AccessPattern ap;

  if (argc < 4) {
    std::cerr << "Usage " << argv[0] << " chunk_size threads ap " << std::endl;
    std::cerr << "   ap : seq ,rand " << std::endl;
    exit(-1);
  }
  
  chunk_size = atol(argv[1]);
  threads = atol(argv[2]);
  if (strcmp("seq", argv[3]) == 0) {
    ap = SEQ;
  } else if (strcmp("rand", argv[3]) == 0) {
    ap = RAND;
  } else {
    std::cerr << " ap shold be seq or rand " << std::endl;
    exit(-1);
  }


  //pthread_t  thread_array [threads];
  boost::thread **  thread_array =  new boost::thread * [threads];
  mem_speed **obj = new mem_speed *[threads];
 
  //pthread_barrier_t thread_sync;
  //verify(pthread_barrier_init(&thread_sync, NULL, threads)); 

  boost::barrier * thread_sync = new boost::barrier(threads);

  unsigned long alloc_bytes = BUFFER_SIZE;
  if ( ap == RAND ) {
    while (buffer_items * chunk_size < MIN_BUFFER_SIZE ) {
      buffer_items = buffer_items  * 2;
    }
    alloc_bytes = buffer_items * chunk_size;
  }

  for (unsigned long i = 0;i < threads; i++) {
    // allocate local buffer and remote buffer
    unsigned char *l_buffer = (unsigned char *)numa_alloc_node(alloc_bytes, local_node);
    unsigned char *r_buffer = (unsigned char *)numa_alloc_node(alloc_bytes, remote_node);

    //obj[i] = new mem_speed(local_cpu, l_buffer, r_buffer, chunk_size, &thread_sync, ap);
    obj[i] = new mem_speed(local_cpu, l_buffer, r_buffer, chunk_size, thread_sync, ap);
    //Pthread_create(thread_array[i], NULL, std::ref(*obj[i]), NULL);
    thread_array[i] = new boost::thread(boost::ref(*obj[i]));
  }


  for (unsigned long i = 0;i < threads; i++ ) {
    //Pthread_join(thread_array[i], NULL);
    thread_array[i]->join();
  }

  return 0;
}





