#define _GNU_SOURCE
#include <jni.h>
#include <time.h>
#include <arpa/inet.h>
#include <linux/net_tstamp.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define MAX_QUEUE_SIZE 32
#define MASK (MAX_QUEUE_SIZE - 1)
#define RECIEVE_BUF_SIZE 1048576
#define RECV_BATCH 8
#define WORKER_CPU 1
#define CACHE_LINE 64

// Branch prediction hints
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

// Structs that are given from our rust code (Packed to match Java)
typedef struct __attribute__((packed))
{
  double x, y, rot;
} RobotPos;

typedef struct __attribute__((packed))
{
  double x, y, rot;
} VisionUncertainty;

typedef struct __attribute__((packed))
{
  RobotPos pose;
  VisionUncertainty stds;
  uint64_t ts;
  uint8_t camera_id;
  uint8_t num_tags;
  uint8_t padding[6];
} VisionMeasurement;

// Cache-line-padded ring buffer (prevents false sharing)
typedef struct
{
  volatile VisionMeasurement data[MAX_QUEUE_SIZE];

  // Written by Worker
  atomic_int head __attribute__((aligned(CACHE_LINE)));
  char _pad_head[CACHE_LINE - sizeof(atomic_int)];

  // Read by Main
  atomic_int tail __attribute__((aligned(CACHE_LINE)));
  char _pad_tail[CACHE_LINE - sizeof(atomic_int)];
} LockFreeQueue;

// Global states
static LockFreeQueue vq = {.head = 0, .tail = 0};

// Broadcast globals
static int broadcast_fd = -1;
static struct sockaddr_in broadcast_addr;

// Helper function to get the Monotonic micros
static inline uint64_t get_monotonic_micros(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

// Extract SO_TIMESTAMPNS from cmsg ancillary data
static inline uint64_t ts_from_cmsg(struct msghdr *msg)
{
  for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(msg);
       cmsg != NULL;
       cmsg = CMSG_NXTHDR(msg, cmsg))
  {
    if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_TIMESTAMPNS)
    {
      struct timespec *ts = (struct timespec *)CMSG_DATA(cmsg);
      return (uint64_t)ts->tv_sec * 1000000ULL + (uint64_t)ts->tv_nsec / 1000ULL;
    }
  }
  // Fallback (should not happen)
  return get_monotonic_micros();
}

// Worker thread for recieving cam updates and pushing them to the queue
static void *vision_worker_thread(void *arg)
{
  int listenfd = *(int *)arg;
  free(arg);

  pthread_setname_np(pthread_self(), "VisionUDPRecv");

  // Pin to WORKER_CPU (keeps L1 cache hot)
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(WORKER_CPU, &cpuset);
  if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0)
    perror("[VisionNative-c] Warning: CPU affinity failed");

  // Elevate to SCHED_FIFO real-time priority (preempts JVM)
  struct sched_param sp = {.sched_priority = 50};
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
    perror("[VisionNative-c] Warning: SCHED_FIFO failed (need RT permissions)");

  // Hoist all recvmmsg structures out of the hot loop
  VisionMeasurement recv_bufs[RECV_BATCH];
  struct iovec iovecs[RECV_BATCH];
  struct mmsghdr msgs[RECV_BATCH];

  // Control message buffers for SO_TIMESTAMPNS (one per message)
  char ctrl_bufs[RECV_BATCH][CMSG_SPACE(sizeof(struct timespec))];

  memset(msgs, 0, sizeof(msgs));
  for (int i = 0; i < RECV_BATCH; i++)
  {
    iovecs[i].iov_base = &recv_bufs[i];
    iovecs[i].iov_len  = sizeof(VisionMeasurement);

    msgs[i].msg_hdr.msg_iov     = &iovecs[i];
    msgs[i].msg_hdr.msg_iovlen  = 1;
    msgs[i].msg_hdr.msg_control    = ctrl_bufs[i];
    msgs[i].msg_hdr.msg_controllen = sizeof(ctrl_bufs[i]);
  }

  while (1)
  {
    // Reset controllen before each call
    for (int i = 0; i < RECV_BATCH; i++)
      msgs[i].msg_hdr.msg_controllen = sizeof(ctrl_bufs[i]);

    // Block on first packet, return up to RECV_BATCH at once
    int n = recvmmsg(listenfd, msgs, RECV_BATCH, MSG_WAITFORONE, NULL);
    if (unlikely(n <= 0))
      continue;

    for (int i = 0; i < n; i++)
    {
      if (unlikely(msgs[i].msg_len != sizeof(VisionMeasurement)))
      {
        printf("[VisionNative-c] Warning: Received packet of unexpected size %u\n",
               msgs[i].msg_len);
        continue;
      }

      // Get kernel-level adapter timestamp
      uint64_t adapter_us = ts_from_cmsg(&msgs[i].msg_hdr);

      // Calculate absolute Monotonic timestamp from delay
      VisionMeasurement *pkt = &recv_bufs[i];
      uint64_t abs_ts = adapter_us - pkt->ts;

      // Load current indices
      int h = atomic_load_explicit(&vq.head, memory_order_relaxed);
      int t = atomic_load_explicit(&vq.tail, memory_order_acquire);
      int next_h = (h + 1) & MASK;

      // If queue is full, we push the tail forward to overwrite oldest
      if (unlikely(next_h == t))
      {
        atomic_store_explicit(&vq.tail, (t + 1) & MASK, memory_order_release);
        printf("[VisionNative-c] Warning: Queue full, dropping oldest packet\n");
      }

      // Write the data to the buffer
      pkt->ts = abs_ts;
      vq.data[h] = *pkt;
      atomic_store_explicit(&vq.head, next_h, memory_order_release);
    }
  }
  return NULL;
}

// --- JNI EXPORTS ---

JNIEXPORT void JNICALL Java_frc_robot_util_VisionNative_startServer(JNIEnv *env, jclass cls, jint port)
{
  int listenfd;
  struct sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof(servaddr));

  // Create a UDP Socket
  listenfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (listenfd < 0)
  {
    perror("[VisionNative-c] Socket creation failed");
    return;
  }

  // Set socket options
  int rcvbuf = RECIEVE_BUF_SIZE;
  setsockopt(listenfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
  int reuse = 1;
  setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  // Enable kernel-level nanosecond timestamps on received packets
  int ts_on = 1;
  if (setsockopt(listenfd, SOL_SOCKET, SO_TIMESTAMPNS, &ts_on, sizeof(ts_on)) < 0)
    perror("[VisionNative-c] Warning: SO_TIMESTAMPNS failed");

  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(port);
  servaddr.sin_family = AF_INET;

  // Bind server address to socket descriptor
  if (bind(listenfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1)
  {
    perror("[VisionNative-c] Bind failed");
    close(listenfd);
    return;
  }
  printf("[VisionNative-c] Ready to receive on port %d\n", port);

  // Use malloc so the FD pointer persists for the thread
  int *arg = malloc(sizeof(int));
  *arg = listenfd;

  // Create background thread with RT priority
  pthread_t thread_id;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  struct sched_param sp = {.sched_priority = 50};
  pthread_attr_setschedparam(&attr, &sp);

  if (pthread_create(&thread_id, &attr, vision_worker_thread, arg) != 0)
  {
    // Fallback to normal thread if RT creation fails
    printf("[VisionNative-c] RT thread creation failed, falling back to normal thread\n");
    pthread_create(&thread_id, NULL, vision_worker_thread, arg);
  }
  pthread_attr_destroy(&attr);

  // Initialize broadcast socket
  if (broadcast_fd != -1) return;
  printf("[VisionNative-c] Initializing broadcast socket\n");

  int b_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (b_fd < 0)
  {
    perror("[VisionNative-c] Broadcast socket creation failed");
    return;
  }

  int broadcast = 1;
  int b_reuse = 1;
  setsockopt(b_fd, SOL_SOCKET, SO_REUSEADDR, &b_reuse, sizeof(b_reuse));
  if (setsockopt(b_fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
  {
    perror("[VisionNative-c] Error setting broadcast permission");
    close(b_fd);
    return;
  }

  memset(&broadcast_addr, 0, sizeof(broadcast_addr));
  broadcast_addr.sin_family = AF_INET;
  broadcast_addr.sin_port = htons(7002);
  broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  broadcast_fd = b_fd; // Only set global once fully ready
}

JNIEXPORT void JNICALL Java_frc_robot_util_VisionNative_broadcastRobotHeading(JNIEnv *env, jclass cls, jdouble angle)
{
  if (likely(broadcast_fd != -1))
  {
    sendto(broadcast_fd, &angle, sizeof(double), 0, (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));
  }
}

// Gets all packets recieved and waiting in queue
JNIEXPORT jint JNICALL Java_frc_robot_util_VisionNative_drainPackets(JNIEnv *env, jclass cls, jobject byte_buffer, jlong current_hal_time)
{
  VisionMeasurement *out_buffer =
      (VisionMeasurement *)(*env)->GetDirectBufferAddress(env, byte_buffer);

  if (unlikely(out_buffer == NULL))
    return 0;

  // Calculate offset to sync C clock to Java clock
  uint64_t now_monotonic = get_monotonic_micros();
  uint64_t offset = (uint64_t)current_hal_time - now_monotonic;

  int t = atomic_load_explicit(&vq.tail, memory_order_relaxed);
  int h = atomic_load_explicit(&vq.head, memory_order_acquire);

  int count = 0;
  while (likely(t != h))
  {
    // Copy from ring buffer into local variable
    VisionMeasurement m = vq.data[t];

    // Apply offset to convert packet from Monotonic to FPGA time
    m.ts += offset;

    out_buffer[count] = m;
    t = (t + 1) & MASK;
    count++;

    if (unlikely(count >= MAX_QUEUE_SIZE))
      break;
  }

  // Update the ring buf tail
  atomic_store_explicit(&vq.tail, t, memory_order_release);
  return count;
}
