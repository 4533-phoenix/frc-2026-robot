// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file
// at the root directory of this project.

#define _GNU_SOURCE
#include <jni.h>
#include <stdint.h>
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
#include <errno.h>

// Constants
#define MAX_QUEUE_SIZE 64
#define MASK (MAX_QUEUE_SIZE - 1)
#define RECIEVE_BUF_SIZE 4194304
#define RECV_BATCH 16
#define CACHE_LINE 64

// Branch prediction hints
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

// Structs that are given from our rust code
typedef struct __attribute__((packed)) {
  double x, y, rot;
} RobotPos;

typedef struct __attribute__((packed)) {
  double x, y, rot;
} VisionUncertainty;

typedef struct __attribute__((packed)) {
  RobotPos pose;
  VisionUncertainty stds;
  uint64_t ts;
  uint8_t camera_id;
  uint8_t num_tags;
  uint8_t padding[6];
} VisionMeasurement;

typedef struct __attribute__((packed)) {
  uint64_t fpga_timestamp;
  double heading;
  double angular_velocity;
} GyroPacket;

// Cache-line-padded ring buffer
typedef struct {
  // We use atomic indices + memcpy to ensure ordering
  VisionMeasurement data[MAX_QUEUE_SIZE];

  // Written by Worker
  atomic_int head __attribute__((aligned(CACHE_LINE)));

  char _pad_head[CACHE_LINE - sizeof(atomic_int)];

  // Read by Main
  atomic_int tail __attribute__((aligned(CACHE_LINE)));
  char _pad_tail[CACHE_LINE - sizeof(atomic_int)];

  // Atomic counter to track drops without blocking printf
  atomic_ulong dropped_packets;
} LockFreeQueue;

// Global states
static LockFreeQueue vq = { .head = 0, .tail = 0, .dropped_packets = 0 };

// Broadcast globals
static int broadcast_fd = -1;
static struct sockaddr_in broadcast_addr;

// Helper function to get the Monotonic micros
static inline uint64_t get_monotonic_micros(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t) ts.tv_sec * 1000000ULL + (uint64_t) ts.tv_nsec / 1000ULL;
}

// Helper function to get the Realtime micros (System Clock)
static inline uint64_t get_realtime_micros(void) {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return (uint64_t) ts.tv_sec * 1000000ULL + (uint64_t) ts.tv_nsec / 1000ULL;
}

// Extract SO_TIMESTAMPNS from cmsg ancillary data
static inline uint64_t ts_from_cmsg(struct msghdr *msg) {
  for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(msg); cmsg != NULL; cmsg =
      CMSG_NXTHDR(msg, cmsg)) {
    if (cmsg->cmsg_level == SOL_SOCKET
        && cmsg->cmsg_type == SCM_TIMESTAMPNS) {
      struct timespec *ts = (struct timespec*) CMSG_DATA(cmsg);
      return (uint64_t) ts->tv_sec * 1000000ULL
          + (uint64_t) ts->tv_nsec / 1000ULL;
    }
  }
  // Fallback to Realtime if cmsg is missing
  return get_realtime_micros();
}

// Worker thread for receiving cam updates and pushing them to the queue
static void* vision_worker_thread(void *arg) {
  int listenfd = *(int*) arg;
  free(arg);

  pthread_setname_np(pthread_self(), "VisionUDPRecv");

  // Elevate to SCHED_FIFO real-time priority
  struct sched_param sp = { .sched_priority = 50 };
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
    perror("[Whacknet-C] Warning: SCHED_FIFO failed (need RT permissions)");

  // Hoist all recvmmsg structures out of the hot loop
  VisionMeasurement recv_bufs[RECV_BATCH];
  struct iovec iovecs[RECV_BATCH];
  struct mmsghdr msgs[RECV_BATCH];

  // Control message buffers for SO_TIMESTAMPNS
  union {
    char buf[CMSG_SPACE(sizeof(struct timespec))];
    struct timespec align;
  } ctrl_bufs[RECV_BATCH];

  memset(msgs, 0, sizeof(msgs));
  for (int i = 0; i < RECV_BATCH; i++) {
    iovecs[i].iov_base = &recv_bufs[i];
    iovecs[i].iov_len = sizeof(VisionMeasurement);

    msgs[i].msg_hdr.msg_iov = &iovecs[i];
    msgs[i].msg_hdr.msg_iovlen = 1;
    msgs[i].msg_hdr.msg_control = ctrl_bufs[i].buf;
    msgs[i].msg_hdr.msg_controllen = sizeof(ctrl_bufs[i].buf);
  }

  while (1) {
    // Reset controllen before each call
    for (int i = 0; i < RECV_BATCH; i++)
      msgs[i].msg_hdr.msg_controllen = sizeof(ctrl_bufs[i].buf);

    // Block on first packet, return up to RECV_BATCH at once
    int n = recvmmsg(listenfd, msgs, RECV_BATCH, MSG_WAITFORONE, NULL);
    if (unlikely(n <= 0))
      continue;

    // Sample clock offset to convert Realtime (socket) to Monotonic (internal queue)
    uint64_t m_now = get_monotonic_micros();
    uint64_t r_now = get_realtime_micros();
    int64_t r_to_m_offset = (int64_t) m_now - (int64_t) r_now;

    // Load head/tail once per batch
    int h = atomic_load_explicit(&vq.head, memory_order_relaxed);
    int t = atomic_load_explicit(&vq.tail, memory_order_acquire);

    for (int i = 0; i < n; i++) {
      if (unlikely(msgs[i].msg_len != sizeof(VisionMeasurement))) {
        // Silently ignore bad sizes to avoid printf spam in hot loop
        continue;
      }

      // Get kernel-level adapter timestamp
      uint64_t adapter_us_raw = ts_from_cmsg(&msgs[i].msg_hdr);

      // Convert Realtime adapter timestamp to Monotonic
      uint64_t adapter_us_monotonic = (uint64_t)(
          (int64_t) adapter_us_raw + r_to_m_offset);

      // Calculate absolute Monotonic timestamp from packet processing delay
      VisionMeasurement *pkt = &recv_bufs[i];
      uint64_t abs_ts_monotonic = adapter_us_monotonic - pkt->ts;

      int next_h = (h + 1) & MASK;

      // If queue is full...
      if (unlikely(next_h == t)) {
        // Do NOT move tail. Moving tail here causes data corruption/race conditions
        // where the reader reads half-written data. Instead, drop the packet.
        atomic_fetch_add_explicit(&vq.dropped_packets, 1,
            memory_order_relaxed);
        continue;
      }

      // Write the data to the buffer
      pkt->ts = abs_ts_monotonic;
      memcpy((void*) &vq.data[h], pkt, sizeof(VisionMeasurement));

      atomic_store_explicit(&vq.head, next_h, memory_order_release);
      h = next_h;
    }
  }
  return NULL;
}

// --- JNI EXPORTS ---

JNIEXPORT void JNICALL Java_frc_lib_lowlevel_Whacknet_startServer(JNIEnv *env, jclass cls, jint rport, jint bport)
{
  int listenfd;
  struct sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof(servaddr));

  // Create a UDP Socket
  listenfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (listenfd < 0)
  {
    perror("[Whacknet-C] Socket creation failed");
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
    perror("[Whacknet-C] Warning: SO_TIMESTAMPNS failed");

  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(rport);
  servaddr.sin_family = AF_INET;

  // Bind server address to socket descriptor
  if (bind(listenfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1)
  {
    perror("[Whacknet-C] Bind failed");
    close(listenfd);
    return;
  }
  printf("[Whacknet-C] Ready to receive on port %d\n", rport);

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
    printf("[Whacknet-C] RT thread creation failed, falling back to normal thread\n");
    pthread_create(&thread_id, NULL, vision_worker_thread, arg);
  }
  pthread_attr_destroy(&attr);

  // Initialize broadcast socket
  if (broadcast_fd != -1) return;
  printf("[Whacknet-C] Initializing broadcast socket\n");

  int b_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (b_fd < 0)
  {
    perror("[Whacknet-C] Broadcast socket creation failed");
    return;
  }

  int broadcast = 1;
  int b_reuse = 1;
  setsockopt(b_fd, SOL_SOCKET, SO_REUSEADDR, &b_reuse, sizeof(b_reuse));
  if (setsockopt(b_fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
  {
    perror("[Whacknet-C] Error setting broadcast permission");
    close(b_fd);
    return;
  }

  memset(&broadcast_addr, 0, sizeof(broadcast_addr));
  broadcast_addr.sin_family = AF_INET;
  broadcast_addr.sin_port = htons(bport);
  broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  broadcast_fd = b_fd; // Only set global once fully ready
}

JNIEXPORT void JNICALL Java_frc_lib_lowlevel_Whacknet_broadcastRobotTelemetry(JNIEnv *env, jclass cls, jlong timestamp, jdouble heading, jdouble velocity)
{
  if (likely(broadcast_fd != -1)
)
{
  GyroPacket pkt = {(uint64_t)timestamp, heading, velocity};
  sendto(broadcast_fd, &pkt, sizeof(GyroPacket), 0, (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));
}
}

// Gets all packets received and waiting in queue
JNIEXPORT jint JNICALL Java_frc_lib_lowlevel_Whacknet_drainPackets(JNIEnv *env,
  jclass cls, jobject byte_buffer, jlong current_hal_time) {
VisionMeasurement *out_buffer =
    (VisionMeasurement*) (*env)->GetDirectBufferAddress(env, byte_buffer);

if (unlikely(out_buffer == NULL))
  return 0;

// Calculate offset to sync C clock to Java clock
uint64_t now_monotonic = get_monotonic_micros();
int64_t offset = (int64_t) current_hal_time - (int64_t) now_monotonic;

int t = atomic_load_explicit(&vq.tail, memory_order_relaxed);
int h = atomic_load_explicit(&vq.head, memory_order_acquire);

// Check drops rarely to avoid spamming console
static uint64_t last_check = 0;
if (unlikely(now_monotonic - last_check > 2000000)) { // 2 seconds
  unsigned long drops = atomic_exchange_explicit(&vq.dropped_packets, 0,
      memory_order_relaxed);
  if (drops > 0) {
    printf("[Whacknet-C] Warning: Dropped %lu packets due to full queue\n",
        drops);
  }
  last_check = now_monotonic;
}

int count = 0;
while (likely(t != h)) {
  // Copy from ring buffer into local variable safely
  VisionMeasurement m;
  memcpy(&m, (const void*) &vq.data[t], sizeof(VisionMeasurement));

  // Apply offset to convert packet from Monotonic to FPGA time
  m.ts = (uint64_t)((int64_t) m.ts + offset);

  // Write to Java buffer
  memcpy(&out_buffer[count], &m, sizeof(VisionMeasurement));

  t = (t + 1) & MASK;
  count++;

  if (unlikely(count >= MAX_QUEUE_SIZE))
    break;
}

// Update the ring buf tail
atomic_store_explicit(&vq.tail, t, memory_order_release);
return count;
}
