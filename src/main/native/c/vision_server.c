#define _GNU_SOURCE
#include <jni.h>
#include <hal/HAL.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
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

// Structs that are given from our rust code (Packed to match Java)
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
  uint64_t camera_id;
} VisionMeasurement;

// Ring buffer struct
typedef struct {
  volatile VisionMeasurement data[MAX_QUEUE_SIZE];
  atomic_int head; // Written by Worker
  atomic_int tail; // Read by Main
} LockFreeQueue;

// Global states
LockFreeQueue vq = {.head = 0, .tail = 0};

// Worker thread for recieving cam updates and pushing them to the queue
void *vision_worker_thread(void *arg) {
  int listenfd = *(int *)arg;
  free(arg);

  VisionMeasurement incoming;
  int32_t status = 0;
  pthread_setname_np(pthread_self(), "VisionUDPWorker");

  while (1) {
    ssize_t len =
        recvfrom(listenfd, &incoming, sizeof(incoming), 0, NULL, NULL);

    if (len == sizeof(VisionMeasurement)) {
      uint64_t now = HAL_GetFPGATime(&status);
      
      // Calculate absolute FPGA timestamp from delay
      incoming.ts = now - incoming.ts;

      // Load current indices
      int h = atomic_load_explicit(&vq.head, memory_order_relaxed);
      int t = atomic_load_explicit(&vq.tail, memory_order_acquire);

      int next_h = (h + 1) & MASK;

      // If queue is full, we push the tail forward to overwrite oldest
      if (next_h == t) {
        atomic_store_explicit(&vq.tail, (t + 1) & MASK, memory_order_release);
      }

      // Write the data to the buffer
      vq.data[h] = incoming;
      atomic_store_explicit(&vq.head, next_h, memory_order_release);
    }
  }
  return NULL;
}

// --- JNI EXPORTS ---

JNIEXPORT void JNICALL Java_frc_robot_util_VisionNative_startServer(JNIEnv *env, jclass cls, jint port) {
  int listenfd;
  struct sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof(servaddr));

  // Create a UDP Socket
  listenfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (listenfd < 0) {
    perror("Socket creation failed");
    return;
  }

  // Set socket options
  int rcvbuf = RECIEVE_BUF_SIZE;
  setsockopt(listenfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(port);
  servaddr.sin_family = AF_INET;

  // Bind server address to socket descriptor
  if (bind(listenfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1) {
    perror("Bind failed");
    close(listenfd);
    return;
  }
  printf("Ready to recive on port %d\n", port);

  // Use malloc so the FD pointer persists for the thread
  int *arg = malloc(sizeof(int));
  *arg = listenfd;

  // Create background thread
  pthread_t thread_id;
  pthread_create(&thread_id, NULL, vision_worker_thread, arg);
}

// Gets all packets recieved and waiting in queue
JNIEXPORT jint JNICALL Java_frc_robot_util_VisionNative_drainPackets(JNIEnv *env, jclass cls, jobject byte_buffer) {
  VisionMeasurement *out_buffer = (VisionMeasurement *)(*env)->GetDirectBufferAddress(env, byte_buffer);
  
  if (out_buffer == NULL) return 0;

  int t = atomic_load_explicit(&vq.tail, memory_order_relaxed);
  int h = atomic_load_explicit(&vq.head, memory_order_acquire);

  int count = 0;
  while (t != h) {
    out_buffer[count] = vq.data[t];
    t = (t + 1) & MASK;
    count++;

    // Prevent infinite loops
    if (count >= MAX_QUEUE_SIZE)
      break;
  }

  // Update the ring buf tail
  atomic_store_explicit(&vq.tail, t, memory_order_release);
  return count;
}