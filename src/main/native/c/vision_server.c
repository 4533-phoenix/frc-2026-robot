#define _GNU_SOURCE
#include <jni.h>
#include <time.h>
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

// Ring buffer struct
typedef struct
{
  volatile VisionMeasurement data[MAX_QUEUE_SIZE];
  atomic_int head; // Written by Worker
  atomic_int tail; // Read by Main
} LockFreeQueue;

// Global states
LockFreeQueue vq = {.head = 0, .tail = 0};

// Broadcast globals
int broadcast_fd = -1;
struct sockaddr_in broadcast_addr;

// Helper function to get the Monotonic micros
uint64_t get_monotonic_micros()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)(ts.tv_sec * 1000000 + ts.tv_nsec / 1000);
}

// Worker thread for recieving cam updates and pushing them to the queue
void *vision_worker_thread(void *arg)
{
  int listenfd = *(int *)arg;
  free(arg);

  VisionMeasurement incoming;
  pthread_setname_np(pthread_self(), "VisionUDPWorker");

  while (1)
  {
    ssize_t len =
        recvfrom(listenfd, &incoming, sizeof(incoming), 0, NULL, NULL);

    if (len == sizeof(VisionMeasurement))
    {
      uint64_t now = get_monotonic_micros();

      // Calculate absolute Monotonic timestamp from delay
      incoming.ts = now - incoming.ts;

      // Load current indices
      int h = atomic_load_explicit(&vq.head, memory_order_relaxed);
      int t = atomic_load_explicit(&vq.tail, memory_order_acquire);

      int next_h = (h + 1) & MASK;

      // If queue is full, we push the tail forward to overwrite oldest
      if (next_h == t)
      {
        atomic_store_explicit(&vq.tail, (t + 1) & MASK, memory_order_release);
        printf("[VisionNative-c] Warning: Queue full, dropping oldest packet\n");
      }

      // Write the data to the buffer
      vq.data[h] = incoming;
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

  // Create background thread
  pthread_t thread_id;
  pthread_create(&thread_id, NULL, vision_worker_thread, arg);

  // Initialize broadcast socket
  if (broadcast_fd == -1)
  {
    broadcast_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (broadcast_fd < 0)
    {
      perror("Broadcast socket creation failed");
    }
    else
    {
      int broadcast = 1;
      if (setsockopt(broadcast_fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
      {
        perror("Error setting broadcast permission");
        close(broadcast_fd);
        broadcast_fd = -1;
      }
      else
      {
        memset(&broadcast_addr, 0, sizeof(broadcast_addr));
        broadcast_addr.sin_family = AF_INET;
        broadcast_addr.sin_port = htons(7002);
        broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
      }
    }
  }
}

JNIEXPORT void JNICALL Java_frc_robot_util_VisionNative_broadcastRobotHeading(JNIEnv *env, jclass cls, jdouble angle)
{
  if (broadcast_fd != -1)
  {
    sendto(broadcast_fd, &angle, sizeof(double), 0, (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));
  }
}

// Gets all packets recieved and waiting in queue
JNIEXPORT jint JNICALL Java_frc_robot_util_VisionNative_drainPackets(JNIEnv *env, jclass cls, jobject byte_buffer, jlong current_hal_time)
{
  VisionMeasurement *out_buffer = (VisionMeasurement *)(*env)->GetDirectBufferAddress(env, byte_buffer);

  if (out_buffer == NULL)
    return 0;

  // Calculate offset to sync C clock to Java clock
  uint64_t now_monotonic = get_monotonic_micros();
  uint64_t offset = (uint64_t)current_hal_time - now_monotonic;

  int t = atomic_load_explicit(&vq.tail, memory_order_relaxed);
  int h = atomic_load_explicit(&vq.head, memory_order_acquire);

  int count = 0;
  while (t != h)
  {
    out_buffer[count] = vq.data[t];

    // Apply offset to convert packet from Monotonic to FPGA time
    out_buffer[count].ts += offset;

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
