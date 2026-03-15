// Copyright (c) 2026 FRC Team 4533 (Phoenix)
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file
// at the root directory of this project.

#define _GNU_SOURCE
#include <jni.h>
#include <stdint.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <pthread.h>
#include <sched.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <hal/CAN.h>
#include <hal/HALBase.h>

#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#define MAX_MOTORS 64
#define STATUS_FRAMES 7
#define SLOT_SIZE 24
#define METADATA_SIZE 24
#define MOTOR_BLOCK_SIZE ((SLOT_SIZE * STATUS_FRAMES) + METADATA_SIZE)
#define TOTAL_BUFFER_SIZE (MOTOR_BLOCK_SIZE * MAX_MOTORS)

#define WORKER_CPU 1
#define CACHE_LINE 64

static uint8_t *shared_buffer = NULL;
static pthread_mutex_t init_mutex = PTHREAD_MUTEX_INITIALIZER;
static _Atomic bool running = false;
static uint32_t streamHandle = 0;
static pthread_t worker_thread;

static inline __attribute__((always_inline)) void update_slot(uint8_t *restrict slot, const uint8_t *restrict data, uint64_t ts) {
  _Atomic uint32_t *seq_ptr = (_Atomic uint32_t*)slot;
  uint32_t seq = atomic_load_explicit(seq_ptr, memory_order_relaxed);
  // Odd (busy)
  atomic_store_explicit(seq_ptr, seq + 1, memory_order_release);
  
  if (likely(data != NULL)) {
    memcpy(slot + 8, data, 8);
  }
  memcpy(slot + 16, &ts, 8);
  
  // Even (stable)
  atomic_store_explicit(seq_ptr, seq + 2, memory_order_release);
}

static void* worker_loop(void *arg) {
  struct HAL_CANStreamMessage messages[32];
  uint32_t messageCount = 0;
  int32_t status = 0;

  pthread_setname_np(pthread_self(), "SparkTapWorker");

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(WORKER_CPU, &cpuset);
  if (unlikely(pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0)) {
    perror("[SparkTap-C] Warning: CPU affinity failed");
  }

  while (likely(atomic_load_explicit(&running, memory_order_relaxed))) {
    status = 0;
    HAL_CAN_ReadStreamSession(streamHandle, messages, 32, &messageCount,
        &status);

    if (status != 0) {
      usleep(100000);
      continue;
    }

    if (messageCount > 0) {
      int32_t ts_status = 0;
      uint64_t current_ts = HAL_GetFPGATime(&ts_status);

      for (uint32_t i = 0; i < messageCount; i++) {
        uint32_t msgId = messages[i].messageID;
        int deviceId = msgId & 0x3F;
        int apiId = (msgId >> 6) & 0x3FF;

        int frameIdx = -1;
        if (apiId >= 0x60 && apiId <= 0x66)
          frameIdx = apiId - 0x60;
        else if (apiId >= 0x2E0 && apiId <= 0x2E6)
          frameIdx = apiId - 0x2E0;

        if (deviceId < MAX_MOTORS && frameIdx != -1) {
          uint8_t *motor_base = shared_buffer
              + (deviceId * MOTOR_BLOCK_SIZE);

          update_slot(motor_base + (frameIdx * SLOT_SIZE),
              messages[i].data, current_ts);
          update_slot(motor_base + (STATUS_FRAMES * SLOT_SIZE), NULL,
              current_ts);
        }
      }
    }

    if (likely(messageCount < 32)) {
      usleep(250);
    }
  }
  return NULL;
}

JNIEXPORT jobject JNICALL Java_frc_robot_util_SparkTap_initNative(
    JNIEnv *env, jclass clazz) {
  pthread_mutex_lock(&init_mutex);
  if (shared_buffer == NULL) {
    // Allocate shared buffer with space for all motors and status frames, aligned to cache line size
    if (posix_memalign((void**)&shared_buffer, CACHE_LINE, TOTAL_BUFFER_SIZE) != 0) {
      perror("[SparkTap-C] Memory allocation failed");
      pthread_mutex_unlock(&init_mutex);
      return NULL;
    }
    memset(shared_buffer, 0, TOTAL_BUFFER_SIZE);

    // Open CAN stream session for all motor status frames
    int32_t status = 0;
    HAL_CAN_OpenStreamSession(&streamHandle, 0x02050000, 0x1FFF0000, 100,
        &status);
    atomic_store(&running, true);

    // Create background thread with RT priority
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    struct sched_param sp = {.sched_priority = 45};
    pthread_attr_setschedparam(&attr, &sp);

    if (pthread_create(&worker_thread, &attr, worker_loop, NULL) != 0) {
      printf("[SparkTap-C] RT thread creation failed, falling back to normal thread\n");
      pthread_create(&worker_thread, NULL, worker_loop, NULL);
    }
    pthread_attr_destroy(&attr);
  }
  pthread_mutex_unlock(&init_mutex);
  return (*env)->NewDirectByteBuffer(env, shared_buffer, TOTAL_BUFFER_SIZE);
}
