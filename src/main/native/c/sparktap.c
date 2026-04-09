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
#include <time.h>
#include <hal/CAN.h>
#include <hal/HALBase.h>

// Constants
#define MAX_MOTORS 64
#define STATUS_FRAMES 7
#define SLOT_SIZE 24
#define MOTOR_BLOCK_SIZE (SLOT_SIZE * STATUS_FRAMES)
#define TOTAL_BUFFER_SIZE (MOTOR_BLOCK_SIZE * MAX_MOTORS)
#define CACHE_LINE 64

// Branch prediction hints
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

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

  // Direct 64-bit copy instead of memcpy and NULL checks
  *(uint64_t*)(slot + 8) = *(const uint64_t*)data;
  *(uint64_t*)(slot + 16) = ts;

  // Even (stable)
  atomic_store_explicit(seq_ptr, seq + 2, memory_order_release);
}

static void* worker_loop(void *arg) {
  struct HAL_CANStreamMessage messages[100];
  uint32_t messageCount = 0;
  int32_t status = 0;
  int64_t offset_us = 0;
  uint32_t sync_counter = 0;

  pthread_setname_np(pthread_self(), "SparkTapWorker");

  while (likely(atomic_load_explicit(&running, memory_order_relaxed))) {
    status = 0;
    HAL_CAN_ReadStreamSession(streamHandle, messages, 100, &messageCount,
        &status);

    if (unlikely(status != 0)) {
      usleep(100000);
      continue;
    }

    if (messageCount > 0) {
      // The FPGA clock and Linux Monotonic clock drift incredibly slowly.
      if (unlikely(sync_counter++ % 1000 == 0)) {
        int32_t ts_status = 0;
        uint64_t current_fpga_us = HAL_GetFPGATime(&ts_status);

        struct timespec ts_mono;
        clock_gettime(CLOCK_MONOTONIC, &ts_mono);
        uint64_t current_mono_us = (ts_mono.tv_sec * 1000000ULL)
            + (ts_mono.tv_nsec / 1000ULL);

        offset_us = current_fpga_us - current_mono_us;
      }

      for (uint32_t i = 0; i < messageCount; i++) {
        uint32_t msgId = messages[i].messageID;
        uint32_t deviceId = msgId & 0x3F;
        uint32_t apiId = (msgId >> 6) & 0x3FF;

        uint32_t base1 = apiId - 0x60;
        uint32_t base2 = apiId - 0x2E0;
        uint32_t frameIdx;

        // Use fast unsigned bounds checking to eliminate multiple branches
        if (likely(base1 <= 6)) {
          frameIdx = base1;
        } else if (unlikely(base2 <= 6)) {
          frameIdx = base2;
        } else {
          continue;
        }

        if (likely(deviceId < MAX_MOTORS)) {
          // Compute accurate timestamp using message's monotonic timestamp
          uint64_t msg_ts_us = (messages[i].timeStamp * 1000ULL)
              + offset_us;

          uint8_t *motor_base = shared_buffer
              + (deviceId * MOTOR_BLOCK_SIZE);

          update_slot(motor_base + (frameIdx * SLOT_SIZE),
              messages[i].data, msg_ts_us);
        }
      }
    }

    // Always sleep to yield the core, preventing CPU lockup regardless of message count.
    if (likely(messageCount < 100)) {
      usleep(250);
    } else {
      usleep(50);
    }
  }
  return NULL;
}

JNIEXPORT jobject JNICALL Java_frc_lib_lowlevel_SparkTap_initNative(JNIEnv *env,
    jclass clazz) {
  pthread_mutex_lock(&init_mutex);
  if (shared_buffer == NULL) {
    // Allocate shared buffer with space for all motors and status frames, aligned to cache line size
    if (posix_memalign((void**) &shared_buffer, CACHE_LINE,
    TOTAL_BUFFER_SIZE) != 0) {
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

    // Create background thread with SCHED_RR real-time priority
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);

    struct sched_param param;
    param.sched_priority = 15;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    if (pthread_create(&worker_thread, &attr, worker_loop, NULL) != 0) {
      printf(
          "[SparkTap-C] RT thread creation failed, falling back to normal thread\n");
      pthread_create(&worker_thread, NULL, worker_loop, NULL);
    }
    pthread_attr_destroy(&attr);
  }
  pthread_mutex_unlock(&init_mutex);
  return (*env)->NewDirectByteBuffer(env, shared_buffer, TOTAL_BUFFER_SIZE);
}
