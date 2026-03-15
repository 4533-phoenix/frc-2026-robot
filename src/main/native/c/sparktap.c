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
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <hal/CAN.h>
#include <hal/HALBase.h>

#define MAX_MOTORS 64
#define STATUS_FRAMES 7
#define SLOT_SIZE 24
#define MOTOR_BLOCK_SIZE (SLOT_SIZE * STATUS_FRAMES)
#define TOTAL_BUFFER_SIZE (MOTOR_BLOCK_SIZE * MAX_MOTORS)

static uint8_t *shared_buffer = NULL;
static pthread_mutex_t init_mutex = PTHREAD_MUTEX_INITIALIZER;
static _Atomic bool running = false;
static uint32_t streamHandle = 0;
static pthread_t worker_thread;

static void* worker_loop(void *arg) {
	struct HAL_CANStreamMessage messages[32];
	uint32_t messageCount = 0;
	int32_t status = 0;

	pthread_setname_np(pthread_self(), "SparkTapWorker");

	while (atomic_load (&running)) {
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
					uint8_t *slot = shared_buffer
							+ (deviceId * MOTOR_BLOCK_SIZE)
							+ (frameIdx * SLOT_SIZE);

					_Atomic uint32_t
					*seq_ptr = (_Atomic uint32_t*)slot;
					uint32_t seq = atomic_load_explicit(seq_ptr,
							memory_order_relaxed);

					// Increment to odd (active write)
					atomic_store_explicit(seq_ptr, seq + 1,
							memory_order_release);

					memcpy(slot + 8, messages[i].data, 8);
					memcpy(slot + 16, &current_ts, 8);

					// Increment to even (stable)
					atomic_store_explicit(seq_ptr, seq + 2,
							memory_order_release);
				}
			}
		}

		if (messageCount < 32) {
			usleep(250);
		}
	}
	return NULL;
}

JNIEXPORT jobject JNICALL Java_frc_robot_util_sparktap_SparkTap_initNative(
		JNIEnv *env, jclass clazz) {
	pthread_mutex_lock(&init_mutex);
	if (shared_buffer == NULL) {
		shared_buffer = (uint8_t*) calloc(1, TOTAL_BUFFER_SIZE);
		int32_t status = 0;
		HAL_CAN_OpenStreamSession(&streamHandle, 0x02050000, 0x1FFF0000, 100,
				&status);
		atomic_store(&running, true);
		pthread_create(&worker_thread, NULL, worker_loop, NULL);
	}
	pthread_mutex_unlock(&init_mutex);
	return (*env)->NewDirectByteBuffer(env, shared_buffer, TOTAL_BUFFER_SIZE);
}
