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
#include <sched.h>
#include <unistd.h>
#include <hal/CAN.h>
#include <hal/HALBase.h>

#define MAX_MOTORS 64
#define STATUS_FRAMES 7
#define SLOT_SIZE 16
#define MOTOR_BLOCK_SIZE (SLOT_SIZE * STATUS_FRAMES)
#define TOTAL_BUFFER_SIZE (MOTOR_BLOCK_SIZE * MAX_MOTORS)

static uint8_t *shared_buffer = NULL;
static pthread_mutex_t wakeup_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t wakeup_cv = PTHREAD_COND_INITIALIZER;
static atomic_uint_least8_t last_updated_mask[MAX_MOTORS];

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

		if (messageCount > 0) {
			pthread_mutex_lock(&wakeup_mutex);
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
					memcpy(slot, messages[i].data, 8);

					int32_t ts_status = 0;
					uint64_t ts = HAL_GetFPGATime(&ts_status);
					memcpy(slot + 8, &ts, 8);

					atomic_fetch_or(&last_updated_mask[deviceId],
							(1 << frameIdx));
				}
			}
			pthread_cond_broadcast(&wakeup_cv);
			pthread_mutex_unlock(&wakeup_mutex);
		}
		usleep(250);
	}
	return NULL;
}

JNIEXPORT jobject JNICALL Java_frc_robot_util_sparktap_SparkTapJNI_init(
		JNIEnv *env, jclass clazz) {
	if (shared_buffer == NULL) {
		shared_buffer = (uint8_t*) calloc(1, TOTAL_BUFFER_SIZE);
		int32_t status = 0;
		// Match all REV (0x0205XXXX)
		HAL_CAN_OpenStreamSession(&streamHandle, 0x02050000, 0x1FFF0000, 100,
				&status);
		atomic_store(&running, true);
		pthread_create(&worker_thread, NULL, worker_loop, NULL);
	}
	return (*env)->NewDirectByteBuffer(env, shared_buffer, TOTAL_BUFFER_SIZE);
}

JNIEXPORT jboolean JNICALL Java_frc_robot_util_sparktap_SparkTapJNI_waitForFrame(
		JNIEnv *env, jclass clazz, jint deviceId, jint frameIdx) {
	if (deviceId >= MAX_MOTORS || frameIdx >= STATUS_FRAMES)
		return false;

	uint8_t bit = 1 << frameIdx;
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_nsec += 20000000;
	if (ts.tv_nsec >= 1000000000) {
		ts.tv_sec++;
		ts.tv_nsec -= 1000000000;
	}

	pthread_mutex_lock(&wakeup_mutex);
	atomic_fetch_and(&last_updated_mask[deviceId], ~bit);

	int result = 0;
	while (!(atomic_load(&last_updated_mask[deviceId]) & bit) && result == 0) {
		result = pthread_cond_timedwait(&wakeup_cv, &wakeup_mutex, &ts);
	}
	pthread_mutex_unlock(&wakeup_mutex);
	return (result == 0);
}
