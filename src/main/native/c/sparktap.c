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

// Shared buffer
static uint8_t *shared_buffer = NULL;

// Synchronization using Pthreads
static pthread_mutex_t wakeup_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t wakeup_cv = PTHREAD_COND_INITIALIZER;
static atomic_uint_least64_t last_updated_mask[MAX_MOTORS];

static _Atomic bool running = false;
static uint32_t streamHandle = 0;
static pthread_t worker_thread;

// REV Constants
// Manufacturer 5, Device Type 2
static const int kRevBaseId = ((2 & 0x1f) << 24) | ((5 & 0xff) << 16);
static const int kRevMask = 0x1fff0000;

static void* worker_loop(void *arg) {
	// In C, you must use the 'struct' keyword for HAL types
	struct HAL_CANStreamMessage messages[32];
	uint32_t messageCount = 0;
	int32_t status = 0;

	pthread_setname_np(pthread_self(), "SparkTapWorker");

	while (atomic_load (&running)) {
		status = 0;
		// Pass the messages array correctly
		HAL_CAN_ReadStreamSession(streamHandle, messages, 32, &messageCount,
				&status);

		if (messageCount > 0) {
			pthread_mutex_lock(&wakeup_mutex);
			for (uint32_t i = 0; i < messageCount; i++) {
				uint32_t msgId = messages[i].messageID;
				int deviceId = msgId & 0x3F;
				int apiId = (msgId >> 6) & 0x3FF;

				int frameIdx = apiId - 0x60;
				if (deviceId < MAX_MOTORS
						&& frameIdx >= 0&& frameIdx < STATUS_FRAMES) {
					uint8_t *slot = shared_buffer
							+ (deviceId * MOTOR_BLOCK_SIZE)
							+ (frameIdx * SLOT_SIZE);

					memcpy(slot, messages[i].data, 8);

					int32_t ts_status = 0;
					uint64_t ts = HAL_GetFPGATime(&ts_status);
					memcpy(slot + 8, &ts, 8);

					uint64_t bit = 1ULL << frameIdx;
					atomic_fetch_or(&last_updated_mask[deviceId], bit);
				}
			}
			pthread_cond_broadcast(&wakeup_cv);
			pthread_mutex_unlock(&wakeup_mutex);
		}
		usleep(250);
	}
	return NULL;
}

JNIEXPORT jobject JNICALL Java_frc_robot_util_SparkTapJNI_init(JNIEnv *env,
		jclass clazz) {
	if (shared_buffer == NULL) {
		shared_buffer = (uint8_t*) calloc(1, TOTAL_BUFFER_SIZE);
		int32_t status = 0;

		HAL_CAN_OpenStreamSession(&streamHandle, kRevBaseId, kRevMask, 100,
				&status);

		atomic_store(&running, true);

		pthread_attr_t attr;
		pthread_attr_init(&attr);
		struct sched_param param;
		param.sched_priority = 45;
		pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
		pthread_attr_setschedparam(&attr, &param);

		pthread_create(&worker_thread, &attr, worker_loop, NULL);
		pthread_attr_destroy(&attr);
	}
	return (*env)->NewDirectByteBuffer(env, shared_buffer, TOTAL_BUFFER_SIZE);
}

JNIEXPORT void JNICALL Java_frc_robot_util_SparkTapJNI_waitForFrame(JNIEnv* env, jclass clazz, jint deviceId, jint frameIdx) {
	if (deviceId >= MAX_MOTORS || frameIdx >= STATUS_FRAMES) return;

	uint64_t bit = 1ULL << frameIdx;
	pthread_mutex_lock(&wakeup_mutex);

	// Clear the bit so we know we are waiting for a NEW update
	atomic_fetch_and(&last_updated_mask[deviceId], ~bit);

	while (!(atomic_load(&last_updated_mask[deviceId]) & bit)) {
		pthread_cond_wait(&wakeup_cv, &wakeup_mutex);
	}

	pthread_mutex_unlock(&wakeup_mutex);
}
