#include "storage_service.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "config_store.h"
#include "storage_self_test.h"

#ifndef STORAGE_RUN_BOOT_SELF_TEST
#define STORAGE_RUN_BOOT_SELF_TEST 0
#endif

static SemaphoreHandle_t storage_mutex = NULL;
static bool storage_ready = false;

static bool storage_init_sfud(void) {
    if (sfud_init() != SFUD_SUCCESS) {
        printf("[storage] sfud_init failed\r\n");
        return false;
    }

    return true;
}

bool storage_init(void) {
    if (storage_ready) {
        return true;
    }

    if (storage_mutex == NULL) {
        storage_mutex = xSemaphoreCreateRecursiveMutex();
        if (storage_mutex == NULL) {
            printf("[storage] mutex create failed\r\n");
            return false;
        }
    }

    if (!storage_init_sfud()) {
        return false;
    }
    printf("[storage] sfud ready\r\n");

    if (fal_init() < 0) {
        printf("[storage] fal_init failed\r\n");
        return false;
    }
    printf("[storage] fal ready\r\n");

#if STORAGE_RUN_BOOT_SELF_TEST
    printf("[storage] boot self-test begin\r\n");
    if (!storage_self_test_run()) {
        printf("[storage] boot self-test failed\r\n");
        return false;
    }
    printf("[storage] boot self-test passed\r\n");
#endif

    if (!config_init()) {
        printf("[storage] config_init failed\r\n");
        return false;
    }
    printf("[storage] config ready\r\n");

    storage_ready = true;
    return true;
}

bool storage_is_ready(void) {
    return storage_ready;
}

const sfud_flash *storage_flash(void) {
    return sfud_get_device(SFUD_W25Q256FV_DEVICE_INDEX);
}

const struct fal_partition *storage_find_partition(const char *name) {
    return fal_partition_find(name);
}

void storage_lock(void) {
    if (storage_mutex != NULL) {
        xSemaphoreTakeRecursive(storage_mutex, portMAX_DELAY);
    }
}

void storage_unlock(void) {
    if (storage_mutex != NULL) {
        xSemaphoreGiveRecursive(storage_mutex);
    }
}

uint32_t storage_crc32(uint32_t crc, const void *data, size_t size) {
    return fdb_calc_crc32(crc, data, size);
}
