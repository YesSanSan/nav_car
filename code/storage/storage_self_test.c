#include "storage_self_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config_store.h"
#include "storage_layout.h"
#include "storage_service.h"

#define STORAGE_SELF_TEST_KEY_U32 "__selftest_u32"
#define STORAGE_SELF_TEST_KEY_STR "__selftest_str"
#define STORAGE_SELF_TEST_KEY_BLOB "__selftest_blob"
#define STORAGE_SELF_TEST_STR_VALUE "nav_car_storage_ok"

__attribute__((section(".sram1_buffer"), aligned(32)))
static uint8_t scratch_backup[STORAGE_FLASH_ERASE_SIZE];
__attribute__((section(".sram1_buffer"), aligned(32)))
static uint8_t scratch_buffer[STORAGE_FLASH_ERASE_SIZE];
__attribute__((section(".sram1_buffer"), aligned(32)))
static uint8_t scratch_verify[STORAGE_FLASH_ERASE_SIZE];

static bool is_all_ff(const uint8_t *buf, size_t len)
{
    size_t i;

    for (i = 0; i < len; ++i) {
        if (buf[i] != 0xFFU) {
            return false;
        }
    }

    return true;
}

static void fill_pattern(uint8_t *buf, size_t len, uint8_t seed)
{
    size_t i;

    for (i = 0; i < len; ++i) {
        buf[i] = (uint8_t)(seed + (uint8_t)(i * 13U) + (uint8_t)(i >> 1U));
    }
}

static bool compare_buffer(const char *label, const uint8_t *expected, const uint8_t *actual, size_t len)
{
    size_t i;

    for (i = 0; i < len; ++i) {
        if (expected[i] != actual[i]) {
            printf("[storage-test] %s mismatch @%lu exp=0x%02X act=0x%02X\r\n",
                   label,
                   (unsigned long)i,
                   expected[i],
                   actual[i]);
            return false;
        }
    }

    return true;
}

static bool partition_read_chunked(const struct fal_partition *part, uint32_t offset, uint8_t *buf, size_t len)
{
    const size_t chunk = 256U;
    size_t done = 0U;

    while (done < len) {
        size_t step = (len - done > chunk) ? chunk : (len - done);
        int ret = fal_partition_read(part, offset + (uint32_t)done, buf + done, step);
        if (ret != (int)step) {
            printf("[storage-test] fal read failed off=0x%08lX step=%lu ret=%d\r\n",
                   (unsigned long)(offset + (uint32_t)done),
                   (unsigned long)step,
                   ret);
            return false;
        }
        done += step;
    }

    return true;
}

static bool partition_write_chunked(const struct fal_partition *part, uint32_t offset, const uint8_t *buf, size_t len)
{
    const size_t chunk = 256U;
    size_t done = 0U;

    while (done < len) {
        size_t step = (len - done > chunk) ? chunk : (len - done);
        int ret = fal_partition_write(part, offset + (uint32_t)done, buf + done, step);
        if (ret != (int)step) {
            printf("[storage-test] fal write failed off=0x%08lX step=%lu ret=%d\r\n",
                   (unsigned long)(offset + (uint32_t)done),
                   (unsigned long)step,
                   ret);
            return false;
        }
        done += step;
    }

    return true;
}

static bool config_test_u32(void)
{
    uint32_t value = 0x1234ABCDUL;
    uint32_t read_back = 0;

    printf("[storage-test] config u32 write\r\n");
    if (!config_set_u32(STORAGE_SELF_TEST_KEY_U32, value)) {
        printf("[storage-test] config_set_u32 failed\r\n");
        return false;
    }
    if (!config_commit()) {
        printf("[storage-test] config u32 commit failed\r\n");
        return false;
    }
    if (!config_get_u32(STORAGE_SELF_TEST_KEY_U32, &read_back)) {
        printf("[storage-test] config_get_u32 failed\r\n");
        return false;
    }
    if (read_back != value) {
        printf("[storage-test] config u32 mismatch exp=0x%08lX act=0x%08lX\r\n",
               (unsigned long)value,
               (unsigned long)read_back);
        return false;
    }

    return true;
}

static bool config_test_string(void)
{
    char read_back[48];

    memset(read_back, 0, sizeof(read_back));
    printf("[storage-test] config string write\r\n");
    if (!config_set_string(STORAGE_SELF_TEST_KEY_STR, STORAGE_SELF_TEST_STR_VALUE)) {
        printf("[storage-test] config_set_string failed\r\n");
        return false;
    }
    if (!config_commit()) {
        printf("[storage-test] config string commit failed\r\n");
        return false;
    }
    if (!config_get_string(STORAGE_SELF_TEST_KEY_STR, read_back, sizeof(read_back))) {
        printf("[storage-test] config_get_string failed\r\n");
        return false;
    }
    if (strcmp(read_back, STORAGE_SELF_TEST_STR_VALUE) != 0) {
        printf("[storage-test] config string mismatch exp=%s act=%s\r\n",
               STORAGE_SELF_TEST_STR_VALUE,
               read_back);
        return false;
    }

    return true;
}

static bool config_test_blob(void)
{
    size_t actual_len = 0;

    fill_pattern(scratch_buffer, 32U, 0x31U);
    memset(scratch_verify, 0, 32U);

    printf("[storage-test] config blob write\r\n");
    if (!config_set_blob(STORAGE_SELF_TEST_KEY_BLOB, scratch_buffer, 32U)) {
        printf("[storage-test] config_set_blob failed\r\n");
        return false;
    }
    if (!config_commit()) {
        printf("[storage-test] config blob commit failed\r\n");
        return false;
    }
    if (!config_get_blob(STORAGE_SELF_TEST_KEY_BLOB, scratch_verify, 32U, &actual_len)) {
        printf("[storage-test] config_get_blob failed\r\n");
        return false;
    }
    if (actual_len != 32U) {
        printf("[storage-test] config blob len mismatch exp=32 act=%lu\r\n", (unsigned long)actual_len);
        return false;
    }
    if (!compare_buffer("config blob", scratch_buffer, scratch_verify, 32U)) {
        return false;
    }

    return true;
}

static void config_cleanup(void)
{
    (void)config_delete(STORAGE_SELF_TEST_KEY_U32);
    (void)config_delete(STORAGE_SELF_TEST_KEY_STR);
    (void)config_delete(STORAGE_SELF_TEST_KEY_BLOB);
    (void)config_commit();
}

bool storage_self_test_run(void)
{
    const sfud_flash *flash;
    const struct fal_partition *cfg_part;
    uint32_t scratch_offset;
    uint32_t scratch_abs_addr;
    bool raw_ok = false;
    bool fal_ok = false;
    bool config_ok = false;
    bool config_tested = false;
    bool restore_ok = false;
    bool backup_blank = false;

    printf("[storage-test] begin\r\n");

    flash = storage_flash();
    cfg_part = storage_find_partition("cfg");
    if (flash == NULL || cfg_part == NULL) {
        printf("[storage-test] flash/partition missing\r\n");
        return false;
    }

    scratch_offset = (uint32_t)(cfg_part->len - STORAGE_FLASH_ERASE_SIZE);
    scratch_abs_addr = (uint32_t)(cfg_part->offset + scratch_offset);
    printf("[storage-test] scratch cfg+0x%08lX abs=0x%08lX\r\n",
           (unsigned long)scratch_offset,
           (unsigned long)scratch_abs_addr);

    printf("[storage-test] backup read start\r\n");
    storage_lock();
    if (!partition_read_chunked(cfg_part, scratch_offset, scratch_backup, sizeof(scratch_backup))) {
        printf("[storage-test] backup read failed\r\n");
        storage_unlock();
        return false;
    }
    storage_unlock();
    backup_blank = is_all_ff(scratch_backup, sizeof(scratch_backup));
    printf("[storage-test] backup read ok blank=%s\r\n", backup_blank ? "yes" : "no");

    printf("[storage-test] raw erase\r\n");
    if (sfud_erase(flash, scratch_abs_addr, STORAGE_FLASH_ERASE_SIZE) == SFUD_SUCCESS &&
        sfud_read(flash, scratch_abs_addr, sizeof(scratch_verify), scratch_verify) == SFUD_SUCCESS &&
        is_all_ff(scratch_verify, sizeof(scratch_verify))) {
        fill_pattern(scratch_buffer, 256U, 0x5AU);
        printf("[storage-test] raw write/read\r\n");
        if (sfud_write(flash, scratch_abs_addr, 256U, scratch_buffer) == SFUD_SUCCESS &&
            sfud_read(flash, scratch_abs_addr, 256U, scratch_verify) == SFUD_SUCCESS &&
            compare_buffer("raw", scratch_buffer, scratch_verify, 256U)) {
            raw_ok = true;
        }
    } else {
        printf("[storage-test] raw erase/read verify failed\r\n");
    }

    printf("[storage-test] fal erase/write/read\r\n");
    if (fal_partition_erase(cfg_part, scratch_offset, STORAGE_FLASH_ERASE_SIZE) == (int)STORAGE_FLASH_ERASE_SIZE) {
        fill_pattern(scratch_buffer, 256U, 0xA5U);
        memset(scratch_verify, 0, 256U);
        if (partition_write_chunked(cfg_part, scratch_offset, scratch_buffer, 256U) &&
            partition_read_chunked(cfg_part, scratch_offset, scratch_verify, 256U) &&
            compare_buffer("fal", scratch_buffer, scratch_verify, 256U)) {
            fal_ok = true;
        }
    } else {
        printf("[storage-test] fal erase failed\r\n");
    }

    printf("[storage-test] restore scratch\r\n");
    if (fal_partition_erase(cfg_part, scratch_offset, STORAGE_FLASH_ERASE_SIZE) == (int)STORAGE_FLASH_ERASE_SIZE) {
        if (backup_blank) {
            restore_ok = true;
        } else if (partition_write_chunked(cfg_part, scratch_offset, scratch_backup, sizeof(scratch_backup)) &&
                   partition_read_chunked(cfg_part, scratch_offset, scratch_verify, sizeof(scratch_verify)) &&
                   compare_buffer("restore", scratch_backup, scratch_verify, sizeof(scratch_backup))) {
            restore_ok = true;
        }
    } else {
        printf("[storage-test] restore erase failed\r\n");
    }

    if (config_is_ready()) {
        config_tested = true;
        config_ok = config_test_u32() && config_test_string() && config_test_blob();
        config_cleanup();
    } else {
        printf("[storage-test] config skipped (not ready)\r\n");
    }

    printf("[storage-test] result raw=%s fal=%s restore=%s config=%s\r\n",
           raw_ok ? "PASS" : "FAIL",
           fal_ok ? "PASS" : "FAIL",
           restore_ok ? "PASS" : "FAIL",
           config_tested ? (config_ok ? "PASS" : "FAIL") : "SKIP");

    return raw_ok && fal_ok && restore_ok && (!config_tested || config_ok);
}
