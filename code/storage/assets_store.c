#if 0
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lfs.h"

#include "assets_store.h"
#include "fal.h"
#include "storage_service.h"

#define ASSET_META_MAGIC 0x41535431UL
#define ASSET_META_PATH_PRIMARY 0U
#define ASSET_META_PATH_SECONDARY 1U
#define ASSET_MANIFEST_PATH "/manifest.txt"

typedef struct asset_meta_record {
    uint32_t magic;
    uint32_t sequence;
    uint32_t active_slot;
    uint32_t asset_version[2];
    uint32_t firmware_compat[2];
    uint32_t image_size[2];
    uint32_t image_crc32[2];
    uint32_t crc32;
} asset_meta_record_t;

typedef struct asset_update_state {
    bool in_progress;
    storage_asset_slot_t slot;
    uint32_t bytes_written;
    uint32_t running_crc;
    asset_update_manifest_t manifest;
} asset_update_state_t;

static lfs_t asset_lfs;
static struct lfs_config asset_lfs_cfg;
static uint8_t asset_read_buffer[256];
static uint8_t asset_prog_buffer[256];
static uint32_t asset_lookahead_buffer[8];
static const struct fal_partition *asset_partitions[2];
static const struct fal_partition *asset_meta_partition;
static asset_meta_record_t asset_meta;
static bool asset_meta_loaded = false;
static bool asset_fs_mounted = false;
static asset_update_state_t update_state;

static const struct fal_partition *slot_partition(storage_asset_slot_t slot) {
    return asset_partitions[(int)slot];
}

static uint32_t meta_record_crc(const asset_meta_record_t *record) {
    return storage_crc32(0, record, offsetof(asset_meta_record_t, crc32));
}

static bool meta_record_valid(const asset_meta_record_t *record) {
    return record->magic == ASSET_META_MAGIC && record->crc32 == meta_record_crc(record) &&
           record->active_slot <= STORAGE_ASSET_SLOT_B;
}

static void build_default_meta(asset_meta_record_t *record) {
    memset(record, 0, sizeof(*record));
    record->magic = ASSET_META_MAGIC;
    record->sequence = 1;
    record->active_slot = STORAGE_ASSET_SLOT_A;
    record->crc32 = meta_record_crc(record);
}

static bool load_asset_meta(asset_meta_record_t *record) {
    asset_meta_record_t candidates[2];
    int read0;
    int read1;

    read0 = fal_partition_read(asset_meta_partition, ASSET_META_PATH_PRIMARY * STORAGE_FLASH_ERASE_SIZE,
                               (uint8_t *)&candidates[0], sizeof(candidates[0]));
    read1 = fal_partition_read(asset_meta_partition, ASSET_META_PATH_SECONDARY * STORAGE_FLASH_ERASE_SIZE,
                               (uint8_t *)&candidates[1], sizeof(candidates[1]));

    if (read0 == sizeof(candidates[0]) && meta_record_valid(&candidates[0]) &&
        read1 == sizeof(candidates[1]) && meta_record_valid(&candidates[1])) {
        *record = candidates[candidates[0].sequence >= candidates[1].sequence ? 0 : 1];
        return true;
    }

    if (read0 == sizeof(candidates[0]) && meta_record_valid(&candidates[0])) {
        *record = candidates[0];
        return true;
    }

    if (read1 == sizeof(candidates[1]) && meta_record_valid(&candidates[1])) {
        *record = candidates[1];
        return true;
    }

    build_default_meta(record);
    return false;
}

static bool save_asset_meta(const asset_meta_record_t *record) {
    asset_meta_record_t next = *record;
    uint32_t target_sector;
    int erase_size;
    int write_size;

    next.crc32 = meta_record_crc(&next);
    target_sector = (next.sequence % 2U) * STORAGE_FLASH_ERASE_SIZE;

    erase_size = fal_partition_erase(asset_meta_partition, target_sector, STORAGE_FLASH_ERASE_SIZE);
    if (erase_size != STORAGE_FLASH_ERASE_SIZE) {
        return false;
    }

    write_size = fal_partition_write(asset_meta_partition, target_sector, (const uint8_t *)&next, sizeof(next));
    if (write_size != sizeof(next)) {
        return false;
    }

    asset_meta = next;
    asset_meta_loaded = true;
    return true;
}

static int asset_bd_read(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
    const struct fal_partition *part = (const struct fal_partition *)cfg->context;
    uint32_t address = (uint32_t)block * cfg->block_size + off;

    return fal_partition_read(part, address, buffer, size) == (int)size ? 0 : LFS_ERR_IO;
}

static int asset_bd_prog(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
    const struct fal_partition *part = (const struct fal_partition *)cfg->context;
    uint32_t address = (uint32_t)block * cfg->block_size + off;

    return fal_partition_write(part, address, buffer, size) == (int)size ? 0 : LFS_ERR_IO;
}

static int asset_bd_erase(const struct lfs_config *cfg, lfs_block_t block) {
    const struct fal_partition *part = (const struct fal_partition *)cfg->context;
    uint32_t address = (uint32_t)block * cfg->block_size;

    return fal_partition_erase(part, address, cfg->block_size) == (int)cfg->block_size ? 0 : LFS_ERR_IO;
}

static int asset_bd_sync(const struct lfs_config *cfg) {
    (void)cfg;
    return 0;
}

static int asset_bd_lock(const struct lfs_config *cfg) {
    (void)cfg;
    storage_lock();
    return 0;
}

static int asset_bd_unlock(const struct lfs_config *cfg) {
    (void)cfg;
    storage_unlock();
    return 0;
}

static void configure_lfs(const struct fal_partition *part) {
    memset(&asset_lfs_cfg, 0, sizeof(asset_lfs_cfg));
    asset_lfs_cfg.context = (void *)part;
    asset_lfs_cfg.read = asset_bd_read;
    asset_lfs_cfg.prog = asset_bd_prog;
    asset_lfs_cfg.erase = asset_bd_erase;
    asset_lfs_cfg.sync = asset_bd_sync;
    asset_lfs_cfg.lock = asset_bd_lock;
    asset_lfs_cfg.unlock = asset_bd_unlock;
    asset_lfs_cfg.read_size = 16;
    asset_lfs_cfg.prog_size = 16;
    asset_lfs_cfg.block_size = STORAGE_FLASH_ERASE_SIZE;
    asset_lfs_cfg.block_count = part->len / STORAGE_FLASH_ERASE_SIZE;
    asset_lfs_cfg.block_cycles = 500;
    asset_lfs_cfg.cache_size = sizeof(asset_read_buffer);
    asset_lfs_cfg.lookahead_size = sizeof(asset_lookahead_buffer);
    asset_lfs_cfg.read_buffer = asset_read_buffer;
    asset_lfs_cfg.prog_buffer = asset_prog_buffer;
    asset_lfs_cfg.lookahead_buffer = asset_lookahead_buffer;
}

static bool partition_has_manifest(const struct fal_partition *part) {
    lfs_t tmp_lfs;
    struct lfs_config tmp_cfg;
    lfs_file_t file;
    int err;

    configure_lfs(part);
    tmp_cfg = asset_lfs_cfg;

    err = lfs_mount(&tmp_lfs, &tmp_cfg);
    if (err != 0) {
        return false;
    }

    err = lfs_file_open(&tmp_lfs, &file, ASSET_MANIFEST_PATH, LFS_O_RDONLY);
    if (err == 0) {
        lfs_file_close(&tmp_lfs, &file);
    }
    lfs_unmount(&tmp_lfs);

    return err == 0;
}

bool assets_init(void) {
    asset_partitions[STORAGE_ASSET_SLOT_A] = storage_find_partition("asset_a");
    asset_partitions[STORAGE_ASSET_SLOT_B] = storage_find_partition("asset_b");
    asset_meta_partition = storage_find_partition("asset_meta");

    if (asset_partitions[0] == NULL || asset_partitions[1] == NULL || asset_meta_partition == NULL) {
        printf("[assets] partition table incomplete\r\n");
        return false;
    }

    (void)load_asset_meta(&asset_meta);
    asset_meta_loaded = true;
    if (!assets_mount_active()) {
        printf("[assets] no mountable resource slot yet\r\n");
    }
    return true;
}

void assets_unmount(void) {
    if (asset_fs_mounted) {
        lfs_unmount(&asset_lfs);
        asset_fs_mounted = false;
    }
}

bool assets_mount_active(void) {
    storage_asset_slot_t preferred;
    storage_asset_slot_t fallback;

    if (!asset_meta_loaded) {
        return false;
    }

    assets_unmount();

    preferred = (storage_asset_slot_t)asset_meta.active_slot;
    fallback = preferred == STORAGE_ASSET_SLOT_A ? STORAGE_ASSET_SLOT_B : STORAGE_ASSET_SLOT_A;

    configure_lfs(slot_partition(preferred));
    if (lfs_mount(&asset_lfs, &asset_lfs_cfg) == 0) {
        asset_fs_mounted = true;
        return true;
    }

    configure_lfs(slot_partition(fallback));
    if (lfs_mount(&asset_lfs, &asset_lfs_cfg) == 0) {
        asset_meta.sequence += 1U;
        asset_meta.active_slot = fallback;
        if (!save_asset_meta(&asset_meta)) {
            printf("[assets] failed to persist fallback slot\r\n");
        }
        asset_fs_mounted = true;
        return true;
    }

    return false;
}

bool assets_begin_update(const asset_update_manifest_t *manifest) {
    const struct fal_partition *target_part;
    storage_asset_slot_t target_slot;

    if (manifest == NULL || manifest->image_size == 0U) {
        return false;
    }

    target_slot = asset_meta.active_slot == STORAGE_ASSET_SLOT_A ? STORAGE_ASSET_SLOT_B : STORAGE_ASSET_SLOT_A;
    target_part = slot_partition(target_slot);

    if (target_part == NULL || manifest->image_size > target_part->len) {
        return false;
    }

    assets_unmount();

    if (fal_partition_erase_all(target_part) != (int)target_part->len) {
        printf("[assets] erase inactive slot failed\r\n");
        return false;
    }

    memset(&update_state, 0, sizeof(update_state));
    update_state.in_progress = true;
    update_state.slot = target_slot;
    update_state.manifest = *manifest;
    return true;
}

bool assets_write_chunk(const void *data, size_t len) {
    const struct fal_partition *target_part;
    int write_size;

    if (!update_state.in_progress || data == NULL || len == 0U) {
        return false;
    }

    target_part = slot_partition(update_state.slot);
    if (target_part == NULL || update_state.bytes_written + len > target_part->len) {
        return false;
    }

    write_size = fal_partition_write(target_part, update_state.bytes_written, data, len);
    if (write_size != (int)len) {
        printf("[assets] chunk write failed @%lu\r\n", (unsigned long)update_state.bytes_written);
        return false;
    }

    update_state.running_crc = storage_crc32(update_state.running_crc, data, len);
    update_state.bytes_written += len;
    return true;
}

bool assets_finalize_update(void) {
    const struct fal_partition *target_part;

    if (!update_state.in_progress) {
        return false;
    }

    target_part = slot_partition(update_state.slot);
    if (target_part == NULL) {
        return false;
    }

    if (update_state.bytes_written != update_state.manifest.image_size) {
        printf("[assets] size mismatch %lu != %lu\r\n",
               (unsigned long)update_state.bytes_written,
               (unsigned long)update_state.manifest.image_size);
        return false;
    }

    if (update_state.manifest.image_crc32 != 0U && update_state.running_crc != update_state.manifest.image_crc32) {
        printf("[assets] crc mismatch 0x%08lx != 0x%08lx\r\n",
               (unsigned long)update_state.running_crc,
               (unsigned long)update_state.manifest.image_crc32);
        return false;
    }

    if (!partition_has_manifest(target_part)) {
        printf("[assets] manifest missing in target slot\r\n");
        return false;
    }

    asset_meta.sequence += 1U;
    asset_meta.active_slot = update_state.slot;
    asset_meta.asset_version[update_state.slot] = update_state.manifest.asset_version;
    asset_meta.firmware_compat[update_state.slot] = update_state.manifest.firmware_compat;
    asset_meta.image_size[update_state.slot] = update_state.manifest.image_size;
    asset_meta.image_crc32[update_state.slot] = update_state.manifest.image_crc32;

    if (!save_asset_meta(&asset_meta)) {
        printf("[assets] metadata save failed\r\n");
        return false;
    }

    update_state.in_progress = false;
    return assets_mount_active();
}

uint32_t assets_get_version(void) {
    return asset_meta.asset_version[asset_meta.active_slot];
}

storage_asset_slot_t assets_get_active_slot(void) {
    return (storage_asset_slot_t)asset_meta.active_slot;
}

bool assets_read_file(const char *path, void *buf, size_t buf_len, size_t *read_len) {
    lfs_file_t file;
    lfs_ssize_t result;

    if (!asset_fs_mounted || path == NULL || buf == NULL) {
        return false;
    }

    if (lfs_file_open(&asset_lfs, &file, path, LFS_O_RDONLY) != 0) {
        return false;
    }

    result = lfs_file_read(&asset_lfs, &file, buf, buf_len);
    lfs_file_close(&asset_lfs, &file);

    if (result < 0) {
        return false;
    }

    if (read_len != NULL) {
        *read_len = (size_t)result;
    }

    return true;
}

lfs_t *assets_lfs_handle(void) {
    return asset_fs_mounted ? &asset_lfs : NULL;
}

#endif
