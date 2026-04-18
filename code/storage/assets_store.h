#ifndef ASSETS_STORE_H
#define ASSETS_STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lfs.h"
#include "storage_layout.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct asset_update_manifest {
    uint32_t image_size;
    uint32_t image_crc32;
    uint32_t asset_version;
    uint32_t firmware_compat;
} asset_update_manifest_t;

bool assets_init(void);
bool assets_mount_active(void);
void assets_unmount(void);

bool assets_begin_update(const asset_update_manifest_t *manifest);
bool assets_write_chunk(const void *data, size_t len);
bool assets_finalize_update(void);

uint32_t assets_get_version(void);
storage_asset_slot_t assets_get_active_slot(void);

bool assets_read_file(const char *path, void *buf, size_t buf_len, size_t *read_len);
lfs_t *assets_lfs_handle(void);

#ifdef __cplusplus
}
#endif

#endif
