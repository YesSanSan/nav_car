#ifndef STORAGE_SERVICE_H
#define STORAGE_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "flashdb.h"
#include "fal.h"
#include "sfud/sfud.h"

#ifdef __cplusplus
extern "C" {
#endif

bool storage_init(void);
bool storage_is_ready(void);

const sfud_flash *storage_flash(void);
const struct fal_partition *storage_find_partition(const char *name);

void storage_lock(void);
void storage_unlock(void);

uint32_t storage_crc32(uint32_t crc, const void *data, size_t size);

#ifdef __cplusplus
}
#endif

#endif
