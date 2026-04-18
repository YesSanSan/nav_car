#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fal.h"
#include "sfud/sfud.h"
#include "storage_layout.h"

static sfud_flash *get_storage_flash_device(void) {
    return sfud_get_device(SFUD_W25Q256FV_DEVICE_INDEX);
}

static int storage_flash_init(void) {
    sfud_flash *flash = get_storage_flash_device();

    if (flash == NULL || !flash->init_ok) {
        return -1;
    }

    storage_sfud_flash_dev.blk_size = flash->chip.erase_gran;
    storage_sfud_flash_dev.len = flash->chip.capacity;
    storage_sfud_flash_dev.write_gran = 1;
    return 0;
}

static int storage_flash_read(long offset, uint8_t *buf, size_t size) {
    sfud_flash *flash = get_storage_flash_device();

    if (flash == NULL || !flash->init_ok) {
        return -1;
    }

    return sfud_read(flash, storage_sfud_flash_dev.addr + (uint32_t)offset, size, buf) == SFUD_SUCCESS ? (int)size : -1;
}

static int storage_flash_write(long offset, const uint8_t *buf, size_t size) {
    sfud_flash *flash = get_storage_flash_device();

    if (flash == NULL || !flash->init_ok) {
        return -1;
    }

    return sfud_write(flash, storage_sfud_flash_dev.addr + (uint32_t)offset, size, buf) == SFUD_SUCCESS ? (int)size : -1;
}

static int storage_flash_erase(long offset, size_t size) {
    sfud_flash *flash = get_storage_flash_device();

    if (flash == NULL || !flash->init_ok) {
        return -1;
    }

    return sfud_erase(flash, storage_sfud_flash_dev.addr + (uint32_t)offset, size) == SFUD_SUCCESS ? (int)size : -1;
}

struct fal_flash_dev storage_sfud_flash_dev = {
    .name = STORAGE_FLASH_DEV_NAME,
    .addr = 0,
    .len = STORAGE_FLASH_TOTAL_SIZE,
    .blk_size = STORAGE_FLASH_ERASE_SIZE,
    .ops =
        {
            .init = storage_flash_init,
            .read = storage_flash_read,
            .write = storage_flash_write,
            .erase = storage_flash_erase,
        },
    .write_gran = 1,
};
