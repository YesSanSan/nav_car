#ifndef FAL_CFG_H
#define FAL_CFG_H

#define FAL_PART_HAS_TABLE_CFG

#define STORAGE_FLASH_DEV_NAME "norflash0"

extern struct fal_flash_dev storage_sfud_flash_dev;

#define FAL_FLASH_DEV_TABLE                  \
    {                                       \
        &storage_sfud_flash_dev,            \
    }

#define FAL_PART_MAGIC_WORD 0x45503130

#define FAL_PART_TABLE                                                               \
    {                                                                                \
        {FAL_PART_MAGIC_WORD, "cfg",        STORAGE_FLASH_DEV_NAME, 0x00000000, 0x00100000, 0}, \
        {FAL_PART_MAGIC_WORD, "asset_meta", STORAGE_FLASH_DEV_NAME, 0x00100000, 0x00010000, 0}, \
        {FAL_PART_MAGIC_WORD, "asset_a",    STORAGE_FLASH_DEV_NAME, 0x00110000, 0x00F78000, 0}, \
        {FAL_PART_MAGIC_WORD, "asset_b",    STORAGE_FLASH_DEV_NAME, 0x01088000, 0x00F78000, 0}, \
    }

#endif
