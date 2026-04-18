#ifndef FDB_CFG_H
#define FDB_CFG_H

#define FDB_USING_KVDB
#define FDB_USING_FAL_MODE

/* W25Q256 is a NOR flash, so it supports bit-level 1->0 programming. */
#define FDB_WRITE_GRAN 1

#define FDB_KV_NAME_MAX 64
#define FDB_KV_CACHE_TABLE_SIZE 32
#define FDB_SECTOR_CACHE_TABLE_SIZE 8

#endif
