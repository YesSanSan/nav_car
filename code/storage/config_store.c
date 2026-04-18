#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "timers.h"

#include "config_store.h"
#include "storage_layout.h"
#include "storage_service.h"

#define CONFIG_DB_NAME "cfg"
#define CONFIG_PART_NAME "cfg"
#define CONFIG_COMMIT_DELAY_MS 1500U
#define CONFIG_PENDING_MAX 8U
#define CONFIG_PENDING_VALUE_MAX 256U

typedef enum config_pending_kind {
    CONFIG_PENDING_EMPTY = 0,
    CONFIG_PENDING_BLOB,
    CONFIG_PENDING_STRING,
    CONFIG_PENDING_DELETE,
} config_pending_kind_t;

typedef struct config_pending_entry {
    config_pending_kind_t kind;
    char key[FDB_KV_NAME_MAX];
    size_t len;
    uint8_t value[CONFIG_PENDING_VALUE_MAX];
} config_pending_entry_t;

static struct fdb_kvdb config_db;
static bool config_ready = false;
static TimerHandle_t commit_timer = NULL;
static config_pending_entry_t pending_entries[CONFIG_PENDING_MAX];
static uint32_t default_schema_version = STORAGE_CONFIG_SCHEMA_VERSION;
static struct fdb_default_kv_node default_kv_nodes[] = {
    {"schema_version", &default_schema_version, sizeof(default_schema_version)},
};
static struct fdb_default_kv default_kv = {
    .kvs = default_kv_nodes,
    .num = sizeof(default_kv_nodes) / sizeof(default_kv_nodes[0]),
};

static size_t config_string_len_bounded(const char *value) {
    size_t len = 0U;

    while (len < (CONFIG_PENDING_VALUE_MAX - 1U) && value[len] != '\0') {
        ++len;
    }

    return len;
}

static void clear_pending_entries(void) {
    size_t i;

    for (i = 0; i < CONFIG_PENDING_MAX; ++i) {
        memset(&pending_entries[i], 0, sizeof(pending_entries[i]));
    }
}

static bool config_format_and_restore_defaults(void) {
    fdb_err_t result;

    printf("[config] formatting cfg partition\r\n");
    result = fdb_kv_set_default(&config_db);
    if (result != FDB_NO_ERR) {
        printf("[config] fdb_kv_set_default failed (%d)\r\n", result);
        return false;
    }

    clear_pending_entries();
    return true;
}

static config_pending_entry_t *find_pending_entry(const char *key) {
    size_t i;

    for (i = 0; i < CONFIG_PENDING_MAX; ++i) {
        if (pending_entries[i].kind != CONFIG_PENDING_EMPTY &&
            strncmp(pending_entries[i].key, key, FDB_KV_NAME_MAX) == 0) {
            return &pending_entries[i];
        }
    }

    return NULL;
}

static config_pending_entry_t *acquire_pending_entry(const char *key) {
    size_t i;
    config_pending_entry_t *entry = find_pending_entry(key);

    if (entry != NULL) {
        return entry;
    }

    for (i = 0; i < CONFIG_PENDING_MAX; ++i) {
        if (pending_entries[i].kind == CONFIG_PENDING_EMPTY) {
            memset(&pending_entries[i], 0, sizeof(pending_entries[i]));
            strncpy(pending_entries[i].key, key, sizeof(pending_entries[i].key) - 1U);
            return &pending_entries[i];
        }
    }

    return NULL;
}

static void schedule_commit(void) {
    if (commit_timer != NULL) {
        xTimerReset(commit_timer, 0);
    }
}

static bool write_pending_entry(config_pending_entry_t *entry) {
    fdb_err_t result = FDB_NO_ERR;
    struct fdb_blob blob;

    switch (entry->kind) {
        case CONFIG_PENDING_BLOB:
            result = fdb_kv_set_blob(&config_db, entry->key, fdb_blob_make(&blob, entry->value, entry->len));
            break;
        case CONFIG_PENDING_STRING:
            result = fdb_kv_set(&config_db, entry->key, (const char *)entry->value);
            break;
        case CONFIG_PENDING_DELETE:
            result = fdb_kv_del(&config_db, entry->key);
            break;
        case CONFIG_PENDING_EMPTY:
        default:
            return true;
    }

    if (result != FDB_NO_ERR) {
        printf("[config] write failed for %s (%d)\r\n", entry->key, result);
        return false;
    }

    memset(entry, 0, sizeof(*entry));
    return true;
}

static void commit_timer_callback(TimerHandle_t timer) {
    (void)timer;
    (void)config_commit();
}

bool config_commit(void) {
    size_t i;
    bool ok = true;

    if (!config_ready) {
        return false;
    }

    storage_lock();
    for (i = 0; i < CONFIG_PENDING_MAX; ++i) {
        if (!write_pending_entry(&pending_entries[i])) {
            ok = false;
        }
    }
    storage_unlock();

    return ok;
}

static bool config_migrate_schema(void) {
    uint32_t current_version = 0;

    printf("[config] read schema_version\r\n");
    if (!config_get_u32("schema_version", &current_version)) {
        current_version = 0;
    }
    printf("[config] current schema=%lu target=%lu\r\n",
           (unsigned long)current_version,
           (unsigned long)STORAGE_CONFIG_SCHEMA_VERSION);

    if (current_version < STORAGE_CONFIG_SCHEMA_VERSION) {
        printf("[config] write schema_version\r\n");
        if (!config_set_u32("schema_version", STORAGE_CONFIG_SCHEMA_VERSION)) {
            printf("[config] config_set_u32 failed\r\n");
            return false;
        }
        printf("[config] commit schema_version\r\n");
        return config_commit();
    }

    return true;
}

bool config_init(void) {
    const struct fal_partition *partition;
    uint32_t sector_size;
    fdb_err_t check_result;

    if (config_ready) {
        return true;
    }

    partition = storage_find_partition(CONFIG_PART_NAME);
    if (partition == NULL) {
        printf("[config] partition not found\r\n");
        return false;
    }
    printf("[config] partition=%s len=0x%08lx\r\n", partition->name, (unsigned long)partition->len);

    sector_size = STORAGE_FLASH_ERASE_SIZE;
    fdb_kvdb_control(&config_db, FDB_KVDB_CTRL_SET_SEC_SIZE, &sector_size);
    printf("[config] init flashdb sector=0x%08lx\r\n", (unsigned long)sector_size);

    if (fdb_kvdb_init(&config_db, CONFIG_DB_NAME, CONFIG_PART_NAME, &default_kv, NULL) != FDB_NO_ERR) {
        printf("[config] fdb_kvdb_init failed\r\n");
        return false;
    }
    printf("[config] flashdb ready\r\n");

    check_result = fdb_kvdb_check(&config_db);
    if (check_result != FDB_NO_ERR) {
        printf("[config] flashdb check failed (%d)\r\n", check_result);
        if (!config_format_and_restore_defaults()) {
            return false;
        }
        check_result = fdb_kvdb_check(&config_db);
        if (check_result != FDB_NO_ERR) {
            printf("[config] flashdb recheck failed (%d)\r\n", check_result);
            return false;
        }
        printf("[config] flashdb recovered\r\n");
    }

    if (commit_timer == NULL) {
        commit_timer = xTimerCreate("cfg_commit", pdMS_TO_TICKS(CONFIG_COMMIT_DELAY_MS), pdFALSE, NULL, commit_timer_callback);
    }

    config_ready = commit_timer != NULL;
    if (!config_ready) {
        printf("[config] commit timer create failed\r\n");
        return false;
    }

    printf("[config] migrate schema\r\n");
    return config_migrate_schema();
}

bool config_is_ready(void) {
    return config_ready;
}

bool config_get_u32(const char *key, uint32_t *value) {
    size_t actual_len = 0;

    if (value == NULL) {
        return false;
    }

    return config_get_blob(key, value, sizeof(*value), &actual_len) && actual_len == sizeof(*value);
}

bool config_set_u32(const char *key, uint32_t value) {
    return config_set_blob(key, &value, sizeof(value));
}

bool config_get_blob(const char *key, void *buf, size_t buf_len, size_t *actual_len) {
    config_pending_entry_t *entry;
    struct fdb_blob blob;
    size_t read_size;

    if (!config_ready || key == NULL || buf == NULL) {
        return false;
    }

    storage_lock();
    entry = find_pending_entry(key);
    if (entry != NULL) {
        if (entry->kind == CONFIG_PENDING_DELETE || entry->len > buf_len) {
            storage_unlock();
            return false;
        }

        memcpy(buf, entry->value, entry->len);
        if (actual_len != NULL) {
            *actual_len = entry->len;
        }
        storage_unlock();
        return true;
    }

    read_size = fdb_kv_get_blob(&config_db, key, fdb_blob_make(&blob, buf, buf_len));
    storage_unlock();

    if (actual_len != NULL) {
        *actual_len = read_size;
    }

    return read_size > 0;
}

bool config_set_blob(const char *key, const void *data, size_t len) {
    config_pending_entry_t *entry;

    if (!config_ready || key == NULL || data == NULL || len == 0 || len > CONFIG_PENDING_VALUE_MAX) {
        return false;
    }

    storage_lock();
    entry = acquire_pending_entry(key);
    if (entry == NULL) {
        storage_unlock();
        return false;
    }

    entry->kind = CONFIG_PENDING_BLOB;
    entry->len = len;
    memcpy(entry->value, data, len);
    storage_unlock();

    schedule_commit();
    return true;
}

bool config_get_string(const char *key, char *buf, size_t buf_len) {
    config_pending_entry_t *entry;
    char *value;

    if (!config_ready || key == NULL || buf == NULL || buf_len == 0) {
        return false;
    }

    storage_lock();
    entry = find_pending_entry(key);
    if (entry != NULL) {
        if (entry->kind != CONFIG_PENDING_STRING) {
            storage_unlock();
            return false;
        }

        strncpy(buf, (const char *)entry->value, buf_len - 1U);
        buf[buf_len - 1U] = '\0';
        storage_unlock();
        return true;
    }

    value = fdb_kv_get(&config_db, key);
    if (value != NULL) {
        strncpy(buf, value, buf_len - 1U);
        buf[buf_len - 1U] = '\0';
    }
    storage_unlock();

    return value != NULL;
}

bool config_set_string(const char *key, const char *value) {
    config_pending_entry_t *entry;
    size_t len;

    if (!config_ready || key == NULL || value == NULL) {
        return false;
    }

    len = config_string_len_bounded(value) + 1U;

    storage_lock();
    entry = acquire_pending_entry(key);
    if (entry == NULL) {
        storage_unlock();
        return false;
    }

    entry->kind = CONFIG_PENDING_STRING;
    entry->len = len;
    memcpy(entry->value, value, len);
    storage_unlock();

    schedule_commit();
    return true;
}

bool config_delete(const char *key) {
    config_pending_entry_t *entry;

    if (!config_ready || key == NULL) {
        return false;
    }

    storage_lock();
    entry = acquire_pending_entry(key);
    if (entry == NULL) {
        storage_unlock();
        return false;
    }

    entry->kind = CONFIG_PENDING_DELETE;
    entry->len = 0;
    storage_unlock();

    schedule_commit();
    return true;
}

bool config_reset_factory(void) {
    if (!config_ready) {
        return false;
    }

    storage_lock();
    if (!config_format_and_restore_defaults()) {
        storage_unlock();
        return false;
    }
    storage_unlock();

    return config_migrate_schema();
}
