#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool config_init(void);
bool config_is_ready(void);

bool config_get_u32(const char *key, uint32_t *value);
bool config_set_u32(const char *key, uint32_t value);

bool config_get_blob(const char *key, void *buf, size_t buf_len, size_t *actual_len);
bool config_set_blob(const char *key, const void *data, size_t len);

bool config_get_string(const char *key, char *buf, size_t buf_len);
bool config_set_string(const char *key, const char *value);

bool config_delete(const char *key);
bool config_commit(void);
bool config_reset_factory(void);

#ifdef __cplusplus
}
#endif

#endif
