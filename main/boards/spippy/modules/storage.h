#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t storage_init(void);
esp_err_t storage_start(void);
esp_err_t storage_load_blob(const char *ns, const char *key, void *data, size_t *size);
esp_err_t storage_save_blob(const char *ns, const char *key, const void *data, size_t size);
esp_err_t storage_delete_key(const char *ns, const char *key);
esp_err_t storage_load_u8(const char *ns, const char *key, uint8_t *value);
esp_err_t storage_save_u8(const char *ns, const char *key, uint8_t value);

#ifdef __cplusplus
}
#endif
