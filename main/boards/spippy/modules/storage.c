#include "modules/storage.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "esp_log.h"

static const char *TAG = "Storage";

esp_err_t storage_init(void)
{
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        err = nvs_flash_erase();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "nvs erase failed: %s", esp_err_to_name(err));
            return err;
        }

        err = nvs_flash_init();
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "nvs initialized");
    return ESP_OK;
}

esp_err_t storage_start(void)
{
    ESP_LOGI(TAG, "ready");
    return ESP_OK;
}

esp_err_t storage_load_blob(const char *ns, const char *key, void *data, size_t *size)
{
    if ((ns == NULL) || (key == NULL) || (data == NULL) || (size == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(ns, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    if (err != ESP_OK) {
        return err;
    }

    size_t required_size = *size;
    err = nvs_get_blob(handle, key, data, &required_size);
    nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    if (err == ESP_OK) {
        *size = required_size;
    }
    return err;
}

esp_err_t storage_save_blob(const char *ns, const char *key, const void *data, size_t size)
{
    if ((ns == NULL) || (key == NULL) || (data == NULL) || (size == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(handle, key, data, size);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t storage_delete_key(const char *ns, const char *key)
{
    if ((ns == NULL) || (key == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_key(handle, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        return ESP_ERR_NOT_FOUND;
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

esp_err_t storage_load_u8(const char *ns, const char *key, uint8_t *value)
{
    if ((ns == NULL) || (key == NULL) || (value == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(ns, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_ERR_NOT_FOUND : err;
    }
    err = nvs_get_u8(handle, key, value);
    nvs_close(handle);
    return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_ERR_NOT_FOUND : err;
}

esp_err_t storage_save_u8(const char *ns, const char *key, uint8_t value)
{
    if ((ns == NULL) || (key == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}
