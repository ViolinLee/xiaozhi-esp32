#pragma once

#include "esp_err.h"
#include "spippy_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t power_manager_init(void);
esp_err_t power_manager_start(void);
esp_err_t power_manager_get_status(spippy_runtime_status_t *status);
esp_err_t power_manager_get_telemetry(spippy_power_telemetry_t *telemetry);
esp_err_t power_manager_clear_low_power_latch(bool force);
esp_err_t power_manager_tick(void);

#ifdef __cplusplus
}
#endif
