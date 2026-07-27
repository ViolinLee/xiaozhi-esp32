#pragma once

#include "esp_err.h"

#include "spippy_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t sensor_manager_init(void);
esp_err_t sensor_manager_start(void);
esp_err_t sensor_manager_get_ultrasonic_reading(spippy_ultrasonic_reading_t *reading);
esp_err_t sensor_manager_tick(void);

#ifdef __cplusplus
}
#endif
