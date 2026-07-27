#pragma once

#include "esp_err.h"
#include "spippy_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t buzzer_manager_init(void);
esp_err_t buzzer_manager_start(void);
esp_err_t buzzer_manager_set_low_power_alert(bool enabled);
esp_err_t buzzer_manager_get_status(spippy_buzzer_status_t *status);

#ifdef __cplusplus
}
#endif
