#pragma once

#include "esp_err.h"
#include "battery_monitor.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t can_driver_init(void);
esp_err_t can_driver_send_status(const battery_status_t *status);

#ifdef __cplusplus
}
#endif