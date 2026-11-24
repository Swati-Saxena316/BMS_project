#pragma once
#include "esp_err.h"
typedef struct {
    float voltage_v;
    float current_a;
    float soc_percent;
    float temperature_c;
} battery_status_t;
esp_err_t battery_monitor_init(void);
esp_err_t battery_monitor_read(battery_status_t *out_status);
