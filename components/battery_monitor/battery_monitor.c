#include "battery_monitor.h"
#include "esp_log.h"
static const char *TAG = "battery_monitor";
esp_err_t battery_monitor_init(void) {
    ESP_LOGI(TAG, "Initializing battery monitor");
    return ESP_OK;
}
esp_err_t battery_monitor_read(battery_status_t *out_status) {
    if (out_status == NULL) { return ESP_ERR_INVALID_ARG; }
    out_status->voltage_v = 48.0f;
    out_status->current_a = 2.5f;
    out_status->soc_percent = 87.3f;
    out_status->temperature_c = 29.0f;
    return ESP_OK;
}
