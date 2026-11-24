#include <string.h>
#include "can_driver.h"
#include "driver/twai.h"
#include "esp_log.h"

static const char *TAG = "can_driver";

#define CAN_TX_GPIO   GPIO_NUM_5
#define CAN_RX_GPIO   GPIO_NUM_4
#define BMS_STATUS_CAN_ID 0x100

esp_err_t can_driver_init(void)
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed: %d", err);
        return err;
    }

    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed: %d", err);
        return err;
    }

    ESP_LOGI(TAG, "CAN (TWAI) driver started");
    return ESP_OK;
}

esp_err_t can_driver_send_status(const battery_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_message_t message = {
        .identifier = BMS_STATUS_CAN_ID,
        .data_length_code = 8,
        .flags = TWAI_MSG_FLAG_NONE,
    };

    int16_t voltage_x100 = (int16_t)(status->voltage_v * 100);
    int16_t current_x100 = (int16_t)(status->current_a * 100);
    uint8_t soc = (uint8_t)status->soc_percent;
    int8_t temp = (int8_t)status->temperature_c;

    memcpy(&message.data[0], &voltage_x100, sizeof(voltage_x100));
    memcpy(&message.data[2], &current_x100, sizeof(current_x100));
    message.data[4] = soc;
    message.data[5] = (uint8_t)temp;

    esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "twai_transmit failed: %d", err);
    }

    return err;
}