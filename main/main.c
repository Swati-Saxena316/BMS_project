#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#ifndef CONFIG_ESP_WIFI_SSID
#define CONFIG_ESP_WIFI_SSID "YOUR_WIFI_NAME"
#endif

#ifndef CONFIG_ESP_WIFI_PASSWORD
#define CONFIG_ESP_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

#ifndef CONFIG_ESP_MAXIMUM_RETRY
#define CONFIG_ESP_MAXIMUM_RETRY 5
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_bit_defs.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "battery_monitor.h"
#include "can_driver.h"
#include "modbus_slave.h"

static const char *TAG = "BMS_MAIN";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num;

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    (void)arg;

    if ((event_base == WIFI_EVENT) &&
        (event_id == WIFI_EVENT_STA_START)) {
        esp_wifi_connect();
    }
    else if ((event_base == WIFI_EVENT) &&
             (event_id == WIFI_EVENT_STA_DISCONNECTED)) {

        if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
            s_retry_num++;

            ESP_LOGW(TAG, "Retrying Wi-Fi connection (%d/%d)",
                     s_retry_num, CONFIG_ESP_MAXIMUM_RETRY);

            esp_wifi_connect();
        }
        else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    }
    else if ((event_base == IP_EVENT) &&
             (event_id == IP_EVENT_STA_GOT_IP)) {

        const ip_event_got_ip_t *event =
            (const ip_event_got_ip_t *)event_data;

        ESP_LOGI(TAG, "Got IP: " IPSTR,
                 IP2STR(&event->ip_info.ip));

        s_retry_num = 0;

        xEventGroupSetBits(s_wifi_event_group,
                           WIFI_CONNECTED_BIT);
    }
}

static bool wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create Wi-Fi event group");
        return false;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    if (esp_netif_create_default_wifi_sta() == NULL) {
        ESP_LOGE(TAG, "Failed to create Wi-Fi station interface");
        return false;
    }

    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL));

    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL));

    wifi_config_t wifi_config = {0};

    snprintf((char *)wifi_config.sta.ssid,
             sizeof(wifi_config.sta.ssid),
             "%s",
             CONFIG_ESP_WIFI_SSID);

    snprintf((char *)wifi_config.sta.password,
             sizeof(wifi_config.sta.password),
             "%s",
             CONFIG_ESP_WIFI_PASSWORD);

    wifi_config.sta.threshold.authmode =
        (strlen(CONFIG_ESP_WIFI_PASSWORD) == 0U) ?
        WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(esp_wifi_set_config(
        WIFI_IF_STA,
        &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to Wi-Fi SSID '%s'...",
             CONFIG_ESP_WIFI_SSID);

    const EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    return ((bits & WIFI_CONNECTED_BIT) != 0);
}

static void bms_task(void *arg)
{
    (void)arg;

    battery_status_t status = {0};

    while (true) {

        if (battery_monitor_read(&status) == ESP_OK) {

            ESP_LOGI(TAG,
                     "Voltage: %.2f V | Current: %.2f A | SoC: %.1f %%",
                     (double)status.voltage_v,
                     (double)status.current_a,
                     (double)status.soc_percent);

            can_driver_send_status(&status);

            const esp_err_t mb_err =
                modbus_slave_update_status(&status);

            if (mb_err != ESP_OK) {
                ESP_LOGW(TAG,
                         "Failed to update Modbus status: %s",
                         esp_err_to_name(mb_err));
            }
        }
        else {
            ESP_LOGW(TAG, "Failed to read battery status");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "BMS starting up...");

    esp_err_t err = nvs_flash_init();

    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) ||
        (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {

        ESP_ERROR_CHECK(nvs_flash_erase());

        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(battery_monitor_init());

    ESP_ERROR_CHECK(can_driver_init());

    if (wifi_init_sta()) {

        ESP_LOGI(TAG,
                 "Wi-Fi connected; starting Modbus TCP server");

        ESP_ERROR_CHECK(modbus_slave_init());
    }
    else {

        ESP_LOGE(TAG,
                 "Wi-Fi connection failed; Modbus TCP server will not start");
    }

    const BaseType_t task_result = xTaskCreate(
        bms_task,
        "bms_task",
        4096,
        NULL,
        5,
        NULL);

    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create BMS task");
    }
}