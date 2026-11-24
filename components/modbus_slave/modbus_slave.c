#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "modbus_slave.h"

#define TAG "MODBUS_TCP"

#define MODBUS_TCP_PORT          502
#define MODBUS_TCP_BACKLOG       4
#define MODBUS_TCP_TASK_STACK    4096
#define MODBUS_TCP_TASK_PRIORITY 5

#define MODBUS_MAX_ADU_LENGTH 260

#define MODBUS_FUNCTION_READ_HOLDING_REGISTERS 0x03

#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION       0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS   0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE     0x03

static uint16_t s_holding_registers[MODBUS_HOLDING_REG_COUNT];

static portMUX_TYPE s_register_lock =
    portMUX_INITIALIZER_UNLOCKED;

static bool s_server_started;

/* Receive the requested number of bytes */
static bool recv_all(int sock,
                     uint8_t *buffer,
                     size_t length)
{
    size_t received = 0;

    while (received < length) {

        const int result = recv(
            sock,
            buffer + received,
            length - received,
            0);

        if (result <= 0) {
            return false;
        }

        received += (size_t)result;
    }

    return true;
}

/* Send the requested number of bytes */
static bool send_all(int sock,
                     const uint8_t *buffer,
                     size_t length)
{
    size_t sent = 0;

    while (sent < length) {

        const int result = send(
            sock,
            buffer + sent,
            length - sent,
            0);

        if (result <= 0) {
            return false;
        }

        sent += (size_t)result;
    }

    return true;
}

/* Send Modbus exception response */
static void send_exception(
    int client,
    const uint8_t request_header[7],
    uint8_t function,
    uint8_t exception_code)
{
    uint8_t response[9];

    memcpy(response, request_header, 7);

    response[4] = 0;
    response[5] = 3;

    response[7] = (uint8_t)(function | 0x80U);
    response[8] = exception_code;

    (void)send_all(
        client,
        response,
        sizeof(response));
}

/* Handle Function Code 0x03 */
static void handle_read_holding_registers(
    int client,
    const uint8_t header[7],
    const uint8_t *pdu,
    size_t pdu_length)
{
    if (pdu_length != 5U) {

        send_exception(
            client,
            header,
            MODBUS_FUNCTION_READ_HOLDING_REGISTERS,
            MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);

        return;
    }

    const uint16_t start =
        ((uint16_t)pdu[1] << 8) | pdu[2];

    const uint16_t quantity =
        ((uint16_t)pdu[3] << 8) | pdu[4];

    if ((quantity == 0U) || (quantity > 125U)) {

        send_exception(
            client,
            header,
            MODBUS_FUNCTION_READ_HOLDING_REGISTERS,
            MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);

        return;
    }

    if (((uint32_t)start + quantity) >
        MODBUS_HOLDING_REG_COUNT) {

        send_exception(
            client,
            header,
            MODBUS_FUNCTION_READ_HOLDING_REGISTERS,
            MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);

        return;
    }

    uint8_t response[
        7U + 2U + (2U * MODBUS_HOLDING_REG_COUNT)
    ];

    const size_t byte_count =
        (size_t)quantity * 2U;

    const size_t response_length =
        9U + byte_count;

    memcpy(response, header, 7);

    response[4] = 0;

    response[5] = (uint8_t)(3U + byte_count);

    response[7] =
        MODBUS_FUNCTION_READ_HOLDING_REGISTERS;

    response[8] = (uint8_t)byte_count;

    portENTER_CRITICAL(&s_register_lock);

    for (uint16_t i = 0; i < quantity; i++) {

        const uint16_t value =
            s_holding_registers[start + i];

        response[9U + (2U * i)] =
            (uint8_t)(value >> 8);

        response[10U + (2U * i)] =
            (uint8_t)(value & 0xFFU);
    }

    portEXIT_CRITICAL(&s_register_lock);

    (void)send_all(
        client,
        response,
        response_length);
}

/* Handle connected Modbus TCP client */
static void handle_client(int client)
{
    uint8_t header[7];

    uint8_t adu[MODBUS_MAX_ADU_LENGTH];

    while (recv_all(client, header, sizeof(header))) {

        const uint16_t protocol_id =
            ((uint16_t)header[2] << 8) | header[3];

        const uint16_t mbap_length =
            ((uint16_t)header[4] << 8) | header[5];

        /*
         * MBAP length includes Unit ID and PDU,
         * but not the first 6 bytes.
         */

        if ((protocol_id != 0U) ||
            (mbap_length < 2U) ||
            (mbap_length > (MODBUS_MAX_ADU_LENGTH - 6U))) {

            ESP_LOGW(TAG,
                     "Invalid Modbus TCP MBAP header");

            break;
        }

        const size_t pdu_length =
            (size_t)mbap_length - 1U;

        if (!recv_all(client, adu, pdu_length)) {
            break;
        }

        const uint8_t function = adu[0];

        if (function ==
            MODBUS_FUNCTION_READ_HOLDING_REGISTERS) {

            handle_read_holding_registers(
                client,
                header,
                adu,
                pdu_length);
        }
        else {

            send_exception(
                client,
                header,
                function,
                MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
        }
    }
}

/* Modbus TCP server task */
static void modbus_server_task(void *arg)
{
    (void)arg;

    const int server = socket(
        AF_INET,
        SOCK_STREAM,
        IPPROTO_IP);

    if (server < 0) {

        ESP_LOGE(TAG,
                 "Unable to create socket: errno %d",
                 errno);

        vTaskDelete(NULL);
        return;
    }

    const int reuse = 1;

    (void)setsockopt(
        server,
        SOL_SOCKET,
        SO_REUSEADDR,
        &reuse,
        sizeof(reuse));

    struct sockaddr_in address = {0};

    address.sin_family = AF_INET;

    address.sin_port =
        htons(MODBUS_TCP_PORT);

    address.sin_addr.s_addr =
        htonl(INADDR_ANY);

    if (bind(
            server,
            (struct sockaddr *)&address,
            sizeof(address)) != 0) {

        ESP_LOGE(TAG,
                 "Socket bind failed on port %d: errno %d",
                 MODBUS_TCP_PORT,
                 errno);

        close(server);

        vTaskDelete(NULL);
        return;
    }

    if (listen(server, MODBUS_TCP_BACKLOG) != 0) {

        ESP_LOGE(TAG,
                 "Socket listen failed: errno %d",
                 errno);

        close(server);

        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG,
             "Modbus TCP server listening on port %d",
             MODBUS_TCP_PORT);

    while (true) {

        struct sockaddr_in client_address;

        socklen_t address_length =
            sizeof(client_address);

        const int client = accept(
            server,
            (struct sockaddr *)&client_address,
            &address_length);

        if (client < 0) {

            ESP_LOGW(TAG,
                     "Accept failed: errno %d",
                     errno);

            vTaskDelay(pdMS_TO_TICKS(100));

            continue;
        }

        ESP_LOGI(TAG,
                 "Modbus TCP client connected");

        handle_client(client);

        shutdown(client, SHUT_RDWR);

        close(client);

        ESP_LOGI(TAG,
                 "Modbus TCP client disconnected");
    }
}

/* Update Modbus registers with battery values */
esp_err_t modbus_slave_update_status(
    const battery_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t voltage_scaled =
        (int32_t)(status->voltage_v * 100.0f);

    int32_t current_scaled =
        (int32_t)(status->current_a * 100.0f);

    int32_t soc_scaled =
        (int32_t)(status->soc_percent * 10.0f);

    /* Voltage limits */
    if (voltage_scaled < 0) {
        voltage_scaled = 0;
    }

    if (voltage_scaled > 65535) {
        voltage_scaled = 65535;
    }

    /* Current limits */
    if (current_scaled < -32768) {
        current_scaled = -32768;
    }

    if (current_scaled > 32767) {
        current_scaled = 32767;
    }

    /* SoC limits */
    if (soc_scaled < 0) {
        soc_scaled = 0;
    }

    if (soc_scaled > 1000) {
        soc_scaled = 1000;
    }

    portENTER_CRITICAL(&s_register_lock);

    s_holding_registers[MODBUS_REG_PACK_VOLTAGE] =
        (uint16_t)voltage_scaled;

    s_holding_registers[MODBUS_REG_PACK_CURRENT] =
        (uint16_t)(int16_t)current_scaled;

    s_holding_registers[MODBUS_REG_SOC] =
        (uint16_t)soc_scaled;

    portEXIT_CRITICAL(&s_register_lock);

    return ESP_OK;
}

/* Initialize Modbus TCP server */
esp_err_t modbus_slave_init(void)
{
    if (s_server_started) {
        return ESP_OK;
    }

    const BaseType_t result = xTaskCreate(
        modbus_server_task,
        "modbus_tcp",
        MODBUS_TCP_TASK_STACK,
        NULL,
        MODBUS_TCP_TASK_PRIORITY,
        NULL);

    if (result != pdPASS) {

        ESP_LOGE(TAG,
                 "Failed to create Modbus TCP server task");

        return ESP_ERR_NO_MEM;
    }

    s_server_started = true;

    return ESP_OK;
}