#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "battery_monitor.h"

/*
 * Modbus TCP Holding Register Map
 *
 * Register 0: Pack Voltage
 *             Unsigned 16-bit
 *             0.01 V per count
 *
 * Register 1: Pack Current
 *             Signed 16-bit
 *             0.01 A per count
 *
 * Register 2: State of Charge
 *             Unsigned 16-bit
 *             0.1 % per count
 *
 * Function Code: 0x03
 * Read Holding Registers
 */

#define MODBUS_REG_PACK_VOLTAGE  0U
#define MODBUS_REG_PACK_CURRENT  1U
#define MODBUS_REG_SOC           2U

#define MODBUS_HOLDING_REG_COUNT 3U

esp_err_t modbus_slave_init(void);

esp_err_t modbus_slave_update_status(
    const battery_status_t *status);