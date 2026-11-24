# BMS Project v2

ESP32 Battery Management System built with ESP-IDF.

## Components
- battery_monitor: reads cell voltages and battery state
- can_driver: CAN bus communication
- modbus_slave: Modbus slave interface

## Build
idf.py build
idf.py flash monitor
