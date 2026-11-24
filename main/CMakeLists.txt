idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "include"
    REQUIRES battery_monitor can_driver modbus_slave esp_wifi esp_event esp_netif nvs_flash
)