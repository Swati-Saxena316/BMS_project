idf_component_register(
    SRCS "can_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES driver battery_monitor
)