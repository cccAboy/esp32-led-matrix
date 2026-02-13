set(src_dirs
    SIM
    FLIP
    GRAVITY
)

set(include_dirs
    SIM
    FLIP
    GRAVITY
)

set(requires
    BSP
    driver
    esp_event
    esp_wifi
    esp_netif
    nvs_flash
    lwip
)

idf_component_register(
    SRC_DIRS ${src_dirs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
)

component_compile_options(-O3 -ffast-math -Wno-error=format -Wno-format)
