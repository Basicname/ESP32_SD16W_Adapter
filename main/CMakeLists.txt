set(src_dirs     ./)
set(include_dirs ./)

set(requires
    driver
    esp_timer
    esp_wifi
    esp_event
    esp_netif
    esp_http_server
    nvs_flash
    iray_model
)

idf_component_register(
    SRC_DIRS     ${src_dirs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES     ${requires}
    EMBED_TXTFILES index.html
)
