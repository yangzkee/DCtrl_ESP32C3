idf_component_register(
    SRCS "param_store.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash freertos
)
