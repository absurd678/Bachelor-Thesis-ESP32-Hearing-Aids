idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_gpio esp_driver_i2s esp_hw_support log esp-tflite-micro
)
