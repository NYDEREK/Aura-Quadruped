idf_component_register(
    SRCS "main.c"
         "barsotion/src/ICM42688_Barsotion.c"
         "barsotion/src/ICM42688_SPI.c"
         "barsotion/src/ICM42688_I2C.c"
         "barsotion/src/ICM42688_Filter.c"
    INCLUDE_DIRS "." "barsotion/src"
    REQUIRES driver esp_driver_gpio esp_driver_spi esp_driver_i2c
)
