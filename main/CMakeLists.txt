idf_component_register(SRCS "ps5_transport.c" "main.c" "app_state.c" "aura_radio.c" "aura_network.c"
                            "board_i2c.c" "icm42688.c" "mpu6050.c" "attitude_filter.c" "power.c" "servo_bus.c" "robot_model.c" "robot_control.c" "joint_trajectory.c" "robot_gait_profile.c" "robot_locomotion.c" "robot_predictive_support.c" "robot_body_trajectory.c" "robot_balance.c" "robot_static_gait_profile.c" "robot_static_balance.c" "robot_gait_sync.c" "robot_contact.c" "robot_kinematics.c" "robot_odometry.c" "robot_gait.c"
                            "servo_protocol.c" "dualsense.c" "tof.c" "vl53l4cd_platform.c"
                            "vl53l4cd_uld/vl53l4cd_api.c" "ws2812.c"
                       INCLUDE_DIRS "." "vl53l4cd_uld"
                       REQUIRES esp_driver_gpio esp_driver_spi esp_driver_i2c
                                esp_driver_uart esp_driver_rmt esp_adc esp_timer esp_rom freertos vfs bt esp_hid nvs_flash esp_wifi esp_netif esp_event lwip mdns)

# Compile against this ESP-IDF checkout's Bluedroid declarations, not vendored ABI headers.
target_include_directories(${COMPONENT_LIB} PRIVATE
    "$ENV{IDF_PATH}/components/bt/host/bluedroid/stack/include"
    "$ENV{IDF_PATH}/components/bt/host/bluedroid/common/include"
    "$ENV{IDF_PATH}/components/bt/common/osi/include"
    "$ENV{IDF_PATH}/components/bt/common/include")
