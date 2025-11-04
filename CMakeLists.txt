cmake_minimum_required(VERSION 3.5)

add_definitions( 
    -DARDUINO_USB_MODE 
    -DARDUINO_USB_CDC_ON_BOOT 
    -DARDUINO_PARTITION_esp_sr_16
    -DHTTPCLIENT_NOSECURE
) 
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(MyVoiceBox)
idf_build_set_property(COMPILE_OPTIONS "-Wno-missing-field-initializers" APPEND)
littlefs_create_partition_image(littlefs data FLASH_IN_PROJECT)

set(EXCLUDE_COMPONENTS 
    espressif__esp_rainmaker 
    espressif__esp_insights
    espressif__esp_modbus
)