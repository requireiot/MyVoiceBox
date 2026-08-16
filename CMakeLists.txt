cmake_minimum_required(VERSION 3.5)

add_definitions( 
    -DARDUINO_USB_MODE 
    -DARDUINO_USB_CDC_ON_BOOT 
    -DARDUINO_PARTITION_esp_sr_16
    -DHTTPCLIENT_NOSECURE
    -DLV_CONF_INCLUDE_SIMPLE=1
    -DLV_CONF_PATH="${CMAKE_SOURCE_DIR}/main/lv_conf.h"
) 
add_compile_options("-I${CMAKE_SOURCE_DIR}/main/")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(MyVoiceBox)
idf_build_set_property(COMPILE_OPTIONS "-Wno-missing-field-initializers" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-deprecated" APPEND)

littlefs_create_partition_image(littlefs data FLASH_IN_PROJECT)

set(EXCLUDE_COMPONENTS 
    espressif__esp_rainmaker 
    espressif__esp_rmaker_common
    espressif__esp_insights
    espressif__esp-modbus
    espressif__esp-zigbee-lib
    espressif__esp-zboss-lib
)