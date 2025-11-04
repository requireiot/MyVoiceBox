idf_component_register(SRCS "src/ESP_SRx.cpp" "src/esp32-hal-srx.c"
                      INCLUDE_DIRS "./src"
                      REQUIRES arduino-esp32 esp-sr
                      )