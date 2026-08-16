file( GLOB ALLFONTS "fonts/*.c")
file( GLOB ALLHARDWARE "hardware/*.cpp")
idf_component_register(
    SRCS 
        "main.cpp" 
        "sr_commands.cpp"
        "uploadWAV.cpp" 
        "rt_stats.cpp"
        "my_display.cpp"
        "my_sensors.cpp"
        "helpers/rsyslog.cpp"
        "helpers/SimpleMqttClientESP.cpp"
        "helpers/SimpleOTA.cpp"
        "helpers/SimpleReports.cpp"
        "helpers/StatusLED.cpp" 
        ${ALLFONTS}
        ${ALLHARDWARE}
    INCLUDE_DIRS 
        "hardware"
        "helpers"
)
