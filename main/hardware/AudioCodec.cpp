#include "esp_err.h"
#include "esp_log.h"
#include "AudioCodec.h"

#define TAG "AudioCodec"


bool AudioCodec::start()
{
    ESP_ERROR_CHECK_RETURN_FALSE(i2s_channel_enable(_spk_handle));
    ESP_ERROR_CHECK_RETURN_FALSE(i2s_channel_enable(_mic_handle));

    enableSpk();
    enableMic();
    ESP_LOGI(TAG, "Audio codec started");
    return true;
}


int AudioCodec::audio_read(int16_t* dest, int samples) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        esp_codec_dev_read(
            _mic_dev, (void*)dest, samples * sizeof(int16_t) * _mic_channels
        )
    );
    return samples;
}


int AudioCodec::audio_write(const int16_t* data, int samples) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        esp_codec_dev_write(
            _spk_dev, (void*)data, samples * sizeof(int16_t) * _spk_channels
        )
    );
    return samples;
}
        

esp_err_t AudioCodec::_init_I2C( 
    int port,
    int sda,
    int scl
)
{
    _i2c_port = port;
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = port,
        .sda_io_num = (gpio_num_t) sda,
        .scl_io_num = (gpio_num_t) scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    auto err = i2c_new_master_bus(&i2c_bus_cfg, &_i2c_bus);
    if (ESP_OK == err)
        ESP_LOGI(TAG, "I2C bus initialized");
    return err;
}
