#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "math.h"

#define I2C_MASTER_SCL_IO           22  // SCL Pin
#define I2C_MASTER_SDA_IO           21  // SDA Pin
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define MPU9250_ADDRESS             0x68  // Device address of MPU9250

#define I2S_NUM                     I2S_NUM_0
#define SAMPLE_RATE                 44100
#define PI                          3.14159

static const char *TAG = "ESP32_I2C_I2S";

// Initialize I2C for MPU9250
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                                       I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
}

// Initialize I2S for audio output
void i2s_config_init() {
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = 27,
        .ws_io_num = 26,
        .data_out_num = 25,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM, &pin_config));
}

// Function to generate and play tone
void play_tone(float freq, int duration_ms) {
    int num_samples = SAMPLE_RATE * duration_ms / 1000;
    int samples_per_wave_length = SAMPLE_RATE / freq;
    float radians_per_sample = 2.0 * PI / (float)samples_per_wave_length;
    size_t bytes_written = 0;

    for (int i = 0; i < num_samples; i++) {
        float sample_val = sin(radians_per_sample * i);
        int16_t sample = (int16_t)(sample_val * 32767);
        i2s_write(I2S_NUM, &sample, sizeof(sample), &bytes_written, portMAX_DELAY);
    }
}

// Main application
void app_main() {
    ESP_LOGI(TAG, "Initializing I2C and I2S");
    i2c_master_init();
    i2s_config_init();

    // Read data from MPU9250 and play tones
    uint8_t data[14];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true); // Start with register 0x3B (ACCEL_XOUT_H)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MPU9250 Read Successful");
    } else {
        ESP_LOGE(TAG, "Failed to read from MPU9250");
    }

    // Play simple melody
    play_tone(261.63, 500); // C4
    play_tone(293.66, 500); // D4
    play_tone(329.63, 500); // E4
    play_tone(392.00, 500); // G4
    play_tone(440.00, 500); // A4

    // Clean up
    i2s_driver_uninstall(I2S_NUM);
    i2c_driver_delete(I2C_MASTER_NUM);
}

