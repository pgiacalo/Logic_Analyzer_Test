/**
 * Code for the ESP32 that sends and receives UART, SPI and CAN Bus messages.
 * This code was written to test Dream Source Lab's  DSLogic Plus logic analyzer along with DSView software.
 * Compile this code using Espressif tools (no Arduino libraries are used). 
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "string.h"
#include <inttypes.h>


#define TAG "ProtocolTest"

// UART configuration (YELLOW Probe)
#define UART_NUM 	UART_NUM_1
#define TXD_PIN 	GPIO_NUM_4 	//Yellow Wire
#define RXD_PIN 	GPIO_NUM_5

// SPI configuration 
#define MY_SPI_HOST 	HSPI_HOST
#define PIN_NUM_MISO 	GPIO_NUM_19	//Logic Analyzer Channel 1 (Blue Wire)
#define PIN_NUM_MOSI 	GPIO_NUM_23	//Logic Analyzer Channel NONE (since it's loopback)
#define PIN_NUM_CLK  	GPIO_NUM_18 	//Logic Analyzer Channel 2 (White Wire)
#define SPI_MAX_TRANSFER_SIZE 4096  // Maximum size for SPI transfer in bytes

// CAN configuration (RED Probe) ("twai" - Two Wire Automotive Interface)
#define TX_GPIO_NUM 	21	//Logic Analyzer Channel 3
#define RX_GPIO_NUM 	22

// Function Prototypes
void init_uart();
void init_spi();
void init_can();
void uart_test();
void spi_test();
void can_test();

void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };
    // Configure UART parameters
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 256 * 2, 0, 0, NULL, 0);
    uart_set_loop_back(UART_NUM, true);
}

void init_spi() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_MAX_TRANSFER_SIZE,
        .flags = 0,
        .intr_flags = 0,
    };

    esp_err_t ret = spi_bus_initialize(MY_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }
}

void init_can() {
    const twai_general_config_t g_config = {
        .mode = TWAI_MODE_NORMAL,  // Change to TWAI_MODE_NO_ACK or TWAI_MODE_NORMAL 
        .tx_io = TX_GPIO_NUM,
        .rx_io = RX_GPIO_NUM,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 10,
        .rx_queue_len = 5,
        .alerts_enabled = TWAI_ALERT_ALL,
        .clkout_divider = 0,
    };
    const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver, error: %s", esp_err_to_name(ret));
        return;
    }

    // Start the TWAI driver
    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver, error: %s", esp_err_to_name(ret));
        twai_driver_uninstall();  // Uninstall driver if start fails
        return;
    }
}

void uart_test() {
    ESP_LOGI(TAG, "Starting UART test");
    char *message_uart = "UART";
    uart_write_bytes(UART_NUM, message_uart, strlen(message_uart));
    char data[10] = {0};
    uart_read_bytes(UART_NUM, (uint8_t*)data, strlen(message_uart), pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Received via UART: %s", data);
    ESP_LOGI(TAG, "UART test completed");
}

void spi_test() {
    ESP_LOGI(TAG, "Starting SPI test");
    spi_device_handle_t spi = NULL;
    spi_bus_add_device(MY_SPI_HOST, &(spi_device_interface_config_t){
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .queue_size = 1}, &spi);
    if (!spi) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return;
    }
    char data[10] = {0};
    spi_transaction_t t = {
        .length = 8 * 4,  // 32 bits
        .tx_buffer = "SPI",
        .rx_buffer = data
    };
    if (spi_device_transmit(spi, &t) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transmit SPI data");
    } else {
        ESP_LOGI(TAG, "Received via SPI: %s", (char*)t.rx_buffer);
    }
    spi_bus_remove_device(spi);
    ESP_LOGI(TAG, "SPI test completed");
}

void can_test() {
	//flush the CAN Bus by stopping and starting it
	// twai_stop();
    // twai_start();

    ESP_LOGI(TAG, "Starting CAN test");
    twai_message_t message_can = {
        .identifier = 0x555,
        .data_length_code = 4,
        .data = {'C', 'A', 'N', '\0'}
    };

	esp_err_t err = twai_transmit(&message_can, pdMS_TO_TICKS(100));
	if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send CAN message");
    	if (err == ESP_ERR_TIMEOUT) {
        	ESP_LOGE(TAG, "CAN transmit error: ESP_ERR_TIMEOUT: Timed out waiting for space on TX queue");
    	} if (err == ESP_ERR_INVALID_ARG) {
        	ESP_LOGE(TAG, "CAN transmit error: ESP_ERR_INVALID_ARG: Arguments are invalid");
    	} else if (err == ESP_FAIL) {
        	ESP_LOGE(TAG, "CAN transmit error: ESP_FAIL: TX queue is disabled and another message is currently transmitting");
    	} else if (err == ESP_ERR_INVALID_STATE) {
        	ESP_LOGE(TAG, "CAN transmit error: ESP_ERR_INVALID_STATE: TWAI driver is not in running state, or is not installed");
    	} else if (err == ESP_ERR_NOT_SUPPORTED) {
        	ESP_LOGE(TAG, "CAN transmit error: ESP_ERR_NOT_SUPPORTED: Listen Only Mode does not support transmissions");
    	}
        return;  // Exit if send failed
    } else {
        ESP_LOGI(TAG, "CAN message sent successfully");
    }

    // Check CAN controller state
	twai_status_info_t status_info;
	twai_get_status_info(&status_info);
	printf("TX Errors: %" PRIu32 ", RX Errors: %" PRIu32 "\n", status_info.tx_error_counter, status_info.rx_error_counter);

    if (twai_get_status_info(&status_info) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get CAN status info");
        if (status_info.state == TWAI_STATE_BUS_OFF) {
		    twai_initiate_recovery();
        	ESP_LOGE(TAG, "CAN status error: TWAI_STATE_BUS_OFF: ...recovering...");
		    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for recovery
		    twai_start();
		}else if (status_info.state == ESP_ERR_TIMEOUT) {
        	ESP_LOGE(TAG, "CAN status error: ESP_ERR_TIMEOUT: Timed out waiting for space on TX queue");
    	} else if (status_info.state == ESP_ERR_INVALID_ARG) {
        	ESP_LOGE(TAG, "CAN status error: ESP_ERR_INVALID_ARG: Arguments are invalid");
    	} else if (status_info.state ==  ESP_ERR_INVALID_STATE) {
        	ESP_LOGE(TAG, "CAN status error: ESP_ERR_INVALID_STATE: TWAI driver is not in running state, or is not installed");
        }
    } else {
        ESP_LOGI(TAG, "CAN Controller state: %d, Errors: TX %d, RX %d",
                 (int)status_info.state, (int)status_info.tx_error_counter, (int)status_info.rx_error_counter);
    }


    twai_message_t message_received_can;
    err = twai_receive(&message_received_can, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive CAN message");
    	if (err == ESP_ERR_TIMEOUT) {
        	ESP_LOGE(TAG, "CAN receive error: ESP_ERR_TIMEOUT: Timed out waiting for space on TX queue");
    	} if (err == ESP_ERR_INVALID_ARG) {
        	ESP_LOGE(TAG, "CAN receive error: ESP_ERR_INVALID_ARG: Arguments are invalid");
    	} else if (err == ESP_ERR_INVALID_STATE) {
        	ESP_LOGE(TAG, "CAN receive error: ESP_ERR_INVALID_STATE: TWAI driver is not in running state, or is not installed");
    	} 
    } else {
        ESP_LOGI(TAG, "Received via CAN: %s", message_received_can.data);
    }

    ESP_LOGI(TAG, "CAN test completed");
}

void app_main() {
    // Initialize peripherals once
    init_uart();
    init_spi();
    init_can();

    // Loop to continuously run tests
    while (1) {
        ESP_LOGI(TAG, "Starting protocol tests...");

        uart_test();
        spi_test();
        can_test();

        ESP_LOGI(TAG, "Protocol tests completed. Restarting after a delay...");
        vTaskDelay(pdMS_TO_TICKS(50)); // Delay before restarting tests
    }
}

