#include <Arduino.h>
#include <driver/uart.h>
#include <esp_err.h>
#include <esp32-hal-log.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

uint8_t check[1];
uint8_t controller_array[25];

typedef struct{

    uint32_t brake;
    uint32_t throttle;
    int32_t rx;
    int32_t ry;
    int32_t lx;
    int32_t ly;    

} controller_vars;


void setup() {
      uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity   = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    controller_vars controller = {0};

    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, 13, 11, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, 1024, 0, 0, NULL, 0);

  while(1)
    {
        uart_read_bytes(UART_NUM_2, check, 1, pdMS_TO_TICKS(5));
        if(check[0] == 0xFF)        
        {
            uart_read_bytes(UART_NUM_2, controller_array, 24, pdMS_TO_TICKS(5));

          check[0] = 0;
          controller.brake = 0;
          controller.throttle = 0;
          controller.rx = 0;
          controller.ry = 0;
          controller.lx = 0;
          controller.ly = 0;

            for (int j = 0; j<4; j++){controller.throttle += (controller_array[j]<<(8*(3-j)));}
            for (int j = 4; j<8; j++){controller.brake += (controller_array[j]<<(8*(7-j)));}
            for (int j = 8; j<12; j++){controller.rx += (controller_array[j]<<(8*(11-j)));}
            for (int j = 12; j<16; j++){controller.ry += (controller_array[j]<<(8*(15-j)));}
            for (int j = 16; j<20; j++){controller.lx += (controller_array[j]<<(8*(19-j)));}
            for (int j = 20; j<24; j++){controller.ly += (controller_array[j]<<(8*(23-j)));}    
        }

            printf("T=%d B=%d RX=%d RY=%d LX=%d LY=%d\n", controller.throttle, controller.brake, controller.rx, controller.ry, controller.lx, controller.ly);
            printf("==========================================\n");
            fflush(stdout);

            // ESP_LOGI("UART", "T=%u B=%u RX=%d RY=%d LX=%d LY=%d", controller.throttle, controller.brake, controller.rx, controller.ry, controller.lx, controller.ly);
            // fflush(stdout);


    }


}

void loop() {

}



// #include <Arduino.h>
// #include <driver/uart.h>
// #include <esp_err.h>
// #include <esp32-hal-log.h>
// #include <esp_log.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include <string.h>

// const char *tx_data = "Hi I am from ESP32";
// uint8_t rx_data[128];

// void setup() {
//       uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity   = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_APB,
//     };

//     uart_param_config(UART_NUM_2, &uart_config);
//     uart_set_pin(UART_NUM_2, 13, 11, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//     uart_driver_install(UART_NUM_2, 1024, 0, 0, NULL, 0);

//   while(1)
//     {
//         uart_write_bytes(UART_NUM_2, tx_data, strlen(tx_data));
//         int len = uart_read_bytes(UART_NUM_2, rx_data, 128, pdMS_TO_TICKS(5));
//         if(len > 0)
//         {
//             rx_data[len] = '\0';
//             printf("Length: %d, Data: %s\n", len, rx_data);
//             fflush(stdout);
//         }

//         vTaskDelay(500 / portTICK_PERIOD_MS);
//     }


// }

// void loop() {

// }

// // #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

// #define UART_NUM UART_NUM_0
// #define TX_PIN 10
// #define RX_PIN 11
// #define TAG "UART"


// esp_err_t uart_read (uint8_t* data_storage){
//   // Read data from UART.
//   const uart_port_t uart_num = UART_NUM_0;
//   uint8_t data[27];
//   int length = 0;
//   ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_0, (size_t*)&length));
//   length = uart_read_bytes(uart_num, data, length, 100);
//   memcpy(data_storage, data, length);
//   return ESP_OK;
// };

//  uint8_t controller_data[27];

// void setup() {
//   Serial.begin(115200);

//   // Setup UART buffered IO with event queue
//   const int uart_buffer_size = (1024 * 2);
//   QueueHandle_t uart_queue;
//   // Install UART driver using an event queue here
//   ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

  
//   uart_config_t uart_config = {
//     .baud_rate = 115200,
//     .data_bits = UART_DATA_8_BITS,
//     .parity    = UART_PARITY_DISABLE,
//     .stop_bits = UART_STOP_BITS_1,
//     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//     .rx_flow_ctrl_thresh = 122,
//   };
//   ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

//   ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

//   esp_log_level_set("*", ESP_LOG_DEBUG);


//   vTaskDelay(2000 / portTICK_PERIOD_MS);
//   ESP_LOGI(TAG, "UART initialized successfully");

// }

// void loop() {
//   vTaskDelay(500/portTICK_PERIOD_MS);
//   ESP_LOGI(TAG, "UART initialized successfully");

//   memset(controller_data, 0, sizeof(controller_data));
//   uart_read(controller_data);

//   ESP_LOGI(TAG, "Received data from UART:");
//     for (int i = 0; i < sizeof(controller_data); i++) {
//     ESP_LOGI(TAG, "%02X ", controller_data[i]);
//   }

// }

// esp_err_t uart_read (uint8_t* data_storage);