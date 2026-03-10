// #include <Arduino.h>
// #include <HardwareSerial.h>

// // #define RX_PIN 19
// // #define TX_PIN 17

// #define RX_PIN 3
// #define TX_PIN 4

// HardwareSerial HC12(1);
//   //17, 19); // HC-12 TX Pin, HC-12 RX Pin

// void setup() {
//   vTaskDelay(1000/portTICK_PERIOD_MS);
//   Serial.begin(9600);             // Serial port to computer
//   HC12.begin(9600,SERIAL_8N1, RX_PIN, TX_PIN);
//   Serial.print("HC12 TEST");
// }

// void loop() {
//   while (HC12.available()) {        // If HC-12 has data
//     Serial.write(HC12.read());      // Send the data to Serial monitor
//   }
//   while (Serial.available()) {      // If Serial monitor has data
//     HC12.write(Serial.read());      // Send that data to HC-12
//   }
// }




#include <Arduino.h>
#include <driver/uart.h>
#include <esp_err.h>
#include <esp32-hal-log.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define RX_PIN 3
#define TX_PIN 4

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
        .baud_rate = 9600, //115200,
        .data_bits = UART_DATA_8_BITS,
        .parity   = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    controller_vars controller = {0};

    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
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

    }


}

void loop() {

}

