/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "example";

#define I2C_MASTER_SCL_IO           6       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           7       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU9250_SENSOR_ADDR         0x69        /*!< Address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR   0x00        /*!< Register addresses of the "who am I" register */
#define MPU9250_PWR_MGMT_1_REG_ADDR 0x6        /*!< Register addresses of the power management register */
#define MPU9250_RESET_BIT           0x41

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t mpu9250_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu9250_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

void app_main(void)
{
    uint8_t data[2];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
    ESP_ERROR_CHECK(mpu9250_register_read(dev_handle, MPU9250_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Demonstrate writing by resetting the MPU9250 */
    ESP_ERROR_CHECK(mpu9250_register_write_byte(dev_handle, MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}






























// #include <stdio.h>
// #include <unistd.h>
// #include <string.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/timers.h"

// #include "esp_log.h"
// #include "esp_system.h"

// #include "icm20948.h"
// #include "icm20948_i2c.h"

// #define TAG "i2c_agmt"
// #define sda_pin 7	
// #define scl_pin 6

// /* i2c bus configuration */
// i2c_config_t conf = {
// 	.mode = I2C_MODE_MASTER,
// 	.sda_io_num = (gpio_num_t) sda_pin,
// 	.sda_pullup_en = GPIO_PULLUP_ENABLE,
// 	.scl_io_num = (gpio_num_t) scl_pin,
// 	.scl_pullup_en = GPIO_PULLUP_ENABLE,
// 	.master.clk_speed = 400000,
// 	.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
// };

// /* ICM 20948 configuration */
// icm0948_config_i2c_t icm_config = {
// 	.i2c_port = I2C_NUM_1,
// 	.i2c_addr = ICM_20948_I2C_ADDR_AD1
// };

// void print_agmt(icm20948_agmt_t agmt)
// {
//   	ESP_LOGI(TAG, "Acc: [ %d, %d, %d ] Gyr: [ %d, %d, %d ] Mag: [ %d, %d, %d ] Tmp: [ %d ]", 
// 		agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z,
// 		agmt.gyr.axes.x, agmt.gyr.axes.y, agmt.gyr.axes.z,
// 		agmt.mag.axes.x, agmt.mag.axes.y, agmt.mag.axes.z,
// 		agmt.tmp.val
// 	);
// }

// void app_main(void)
// {
// 	icm20948_device_t icm;

// 	/* setup i2c */
// 	ESP_ERROR_CHECK(i2c_param_config(icm_config.i2c_port, &conf));
// 	ESP_ERROR_CHECK(i2c_driver_install(icm_config.i2c_port, conf.mode, 0, 0, 0));
	
// 	/* setup ICM20948 device */
// 	icm20948_init_i2c(&icm, &icm_config);
		
// 	/* check ID */
//     while (icm20948_check_id(&icm) != ICM_20948_STAT_OK)
// 	{
// 		ESP_LOGE(TAG, "check id failed");
// 		vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// 	ESP_LOGI(TAG, "check id passed");

// 	/* check whoami */
// 	icm20948_status_e stat = ICM_20948_STAT_ERR;
// 	uint8_t whoami = 0x00;
// 	while ((stat != ICM_20948_STAT_OK) || (whoami != ICM_20948_WHOAMI))
// 	{
// 		whoami = 0x00;
// 		stat = icm20948_get_who_am_i(&icm, &whoami);
// 		ESP_LOGE(TAG, "whoami does not match (0x %d). Halting...", whoami);
// 		vTaskDelay(1000 / portTICK_PERIOD_MS);
// 	}

// 	/* Here we are doing a SW reset to make sure the device starts in a known state */
// 	icm20948_sw_reset(&icm);
// 	vTaskDelay(250 / portTICK_PERIOD_MS);

// 	icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);

// 	// Set Gyro and Accelerometer to a particular sample mode
// 	// optiona: SAMPLE_MODE_CONTINUOUS. SAMPLE_MODE_CYCLED
// 	icm20948_set_sample_mode(&icm, sensors, SAMPLE_MODE_CONTINUOUS); 

// 	// Set full scale ranges for both acc and gyr
// 	icm20948_fss_t myfss;
// 	myfss.a = GPM_2;   // (icm20948_accel_config_fs_sel_e)
// 	myfss.g = DPS_250; // (icm20948_gyro_config_1_fs_sel_e)
// 	icm20948_set_full_scale(&icm, sensors, myfss);

// 	// Set up DLPF configuration
// 	icm20948_dlpcfg_t myDLPcfg;
// 	myDLPcfg.a = ACC_D473BW_N499BW;
// 	myDLPcfg.g = GYR_D361BW4_N376BW5;
// 	icm20948_set_dlpf_cfg(&icm, sensors, myDLPcfg);

// 	// Choose whether or not to use DLPF
// 	icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_ACC, false);
// 	icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_GYR, false);

// 	// Now wake the sensor up
// 	icm20948_sleep(&icm, false);
// 	icm20948_low_power(&icm, false);

//     /* loop */
//     while(1)
// 	{
// 		vTaskDelay(100 / portTICK_PERIOD_MS);

// 		icm20948_agmt_t agmt; // = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0}};
// 		if (icm20948_get_agmt(&icm, &agmt) == ICM_20948_STAT_OK) {
// 			print_agmt(agmt);
// 		} else {
// 			ESP_LOGE(TAG, "Uh oh");
// 		}        
//     }
// }






















// #include <stdio.h>
// #include "sdkconfig.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "driver/i2c_master.h"

// #define sda_pin 1
// #define scl_pin 2

// #define ICM_addr 0x69
// #define ICM_who 0x00
// #define I2C_MASTER_TIMEOUT_MS 1000

// static const char *TAG = "Out:";


// void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
// {
//         i2c_master_bus_config_t bus_config = {
//         .i2c_port = I2C_NUM_0,
//         .sda_io_num = sda_pin,
//         .scl_io_num = scl_pin,
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .glitch_ignore_cnt = 7,
//         .flags.enable_internal_pullup = true,
//         };
//         i2c_new_master_bus(&bus_config, bus_handle);

//         i2c_device_config_t dev_config = {
//         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//         .device_address = ICM_addr,
//         .scl_speed_hz = 100000,
//         };
//         i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle);
// };

// void icm_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
// }


// void app_main() {

// vTaskDelay(3000/portTICK_PERIOD_MS);

// ESP_LOGI(TAG,"Hello ICM20948 !");

// uint8_t data[2];
// i2c_master_bus_handle_t bus_handle;
// i2c_master_dev_handle_t dev_handle;
// i2c_master_init(&bus_handle, &dev_handle);
// ESP_LOGI(TAG, "I2C initialized successfully");

// icm_register_read(dev_handle, ICM_who, data, 1);
//     ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);


// //i2c_master_transmit_receive(dev_handle, ICM_who, 1, data, 1, I2C_MASTER_TIMEOUT_MS);

// // uint8_t write_buf[2] = {ICM_who, data};
// // i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);

// }
 
// void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);

// void icm_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);