#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Adafruit_DPS310.h>
#include <Wire.h>

#include <sensors.h>
#include <madgwickfilter.h>
#include <pids.h>
#include <marg.h>

// #define mag_offset_x -70.35
// #define mag_offset_y -76.35
// #define mag_offset_z -16.35
// #define mag_scale_x 55.65
// #define mag_scale_y 118.20
// #define mag_scale_z 104.10
  
Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp = dps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = dps.getPressureSensor();

sensors_event_t accelerometer;
sensors_event_t gyrometer;
sensors_event_t magnetometer;
sensors_event_t temperature;

sensors_event_t temp_event;
sensors_event_t pressure_event;

madgwick_ahrs_t marg;


void setup(void) {

  float imu_offset_x = 0.0f;  // Adjust to your actual offset (e.g., 1cm)
  float imu_offset_y = 0.0f;
  float imu_offset_z = 0.0f;

  float cal_mag_x = 0;
  float cal_mag_y = 0;
  float cal_mag_z = 0;

  marg.beta = 0.1f;  // Adjust beta as needed (typically 0.1 to 1.0)
  marg.zeta = 0.0f;  // Zeta for magnetometer bias (usually 0)
  marg.q[0] = 1.0f;  // w
  marg.q[1] = 0.0f;  // x
  marg.q[2] = 0.0f;  // y
  marg.q[3] = 0.0f;  // z

  pid_vars yaw = {0};
  pid_vars roll = {0};
  pid_vars pitch = {0};  

  vTaskDelay(1500 / portTICK_PERIOD_MS);
  printf("ICM20948 demo\n");
  Serial.begin(115200);
  Wire.setPins(7, 8); //sda,scl
  icm20948_init(&icm);

  // while (!Serial)
  //   delay(10); // will pause Zero, Leonardo, etc until serial console opens

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  dps310_init(&dps);

  while(1){


  icm.getEvent(&accelerometer, &gyrometer, &temperature, &magnetometer);

  // Apply magnetometer calibration
  float mx_cal = (magnetometer.magnetic.x - mag_offset_x) / mag_scale_x;
  float my_cal = (magnetometer.magnetic.y - mag_offset_y) / mag_scale_y;
  float mz_cal = (magnetometer.magnetic.z - mag_offset_z) / mag_scale_z;
  if (dps.temperatureAvailable()) {
    dps_temp->getEvent(&temp_event);
  }
  // Reading pressure also reads temp so don't check pressure
  // before temp!
  if (dps.pressureAvailable()) {
    dps_pressure->getEvent(&pressure_event);
  }


    // Serial.print(F("DPS 310 Temperature = "));
    // Serial.print(temp_event.temperature);
    // Serial.println(" *C");
    // Serial.println();

    // Serial.print(F("DPS 310 Pressure = "));
    // Serial.print(pressure_event.pressure);
    // Serial.println(" hPa"); 
    // Serial.println();
    // Serial.println();

  

    float roll = 0.0, pitch = 0.0, yaw = 0.0;

    imu_filter(accelerometer.acceleration.x, accelerometer.acceleration.y, accelerometer.acceleration.z, gyrometer.gyro.x, gyrometer.gyro.y, gyrometer.gyro.z);
    eulerAngles(q_est, &roll, &pitch, &yaw);

    // printf("6DOF: roll: %f, pitch: %f, yaw: %f\n\n", roll, pitch, yaw);

    // static uint64_t last_time = 0;
    // uint64_t now = esp_timer_get_time();
    // float deltat = (last_time == 0) ? 0.01f : (now - last_time) / 1000000.0f;
    // last_time = now;

    MadgwickQuaternionUpdate(accelerometer.acceleration.x, accelerometer.acceleration.y, accelerometer.acceleration.z, gyrometer.gyro.x, gyrometer.gyro.y, gyrometer.gyro.z, mx_cal, my_cal, mz_cal, measurement_delay_us / 100000.0f, &marg);
      ToEulerAngles(&marg, &roll, &pitch, &yaw);

      
      printf("9DOF: roll: %f, pitch: %f, yaw: %f\n\n", roll, pitch, yaw);

      Serial.println(); 
      Serial.println();
      vTaskDelay(10 / portTICK_PERIOD_MS);
    
    }
}

void loop() {
   
}

// YOU CAN DO IT FARES!






















































  // Serial.print("\t\tICM20948 Temperature ");
  // Serial.print(temp.temperature);
  // Serial.println(" deg C");

  // /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("\t\tAccel X: ");
  // Serial.print(accel.acceleration.x);
  // Serial.print(" \tY: ");
  // Serial.print(accel.acceleration.y);
  // Serial.print(" \tZ: ");
  // Serial.print(accel.acceleration.z);
  // Serial.println(" m/s^2 ");

  // Serial.print("\t\tMag X: ");
  // Serial.print(mag.magnetic.x);
  // Serial.print(" \tY: ");
  // Serial.print(mag.magnetic.y);
  // Serial.print(" \tZ: ");
  // Serial.print(mag.magnetic.z);
  // Serial.println(" uT");

  // /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("\t\tGyro X: ");
  // Serial.print(gyro.gyro.x);
  // Serial.print(" \tY: ");
  // Serial.print(gyro.gyro.y);
  // Serial.print(" \tZ: ");
  // Serial.print(gyro.gyro.z);
  // Serial.println(" radians/s ");
  // Serial.println();



















//delay(100);

  //  Serial.print(temp.temperature);
  //
  //  Serial.print(",");
  //
  //  Serial.print(accel.acceleration.x);
  //  Serial.print(","); Serial.print(accel.acceleration.y);
  //  Serial.print(","); Serial.print(accel.acceleration.z);
  //
  //  Serial.print(",");
  //  Serial.print(gyro.gyro.x);
  //  Serial.print(","); Serial.print(gyro.gyro.y);
  //  Serial.print(","); Serial.print(gyro.gyro.z);
  //
  //  Serial.print(",");
  //  Serial.print(mag.magnetic.x);
  //  Serial.print(","); Serial.print(mag.magnetic.y);
  //  Serial.print(","); Serial.print(mag.magnetic.z);

  //  Serial.println();
  //
  //  delayMicroseconds(measurement_delay_us);










































//start of icm20948 init function ==========================================
  // Serial.println("Adafruit ICM20948 test!");

  // // Try to initialize!
  // if (!icm.begin_I2C(0x69, &Wire, 0)) {

  //   Serial.println("Failed to find ICM20948 chip");
  //   while (1) {
  //     delay(10);
  //   }
  // }
  // Serial.println("ICM20948 Found!");
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  // Serial.print("Accelerometer range set to: ");
  // switch (icm.getAccelRange()) {
  // case ICM20948_ACCEL_RANGE_2_G:
  //   Serial.println("+-2G");
  //   break;
  // case ICM20948_ACCEL_RANGE_4_G:
  //   Serial.println("+-4G");
  //   break;
  // case ICM20948_ACCEL_RANGE_8_G:
  //   Serial.println("+-8G");
  //   break;
  // case ICM20948_ACCEL_RANGE_16_G:
  //   Serial.println("+-16G");
  //   break;
  // }
  // Serial.println("OK");

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  // Serial.print("Gyro range set to: ");
  // switch (icm.getGyroRange()) {
  // case ICM20948_GYRO_RANGE_250_DPS:
  //   Serial.println("250 degrees/s");
  //   break;
  // case ICM20948_GYRO_RANGE_500_DPS:
  //   Serial.println("500 degrees/s");
  //   break;
  // case ICM20948_GYRO_RANGE_1000_DPS:
  //   Serial.println("1000 degrees/s");
  //   break;
  // case ICM20948_GYRO_RANGE_2000_DPS:
  //   Serial.println("2000 degrees/s");
  //   break;
  // }

  // //  icm.setAccelRateDivisor(4095);
  // uint16_t accel_divisor = icm.getAccelRateDivisor();
  // float accel_rate = 1125 / (1.0 + accel_divisor);

  // Serial.print("Accelerometer data rate divisor set to: ");
  // Serial.println(accel_divisor);
  // Serial.print("Accelerometer data rate (Hz) is approximately: ");
  // Serial.println(accel_rate);

  // //  icm.setGyroRateDivisor(255);
  // uint8_t gyro_divisor = icm.getGyroRateDivisor();
  // float gyro_rate = 1100 / (1.0 + gyro_divisor);

  // Serial.print("Gyro data rate divisor set to: ");
  // Serial.println(gyro_divisor);
  // Serial.print("Gyro data rate (Hz) is approximately: ");
  // Serial.println(gyro_rate);

  // // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  // Serial.print("Magnetometer data rate set to: ");
  // switch (icm.getMagDataRate()) {
  // case AK09916_MAG_DATARATE_SHUTDOWN:
  //   Serial.println("Shutdown");
  //   break;
  // case AK09916_MAG_DATARATE_SINGLE:
  //   Serial.println("Single/One shot");
  //   break;
  // case AK09916_MAG_DATARATE_10_HZ:
  //   Serial.println("10 Hz");
  //   break;
  // case AK09916_MAG_DATARATE_20_HZ:
  //   Serial.println("20 Hz");
  //   break;
  // case AK09916_MAG_DATARATE_50_HZ:
  //   Serial.println("50 Hz");
  //   break;
  // case AK09916_MAG_DATARATE_100_HZ:
  //   Serial.println("100 Hz");
  //   break;
  // }
  // Serial.println();

  //end of icm20948 init function ==========================================