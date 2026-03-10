#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <adafruit_dps310.h>
#include <sensors.h>


void icm20948_init(Adafruit_ICM20948 *icm){
     Serial.println("Adafruit ICM20948 test!");

  // Try to initialize!
  if (!icm->begin_I2C(0x69, &Wire, 0)) {

    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");
  icm->setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm->getAccelRange()) {
  case ICM20948_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.println("OK");

  icm->setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm->getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm->getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm->getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm->getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }
  
  Serial.println("ICM20948 INITIALISATION SUCCESSFUL :)");
  vTaskDelay(1000 / portTICK_PERIOD_MS);

}

void dps310_init(Adafruit_DPS310 *dps){
    Serial.println("DPS310");
    if (! dps->begin_I2C(0x77, &Wire)) {
      Serial.println("Failed to find DPS");
      //while (1) yield();
    }
      
    Serial.println("DPS OK!");

    // Setup highest precision
    dps->configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps->configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

    Adafruit_Sensor *dps_temp = dps->getTemperatureSensor();
    Adafruit_Sensor *dps_pressure = dps->getPressureSensor();

    dps_temp->printSensorDetails();
    dps_pressure->printSensorDetails();
    Serial.println("DPS310 INITIALISATION SUCCESSFUL :)");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// float convert_pressure_to_altitude(float pressure_hPa, float sea_level_pressure_hPa) {
//   float
// }

void calibrate_magnetometer(Adafruit_ICM20948 *icm, float *offset_x, float *offset_y, float *offset_z, float *scale_x, float *scale_y, float *scale_z) {
  Serial.println("Calibrating magnetometer... Rotate the device in all directions for 10 seconds");

  float min_x = 10000.0, max_x = -10000.0;
  float min_y = 10000.0, max_y = -10000.0;
  float min_z = 10000.0, max_z = -10000.0;

  sensors_event_t accel, gyro, temp, mag;

  unsigned long start_time = millis();
  while (millis() - start_time < 10000) {
    icm->getEvent(&accel, &gyro, &temp, &mag);

    min_x = min(min_x, mag.magnetic.x);
    max_x = max(max_x, mag.magnetic.x);
    min_y = min(min_y, mag.magnetic.y);
    max_y = max(max_y, mag.magnetic.y);
    min_z = min(min_z, mag.magnetic.z);
    max_z = max(max_z, mag.magnetic.z);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  *offset_x = (min_x + max_x) / 2.0;
  *offset_y = (min_y + max_y) / 2.0;
  *offset_z = (min_z + max_z) / 2.0;

  *scale_x = (max_x - min_x) / 2.0;
  *scale_y = (max_y - min_y) / 2.0;
  *scale_z = (max_z - min_z) / 2.0;

  Serial.printf("Magnetometer calibration complete! Offsets: X=%.2f, Y=%.2f, Z=%.2f Scales: X=%.2f, Y=%.2f, Z=%.2f\n", *offset_x, *offset_y, *offset_z, *scale_x, *scale_y, *scale_z);
}
