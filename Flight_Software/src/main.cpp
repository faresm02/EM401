#include <Arduino.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_pm.h"
#include <esp_timer.h>
#include <pids.h>

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>

#include <sensors.h>
#include <madgwickfilter.h>
#include <marg.h>

#define max_pulse 250
#define min_pulse 125

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp = dps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = dps.getPressureSensor();

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t temp;

sensors_event_t temp_event;
sensors_event_t pressure_event;


// IMU offset from center of rotation (in meters, body frame)
float imu_offset_x = 0.0f;  // Adjust to your actual offset (e.g., 1cm)
float imu_offset_y = 0.0f;
float imu_offset_z = 0.0f;

float cal_mag_x = 0;
float cal_mag_y = 0;
float cal_mag_z = 0;

// float mag_offset_x = 0;
// float mag_offset_y = 0;
// float mag_offset_z = 0;

// float mag_scale_x = 0.0;
// float mag_scale_y = 0.0;
// float mag_scale_z = 0.0;

madgwick_ahrs_t marg;


void setup() {

  pid_vars yaw_pid = {0};
  pid_vars roll_pid = {0};
  pid_vars pitch_pid = {0}; 

  float m1_pulse = 0.0;
  float m2_pulse = 0.0;
  float m3_pulse = 0.0;
  float m4_pulse = 0.0;
  float thrust = 0.0;

  float roll = 0.0; 
  float pitch = 0.0;
  float yaw = 0.0;

  // Initialize Madgwick filter
  marg.beta = 0.1f;  // Adjust beta as needed (typically 0.1 to 1.0)
  marg.zeta = 0.0f;  // Zeta for magnetometer bias (usually 0)
  marg.q[0] = 1.0f;  // w
  marg.q[1] = 0.0f;  // x
  marg.q[2] = 0.0f;  // y
  marg.q[3] = 0.0f;  // z

  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for Serial to initialize
  Serial.println("Hello, World!\n");
  vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second

  printf("ICM20948 demo\n");
  Wire.setPins(7, 6); //sda,scl

  icm20948_init(&icm);
  dps310_init(&dps);

  //calibrate_magnetometer(&icm, &mag_offset_x, &mag_offset_y, &mag_offset_z, &mag_scale_x, &mag_scale_y, &mag_scale_z);

  Serial.println("Magnetometer calibration Values:");
  Serial.printf("Offsets: X=%.2f, Y=%.2f, Z=%.2f\n", mag_offset_x, mag_offset_y, mag_offset_z);
  Serial.printf("Scales: X=%.2f, Y=%.2f, Z=%.2f\n", mag_scale_x, mag_scale_y, mag_scale_z);




  // init pwm 
  pwm_init(M1, m1_chan);
  pwm_init(M2, m2_chan);
  pwm_init(M3, m3_chan);
  pwm_init(M4, m4_chan);
  // Set duty to 0%
  vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait for 5 seconds


  //PID coefficients
  roll_pid.Kp = 2.50;
  roll_pid.Kd = 1.3;
  roll_pid.Ki = 0;

  pitch_pid.Kp = 2.50;
  pitch_pid.Kd = 1.5;
  pitch_pid.Ki = 0;

  yaw_pid.Kp = 0; 
  yaw_pid.Kd = 0; 
  yaw_pid .Ki = 0; 

  roll_pid.r = 1;
  pitch_pid.r = 1;
  yaw_pid.r = 0.95;

  while(1){

  icm.getEvent(&accel, &gyro, &temp, &mag);

  // if (dps.temperatureAvailable()) {
  //   dps_temp->getEvent(&temp_event);
  // }
  // // Reading pressure also reads temp so don't check pressure
  // // before temp!
  // if (dps.pressureAvailable()) {
  //   dps_pressure->getEvent(&pressure_event);
  // }

  // Apply magnetometer calibration
  float mx_cal = (mag.magnetic.x - mag_offset_x) / mag_scale_x;
  float my_cal = (mag.magnetic.y - mag_offset_y) / mag_scale_y;
  float mz_cal = (mag.magnetic.z - mag_offset_z) / mag_scale_z;

  MadgwickQuaternionUpdate(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, mx_cal, my_cal, mz_cal, measurement_delay_us / 100000.0f, &marg);

  ToEulerAngles(&marg, &roll, &pitch, &yaw);
  // printf("roll: %f, pitch: %f, yaw: %f\n", roll, pitch, yaw);

  roll_pid.sp = 0;
  pitch_pid.sp = 0;
  yaw_pid.sp = 0;

  roll_pid.pv = roll;
  pitch_pid.pv = pitch;
//  yaw_pid.pv = yaw;
  yaw_pid.pv = 0;


  get_pids(&roll_pid);
  get_pids(&pitch_pid);
  get_pids(&yaw_pid);

    //set motor values
  m1_pulse = 125 + thrust - pitch_pid.pulse - roll_pid.pulse - yaw_pid.pulse /*-cont_yaw*/;   // Front left motor 
  m3_pulse = 125 + thrust - pitch_pid.pulse + roll_pid.pulse + yaw_pid.pulse /*+ cont_yaw*/;   // Front right motor 
  m2_pulse = 125 + thrust + pitch_pid.pulse - roll_pid.pulse + yaw_pid.pulse /*+ cont_yaw*/;   // Back left motor
  m4_pulse = 125 + thrust + pitch_pid.pulse + roll_pid.pulse - yaw_pid.pulse /*- cont_yaw*/;   // Back right motor 
 
  printf("\nPID Outputs:\n\n");
  printf("Roll P: %f, I: %f, D: %f\n", roll_pid.P, roll_pid.I, roll_pid.D);
  printf("Pitch P: %f, I: %f, D: %f\n", pitch_pid.P, pitch_pid.I, pitch_pid.D);
  printf("\n");

if (m1_pulse > max_pulse){m1_pulse = max_pulse;}
if (m1_pulse < min_pulse){m1_pulse = min_pulse;}

if (m2_pulse > max_pulse){m2_pulse = max_pulse;}
if (m2_pulse < min_pulse){m2_pulse = min_pulse;}

if (m3_pulse > max_pulse){m3_pulse = max_pulse;}
if (m3_pulse < min_pulse){m3_pulse = min_pulse;}

if (m4_pulse > max_pulse){m4_pulse = max_pulse;}
if (m4_pulse < min_pulse){m4_pulse = min_pulse;}
  
  update_pwm(m1_pulse, m1_chan);
  update_pwm(m2_pulse, m2_chan); 
  update_pwm(m3_pulse, m3_chan);
  update_pwm(m4_pulse, m4_chan); 

   
  printf("\nThrust: %f\n", thrust);
  printf("M1: %f\n", m1_pulse);
  printf("M2: %f\n", m2_pulse);
  printf("M3: %f\n", m3_pulse);
  printf("M4: %f\n", m4_pulse);

  vTaskDelay(1 / portTICK_PERIOD_MS); // Delay for stability
  }
}

void loop() {

}


  // imu_filter(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
  // eulerAngles(q_est, &roll, &pitch, &yaw);
  // printf("roll: %f, pitch: %f, yaw: %f\n", roll, pitch, yaw);







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