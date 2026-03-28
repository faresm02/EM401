#include <Arduino.h>
#include <Wire.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include <esp_timer.h>
#include <pids.h>

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>

#include <sensors.h>
#include <madgwickfilter.h>
#include <marg.h>
#include <controller.h>

#define LOOP_RATE_HZ     50000
#define LOOP_PERIOD_US   (1000000 / LOOP_RATE_HZ)

#define max_pulse 250
#define min_pulse 125

Adafruit_ICM20948 icm;
// uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp = dps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = dps.getPressureSensor();

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t temp;

sensors_event_t temp_event;
sensors_event_t pressure_event;

esp_timer_handle_t flight_timer;

madgwick_ahrs_t marg; // for 9dof

quaternion quat; // for 6dof


// IMU offset from center of rotation (in meters, body frame)
float imu_offset_x = 0.0f;  // Adjust to your actual offset (e.g., 1cm)
float imu_offset_y = 0.0f;
float imu_offset_z = 0.0f;

float cal_mag_x = 0;
float cal_mag_y = 0;
float cal_mag_z = 0;

const char* axes[] = {"Roll", "Pitch", "Yaw"};
const char* params[] = {"Kp", "Kd", "Ki"};

// float mag_offset_x = 0;
// float mag_offset_y = 0;
// float mag_offset_z = 0;

// float mag_scale_x = 0.0;
// float mag_scale_y = 0.0;
// float mag_scale_z = 0.0;

void setup() {

  // pid_vars yaw_pid = {0};
  pid_vars roll_pid = {0};
  pid_vars pitch_pid = {0}; 

  pid_vars roll_angle = {0};
  pid_vars roll_rate = {0};
  pid_vars pitch_angle = {0};
  pid_vars pitch_rate = {0};
  pid_vars yaw_rate = {0};

  int16_t rx_input=0; 
  int16_t ry_input=0;






// Outer loop - angle (P only)
roll_angle.Kp = 0.750;//1.020;
roll_angle.Ki = 0;//0.4;
roll_angle.Kd = 0;//0.5;
roll_angle.r = 1;

pitch_angle.Kp = 0.750;//1.030;//1.3;
pitch_angle.Ki = 0;
pitch_angle.Kd = 0;
pitch_angle.r = 1;

// Inner loop - rate
roll_rate.Kp = 0.20;//1.0;
roll_rate.Kd = 4.5;//4.5;
roll_rate.Ki = 0;
roll_rate.r = 1;

pitch_rate.Kp = 0.20;//0.5;
pitch_rate.Kd = 5.0;//4.5;
pitch_rate.Ki = 0;
pitch_rate.r = 1;

yaw_rate.Kp = 0.5;
yaw_rate.Ki = 0.0;
yaw_rate.Kd = 1.0;








  float m1_pulse = 0.0;
  float m2_pulse = 0.0;
  float m3_pulse = 0.0;
  float m4_pulse = 0.0;
  float thrust = 0.0;

  float roll = 0.0; 
  float pitch = 0.0;
  float yaw = 0.0;

  float curved_R2 = 0.0  ; // Curved throttle value for better low-end control
  float curved_L2 = 0.0  ; // Curved brake value for better low-end control
  float raw_R2 = 0.0;
  float raw_L2 = 0.0;

  float max_thrust = 55.0f;                          // 125 + 55 = 180 max pulse




  float var_beta;
  // Initialize Madgwick filter
  marg.beta = var_beta = 0.05f;  // Adjust beta as needed (typically 0.1 to 1.0)
  marg.zeta = 0.0f;  // Zeta for magnetometer bias (usually 0)
  marg.q[0] = 1.0f;  // w
  marg.q[1] = 0.0f;  // x
  marg.q[2] = 0.0f;  // y
  marg.q[3] = 0.0f;  // z

  Serial.begin(115200);
  vTaskDelay(100/portTICK_PERIOD_MS); // Wait for Serial to initialize
  Serial.println("Hello, World !\n");
  vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for 1 second


  controller_espnow_init();

  printf("Hello I2C !\n");
  Wire.setPins(7, 8); //sda,scl
  Wire.begin();
  // Wire.setClock(400000);
 Wire.setClock(1000000);
  

  icm20948_init(&icm);
  dps310_init(&dps);

  //calibrate_magnetometer(&icm, &mag_offset_x, &mag_offset_y, &mag_offset_z, &mag_scale_x, &mag_scale_y, &mag_scale_z);

  Serial.println("Magnetometer calibration Values:");
  Serial.printf("Offsets: X=%.2f, Y=%.2f, Z=%.2f\n", mag_offset_x, mag_offset_y, mag_offset_z);
  Serial.printf("Scales: X=%.2f, Y=%.2f, Z=%.2f\n", mag_scale_x, mag_scale_y, mag_scale_z);



  vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for 1 second
  // init pwm 
  pwm_init(M1, m1_chan);
  pwm_init(M2, m2_chan);
  pwm_init(M3, m3_chan);
  pwm_init(M4, m4_chan);
  // Set duty to 0%

  update_pwm(125, m1_chan); // Set duty to 0% after calibration
  update_pwm(125, m2_chan);
  update_pwm(125, m3_chan);
  update_pwm(125, m4_chan);

  vTaskDelay(3000 / portTICK_PERIOD_MS); // Wait for 1 second

  // calibrate_motors(M1, m1_chan);
  // calibrate_motors(M2, m2_chan);
  // calibrate_motors(M3, m3_chan);
  // calibrate_motors(M4, m4_chan);

  // vTaskDelay(10000 / portTICK_PERIOD_MS); // Run motors at full throttle for 5 seconds to calibrate ESCs
  // update_pwm(125, m1_chan); // Set duty to 0% after calibration
  // update_pwm(125, m2_chan);
  // update_pwm(125, m3_chan);
  // update_pwm(125, m4_chan);

  // vTaskDelay(10000/ portTICK_PERIOD_MS); // Wait for 2 seconds

  // After calibration, test each motor individually
// Serial.println("Testing M1 (GPIO 1)");
// update_pwm(140, m1_chan);  // light spin
// vTaskDelay(3000 / portTICK_PERIOD_MS);
// update_pwm(125, m1_chan);

// Serial.println("Testing M2 (GPIO 2)");
// update_pwm(140, m2_chan);
// vTaskDelay(3000 / portTICK_PERIOD_MS);
// update_pwm(125, m2_chan);

// Serial.println("Testing M3 (GPIO 9)");
// update_pwm(140, m3_chan);
// vTaskDelay(3000 / portTICK_PERIOD_MS);
// update_pwm(125, m3_chan);

// Serial.println("Testing M4 (GPIO 10)");
// update_pwm(140, m4_chan);
// vTaskDelay(3000 / portTICK_PERIOD_MS);
// update_pwm(125, m4_chan);


  //PID coefficients
  // roll_pid.Kp = 0.0;
  // roll_pid.Kd = 0.0;
  // roll_pid.Ki = 0;

  // pitch_pid.Kp = 0.0;
  // pitch_pid.Kd = 0.0;
  // pitch_pid.Ki = 0;

  // yaw_pid.Kp = 0; 
  // yaw_pid.Kd = 0; 
  // yaw_pid .Ki = 0; 

  // roll_pid.r = 1;
  // pitch_pid.r = 1;
  // yaw_pid.r = 1;

  uint64_t prev_time = esp_timer_get_time();

  while(1){
    uint64_t now = esp_timer_get_time();
    float deltat = (now - prev_time) / 1000000.0f;
    prev_time = now;

//    //process_controller_data(&roll_pid, &pitch_pid, &yaw_pid, roll_pid.pv, pitch_pid.pv, yaw_pid.pv);
    // process_controller_data(&roll_rate, &pitch_rate, &yaw_pid, roll_angle.pv, pitch_angle.pv, yaw_pid.pv);
process_controller_data(&roll_angle, &roll_rate,&pitch_angle, &pitch_rate,&yaw_rate,roll_angle.pv, pitch_angle.pv, yaw_rate.pv);
    icm.getEvent(&accel, &gyro, &temp, &mag);

    if (dps.temperatureAvailable()) {
      dps_temp->getEvent(&temp_event);
    }
    // Reading pressure also reads temp so don't check pressure
    // before temp!
    if (dps.pressureAvailable()) {
      dps_pressure->getEvent(&pressure_event);
    }

    // // Apply magnetometer calibration
    // float mx_cal = (mag.magnetic.x - mag_offset_x) / mag_scale_x;
    // float my_cal = (mag.magnetic.y - mag_offset_y) / mag_scale_y;
    // float mz_cal = (mag.magnetic.z - mag_offset_z) / mag_scale_z;

    // float accel_norm = accel.acceleration.x * accel.acceleration.x + 
    //                accel.acceleration.y * accel.acceleration.y + 
    //                accel.acceleration.z * accel.acceleration.z;


    // MadgwickQuaternionUpdate(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
    //                          gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
    //                          0, 0, 0, deltat, &marg);

    // MadgwickQuaternionUpdate(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, mx_cal, my_cal, mz_cal, deltat, &marg);
    // ToEulerAngles(&marg, &roll, &pitch, &yaw);

    imu_filter(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, deltat, var_beta);

    eulerAngles(q_est, &roll, &pitch, &yaw);





//    // roll_pid.sp = 0;
//    // pitch_pid.sp = 0;
//    // yaw_pid.sp = 0;

    //   roll_pid.pv = -roll;
    //   pitch_pid.pv = -pitch;

    
    raw_R2 = rxPacket.throttle / 1023.0f;          // 0 to 1
    raw_L2 = (-1) * rxPacket.brake / 1023.0f;             // 0 to 1
    // curved_R2 = raw_R2 * raw_R2 * raw_R2;
    // curved_L2 = raw_L2 * raw_L2 * raw_L2;

    curved_R2 = raw_R2 * 0.003; 
    curved_L2 = raw_L2 * 0.003; 
//    thrust += curved_R2 * max_thrust; 
    
    thrust += curved_R2 * max_thrust + curved_L2 * max_thrust; // Add brake as negative thrust
    if (thrust > max_thrust) {thrust = max_thrust;}
    if (thrust < 0) {thrust = 0;}

  //map controller inputs to setpoints and motor commmands
    // thrust = rxPacket.throttle * (125.0f / 1023.0f);
    rx_input = (abs(rxPacket.rx) > 30) ? rxPacket.rx : 0;
    ry_input = (abs(rxPacket.ry) > 30) ? rxPacket.ry : 0;

    // roll_pid.sp = rx_input * (15.0f / 512.0f);
    // pitch_pid.sp = ry_input * (15.0f / 512.0f);





    // roll_pid.pv = -pitch;
    // pitch_pid.pv = -roll;

//    // roll_pid.pv = pitch;
//    // pitch_pid.pv = roll;

    // roll_pid.pv = roll;
    // pitch_pid.pv = pitch;

//    // roll_pid.gyropv = gyro.gyro.y;
//    // pitch_pid.gyropv = gyro.gyro.x;
//    //  yaw_pid.pv = yaw;
//    // yaw_pid.pv = 0;

    

//    //get_pids(&roll_pid);
//    //get_pids(&pitch_pid);
//    //get_pids(&yaw_pid);

// Outer loop: angle → desired rate
roll_angle.sp = rx_input * (15.0f / 512.0f);
roll_angle.pv = pitch -roll_trim; // Adjust for level flight trim
get_pids(&roll_angle);

pitch_angle.sp = ry_input * (15.0f / 512.0f);
pitch_angle.pv = roll -pitch_trim; // Adjust for level flight trim
get_pids(&pitch_angle);

// Inner loop: desired rate → motor output
roll_rate.sp = roll_angle.pulse;
roll_rate.pv = -gyro.gyro.y;
roll_rate.gyropv = gyro.gyro.y;
get_pids(&roll_rate);

pitch_rate.sp = pitch_angle.pulse;
pitch_rate.pv = gyro.gyro.x;
pitch_rate.gyropv = gyro.gyro.x;
get_pids(&pitch_rate);

    // yaw_pid.sp = 0;
    // yaw_pid.pv = 0;

int16_t lx_input = (abs(rxPacket.lx) > 30) ? rxPacket.lx : 0;
yaw_rate.sp = lx_input * (2.0f / 512.0f);
yaw_rate.pv = gyro.gyro.z;
yaw_rate.gyropv = gyro.gyro.z;
get_pids(&yaw_rate);  



    

//    //   //set motor values
//    // m1_pulse = 125 + thrust - pitch_pid.pulse - roll_pid.pulse - yaw_pid.pulse /*-cont_yaw*/;   // Front left motor 
//    // m3_pulse = 125 + thrust - pitch_pid.pulse + roll_pid.pulse + yaw_pid.pulse /*+ cont_yaw*/;   // Front right motor 
//    // m2_pulse = 125 + thrust + pitch_pid.pulse - roll_pid.pulse + yaw_pid.pulse /*+ cont_yaw*/;   // Back left motor
//    // m4_pulse = 125 + thrust + pitch_pid.pulse + roll_pid.pulse - yaw_pid.pulse /*- cont_yaw*/;   // Back right motor 


m1_pulse = 125 + thrust - pitch_rate.pulse - roll_rate.pulse - yaw_rate.pulse;
m3_pulse = 125 + thrust - pitch_rate.pulse + roll_rate.pulse + yaw_rate.pulse;
m2_pulse = 125 + thrust + pitch_rate.pulse - roll_rate.pulse + yaw_rate.pulse;
m4_pulse = 125 + thrust + pitch_rate.pulse + roll_rate.pulse - yaw_rate.pulse;

    if (m1_pulse > max_pulse){m1_pulse = max_pulse;}
    if (m1_pulse < min_pulse){m1_pulse = min_pulse;}

    if (m2_pulse > max_pulse){m2_pulse = max_pulse;}
    if (m2_pulse < min_pulse){m2_pulse = min_pulse;}

    if (m3_pulse > max_pulse){m3_pulse = max_pulse;}
    if (m3_pulse < min_pulse){m3_pulse = min_pulse;}

    if (m4_pulse > max_pulse){m4_pulse = max_pulse;}
    if (m4_pulse < min_pulse){m4_pulse = min_pulse;}

  static uint64_t kill_time_short = 0;
    bool circle = rxPacket.buttons & 0x0002;
    if (circle) {
        kill_time_short = esp_timer_get_time();
    }

    if ((esp_timer_get_time() - kill_time_short) < 3000000) {  // 3 seconds
        thrust = 0;
        roll_pid.I = 0;
        pitch_pid.I = 0;
        yaw_rate.I = 0;
        roll_rate.I = 0;
        pitch_rate.I = 0;
        roll_angle.I = 0;
        pitch_angle.I = 0;
        yaw_rate.I = 0;
        m1_pulse = 125;
        m2_pulse = 125;
        m3_pulse = 125;
        m4_pulse = 125;
    }

    if(thrust < 2.0f){
        roll_pid.I = 0; 
        pitch_pid.I = 0;
        yaw_rate.I = 0;
        roll_rate.I = 0;
        pitch_rate.I = 0;
        roll_angle.I = 0;
        pitch_angle.I = 0;
        yaw_rate.I = 0;
        m1_pulse = 125;
        m2_pulse = 125;
        m3_pulse = 125;
        m4_pulse = 125; 
    } 
      
    update_pwm(m1_pulse, m1_chan);
    update_pwm(m2_pulse, m2_chan); 
    update_pwm(m3_pulse, m3_chan);
    update_pwm(m4_pulse, m4_chan); 

    // vTaskDelay(1/ portTICK_PERIOD_MS); // Delay for stability
    
    //=================================================================================
    //PRINT BLOCK FOR DEBUGGING 
    // while ((esp_timer_get_time() - now) < 2000) {}
      static uint32_t last_print = 0;
    if (millis() - last_print >= 150) {
        last_print = millis();

      // printf("\nRoll: %f, pitch: %f, yaw: %f\n", roll_pid.pv, pitch_pid.pv, yaw_pid.pv);

      // printf("DPS310 Temp: %f C, Pressure: %f hPa\n", temp_event.temperature, pressure_event.pressure);

      // printf("\nThrust: %f\n", thrust);
      // printf("M1: %f  ", m1_pulse);
      // printf("M2: %f  ", m2_pulse);
      // printf("M3: %f  ", m3_pulse);
      // printf("M4: %f  ", m4_pulse);

      printf("T:%d B:%d LX:%d LY:%d RX:%d RY:%d\n", rxPacket.throttle, rxPacket.brake, rxPacket.lx, rxPacket.ly, rxPacket.rx, rxPacket.ry);

      printf("Editing: %s %s\r\n", axes[pid_axis], params[pid_param]);

      // printf("Roll  Kp:%.3f Kd:%.3f Ki:%.3f  ", roll_pid.Kp, roll_pid.Kd, roll_pid.Ki);
      // printf("Pitch Kp:%.3f Kd:%.3f Ki:%.3f  ", pitch_pid.Kp, pitch_pid.Kd, pitch_pid.Ki);
      // printf("Yaw   Kp:%.3f Kd:%.3f Ki:%.3f  ", yaw_pid.Kp, yaw_pid.Kd, yaw_pid.Ki);

      printf("[%s] %s %s  ",
       use_rate_loop ? "RATE" : "ANGLE",
       (pid_axis == 0 ? "ROLL" : pid_axis == 1 ? "PITCH" : "YAW"),
       (pid_param == 0 ? "Kp" : pid_param == 1 ? "Kd" : "Ki"));

      printf("Kp:%.3f Kd:%.3f Ki:%.3f\n",active_pid->Kp,active_pid->Kd,active_pid->Ki);

      printf("\nDelta t: %f\n",deltat);

      printf("\nRoll  angle P:%.3f  rate P:%.3f D:%.3f pulse:%.3f\n\n", roll_angle.P, roll_rate.P, roll_rate.D, roll_rate.pulse);
      printf("\nPitch angle P:%.3f  rate P:%.3f D:%.3f pulse:%.3f\n\n", pitch_angle.P, pitch_rate.P, pitch_rate.D, pitch_rate.pulse);
      

      printf("Trim R:%.1f P:%.1f  Mode: %s\n", roll_trim, pitch_trim, trim_mode ? "TRIM" : "PID");
      
      // printf("gyro x: %f, y: %f, z: %f\n", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);

      // printf("\n beta: %f\n", var_beta);

      


    }
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


















  //I2C BUS SCAN 
//   #include <Wire.h>
// #include <Arduino.h>

// void setup() {
  
//     Serial.begin(115200);
//     Wire.setPins(7, 8);  // your SDA, SCL
//     Wire.begin();
//     delay(1000);

//     Serial.println("Scanning I2C bus...");
//     for (byte addr = 1; addr < 127; addr++) {
//         Wire.beginTransmission(addr);
//         byte error = Wire.endTransmission();
//         if (error == 0) {
//             Serial.printf("Device found at 0x%02X\n", addr);
//         }
//     }
//     Serial.println("Scan complete");
// }

// void loop() {}
