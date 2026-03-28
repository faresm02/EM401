#ifndef CONTROLLER_H
#define CONTROLLER_H

// #include <WiFi.h>
// #include <esp_now.h>
#include <Arduino.h>
#include <pids.h>
#define CIRCLE
#define SQUARE
#define TRIANGLE
#define CROSS
#define L1
#define R1
#define DPAD_UP
#define DPAD_DOWN
#define DPAD_LEFT
#define DPAD_RIGHT  

extern uint8_t controllerMAC[];

typedef struct {
    int16_t throttle;
    int16_t brake;
    int16_t lx;
    int16_t ly;
    int16_t rx;
    int16_t ry;
    uint16_t buttons;
    uint8_t dpad;
    // bool transmitter_connection;
} controller_packet_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float roll_sp;
    float pitch_sp;
    float yaw_rate_sp;
    float roll_kp, roll_kd, roll_ki;
    float pitch_kp, pitch_kd, pitch_ki;
    float yaw_kp, yaw_kd, yaw_ki;
    float roll_trim;
    float pitch_trim;
    uint8_t pid_axis;
    uint8_t pid_param;
    uint8_t pid_loop; // 0 for angle, 1 for rate
    // bool receiver_connection;
} telemetry_packet_t;

extern controller_packet_t rxPacket;
extern telemetry_packet_t txTelemetry;
extern volatile bool newData;

extern pid_vars *active_pid;
extern uint8_t pid_axis;
extern uint8_t pid_param;
extern bool use_rate_loop;
extern bool trim_mode;
extern float roll_trim;
extern float pitch_trim;    

// PID tuning state
extern uint8_t pid_axis;    // 0=roll, 1=pitch, 2=yaw
extern uint8_t pid_param;   // 0=Kp, 1=Kd, 2=Ki
extern bool r1_prev;
extern bool l1_prev;
extern uint8_t dpad_prev;

extern pid_vars *active_pid;

extern const char* active_axis_str;
extern const char* active_loop_str;
extern const char* active_param_str;


void onReceive(const uint8_t *mac, const uint8_t *data, int len);

void controller_espnow_init();

//void process_controller_data(pid_vars *roll_pid, pid_vars *pitch_pid, pid_vars *yaw_pid, float roll, float pitch, float yaw);
void process_controller_data(pid_vars *roll_angle,pid_vars *roll_rate,pid_vars *pitch_angle,pid_vars *pitch_rate,pid_vars *yaw_rate,float roll, float pitch, float yaw);


#endif