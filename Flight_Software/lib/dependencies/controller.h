#ifndef CONTROLLER_H
#define CONTROLLER_H

// #include <WiFi.h>
// #include <esp_now.h>
#include <Arduino.h>
#include <pids.h>

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
} controller_packet_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float roll_kp, roll_kd, roll_ki;
    float pitch_kp, pitch_kd, pitch_ki;
    float yaw_kp, yaw_kd, yaw_ki;
    uint8_t pid_axis;
    uint8_t pid_param;
} telemetry_packet_t;

extern controller_packet_t rxPacket;
extern telemetry_packet_t txTelemetry;
extern volatile bool newData;

// PID tuning state
extern uint8_t pid_axis;    // 0=roll, 1=pitch, 2=yaw
extern uint8_t pid_param;   // 0=Kp, 1=Kd, 2=Ki
extern bool r1_prev;
extern bool l1_prev;
extern uint8_t dpad_prev;


void onReceive(const uint8_t *mac, const uint8_t *data, int len);

void controller_espnow_init();

void process_controller_data(pid_vars *roll_pid, pid_vars *pitch_pid, pid_vars *yaw_pid, float roll, float pitch, float yaw);
#endif