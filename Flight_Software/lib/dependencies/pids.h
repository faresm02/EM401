#include <Arduino.h>
#include <math.h>
#include <driver/ledc.h>

#define M1 1
#define m1_chan LEDC_CHANNEL_0
#define M2 2
#define m2_chan LEDC_CHANNEL_1
#define M3 9
#define m3_chan LEDC_CHANNEL_2
#define M4 8
#define m4_chan LEDC_CHANNEL_3

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
// #define LEDC_OUTPUT_IO          (1) // Define the output GPIO
// #define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_CLK_SRC            LEDC_AUTO_CLK
#define LEDC_FREQUENCY          (1600) // Frequency in Hertz. Set frequency at 4 kHz

typedef struct {
    double P;
    double I;   
    double D;
    double Kp;
    double Ki;
    double Kd;
    double E;      
    double prevE;   
    double prevTime;    
    double pulse;   
    double filterD;
    double pv;
    double sp;
    float r;
    double gyropv;
} pid_vars;

void get_pids(pid_vars *pid);

void pwm_init(int motor_pin, ledc_channel_t channel);

void update_pwm(float duty, ledc_channel_t channel);

