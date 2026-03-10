#include <pids.h>
#include <driver/ledc.h>
#include <math.h>


void get_pids(pid_vars *pid){
    double dt;
    uint64_t currTime = xTaskGetTickCount();
    
    //Calculate error
    pid->E = pid->sp-pid->pv;

    //Find change in time
    dt = (currTime-pid->prevTime)/1000000.0;
    if (dt < 0.000001) {dt = 0.000001;}   

    //Proportional part thrust
    pid->P = pid->Kp*(pid->E);

    //Intergral part thrust
    pid->I += pid->Ki*pid->E*dt;

    //Locks intergral part contribution so it doesn't infinitly build
    if(pid->I < -30){pid->I = -30;}
    if(pid->I > 30){pid->I = 30;}

    //Low pass filter to remove noise
    pid->filterD = pid->filterD * (1.0 - pid->r) + pid->r * pid->gyropv;
    //Derivative part thrust
    pid->D = pid->Kd*pid->filterD;
    
    //clamps D term so if the changing degrees is small it doesnt react
    if (pid->gyropv < 15 && pid->gyropv > -15){pid->D=0;}

    //summs the parts together to get final pulse
    pid->pulse = pid->P + pid->I + pid->D;
    //if the error
    if (fabs(pid->E)< 1){pid->pulse = 0;}
    
    pid->prevTime = currTime;
    pid->prevE = pid->E;

}

void pwm_init(int motor_pin, ledc_channel_t channel)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_CLK_SRC,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = motor_pin,
        .speed_mode     = LEDC_MODE,
        .channel        = channel,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0, // Set duty to 0%
        .hpoint          = 0,

    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

}

void update_pwm(float duty, ledc_channel_t channel){
    // float duty_scaled = ((duty/5)+20)/100;
    float duty_scaled = (duty-125)/125;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, (8192)*duty_scaled)); // Set duty to 0%
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
}