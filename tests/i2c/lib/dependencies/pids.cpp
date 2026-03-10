#include <pids.h>
#include <math.h>
// #include <Arduino.h>

void PIDStruct(pid_vars *pid){
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