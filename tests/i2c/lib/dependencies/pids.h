#include <Arduino.h>

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