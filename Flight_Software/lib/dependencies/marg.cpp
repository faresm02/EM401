#include <math.h>
#include <marg.h>
#define PI 3.14159265358979323846f

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat, madgwick_ahrs_t *quat)
{
    float q1 = quat->q[0], q2 = quat->q[1], q3 = quat->q[2], q4 = quat->q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;
    static float gbx=0, gby=0, gbz=0;


    // Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;

    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    
    // normalise step magnitude
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    gbx += quat->zeta * s1 * deltat;
    gby += quat->zeta * s2 * deltat;    

    if (fabsf(s3) > 0.01f) {
    gbz += quat->zeta * s3 * deltat;
    } else {
    gbz += quat->zeta * s3 * deltat;
    }

    const float MAX_BIAS = 1.0f;  
    if (gbz >  MAX_BIAS) gbz =  MAX_BIAS;
    if (gbz < -MAX_BIAS) gbz = -MAX_BIAS;

    gx -= gbx;
    gy -= gby;
    gz -= gbz;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - quat->beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - quat->beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - quat->beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - quat->beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    quat->q[0] = q1 * norm;
    quat->q[1] = q2 * norm;
    quat->q[2] = q3 * norm;
    quat->q[3] = q4 * norm;
}


void ToEulerAngles(madgwick_ahrs_t *quat,float *roll, float *pitch, float *yaw) {

    //w = 0, x = 1, y =2, z = 3    
    static float prevYaw = 0;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (quat->q[0] * quat->q[1] + quat->q[2] * quat->q[3]);
    double cosr_cosp = 1 - 2 * (quat->q[1] * quat->q[1] + quat->q[2] * quat->q[2]);
    *roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (quat->q[0] * quat->q[2] - quat->q[3] * quat->q[1]);
    if (fabs(sinp) >= 1)
        *pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        *pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat->q[0] * quat->q[3] + quat->q[1] * quat->q[2]);
    double cosy_cosp = 1 - 2 * (quat->q[2] * quat->q[2] + quat->q[3] * quat->q[3]);
    *yaw = atan2(siny_cosp, cosy_cosp) ;

     if (*yaw-prevYaw <= 0.0261799 && *yaw-prevYaw >= -0.0261799 ) {
        *yaw  = prevYaw;
    }

    if (*yaw > PI) {
        *yaw -= 2 * PI;
    }

    // Convert to degrees
    *roll *= (180.0 / PI);
    *pitch *= (180.0 / PI);
    *yaw *= (180.0 / PI);

    //printf("\n%f\n",prevYaw);
    prevYaw = *yaw;
}