

typedef struct madgwick_ahrs {
    volatile float beta;
    volatile float zeta;
    volatile float q[4];
} madgwick_ahrs_t;

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat, madgwick_ahrs_t *quat);
void ToEulerAngles(madgwick_ahrs_t *quat,float *roll, float *pitch, float *yaw);
