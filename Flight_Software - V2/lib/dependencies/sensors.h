#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>  

// Magnetometer calibration values
// #define mag_offset_x -58.05
// #define mag_offset_y 58.05
// #define mag_offset_z 78.97
// #define mag_scale_x 25.65
// #define mag_scale_y 29.40
// #define mag_scale_z 34.28

//Offsets: X=-70.35, Y=-76.35, Z=-16.35 Scales: X=55.65, Y=118.20, Z=104.10

#define mag_offset_x -70.35
#define mag_offset_y -76.35
#define mag_offset_z -16.35
#define mag_scale_x 55.65
#define mag_scale_y 118.20
#define mag_scale_z 104.10

void icm20948_init(Adafruit_ICM20948 *icm);

void dps310_init(Adafruit_DPS310 *dps);

void calibrate_magnetometer(Adafruit_ICM20948 *icm, float *offset_x, float *offset_y, float *offset_z, float *scale_x, float *scale_y, float *scale_z);


