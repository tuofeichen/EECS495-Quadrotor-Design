// imu.cpp
#include "imu.h"
#include <wiringPiI2C.h>
#include <math.h>

#define CALI_SAMPLES 100 // # OF calibration samples 

double imu_measure(int dt)
{
    int delta_time = dt;
}
void init_imu(int imu,long int &x_off, long int &y_off, long int &z_off){

    wiringPiI2CWriteReg8(imu,0x10,0xC0);  //turn on and set gyro/ accel output
    wiringPiI2CWriteReg8(imu,0x20,0xC8);
    int cali_cnt = CALI_SAMPLES;
    int val_comp = 0;
    int x_rate = 0;
    int y_rate = 0;
    int z_rate = 0;

    long int x_offset = 0;
    long int y_offset = 0;
    long int z_offset = 0;

    // calculate gyroscope reading offset as time average
    while(cali_cnt--)
    {
        x_rate=wiringPiI2CReadReg16(imu,0x18);
        x_rate = two_comp(x_rate);
        x_offset += x_rate;

        y_rate=wiringPiI2CReadReg16(imu,0x1A);
        y_rate = two_comp(y_rate);
        y_offset += y_rate;

        z_rate=wiringPiI2CReadReg16(imu,0x1C);
        z_rate = two_comp(z_rate);
        z_offset += z_rate;
    }

    y_offset = y_offset/CALI_SAMPLES;
    x_offset = x_offset/CALI_SAMPLES;
    z_offset = z_offset/CALI_SAMPLES;

    x_off = x_offset;
    y_off = y_offset;
    z_off = z_offset;
    
}

void accel_measure(int imu, double& roll_acc, double& pitch_acc)
{
    int x_acc = wiringPiI2CReadReg16(imu,0x28);
    int y_acc = wiringPiI2CReadReg16(imu,0x2A);
    int z_acc = wiringPiI2CReadReg16(imu,0x2C);
    x_acc = two_comp(x_acc);
    y_acc = two_comp(y_acc);
    z_acc = two_comp(z_acc);

    roll_acc = atan2(-y_acc,z_acc);
    pitch_acc = atan2(-x_acc,z_acc);

}

void gyro_measure(int imu, double& delta_roll_gyro, double& delta_pitch_gyro, double& yaw_gyro, int x_offset, int y_offset, int z_offset, float diff)
{
    float x_angle_dps, y_angle_dps, z_angle_dps;
    float delta_z_rotate;
    // float roll_gyro, pitch_gyro, yaw_angle_gyro;

    int x_rate  = wiringPiI2CReadReg16(imu,0x18);
    int y_rate  = wiringPiI2CReadReg16(imu,0x1A);
    int z_rate  = wiringPiI2CReadReg16(imu,0x1C);

    x_rate = two_comp (x_rate);
    y_rate = two_comp (y_rate);
    z_rate = two_comp (z_rate);
    
    x_rate -= x_offset;
    y_rate -= y_offset;
    z_rate -= z_offset;

    //convert to dps
    x_angle_dps = x_rate*((double)500/32767);
    y_angle_dps = y_rate*((double)500/32767);
    z_angle_dps = z_rate*((double)500/32767);
 //compute amount of rotation since last execution
    delta_roll_gyro  = x_angle_dps*diff;
    delta_pitch_gyro = y_angle_dps*diff;
    

    delta_z_rotate = z_angle_dps*diff;
    yaw_gyro += delta_z_rotate;

}
void comp_filter(double A, double sens_proj, double sens_corr, double& output)
{
    output = (sens_corr/3.1415f*180) * A + (1-A)*(sens_proj+output);
}
bool imu_error(double pitch, double roll, double cutoff)
{
    return ((pitch>cutoff)||(pitch<-cutoff)||(roll>cutoff)||(roll<-cutoff));
}

int two_comp(int val)
{
    int val_comp = val;
        if(val_comp>0x8000)
            {
                val_comp = val_comp ^ 0xffff;
                val_comp = -val_comp-1;
            }
    return val_comp;
}