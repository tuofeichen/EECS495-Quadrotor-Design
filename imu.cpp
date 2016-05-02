#include "_imu.hpp"
#include <wiringPiI2C.h>
#include <math.h>

IMU::IMU(){

	_imu=wiringPiI2CSetup (0x6B) ; //accel/gyro address
	wiringPiI2CWriteReg8(_imu,0x10,0xC0);  //turn on and set gyro/ accel output
    wiringPiI2CWriteReg8(_imu,0x20,0xC8);
    int cali_cnt = CALI_SAMPLES;
    int val_comp = 0;
    int x_rate = 0;
    int y_rate = 0;
    int z_rate = 0;

    // calculate gyroscope reading offset as time average
    while(cali_cnt--)
    {
        x_rate=wiringPiI2CReadReg16(_imu,0x18);
        x_rate = two_comp(x_rate);
        _x_offset += x_rate;

        y_rate=wiringPiI2CReadReg16(_imu,0x1A);
        y_rate = two_comp(y_rate);
        _y_offset += y_rate;

        z_rate=wiringPiI2CReadReg16(_imu,0x1C);
        z_rate = two_comp(z_rate);
        _z_offset += z_rate;
    }

    _y_offset = _y_offset/CALI_SAMPLES;
    _x_offset = _x_offset/CALI_SAMPLES;
    _z_offset = _z_offset/CALI_SAMPLES;
}

void IMU::accel_measure()
{
    int x_acc = wiringPiI2CReadReg16(_imu,0x28);
    int y_acc = wiringPiI2CReadReg16(_imu,0x2A);
    int z_acc = wiringPiI2CReadReg16(_imu,0x2C);
    x_acc = two_comp(x_acc);
    y_acc = two_comp(y_acc);
    z_acc = two_comp(z_acc);

    _roll_acc = atan2(-y_acc,z_acc);
    _pitch_acc = atan2(-x_acc,z_acc);

}

void IMU::gyro_measure(float dt)
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

    //compute amount of rotation since last execution, these should all be temperate   
    _d_roll_gyro  = x_angle_dps*dt;
    _d_pitch_gyro = y_angle_dps*dt;
    

    delta_z_rotate = z_angle_dps*dt;
    // only yaw gyro is a member variable 
    yaw_ += delta_z_rotate;

}

void comp_filter(double A)
{
    roll_  = (_roll_acc/3.1415f*180) * A  + (1-A)*(_d_roll_gyro+roll_);
    pitch_ = (_pitch_acc/3.1415f*180) * A + (1-A)*(_d_pitch_gyro+pitch_);
}


bool IMU::measure(float dt, float A, float cutoff)
{
	accel_measure();
	gyro_measure(dt);
	comp_filter(A);
	return (fabs(roll_)>cutoff || fabs(pitch_)>cutoff);
}



