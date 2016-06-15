#ifndef IMU__H__
#define IMU__H__

double imu_measure(int dt);

void init_imu(int imu,long int &x_off, long int &y_off,long int& z_off );

void tune_imu(int imu,long int &x_off, long int &y_off, long int &z_off, double &roll_off, double &pitch_off, double A);

void accel_measure(int imu, double& roll_acc, double& pitch_acc);

void gyro_measure(int imu, double& roll_gyro, double& pitch_gyro, double& yaw_gyro, int x_offset, int y_offset, int z_offset, float diff);

void comp_filter(double A, double sens_proj, double sens_corr, double& output);

bool imu_error(double pitch, double roll, double cutoff);

int two_comp(int val);

#endif