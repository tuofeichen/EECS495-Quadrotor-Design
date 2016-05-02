
#ifndef IMU__H__
#define IMU__H__


#define CALI_SAMPLES 100 // # of calibration samples 

class IMU{
public:
	IMU();
	int _imu; // address ID 
	bool measure(float dt, float A, float cutoff); // output error 
	
	double roll_; 
	double pitch_; 
	double yaw_; 
private:
int _x_offset; 
int _y_offset;
int _z_offset; 

double _roll_acc;
double _pitch_acc;

double _d_roll_gyro;
double _d_pitch_gyro;



void accel_measure();

void gyro_measure(float dt,double& delta_roll_gyro, double& delta_pitch_gyro);

void comp_filter(float A);

int two_comp(int val);

};

#endif