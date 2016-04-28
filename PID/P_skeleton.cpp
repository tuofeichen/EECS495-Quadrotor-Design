#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <signal.h>
#include <curses.h>
#include <termios.h>
#include <stdlib.h>

#include "imu.h"

#define frequency 25000000.0
#define LED0 0x6			//LED0 start register
#define LED0_ON_L 0x6		//LED0 output and brightness control byte 0
#define LED0_ON_H 0x7		//LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8		//LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9		//LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4	// For the other 15 channels

#define IMU_CUTOFF 30
#define PWM_NEUTRAL 1250 // neutral pwm
#define PWM_MAX 1400     // max pwm
int execute,pwm;

//init the pwm board
void init_pwm(int pwm)
{
    float freq =400.0*.95;
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    uint8_t prescale = floor(prescaleval+0.5);
    int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
    int sleep	= settings | 0x10;
    int wake 	= settings & 0xef;
    int restart = wake | 0x80;
    wiringPiI2CWriteReg8(pwm, 0x00, sleep);
    wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
    wiringPiI2CWriteReg8(pwm, 0x00, wake);
    delay(10);
    wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
}



//turn on the motor
void init_motor(int pwm,uint8_t channel)
{
	int on_value=0;

	int time_on_us=900;
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

     time_on_us=1200;
     off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1000;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

}

//set the pwm value of the motor
void set_PWM(int pwm, uint8_t channel, float time_on_us)
{
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);

}

//when cntrl+c pressed, kill motors
void trap(int signal)
{
    // turn off  all 4 motors!! 
    // error condition dealt with in the main function
    execute=0;
    printf("ending program\n\r");
}

int main (int argc, char *argv[])
{
	execute=1;

	signal(SIGINT, &trap);
    struct timespec te;
    int address;
    int mag,imu;
    int data;
	// int display;
    int roll_pos_pwm=PWM_NEUTRAL;
    int roll_neg_pwm=PWM_NEUTRAL;
    int pitch_pos_pwm=PWM_NEUTRAL;
    int pitch_neg_pwm=PWM_NEUTRAL;


    wiringPiSetup () ;

    // gyro init 
    long time_curr=0;
    long time_prev=0;
    double delta_roll_gyro=0;
    double delta_pitch_gyro=0;
    double yaw_gyro = 0;

    long int x_offset = 0;
    long int y_offset = 0;
    long int z_offset = 0;

    // acc init
    double pitch_acc = 0;
    double roll_acc = 0;

    float diff;

    // complimentary filter
    double roll  = 0;
    double roll_prev = 0;
    double pitch_prev = 0;
    double pitch = 0;
    double d_pitch = 0;

    double A= 0.02;

    // p controller gain
    int P = 0;
    int D = 500;
    // error condition
    bool imu_limit = 0;
    bool timeout = 0;

    //setup for pwm
    pwm=wiringPiI2CSetup (0x40);  //connect pwm board to imu
    imu=wiringPiI2CSetup (0x6B) ; //accel/gyro address

    // check i2c connections
    if(imu==-1||pwm==-1)
    {
            printf("cant connect to I2C device %d %d\n",imu,pwm);
            return -1;
    }
    else
    {
        //init pwm , imu and motors
        init_pwm(pwm);
        init_imu(imu,x_offset, y_offset, z_offset);     

        init_motor(pwm,3);
        init_motor(pwm,2);
        init_motor(pwm,1);
        init_motor(pwm,0);

        // error checking
        while(execute&&(!imu_limit)&&(!timeout))
        {

            //get current time in nanoseconds
            timespec_get(&te,TIME_UTC);
            time_curr=te.tv_nsec;

            //compute time since last execution
            diff=time_curr-time_prev;
            //check for rollover
            if(diff<=0)
            {
                diff+=1000000000;
            }
            //convert to seconds
            diff=diff/1000000000;

            gyro_measure(imu, delta_roll_gyro, delta_pitch_gyro, yaw_gyro, x_offset, y_offset, z_offset, diff);
            
            accel_measure(imu, roll_acc, pitch_acc);   //compute amount of rotation since last execution
            
            comp_filter(A, delta_roll_gyro,  roll_acc,  roll);
            comp_filter(A, delta_pitch_gyro, pitch_acc, pitch);
            imu_limit = imu_error(pitch, roll, double(IMU_CUTOFF));
            
            // derivative term
            d_pitch = pitch - pitch_prev;
            pitch_prev = pitch;

            pitch_pos_pwm = PWM_NEUTRAL + d_pitch * D - pitch*P;
            pitch_neg_pwm = PWM_NEUTRAL - d_pitch * D + pitch*P;
            
            pitch_pos_pwm= (pitch_pos_pwm>=PWM_MAX) ? (PWM_MAX):(pitch_pos_pwm);
            pitch_neg_pwm= (pitch_neg_pwm<=PWM_MAX) ? (PWM_MAX):(pitch_neg_pwm);
            
            timeout = (diff > 5e-3) && (time_prev !=0);


            // cap on roll PWM 
            // roll_pos_pwm= ((PWM_NEUTRAL - roll*P)>=PWM_MAX) ? (PWM_MAX):(PWM_NEUTRAL - roll*P);
            // roll_neg_pwm= ((PWM_NEUTRAL + roll*P)>=PWM_MAX) ? (PWM_MAX):(PWM_NEUTRAL + roll*P);

            // pitch_pos_pwm= ((PWM_NEUTRAL - pitch*P)>=PWM_MAX) ? (PWM_MAX):(PWM_NEUTRAL - pitch*P);
            // pitch_neg_pwm= ((PWM_NEUTRAL + pitch*P)>=PWM_MAX) ? (PWM_MAX):(PWM_NEUTRAL + pitch*P);



            // send pwm command
            // set_PWM(pwm,2,roll_neg_pwm); //Roll Negative
            // set_PWM(pwm,3,roll_pos_pwm); //Rol Positive
            set_PWM(pwm, 2, pitch_neg_pwm); //Roll Negative
            set_PWM(pwm, 3, pitch_pos_pwm); //Rol Positive


            //remember the current time
            time_prev=time_curr;
            }
        }

  	     // exit program when the error conditons are met
        set_PWM(pwm, 2, 0);
        set_PWM(pwm, 3, 0);
        printf("Exiting loop");

        return 0;
}
