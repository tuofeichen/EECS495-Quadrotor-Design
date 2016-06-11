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


#include <curses.h>
#include <stdio.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/time.h>


#include "imu.h"

#define frequency 25000000.0
#define LED0 0x6			//LED0 start register
#define LED0_ON_L 0x6		//LED0 output and brightness control byte 0
#define LED0_ON_H 0x7		//LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8		//LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9		//LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4	// For the other 15 channels

#define IMU_CUTOFF 70
#define PWM_NEUTRAL 1400 // neutral pwm
#define PWM_MAX 1700     // max pwm
#define PWM_MIN 1000
#define I_MAX 100        // max integrated error

// #define LOGGING

int execute,pwm;
double A = 0.001;
int P_pitch = 10;
double I_pitch = 0.001;
int D_pitch = 950;

int P_roll = 10;
double I_roll = 0.005;
int D_roll = 1050;

int P_yaw = 200;
double I_yaw = 10;

struct data
{
    int keypress;
    float pitch;
    float roll;
    float yaw;
    float thrust;
    int sequence_num;
 //   int toggle;
 //   int tune;
};

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


void kill_motor(int pwm,uint8_t channel)
 {
     int on_value=0;

     int time_on_us=999;
     uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

     wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
     wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
     wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
     wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);

}

void trap(int signal)
{
   execute=0;
   printf("ending program\n\r");
}

int main (int argc, char *argv[])
{
    execute=1;
    //Initialize user input shared memory
    int segment_id; 
    data* shared_memory; 
    struct shmid_ds shmbuffer; 
    int segment_size; 
    const int shared_segment_size = 0x6400; 
    int smhkey=33222;

    /* Allocate a shared memory segment.  */ 
    segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666); 
    /* Attach the shared memory segment.  */ 
    shared_memory = (data*) shmat (segment_id, 0, 0); 
    printf ("shared memory attached at address %p\n", shared_memory); 
    /* Determine the segment's size. */ 
    shmctl (segment_id, IPC_STAT, &shmbuffer); 
    segment_size = shmbuffer.shm_segsz; 
    printf ("segment size: %d\n", segment_size); 

	signal(SIGINT, &trap);
    struct timespec te;
    int address;
    int mag,imu;
    // int data;
	// int display;
    int roll_pos_pwm  = PWM_NEUTRAL;
    int roll_neg_pwm  = PWM_NEUTRAL;
    int pitch_pos_pwm = PWM_NEUTRAL;
    int pitch_neg_pwm = PWM_NEUTRAL;


    wiringPiSetup () ;

    // gyro init 
    long time_curr = 0;
    long time_prev = 0;
    long time_ctrl = 0;
    long diff_ctrl = 0;
    double delta_roll_gyro=0;
    double delta_pitch_gyro=0;
    double yaw_gyro = 0;

    long int x_offset = 0;
    long int y_offset = 0;
    long int z_offset = 0;

    // acc init
    double pitch_acc = 0;
    double roll_acc = 0;

    double roll_offset = 0;
    double pitch_offset = 0;

    float diff;

    // complimentary filter
    double roll  = 0;
    double roll_prev = 0;
    double d_roll = 0;
    double i_roll = 0;
    double pitch_prev = 0;
    double pitch = 0;
    double d_pitch = 0;
    double i_pitch = 0;
    double pitch_err;
    double roll_err;
    double yaw_err; 
    double i_yaw=0;


    int disp = 0;
    // error condition
    bool imu_limit = 0;
    bool timeout = 0;
    bool arming = 0;
    bool calibration = 0;

    int  seq = 0;

#ifdef LOGGING
    FILE *f;
    f = fopen("PID_controller.csv","w");
    fprintf(f,"pitch_setpoint,pitch,delta_pitch_gyro,pitch_acc,roll_setpoint,roll,delta_roll_gyro,roll_acc\n");
#endif
    //setup pwm and imu
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
            data s= *shared_memory; //Retrieved shared memory
            if(s.keypress==32){
                execute=0;
            }

            //get current time in nanoseconds
            timespec_get(&te,TIME_UTC);
            time_curr=te.tv_nsec;
            
            // update ctrl time out
            if((seq != s.sequence_num) || (time_ctrl == 0))
            {
                time_ctrl = time_curr;
                seq = s.sequence_num;
            }

            diff_ctrl = (time_curr - time_ctrl);
            
            if(diff_ctrl < 0)
            {
                diff_ctrl+=1000000000;
            }

            timeout = diff_ctrl>250000000;
                
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

            roll_acc = roll_acc -  roll_offset;
            pitch_acc = pitch_acc - pitch_offset;


            comp_filter(A, delta_roll_gyro,  roll_acc,  roll);
            comp_filter(A, delta_pitch_gyro, pitch_acc, pitch);
            imu_limit = imu_error(pitch, roll, double(IMU_CUTOFF));
            

            roll_err = roll - s.roll; 
            d_roll = roll - roll_prev;
            roll_prev = roll;
            
            pitch_err = pitch - s.pitch;
    	    d_pitch = pitch - pitch_prev;
            pitch_prev = pitch;

            yaw_err = yaw_gyro - s.yaw; //controlling yaw rate and not angle


            if (i_roll+I_roll*roll > 0)
                i_roll = ((i_roll+I_roll*roll_err)>=I_MAX) ? (I_MAX):(i_roll+I_roll*roll_err);
            else
                i_roll = ((i_roll+I_pitch*roll_err)<=-I_MAX) ? (-I_MAX):(i_roll+I_roll*roll_err);

            if (i_pitch+I_pitch*pitch > 0)
                i_pitch = ((i_pitch+I_pitch*pitch_err)>=I_MAX) ? (I_MAX):(i_pitch+I_pitch*pitch_err);
            else
                i_pitch = ((i_pitch+I_pitch*pitch_err)<=-I_MAX) ? (-I_MAX):(i_pitch+I_pitch*pitch_err);

            if (i_yaw+I_yaw*yaw > 0)
                i_yaw = ((i_yaw+I_yaw*yaw_err)>=I_MAX) ? (I_MAX):(i_yaw+I_yaw*yaw_err);
            else
                i_yaw = ((i_yaw+I_yaw*yaw_err)<=-I_MAX) ? (-I_MAX):(i_yaw+I_yaw*yaw_err);

            //Yaw added to whichever set of propellors spin clockwise to induce counter-clockwise yaw
            pitch_pos_pwm = s.thrust - yaw_err*P_yaw - i_yaw*I_yaw - d_pitch * D_pitch - pitch_err*P_pitch - i_pitch;
            pitch_neg_pwm = s.thrust - yaw_err*P_yaw - i_yaw*I_yaw + d_pitch * D_pitch + pitch_err*P_pitch + i_pitch;
	        pitch_pos_pwm= (pitch_pos_pwm>=PWM_MAX) ? (PWM_MAX):(pitch_pos_pwm);
            pitch_neg_pwm= (pitch_neg_pwm>=PWM_MAX) ? (PWM_MAX):(pitch_neg_pwm);
            pitch_pos_pwm= (pitch_pos_pwm<=PWM_MIN) ? (PWM_MIN):(pitch_pos_pwm);
            pitch_neg_pwm= (pitch_neg_pwm<=PWM_MIN) ? (PWM_MIN):(pitch_neg_pwm);
                       
    	    roll_pos_pwm = s.thrust + yaw_err*P_yaw + i_yaw*I_yaw  - d_roll * D_roll - roll_err*P_roll - i_roll;
            roll_neg_pwm = s.thrust + yaw_err*P_yaw + i_yaw*I_yaw  + d_roll * D_roll + roll_err*P_roll + i_roll;
            roll_pos_pwm= (roll_pos_pwm>=PWM_MAX) ? (PWM_MAX):(roll_pos_pwm);
            roll_neg_pwm= (roll_neg_pwm>=PWM_MAX) ? (PWM_MAX):(roll_neg_pwm);
            roll_pos_pwm= (roll_pos_pwm<=PWM_MIN) ? (PWM_MIN):(roll_pos_pwm);
            roll_neg_pwm= (roll_neg_pwm<=PWM_MIN) ? (PWM_MIN):(roll_neg_pwm);

         
            if(!(disp%50)){
                // printf("pos pwm is %d neg is %d \n\r",roll_pos_pwm, roll_neg_pwm);
                // printf("roll %f roll acc %f ",roll,roll_acc*180/3.1415);
                // printf ("roll gyro %f\n\r",delta_roll_gyro);
                // printf("pos pwm is %d neg is %d \n\r",pitch_pos_pwm, pitch_neg_pwm);
                // printf("pitch %f pitch acc %f ",pitch,pitch_acc*180/3.1415);
                // printf ("pitch gyro %f\n\r",delta_pitch_gyro);
                // printf ("time_ctrl: %d time_curr: %d \n\r",time_ctrl,time_curr);
                // printf ("roll: %f, pitch: %f\n",roll,pitch);
                // printf ("yaw setpoint: %f, yaw_gyro: %f\n",s.yaw,yaw_gyro);
                // printf ("time diff: %f\n\r", diff);
            #ifdef LOGGING
                //pitch_setpoint,pitch,delta_pitch_gyro,pitch_acc,roll_setpoint,roll,delta_roll_gyro,roll_acc
                fprintf(f, "%f,%f,%f,%f,%f,%f,%f,%f\n", s.pitch,pitch,delta_pitch_gyro,pitch_acc*180/3.1415,s.roll,roll,delta_roll_gyro,roll_acc*180/3.1415);
            #endif

            }   

            disp++;

            timeout = timeout || (diff > 5e-1) && (time_prev !=0);

            // send pwm command
            
            if(s.keypress==33)
                arming = 0;
            if(s.keypress==34)
                arming = 1;

            if(s.keypress==35)
                calibration=1;

            if (!arming){
                set_PWM(pwm,0,1000);
                set_PWM(pwm,1,1000);
                set_PWM(pwm,2,1000);
                set_PWM(pwm,3,1000);
            }
            else {
                set_PWM(pwm,0,pitch_pos_pwm);//1000);
                set_PWM(pwm,1,pitch_neg_pwm);
                set_PWM(pwm,2,roll_pos_pwm);
                set_PWM(pwm,3,roll_neg_pwm);
    	    }

            if (calibration){
                tune_imu(imu, x_offset, y_offset, z_offset, roll_offset, pitch_offset, A);
                printf ("New offset values:\n\r x_offset: %d, y_offset: %d, z_offset: %d\n\r roll_offset: %f, pitch_offset: %f \n\r",x_offset, y_offset, z_offset, roll_offset, pitch_offset);
                shared_memory->keypress=0;
                calibration = 0;
            }

            //remember the current time
            time_prev=time_curr;
            }
        }

  	     // exit program when the error conditons are met
        set_PWM(pwm,0,1000);
        delay(100);
        set_PWM(pwm,1,1000);
        delay(100);
        set_PWM(pwm,2,1000);
        delay(100);
        set_PWM(pwm,3,1000);
        delay(100);
    	
        kill_motor(pwm,0);
	    kill_motor(pwm,1);
	    kill_motor(pwm,2);
	    kill_motor(pwm,3);


        printf("motors killed, imu error: %d, timeout error %d \n\r",imu_limit,timeout);


        return 0;
}
