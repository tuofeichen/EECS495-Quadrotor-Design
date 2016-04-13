
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>

#include <termios.h>
#include <signal.h>
#include <stdlib.h>

#define  CALIBRATION_CNT 1000 // Samples for calibration

// Control-C Interrupt Handler
static volatile int exit_flag = 0;
void intHandler(int sig)
{

printf("Exiting the program by control C\n\r");
exit_flag = 1;

}


int main (int argc, char *argv[])
{
  //open file for writing to if needed
	FILE *f;
	f=fopen("roll_plot.csv","a+");
	fprintf(f,"roll_gyro, roll_acc, roll_filtered\n"); // write header


    struct timespec te;
    int address;
    int mag,imu;
    int data;


    wiringPiSetup () ;

    //setup imu on I2C
    imu=wiringPiI2CSetup (0x6B) ; //accel/gyro address
    //read who am i register
  	int whoami = wiringPiI2CReadReg8(imu,0x0F);
		//	printf("I am: 0x %x",whoami);
		if(imu==-1)
    {
            printf("cant connect to I2C device\n");
            return -1;
    }

   else
    {
				// turn on interrupt
			  signal(SIGINT,intHandler);
			  //turn on acc by writing 0xC0 to address 0x10 and 0x20
        wiringPiI2CWriteReg8(imu,0x10,0xC0); //turn on and set accell output
        //turn on and set gyro output
				wiringPiI2CWriteReg8(imu,0x20,0xC8);

				// gyro initialization
        float pitch_angle_gyro=0;
        float roll_angle_gyro=0;
        float yaw_angle_gyro=0;
				int z_rate_gyro = 0;
				int y_rate_gyro = 0;
				int x_rate_gyro = 0;
				float delta_x_rotate=0;
				float delta_y_rotate=0;
				float delta_z_rotate=0;

				// timer initialization
        long time_curr;
        long time_prev;


				int p_cnt= 0;
				int cali_cnt = CALIBRATION_CNT;


				long int z_offset = 0;
			  long int x_offset = 0;

			// acc initialization
				int x_acc = 0;
				int y_acc = 0;
				int z_acc = 0;
				double pitch_angle_acc = 0;
				double roll_angle_acc = 0;

			//complimentary filter
				double roll =0;
				double A= 0.02;
				while(cali_cnt--){
			            z_rate_gyro=wiringPiI2CReadReg16(imu,0x1C);
			            if(z_rate_gyro>0x8000){
			                z_rate_gyro=  z_rate_gyro ^ 0xffff;
			                z_rate_gyro= -z_rate_gyro-1;

			            }
				 z_offset += z_rate_gyro;

				x_rate_gyro=wiringPiI2CReadReg16(imu,0x18);
			            if(x_rate_gyro>0x8000)
			            {
			                x_rate_gyro = x_rate_gyro ^ 0xffff;
			                x_rate_gyro = -x_rate_gyro-1;

			            }
			         x_offset += x_rate_gyro;
			}
	// average the offset
	z_offset = z_offset/CALIBRATION_CNT;
	x_offset = x_offset/CALIBRATION_CNT;
	printf("what is the x-offset? %d\n\r",x_offset);


        while(!exit_flag)
        {

            //read in gyro z rate
            z_rate_gyro=wiringPiI2CReadReg16(imu,0x1C); // read z gyro
	    			x_rate_gyro=wiringPiI2CReadReg16(imu,0x18); // read x gyro

            //convert to 2's complement
            if(z_rate_gyro>0x8000)
            {
                z_rate_gyro=z_rate_gyro ^ 0xffff;
                z_rate_gyro=-z_rate_gyro-1;
            }

	    		 if(x_rate_gyro>0x8000)
            {
                x_rate_gyro = x_rate_gyro ^ 0xffff;
                x_rate_gyro = -x_rate_gyro-1;
            }

            //subtract from calibration value
            z_rate_gyro -= z_offset;
	    			x_rate_gyro -= x_offset;

	    			//convert to dps
            float z_angle_dps = z_rate_gyro*((double)500/32767);
            float x_angle_dps = x_rate_gyro*((double)500/32767);

 						//get current time in nanoseconds
            timespec_get(&te,TIME_UTC);
            time_curr=te.tv_nsec;

            //compute time since last execution
            float diff=time_curr-time_prev;
            //check for rollover
            if(diff<=0){
                diff+=1000000000;
            }
            //convert to seconds
            diff=diff/1000000000;

            //compute amount of rotation since last execution
            delta_z_rotate = z_angle_dps*diff;
	    			delta_x_rotate = x_angle_dps*diff;

						//gyro integration
            yaw_angle_gyro  += delta_z_rotate;
	    			roll_angle_gyro += delta_x_rotate;

						// read from accelerometer
						x_acc = wiringPiI2CReadReg16(imu,0x28);
						y_acc = wiringPiI2CReadReg16(imu,0x2A);
						z_acc = wiringPiI2CReadReg16(imu,0x2C);

						if(x_acc>0x8000){
					                x_acc= x_acc ^ 0xffff;
					                x_acc= -x_acc-1;
					            }
						if(z_acc>0x8000){
					                z_acc= z_acc ^ 0xffff;
					                z_acc= -z_acc-1;
					            }
						if(y_acc>0x8000){
					                y_acc= y_acc  ^ 0xffff;
													y_acc= -y_acc -1;
						}

						// changed x and y direction to follow the convention on the board
						roll_angle_acc = atan2(-y_acc,z_acc);
						pitch_angle_acc = atan2(-x_acc,z_acc);

						// complimentary filter
						roll = (roll_angle_acc/3.1415f*180) * A + (1-A)*(delta_x_rotate+roll);

						// don't output every loop
						p_cnt++;
	  				if(!(p_cnt%1000)){
		        //print to file if desired
	           fprintf(f, "%f,", roll_angle_gyro);
						 fprintf(f, "%f,", roll_angle_acc);
			 		 	 fprintf(f, "%f\n",roll);
	  		 		}
            //remember the current time
            time_prev=time_curr;
        }

  	 }
        return 0;

}
