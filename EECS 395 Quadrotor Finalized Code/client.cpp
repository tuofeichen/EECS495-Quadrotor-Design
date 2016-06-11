#include <curses.h>
#include <stdio.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
//#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <time.h>

struct data
{
	int keypress;
	float pitch;
	float roll;
	float yaw;
	float thrust;
    int sequence_num;
};

int main (int argc, char *argv[])
{

    WINDOW *w =initscr();
    cbreak();
    nodelay(w,TRUE);
	int motor_on=0;
    //shared memory init
    int segment_id;
    data* shared_memory;
    struct shmid_ds shmbuffer;
    int segment_size;
    const int shared_segment_size = 0x6400;
    int smhkey=33222;
    int ch = 0;
    /* Basic initialization of curses lib */
    initscr();
    cbreak();
    noecho(); /* Set this for interactive programs. */
    nonl();
    intrflush(stdscr, FALSE);
    keypad(stdscr, TRUE);

    /* Allocate a shared memory segment.  */
    segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
    /* Attach the shared memory segment.  */
    shared_memory = (data*) shmat (segment_id, 0, 0);
    printf ("shared memory attached at address %p\n", shared_memory);
    /* Determine the segment's size. */
    shmctl (segment_id, IPC_STAT, &shmbuffer);
    segment_size  =               shmbuffer.shm_segsz;
    printf ("segment size: %d\n", segment_size);


    float roll=0;
    float pitch=0;
    float yaw=0;
    float thrust=1500;
    int key=0;
    shared_memory->keypress=0;
    int i=0;

	int debounce=0;
    struct timeval te;
    gettimeofday(&te,NULL);
    long curr_time=te.tv_sec*1000LL+te.tv_usec/1000;
    long heart_beat_timer=te.tv_sec*1000LL+te.tv_usec/1000;


//socket stuff


    int sockfd = 0, n = 0;
    char recvBuff[1024];
    struct sockaddr_in serv_addr;

    if(argc != 2)
    {
        printf("\n Usage: %s <ip of server> \n",argv[0]);
        return 1;
    }

    memset(recvBuff, '0',sizeof(recvBuff));
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Error : Could not create socket \n");
        return 1;
    }

    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(5000);

    if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr)<=0)
    //if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr)<=0)
    {
        printf("\n inet_pton error occured\n");
        return 1;
    }

    if( connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       printf("\n Error : Connect Failed \n");
       return 1;
    }
    //end socket stuff


    while(1)
    {
        //get current time
        gettimeofday(&te,NULL);
        curr_time=te.tv_sec*1000LL+te.tv_usec/1000;


		int n= read(sockfd, recvBuff, sizeof(recvBuff)-1);
		if(n>0)
		{
			recvBuff[n] = 0;
		}


        key=getch();
        if (key!=ERR)
        {

            if(key==32)
            {
                shared_memory->keypress=key;
                printf("\n kill motors and exit program!!\n\r");
            }
            if(key=='a')
            {
                roll+=2.5;
                shared_memory->roll=roll;
                printf("increase roll to %f\n\r",roll);
            }
            if(key=='d')
            {
                roll-=2.5;
                shared_memory->roll=roll;
                printf("decrease roll to %f\n\r",roll);
            }
            if(key=='x')
            {
                roll=0;
                shared_memory->roll=roll;
                printf("reset roll to 0\n\r");
            }
            if(key=='h')
            {
                thrust+=10;
                shared_memory->thrust=thrust;
                printf("increase thrust to %f \n\r",thrust);
            }
            if(key=='n')
            {
                thrust-=10;
                shared_memory->thrust=thrust;
                printf("decrease thrust to %f\n\r",thrust);
            }
            //reset heartbeat timer
            heart_beat_timer=curr_time;
            shared_memory->sequence_num=i++;

        }

		if(n>0)//message received from joystick server
		{
			thrust=1400-1.0*((float)recvBuff[0]-128.0);
			roll =0.15*((float)recvBuff[2]-128.0);
			pitch=0.15*((float)recvBuff[3]-128.0);
			yaw= - 0.001*((float)recvBuff[1]-128.0);
			if(recvBuff[4]>1)
			{
				shared_memory->keypress=32;
				printf("key kill pressed\n\r");
			}
			else if(recvBuff[5]>1)
			{
				shared_memory->keypress=33;
				printf("key pause pressed\n\r");
			}
			else if(recvBuff[6]>1)
			{
				shared_memory->keypress=34;
				printf("key un- pause pressed\n\r");
			}
			else if(recvBuff[7]>1)
			{
				shared_memory->keypress=35;
				printf("calibration pressed\n\r");
			}
			else//no key press
			{
				shared_memory->keypress=36;
			}

			shared_memory->thrust=thrust;
			shared_memory->pitch=pitch;
			shared_memory->roll=roll;
			shared_memory->yaw=yaw;
		//	printf("Thrust %f, Pitch %f, roll %f, yaw %f keypress is %c\n\r ",thrust, pitch, roll, yaw,shared_memory->keypress);
			
            heart_beat_timer=curr_time;
            shared_memory->sequence_num=i++;
			//printf("%d %d %d %d %d %d %d %d \n\r",recvBuff[0],recvBuff[1],recvBuff[2],recvBuff[3],recvBuff[4],recvBuff[5],recvBuff[6],recvBuff[7]);
	}
        //send heartbeat if its been .2 seconds since last message sent
       /* if(curr_time>heart_beat_timer+200)
        {
            printf("heart beat sent\n\r");
            heart_beat_timer=curr_time;
            shared_memory->keypress=36;
            shared_memory->sequence_num=i++;
        }*/
        fflush(stdout);
    }


/* Detach the shared memory segment.  */
shmdt (shared_memory);

/* Deallocate the shared memory segment.  */
endwin();

return 0;
}
