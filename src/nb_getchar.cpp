#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <signal.h>

struct termios initial_settings,
               new_settings;

static volatile int exit_prog = 0;

void intHandler(int sig)
{
	printf("Enter interrupt!\n");
	exit_prog = 1;
}

int main(int argc, char *argv[])
{
  int n;
  unsigned char key;
  tcgetattr(0,&initial_settings);

  new_settings = initial_settings;
  new_settings.c_lflag &= ~ICANON;
  new_settings.c_lflag &= ~ECHO;
  //new_settings.c_lflag &= ~ISIG;
  new_settings.c_cc[VMIN] = 0;
  new_settings.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &new_settings);

  signal(SIGINT,intHandler);
  while(!exit_prog)
  {

    n = getchar();
    if(n != EOF)
    {
      key = n;
      if(key == 27)  /* Escape key pressed */
      {
        break;
      }

	printf("The key pressed is %c\n\r",key);
    }
  }

  tcsetattr(0, TCSANOW, &initial_settings);

  return(0);
}
