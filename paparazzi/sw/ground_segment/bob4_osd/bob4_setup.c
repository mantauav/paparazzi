
// BOB4H OSD Setup Application
//
// Copyright 2011 Aurora Flight Sciences
//  James Peverill   8/11/2010
//
//  This app sets up the BOB4 OSD board with an aurora/skate themed 
// startup script and burns it to flash
//

#include <stdlib.h>
#include <stdio.h>
#include <getopt.h>

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>

#include <ctype.h>
#include <unistd.h>
#include <strings.h>
#include <string.h>

#include <math.h>


#define FALSE 0
#define TRUE 1

//aspect ratio between BOB resolution aspect and screen aspect
//  looks around 1 when TV set to wide mode
#define ASPECT 1


//serial port settings
#define BAUDRATE B115200

unsigned char outputbuffer[100];

#define FDPRINTF(port,...) { sprintf(outputbuffer,__VA_ARGS__); if( write(port,outputbuffer,strlen(outputbuffer)) <= 0 ) { printf(" Term closed"); exit(1); } }

// serial port globals
int TTY;

// verbose mode
int verbose = 0;

void PrintOSD()
{
  //Clear Screen
  FDPRINTF(TTY,"\033[2J");

  //Set Aurora Splash Screen
  FDPRINTF(TTY,"\033[0z");  
  FDPRINTF(TTY,"\033[5;7H");
  FDPRINTF(TTY,"AURORA FLIGHT SCIENCES\r\n");
  FDPRINTF(TTY,"\033[6z");
  FDPRINTF(TTY,"      Skate UAS");

  FDPRINTF(TTY,"\033[0z");
  FDPRINTF(TTY,"\033[10,0H");
  FDPRINTF(TTY,"     Waiting for GCS connection");

  //Wait 1 second or until keypress
  FDPRINTF(TTY,"\033[1;2|");
}

void SetupOSD()
{
  //put in startup screen string
  FDPRINTF(TTY,"\033X");

  PrintOSD();

  FDPRINTF(TTY,"\033\\");

  //set boot script and flash
  FDPRINTF(TTY,"\033[8v");
  FDPRINTF(TTY,"\033[1v");

}


void au_osd_print_help( void ) {
  printf("au_osd_setup help, calling options:\n");
  printf(" -p <device>     Use <device> com port, default is /dev/ttyUSB1\n");
  printf(" -v              Enable verbose mode\n");
  printf(" -t              Output Splash Screen, do not flash\n");
}


ttyConfigure ( int fd ) {
  
  struct termios oldtio,newtio;

  tcgetattr(fd,&oldtio); /* save current serial port settings */
  bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */
        
  /* 
     BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
     CRTSCTS : output hardware flow control (only used if the cable has
     all necessary lines. See sect. 7 of Serial-HOWTO)
     CS8     : 8n1 (8bit,no parity,1 stopbit)
     CLOCAL  : local connection, no modem contol
     CREAD   : enable receiving characters
  */
  newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  
  /*
    IGNPAR  : ignore bytes with parity errors
    ICRNL   : map CR to NL (otherwise a CR input on the other computer
    will not terminate input)
    otherwise make device raw (no other input processing)
  */
  newtio.c_iflag = IGNPAR | ICRNL | IXON | IXOFF;
  
  /*
    Raw output.
  */
  newtio.c_oflag = 0;
  
  /*
    ICANON  : enable canonical input
    disable all echo functionality, and don't send signals to calling program
  */
  newtio.c_lflag = ICANON;
  
  /* 
     now clean the modem line and activate the settings for the port
  */
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd,TCSANOW,&newtio);

}

main (int argc, char**argv)
{
  int print_mode = 0;
  const char* com_port = 0;

	char c;
	while ( (c = getopt (argc, argv, "p:vt")) != EOF) {
		switch (c) {
		case 'p':
		  com_port = optarg;
		  break;
		case 'v':
		  verbose = 1;
		  printf("Verbose debug on\n");
		  break;
		case 't':
		  print_mode = 1;
		  printf("Showing splash screen\n");
		  break;
		default:
		  au_osd_print_help();
		  abort();
		}
	}

	/* handling of environment variable */
	if (!com_port)
	  com_port = "/dev/ttyUSB1";

	printf("AU_OSD: Opening port: %s\n",com_port);

	// open serial port
	TTY = open(com_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (TTY < 0)
	  {
	    printf("AU_OSD: ");
	    perror(com_port);
	    exit(-1);
	  }

	ttyConfigure(TTY);

	if(print_mode == 0) 
	  SetupOSD();
	else
	  PrintOSD();

}
