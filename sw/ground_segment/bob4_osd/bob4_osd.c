// Ivybus->OSD application
//
// Copyright 2011 Aurora Flight Sciences
//  James Peverill   8/11/2010
//
//  This app waits on the ivybus for preprogrammed messages to be sent
//  It then parses them and renders them onto the OSD attached via serial link
//  It is designed to read out to a BOB4 based OSD board
//
//


//
//  messages:
//   BAT   
//   5 BAT 0 125 0 1 306 306 0 
//           dV  flight time in seconds
//   

//  Update History:
//   2/10 - Updated for newest Paparazzi, removed proprietary extensions


#include <stdlib.h>
#include <stdio.h>
#include <getopt.h>
#include <ivy.h>
#include <ivyloop.h>
#include <timer.h>

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


// storage of status variables
unsigned char volts_valid = 1;
float volts = 0.0;

unsigned char flighttime_valid = 1;
int flighttime;

unsigned char altitude_valid = 1;
float altitude = 0.0;

unsigned char groundspeed_valid = 1;
float groundspeed = 0.0;
unsigned char airspeed_valid = 1;
float airspeed = 0.0;

unsigned char telemetry_valid = 1;
char telemetry_status = 0;

unsigned char pitch_angle_valid = 1;
float pitch_angle = 0.0;

unsigned char home_valid = 1;
float home_utm_east = 0.0;
float home_utm_north = 0.0;

//heading_angle_deg is current gps flight heading, degrees clockwise from north
unsigned char heading_valid = 1;
float heading_angle_deg = 0.0;

float compass_heading = 0.0;
float old_compass_heading = 0.0;

unsigned char pos_valid = 1;
float utm_east = 0.0;
float utm_north = 0.0;



// serial port globals
int TTY;


//--------------------------------------------------
// global booleans

// verbose mode
int verbose = 0;
// proprietary mode (turns off proprietary display items)
int proprietary = 0;



void ClearOSD()
{
  //clear out screen
  FDPRINTF(TTY,"\033[2J");
}

volatile int datalink_beat = 0;
//--------------------------------------------------
void Datalink_Heartbeat (IvyClientPtr app, void *data, int argc, char **argv)
{
  datalink_beat = 1;
}



//--------------------------------------------------
void OSD_Heartbeat(TimerId id, void *data, unsigned long delta)
{
  static beat = 0;

  if (beat == 0 ) {
    beat = 1;
    if (datalink_beat == 1) {
      FDPRINTF(TTY,"\033[0;34H*");
    } else {
      FDPRINTF(TTY,"\033[0;34HO");
    }
  } else {
    beat = 0;
    FDPRINTF(TTY,"\033[0;34H ");    
  }

  datalink_beat = 0;

}
  

void SetGenlockOff()
{
  FDPRINTF(TTY,"\033[18;2v");
}

void SetPal()
{
  FDPRINTF(TTY,"\033[16;1v");
}


/* BAT message has:
    centivolts in 2nd
    flight time in 3rd (seconds)
*/ 
void BatteryCallback (IvyClientPtr app, void *data, int argc, char **argv)
{
  if (verbose>0) {
    printf("BAT Received, argc=%u\n",argc);
    printf(" argv=\"%s\"\n",argv[0]);
  }

  char* substring;
  int counter = 0;

  substring = strtok(argv[0]," ");
  while (substring!= NULL) {
    switch (counter) {
    case 1:
      volts = atof(substring)/10;
      volts_valid = 0;
      break;
    case 3: //updated in newer PPZs
      flighttime = atoi(substring);
      flighttime_valid = 0;      
      break;
    }
    counter++;
    substring = strtok(NULL," ");
  }

  if(proprietary == 0) {
    //write out battery voltage
    FDPRINTF(TTY,"\033[0;0H%2.2fV ",volts);
  }

  //write out flight time
  FDPRINTF(TTY,"\033[0;13H  %u:%02u ",flighttime/60,flighttime%60);

}

//
//  ESTIMATOR format is altitude in m, z_dot in m/s
void EstimatorCallback (IvyClientPtr app, void *data, int argc, char **argv)
{  
  static float ground_level=9999;

  if (verbose>0) {
    printf("ESTIMATOR Received, argc=%u\n",argc);
    printf(" argv=\"%s\"\n",argv[0]);
  }

  char* substring;
  int counter = 0;

  substring = strtok(argv[0]," ");
  while (substring!= NULL) {
    switch (counter) {
    case 0:
      altitude = atof(substring);
      altitude_valid = 0;
      break;
    }
    counter++;
    substring = strtok(NULL," ");
  }

  //clip altitude
  if(altitude < -999)
    altitude = -999;
  if(altitude > 9999)
    altitude = 9999;
  
  if(ground_level < altitude) {
    ground_level = altitude;
  }
  altitude -= ground_level;
  
  //write out altitude voltage
  FDPRINTF(TTY,"\033[9;0HAlt:");
  FDPRINTF(TTY,"\033[10;0H%04.1fm",altitude);

}



//--------------------------------------------------
// ATTITUDE callback, gives pitch information
#define PITCH_BARS_HEIGHT 40

#define PITCH_BARS_WIDTH 30
#define PITCH_BARS_CAL_WIDTH 16
#define PITCH_BARS_GAP 4

#define CENTER_X 240
#define CENTER_Y 144

void AttitudeStatusCallback (IvyClientPtr app, void *data, int argc, char **argv)
{

  if (verbose>0) {
    printf("ATTITUDE Received, argc=%u\n",argc);
    printf(" argv=\"%s\"\n",argv[0]);
  }

  char* substring;
  int counter = 0;
  int pitch_pixels=0;
  static int old_pitch_pixels=0;

  substring = strtok(argv[0]," ");
  while (substring!= NULL) {
    switch (counter) {
    case 2:
      pitch_angle = atof(substring);
      pitch_angle_valid = 0;
      break;
    }
    counter++;
    substring = strtok(NULL," ");
  }
    
  //draw pitch angle bars
  // bars are drawn relative to BOB4 sector 12, center of screen
  // max bar rise or drop is PITCH_BARS_HEIGHT pixels
  // corresponding to up or down 90 degrees pitch angle

  if(pitch_angle>90)
    pitch_angle = 90;
  if(pitch_angle<-90)
    pitch_angle = -90;

  pitch_pixels = (int)((pitch_angle/90)*PITCH_BARS_HEIGHT);

  if (verbose>0)
    printf(" pitch: angle: %f  pixels:%d",pitch_angle,pitch_pixels);
  
  

  //draw calibration bars
  //middle bar
  FDPRINTF(TTY,"\033[%d;%d.r",(CENTER_X),(CENTER_Y) );
  FDPRINTF(TTY,"\033[%d;%d-r",(CENTER_X)+(PITCH_BARS_CAL_WIDTH),(CENTER_Y) );
  FDPRINTF(TTY,"\033[/r"); 

  //top bar
  FDPRINTF(TTY,"\033[%d;%d.r",(CENTER_X),(CENTER_Y)+PITCH_BARS_HEIGHT );
  FDPRINTF(TTY,"\033[%d;%d-r",(CENTER_X)+(PITCH_BARS_CAL_WIDTH),(CENTER_Y)+PITCH_BARS_HEIGHT  );
  FDPRINTF(TTY,"\033[/r");
  //bottom bar
  FDPRINTF(TTY,"\033[%d;%d.r",(CENTER_X),(CENTER_Y)-PITCH_BARS_HEIGHT );
  FDPRINTF(TTY,"\033[%d;%d-r",(CENTER_X)+(PITCH_BARS_CAL_WIDTH),(CENTER_Y)-PITCH_BARS_HEIGHT  );
  FDPRINTF(TTY,"\033[/r");



  //draw actual bars
  //erase old ones
  //left bar
  FDPRINTF(TTY,"\033[%d;%d.r",(CENTER_X),(CENTER_Y)+old_pitch_pixels );
  FDPRINTF(TTY,"\033[%d;%d-r",(CENTER_X)-(PITCH_BARS_WIDTH),(CENTER_Y)+old_pitch_pixels );
  FDPRINTF(TTY,"\033[0/r");  

  //draw actual bars
  //left bar
  FDPRINTF(TTY,"\033[%d;%d.r",(CENTER_X),(CENTER_Y)+pitch_pixels );
  FDPRINTF(TTY,"\033[%d;%d-r",(CENTER_X)-(PITCH_BARS_WIDTH),(CENTER_Y)+pitch_pixels );
  FDPRINTF(TTY,"\033[/r"); 


  old_pitch_pixels = pitch_pixels;
  

}


void drawArrow ( int center_x, int center_y, int angle, unsigned char color ) {

  float dx,dy,dx_arrow,dy_arrow;

  dx = sin(angle*(3.141/180))*20;
  dy = -cos(angle*(3.141/180))*20;
  //scale factor?
  dx *= ASPECT;
  
  dx_arrow = sin( (angle-135)*(3.141/180))*5;
  dy_arrow = -cos( (angle-135)*(3.141/180))*5;
  dx_arrow *= ASPECT;

  //draw line? 
  FDPRINTF(TTY,"\033[%u;%u;17.r",center_x,center_y);
  FDPRINTF(TTY,"\033[%u;%u;17-r",(int)dx+center_x,(int)dy+center_y);
  FDPRINTF(TTY,"\033[%u;%u;17-r",(int)dx+(int)dx_arrow+center_x,(int)dy+(int)dy_arrow+center_y);
  FDPRINTF(TTY,"\033[%c/r",color);  

}




void GPSCallback (IvyClientPtr app, void *data, int argc, char **argv)
{
  float dy,dx,heading_to_home,delta_heading;
  float dy_arrow,dx_arrow;

  static float old_dx,old_dy,old_dx_arrow,old_dy_arrow;

  if (verbose>0) {
    printf("GPS Received, argc=%u\n",argc);
    printf(" argv=\"%s\"\n",argv[0]);
  }

  char* substring;
  int counter = 0;

  substring = strtok(argv[0]," ");
  while (substring!= NULL) {
    switch (counter) {
    case 5:
      groundspeed = atof(substring)/100;
      groundspeed_valid = 0;
      break;
    case 1:
      utm_east = atof(substring)/100;
      pos_valid = 0;
      break;
    case 2:
      utm_north = atof(substring)/100;
      pos_valid = 0;
      break;
    case 3:
      heading_angle_deg = atof(substring)/10;
      heading_valid = 0;
      break;
    }
    counter++;
    substring = strtok(NULL," ");
  }
    
  //write out groundspeed
  // clip ground speed
  if (groundspeed>999)
    groundspeed = 999;
  FDPRINTF(TTY,"\033[9;30HGS:");
  FDPRINTF(TTY,"\033[10;30H%03.1f",groundspeed);
  FDPRINTF(TTY,"\033[11;30Hm/s");

  //draw compass pointer
  compass_heading = 360 - heading_angle_deg;
  
  //erase old compass pointer
    dx = -sin(old_compass_heading*(3.141/180))*20;
    dy = -cos(old_compass_heading*(3.141/180))*20;
    //scale factor
    dx *= ASPECT;

    dx_arrow = -sin( (old_compass_heading-135)*(3.141/180))*5;
    dy_arrow = -cos( (old_compass_heading-135)*(3.141/180))*5;
    dx_arrow *= ASPECT;

    //erase old line
    FDPRINTF(TTY,"\033[40;40;8.r");
    FDPRINTF(TTY,"\033[%u;%u;8-r",(int)dx+40,(int)dy+40);
    FDPRINTF(TTY,"\033[%u;%u;8-r",(int)dx+(int)dx_arrow+40,(int)dy+(int)dy_arrow+40);
    FDPRINTF(TTY,"\033[0/r");  

    dx = -sin(compass_heading*(3.141/180))*20;
    dy = -cos(compass_heading*(3.141/180))*20;
    //scale factor
    dx *= ASPECT;

    dx_arrow = -sin( (compass_heading-135)*(3.141/180))*5;
    dy_arrow = -cos( (compass_heading-135)*(3.141/180))*5;
    dx_arrow *= ASPECT;

    //draw new line
    FDPRINTF(TTY,"\033[40;40;8.r");
    FDPRINTF(TTY,"\033[%u;%u;8-r",(int)dx+40,(int)dy+40);
    FDPRINTF(TTY,"\033[%u;%u;8-r",(int)dx+(int)dx_arrow+40,(int)dy+(int)dy_arrow+40);
    FDPRINTF(TTY,"\033[/r");  
  

  old_compass_heading = compass_heading;


  //draw pointer to home
  if (home_valid == 0) {
    //calculate angle to home
    dy = home_utm_north - utm_north;
    dx = home_utm_east - utm_east;
    if(dy<0)
      heading_to_home = atan(dx/dy)*(180.0/3.141)+180;
    else
      heading_to_home = -atan(dx/-dy)*(180.0/3.141);

    if (heading_to_home<0)
      heading_to_home+=360;
    if (heading_to_home>360)
      heading_to_home-=360;
    delta_heading = heading_to_home-heading_angle_deg;
    if (delta_heading<0)
      delta_heading+=360;
      
    if(verbose>0) {
      printf(" dy: %f  dx: %f\n",dy,dx);
      printf(" utm: %f %f   home: %f %f\n",utm_north,utm_east,home_utm_north,home_utm_east);
      printf(" heading to home: %f  heading: %f\n",heading_to_home, heading_angle_deg);
      printf(" delta_heading: %f\n",delta_heading);
    }


    dx = sin(delta_heading*(3.141/180))*20;
    dy = -cos(delta_heading*(3.141/180))*20;
    //scale factor
    dx *= ASPECT;

    dx_arrow = sin( (delta_heading-135)*(3.141/180))*5;
    dy_arrow = -cos( (delta_heading-135)*(3.141/180))*5;
    dx_arrow *= ASPECT;

    //erase old line
    FDPRINTF(TTY,"\033[0;40;17.r");
    FDPRINTF(TTY,"\033[%u;%u;17-r",(int)old_dx,(int)old_dy+40);
    FDPRINTF(TTY,"\033[%u;%u;17-r",(int)old_dx+(int)old_dx_arrow,(int)old_dy+(int)old_dy_arrow+40);
    FDPRINTF(TTY,"\033[0/r");  

    //draw new line
    FDPRINTF(TTY,"\033[0;40;17.r");
    FDPRINTF(TTY,"\033[%u;%u;17-r",(int)dx,(int)dy+40);
    FDPRINTF(TTY,"\033[%u;%u;17-r",(int)dx+(int)dx_arrow,(int)dy+(int)dy_arrow+40);
    FDPRINTF(TTY,"\033[/r");

    old_dx = dx;
    old_dy = dy;
    old_dx_arrow = dx_arrow;
    old_dy_arrow = dy_arrow;

  }      
}

void WpMovedCallback (IvyClientPtr app, void *data, int argc, char **argv)
{
  int wp_num;

 if (verbose>0) {
    printf("WP_MOVED Received, argc=%u\n",argc);
    printf(" argv=\"%s\"\n",argv[0]);
  }

  char* substring;
  int counter = 0;

  substring = strtok(argv[0]," ");
  while (substring!= NULL) {
    switch (counter) {
    case 0:
      wp_num = atoi(substring);
      break;
    case 1:
      if (wp_num == 1) {
	home_utm_east = atof(substring);
	home_valid = 0;
      }
      break;
    case 2:
      if (wp_num == 1) {
	home_utm_north = atof(substring);
	home_valid = 0;
      }
      break;
    }
    counter++;
    substring = strtok(NULL," ");
  }


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

void au_osd_print_help( void ) {

  printf("au_osd help, calling options:\n");
  printf(" -b <ivy_bus>    Use specified Ivy bus\n");
  printf(" -p <device>     Use <device> com port, default is /dev/ttyUSB1\n");
  printf(" -v              Enable verbose mode\n");
  printf(" -o              Turn on proprietary mode (for shows)\n");
  printf(" -t              Use MY_CMDS for velocity/altitude\n");
  printf(" -P              Set PAL explicitely on startup\n");
  printf(" -g              Set genlock mode\n");
}


main (int argc, char**argv)
{
	/* handling of -b option */
	const char* bus = 0;
	const char* com_port = 0;

	int genlockmode = 0;
	int palmode = 0;

	char c;
	while ( (c = getopt (argc, argv, "b:p:voPtg")) != EOF) {
		switch (c) {
		case 'g':
		  printf("Setting Genlock Off\n");
		  genlockmode = 1;
		  break;
		case 'b':
		  bus = optarg;
		  break;
		case 'p':
		  com_port = optarg;
		  break;
		case 'v':
		  verbose = 1;
		  printf("Verbose debug on\n");
		  break;
		case 'P':
		  palmode = 1;
		  printf("Setting PAL mode\n");
		  break;
		case 'o':
		  proprietary = 1;
		  printf("Using OSD in proprietary mode\n");
		  break;
		default:
		  au_osd_print_help();
		  abort();
		}
	}

	/* handling of environment variable */
	if (!bus)
		bus = getenv ("IVYBUS");

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

	/* initializations */
	IvyInit ("IvyOSD", "OSD Startup", 0, 0, 0, 0);
	IvyStart (bus);

	/* Battery message, has voltage, flight time */
	IvyBindMsg (BatteryCallback, 0, "[0-9] BAT (.*)");

	/* Estimator message, has altitude */
	IvyBindMsg (EstimatorCallback, 0, "[0-9] ESTIMATOR (.*)");

	/* VNAV message, has attitude */
	IvyBindMsg (AttitudeStatusCallback, 0, "[0-9] ATTITUDE (.*)");       

	/* GPS message, has position */
	IvyBindMsg (GPSCallback, 0, "[0-9] GPS (.*)");   

	/* WP_MOVED message, has HOME waypoint position */
	IvyBindMsg (WpMovedCallback, 0, "[0-9] WP_MOVED (.*)"); 

	/* Bind Datalink Heartbeat */
	IvyBindMsg (Datalink_Heartbeat,  0, "[0-9] DOWNLINK_STATUS (.*)");

	/* OSD Heartbeat */
	TimerRepeatAfter ( TIMER_LOOP, 1000, OSD_Heartbeat, 0 );

	ClearOSD();


	if (palmode == 1) {
       	  SetPal();
	}
	if (genlockmode == 1) {
	  SetGenlockOff();
	}

	/* main loop */
	IvyMainLoop();
}


