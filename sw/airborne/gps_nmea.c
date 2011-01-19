/*
 *  
 * Copyright (C) 2008 Marcus Wolschon
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

/** 
 * file gps_nmea.c
 * brief Parser for the NMEA protocol
 *
 * This file is a drop-in replacement for gps_ubx.c
 *
 * TODO: THIS NMEA-PARSER IS NOT WELL TESTED AND INCOMPLETE!!!
 * Status:
 *  Parsing GGA and RMC is complete, GSA and other records are
 *  incomplete.
 */

#include <inttypes.h>
#include <string.h> 
#include <math.h>

#ifdef DEBUG_NMEA
// do debug-output if run on the DEBUG_NMEA-target
#include <stdlib.h>
#endif

#include "flight_plan.h"
#include "uart.h"
#include "gps.h"
#include "gps_nmea.h"
#include "nav.h"
#include "latlong.h"

//AD
#include "usb_debug.h"


int32_t  gps_lat;  // latitude in degrees * 1e-7
int32_t  gps_lon;  // longitude in degrees * 1e-7
uint16_t gps_PDOP; // precision
bool_t   gps_pos_available = FALSE;

uint32_t gps_itow;
int32_t  gps_alt;
uint16_t gps_gspeed; // in cm/s
int16_t  gps_climb;
int16_t  gps_course;
int32_t  gps_utm_east, gps_utm_north;
uint8_t  gps_utm_zone;
uint8_t  gps_mode;

char nmea_msg_buf[NMEA_MAXLEN];
int  nmea_msg_len = 0;

#ifdef GPS_CONFIGURE
static uint8_t gps_status_config;
#endif

//AD added gps_configuring variable
//   in gps.h if gps_configuring is true GpsParseOrConfigure() checks value of variable 
//   and calls gps_configure() or parse_gps_msg() depending on its value

bool_t gps_configuring;

// true if parse_ubx() has a complete message and parse_gps_msg() shall parse it
volatile bool_t gps_msg_received = FALSE;

uint8_t  ubx_id, ubx_class; // unused
uint16_t gps_reset;         // unused

uint32_t gps_Pacc, gps_Sacc;
uint8_t gps_numSV;         // number of satelites in view

struct svinfo gps_svinfos[GPS_NB_CHANNELS];
uint8_t gps_nb_channels;
uint8_t gps_nb_ovrn;        // number if incomplete nmea-messages



////////////////////////////////////////////////////////
//       uart-configuration

//AD
// added GPS_CONFIG_INIT define equal to 0
#define GPS_CONFIG_INIT 0
void gps_init( void ) {
#ifdef GPS_CONFIGURE
  gps_status_config = GPS_CONFIG_INIT;
  gps_configuring = TRUE;
#endif
}

void ubxsend_cfg_rst(uint16_t bbr , uint8_t reset_mode) {
}


#ifdef GPS_CONFIGURE
/* GPS dynamic configuration */

#include "uart.h"

void gps_configure_uart ( void ) {
  //UbxSend_CFG_PRT(0x01, 0x0, 0x0, 0x000008D0, GPS_BAUD, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);  
  //while (GpsUartRunning) ; /* FIXME */
  GpsUartInitParam( UART_BAUD(GPS_BAUD),  UART_8N1, UART_FIFO_8);
}

void gps_configure ( void ) {
  /* Initialize GPS receiver into desired operational mode, e.g. set FIX rate and update rates 
     for desired NMEA sentences */
  const unsigned char* fixRateCmd   = "$PMTK300,250,0,0,0,0*2A\r\n";
  /* const unsigned char* fixRateQuery = "$PMTK400*36\r\n"; */
  const unsigned char* nmeaQuetCmd     = "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
  const unsigned char* nmeaOutputCmd   = "$PMTK314,0,1,0,1,1,4,0,0,0,0,0,0,0,0,0,0,0,0,0*2D\r\n";
  /* const unsigned char* nmeaOutputQuery = "$PMTK,414*33\r\n"; */

  // turn off all nmea messages
  WriteGpsBuffer(nmeaQuetCmd); 
  // set gps fix rate to 4Hz
  WriteGpsBuffer(fixRateCmd);
  // output RMC, GGA, GSA at 4Hz and GSV at 1Hz
  WriteGpsBuffer(nmeaOutputCmd);

  gps_configuring=FALSE;	
}
#endif /* GPS_CONFIGURE */


/*****************************************************************************************/

/**
 * Fast forward to the next field.
 * Return 0 if no next field, otherwise return 1 and advance current_idx to the next field in nmea_msg_buf[].
 * Input argument is the current position (index) in the nmea_msg_buf[].
 */
uint8_t nextField(uint8_t* current_idx){
	// if we are currently paused on a ',' simply step off of it
	if(nmea_msg_buf[*current_idx] == ','){
		(*current_idx)++;
		if(*current_idx >= nmea_msg_len) {
			return 0;
		}
		else {
			return 1;
		}
	}

	// otherwise, march to the next ',' and step off of it
	(*current_idx)++;
	while(nmea_msg_buf[*current_idx] != ','){
		if(*current_idx >= nmea_msg_len) return 0;

		(*current_idx)++;
	}

	(*current_idx)++;
	return 1;
}

/**
 * Check if the sentense is error-free
 * Return 1 if sentence is valid, 0 otherwise.
 */
uint8_t isValidSentence(uint8_t current_idx){
  
}

/**
 * Attempt to reject empty sentences.
 * Return 1 if sentence contains no data, 0 otherwise.
 */
uint8_t isEmptySentence(uint8_t current_idx){
  uint8_t ret = 0;
  if(nmea_msg_buf[current_idx]==',' && nmea_msg_buf[current_idx+1]==',') {
    ret = 1;
  }
  return ret;
}


/**
 * parse GPGSA-nmea-messages stored in
 * nmea_msg_buf.
 */
void parse_nmea_GPGSA() {
  uint8_t i = 7; // current position in the message

  //USB_DEBUG_OUT("isGPGSA: %s\n\r", nmea_msg_buf);

  // attempt to detect and discard empty sentences
  if (isEmptySentence(i)) return;
  
  // get auto2D/3D
  // ignored
  if (!nextField(&i)) return;
  
  // get 2D/3D-fix
  // set gps_mode=3=3d, 2=2d, 1=no fix or 0
  gps_mode = atoi(&nmea_msg_buf[i]);
  if (gps_mode == 1){
    gps_mode = 0;
  }
}

/**
 * parse GPRMC-nmea-messages stored in
 * nmea_msg_buf.
 */
void parse_nmea_GPRMC() {
  uint8_t i = 7;     // current position in the message
  char* endptr;  // end of parsed substrings
  
  //USB_DEBUG_OUT("isGPRMC: %s\n\r", nmea_msg_buf);

  // attempt to detect and discard empty sentences
  if (isEmptySentence(i)) return;

  // get time
  // ignored
  if (!nextField(&i)) return;

  // get warning
  // ignored
  if (!nextField(&i)) return;
  
  // get lat
  // ignored
  if (!nextField(&i)) return;
  
  // get North/South
  // ignored
  if (!nextField(&i)) return;
  
  // get lon
  // ignored
  if (!nextField(&i)) return;
  
  // get eath/west
  // ignored
  if (!nextField(&i)) return;
  
  // get ground speed (in m/s)
  if ((gps_mode ==2) || (gps_mode==3)){      
    double speed = strtod(&nmea_msg_buf[i], &endptr);
    gps_gspeed = (uint16_t)(speed * 0.514444444); //convert knots to m/s 
  }
    
  if (!nextField(&i)) return;

  // get course
  if ((gps_mode ==2) || (gps_mode==3)){      
    double course = strtod(&nmea_msg_buf[i], &endptr);
    gps_course = (int16_t)(course * 10);
  }
}


/**
 * parse GPGGA-nmea-messages stored in
 * nmea_msg_buf.
 */
void parse_nmea_GPGGA() {
  uint8_t i = 7;     // current position in the message
  char* endptr;  // end of parsed substrings
  double degrees, minutesfrac;

  //USB_DEBUG_OUT("isGPGGA: %s\n\r", nmea_msg_buf);

  // attempt to detect and discard empty sentences
  if (isEmptySentence(i)) return;
  
  // get UTC time [hhmmss.sss] and convert to ms
  double dtime  = strtod(&nmea_msg_buf[i],&endptr)*1000;
  uint32_t time = (uint32_t)(dtime);
  uint32_t hh   = time/1e7;
  uint32_t mm   = (time - (hh*1e7))/1e5;
  uint32_t ms   = time - hh*1e7 - mm*1e5;
  gps_itow = (hh*60 + mm)*60*1e3 + ms;
  
  if (!nextField(&i)) return;

  // get latitude [ddmm.mmmmm]
  double lat = strtod(&nmea_msg_buf[i], &endptr);
  // convert to pure degrees [dd.dddd] format
  minutesfrac = modf(lat/100, &degrees);
  lat = degrees + (minutesfrac*100)/60;

  if (!nextField(&i)) return;
  
  // correct latitute for N/S
  if(nmea_msg_buf[i] == 'S'){
    lat = -lat;
  }

  gps_lat = (int32_t)(lat * 1e7); // convert to fixed-point

  if (!nextField(&i)) return;
  
  // get longitude [ddmm.mmmmm]
  double lon = strtod(&nmea_msg_buf[i], &endptr);
  // convert to pure degrees [dd.dddd] format
  minutesfrac = modf(lon/100, &degrees);
  lon = degrees + (minutesfrac*100)/60;

  if (!nextField(&i)) return;

  // correct latitute for E/W
  if(nmea_msg_buf[i] == 'W'){
    lon = -lon;
  }
  
  gps_lon = (int32_t)(lon * 1e7); // convert to fixed-point

  if (!nextField(&i)) return;  

  latlong_utm_of(RadOfDeg(lat), RadOfDeg(lon), nav_utm_zone0);
  
  gps_utm_east  = latlong_utm_x * 100;
  gps_utm_north = latlong_utm_y * 100;
  gps_utm_zone  = nav_utm_zone0;
  
  
  // position fix status
  // 0 = Invalid, 1 = Valid SPS, 2 = Valid DGPS, 3 = Valid PPS
  // check for good position fix
  if( (nmea_msg_buf[i] != '0') && (nmea_msg_buf[i] != ',') )  {
    gps_pos_available = TRUE;
  } 
  else {
    gps_pos_available = FALSE;
  }

  if (!nextField(&i)) return;

  // get number of satellites used in GPS solution
  gps_numSV = atoi(&nmea_msg_buf[i]);
  
  if (!nextField(&i)) return;

  // get horizontal dilution of position
  // ignored
  if (!nextField(&i)) return;
  
  // get altitude (in cm)
  double alt = strtod(&nmea_msg_buf[i], &endptr);
  gps_alt = (int32_t)(alt * 100);

  // next field: altitude units, always 'M'
  // ignored
  if (!nextField(&i)) return;
  
  // next field: geoid seperation
  // ignored
  if (!nextField(&i)) return;

  // next field: seperation units
  // ignored
  if (!nextField(&i)) return;

  // next field: DGPS age
  // ignored
  if (!nextField(&i)) return;

  // next field: DGPS station ID
  // ignored
  if (!nextField(&i)) return;
}

/**
 * parse GPGSinfo.gps_numSVV-nmea-messages stored in
 * nmea_msg_buf.
 */
void parse_nmea_GPGSV() {
  uint8_t i = 7;    // current position in the message
  char* endptr;     // end of parsed substrings
  int ch = 0;       // gps satellite channel number
  
  uint8_t         msgnum = 0;
  uint8_t     totnummsgs = 0;
  uint8_t gps_nb_channel = 0;

  const uint8_t channels_per_msg = 4; //up to 4 satellites per GPSVN message

  //USB_DEBUG_OUT("isGPGSV: %s\n\r", nmea_msg_buf);  

  // attempt to detect and discard empty sentences
  if (isEmptySentence(i)) return;
  
  // get total number of messages
  totnummsgs = (uint8_t) strtod(&nmea_msg_buf[i], &endptr);

  // next field: message number
  if (!nextField(&i)) return;

  // get message number and set gps_nb_channel number accordingly
  msgnum = (uint8_t) strtod(&nmea_msg_buf[i], &endptr);
  gps_nb_channel = (msgnum - 1) * channels_per_msg;

  // next field: satellites in view
  if (!nextField(&i)) return;

  // get num satellites in view
  gps_numSV = atoi(&nmea_msg_buf[i]);

  for (ch = 0; ch < channels_per_msg; ch++){ 
    
    // if valid satellite data
    if (gps_nb_channel < gps_numSV){
      
      // next field: satellite number
      if (!nextField(&i)) break;

      // get satellite number
      gps_svinfos[gps_nb_channel].svid= atoi(&nmea_msg_buf[i]);
      
      // next field: elevation in degrees
      if (!nextField(&i)) break;

      // get elevation in degrees
      gps_svinfos[gps_nb_channel].elev = atoi(&nmea_msg_buf[i]);

      // next field: azimuth in degrees
      if (!nextField(&i)) break;

      // get azimuth in degrees to true
      gps_svinfos[gps_nb_channel].azim = atoi(&nmea_msg_buf[i]);
      
      
      // next field: signal to noise ratio (SNR)
      if (!nextField(&i)) break;

      // get SNR in dB
      gps_svinfos[gps_nb_channel].cno = atoi(&nmea_msg_buf[i]);

    }

    // zero out if passed valid sat data
    else {
    	gps_svinfos[gps_nb_channel].svid = 0;
    	gps_svinfos[gps_nb_channel].elev = 0;
    	gps_svinfos[gps_nb_channel].azim = 0;
    	gps_svinfos[gps_nb_channel].cno  = 0;
    }
    
    // get ready for the next channel's data
    gps_nb_channel++;
  } // up to "channels_per_msg" satellites per GPSVN message

  //zero out the rest of gps_svinfos for channels with no fresh sat data
  if (msgnum == totnummsgs) {
    for(ch = msgnum * channels_per_msg; ch < GPS_NB_CHANNELS; ch++){
      gps_svinfos[ch].svid = 0;
      gps_svinfos[ch].elev = 0;
      gps_svinfos[ch].azim = 0;
      gps_svinfos[ch].cno  = 0;
    }
  }
}

/**
 * set of helper query functions                                                
 */
bool_t isGPRMC(){
  return (nmea_msg_len > 5  && !strncmp(nmea_msg_buf, "$GPRMC", 6));
}
bool_t isGPGGA(){
  return (nmea_msg_len > 5 && !strncmp(nmea_msg_buf, "$GPGGA", 6));
}
bool_t isGPGSA(){
  return (nmea_msg_len > 5 && !strncmp(nmea_msg_buf, "$GPGSA", 6));
}
bool_t isGPGSV(){
  return (nmea_msg_len > 5 && !strncmp(nmea_msg_buf, "$GPGSV", 6));
}

void parse_gps_msg( void ) {
  
  nmea_msg_buf[nmea_msg_len] = 0;
  
  if(isGPRMC()){
    parse_nmea_GPRMC();
  } else if (isGPGGA()){
    parse_nmea_GPGGA();
  } else if (isGPGSA()){
    parse_nmea_GPGSA();
  } else if (isGPGSV()){
    parse_nmea_GPGSV();
  } else {
    // nothing to do, ignore unsupported message
  }
  
  // reset message-buffer
  nmea_msg_len = 0;
}


/**
 * This is the actual parser.
 * It reads one character at a time
 * setting gps_msg_received to TRUE
 * after a full line.
 */
void parse_ubx( uint8_t c ) {

  //reject empty lines
  if (nmea_msg_len == 0) {
    if (c == '\r' || c == '\n'){
      return;
    }
  }

  // fill the buffer, unless it's full
  if (nmea_msg_len < NMEA_MAXLEN - 1) {
    // messages end with a linefeed
    if (c == '\r' || c == '\n') {
      gps_msg_received = TRUE;
    } 
    else {
      nmea_msg_buf[nmea_msg_len] = c;
      nmea_msg_len ++;
    }
  }
  
  if (nmea_msg_len >= NMEA_MAXLEN - 1){
    gps_msg_received = TRUE;
  }
  
}
