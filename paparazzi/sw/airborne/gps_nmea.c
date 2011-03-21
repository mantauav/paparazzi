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
 * TODO: THIS NMEA-PARSER IS PARTIALLY TESTED AND INCOMPLETE!!!
 * Status: 
 * 
 */


#ifdef DEBUG_NMEA
// do debug-output if run on the DEBUG_NMEA-target
#endif

#include <stdlib.h>
#include <stdio.h>
#include "string.h"

#include "paparazzi.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "mcu_periph/uart.h"
#include "gps.h"
#include "gps_nmea.h"
#include "subsystems/nav.h"
#include "latlong.h"

#include "ap_downlink.h"
#include "messages.h"

int32_t  gps_lat;  // latitude in degrees * 1e-7
int32_t  gps_lon;  // longitude in degrees * 1e-7
uint16_t gps_PDOP; // precision
bool_t   gps_pos_available = FALSE;

uint32_t gps_itow;
uint16_t gps_week;
int32_t  gps_alt;
uint16_t gps_gspeed; // in cm/s
int16_t  gps_climb;
int16_t  gps_course;
int32_t  gps_utm_east, gps_utm_north;
uint8_t  gps_utm_zone;
uint8_t  gps_mode;

uint8_t debug_msg[80];

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

//void ubxsend_cfg_rst(uint16_t bbr , uint8_t reset_mode) {
//}


#ifdef GPS_CONFIGURE
/* GPS dynamic configuration */

//#include "uart.h"

void gps_configure_uart ( void ) {
  //UbxSend_CFG_PRT(0x01, 0x0, 0x0, 0x000008D0, GPS_BAUD, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);  
  //while (GpsUartRunning) ; /* FIXME */
//  GpsUartInitParam( UART_BAUD(GPS_BAUD),  UART_8N1, UART_FIFO_8);
}

void gpsSend(uint8_t msg[])
{
  int i=0;
  while (msg[i]!=0)
  {
    GpsUartSend1(msg[i]);
    i++;
  }
}
//  uint8_t fixRateCmd[]   = "$PMTK300,250,0,0,0,0*2A\r\n";
  uint8_t fixRateCmd[]   = "$PMTK300,200,0,0,0,0*2F\r\n";
  /* const unsigned char* fixRateQuery = "$PMTK400*36\r\n"; */
  uint8_t nmeaQuetCmd[]     = "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
  uint8_t nmeaOutputCmd[]   = "$PMTK314,0,1,0,1,1,4,0,0,0,0,0,0,0,0,0,0,0,0,0*2D\r\n";

void gps_configure ( void ) {
  /* Initialize GPS receiver into desired operational mode, e.g. set FIX rate and update rates 
     for desired NMEA sentences */
  /* const unsigned char* nmeaOutputQuery = "$PMTK,414*33\r\n"; */

  // turn off all nmea messages
//  WriteGpsBuffer(nmeaQuetCmd); 
  // set gps fix rate to 4Hz
//  WriteGpsBuffer(fixRateCmd);
  gpsSend(fixRateCmd);
  gpsSend(fixRateCmd);
  gpsSend(fixRateCmd);
  // output RMC, GGA, GSA at 4Hz and GSV at 1Hz
//  WriteGpsBuffer(nmeaOutputCmd);
  gps_nb_ovrn=0;
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
  return 0;
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
  char* endptr;  // end of parsed substrings
  uint8_t i = 7; // current position in the message
  uint8_t activesv;
  float pdop_float;

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

  // get active satellite info
  //ignored
  for (activesv = 3;activesv <=14;activesv++){
    if (!nextField(&i)) return;
  }
  
  //get PDOP
  pdop_float = (float)strtod(&nmea_msg_buf[i], &endptr) + 1.0;
  gps_PDOP = (uint16_t)(pdop_float*100);

  if (!nextField(&i)) return;
  //get HDOP
  //ignored

  if (!nextField(&i)) return;
  //get VDOP
  //ignored
  return;
  
}

/**
 * parse GPRMC-nmea-messages stored in
 * nmea_msg_buf.
 */
void parse_nmea_GPRMC() {
  uint8_t i = 7; // current position in the message
  char* endptr;  // end of parsed substrings
  
  //USB_DEBUG_OUT("isGPRMC: %s\n\r", nmea_msg_buf);

  // attempt to detect and discard empty sentences
  if (isEmptySentence(i)) return;

  // get UTC time [hhmmss.sss] and convert to ms
  double dtime  = strtod(&nmea_msg_buf[i],&endptr)*1000;
  uint32_t time = (uint32_t)(dtime);
  uint32_t hh   = time/1e7;
  uint32_t mm   = (time - (hh*1e7))/1e5;
  uint32_t ms   = time - hh*1e7 - mm*1e5;
  // Note: these values are used below to calculate gps_itow
  
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
    gps_gspeed = (uint16_t)(speed * 51.4444444); //convert knots to cm/s 
  }
    
  if (!nextField(&i)) return;

  // get course
  if ((gps_mode ==2) || (gps_mode==3)){      
    double course = strtod(&nmea_msg_buf[i], &endptr);
    gps_course = (int16_t)(course * 10);
  }

  if (!nextField(&i)) return;

  // get date
  uint32_t ddate  = atoi(&nmea_msg_buf[i]);
  uint32_t DD = ddate/1e4;
  uint32_t MM = (ddate - DD*1e4)/1e2;
  uint32_t YY = ddate - DD*1e4 - MM*1e2;
  
  // set gps_itow
  gps_itow = (hh*60 + mm)*60*1e3 + ms +
    get_weekday(DD, MM, YY)*24*60*60*1000;
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

  // get time
  // ignored - done in RMC message parser
  
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
  gps_nb_channels = atoi(&nmea_msg_buf[i]);  //Get number of satellites in view
  gps_nb_channels = Min(gps_nb_channels, GPS_NB_CHANNELS);

  for (ch = 0; ch < channels_per_msg; ch++){ 
    
    // if valid satellite data
    if (gps_nb_channel < gps_nb_channels){
      
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
  return (nmea_msg_len > 5 && !strncmp(nmea_msg_buf, "$GPRMC", 6));
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



/**
 * Converts two ascii characters that represent a hex byte to decimal value.
 * Note does not do any error checking on whether characters are in range 0..9 or A..F
 * Used in checksum calculation
 */
inline uint8_t atoh(uint8_t a,uint8_t b)
{
  uint8_t val;
  if (a<58)
    val=(a-48)*16;
  else
    val=(a-55)*16;
  if (b<58)
    val+=b-48;
  else
    val+=b-55;
  return val;  
}

/**
 * Verify an NMEA checksum
 * checksum consists of XORing each byte between $ and *
 */
uint8_t nmea_checksum_valid(uint8_t buf[],uint8_t buf_len)
{
  uint8_t i=0;
  uint8_t cksum=0;
  uint8_t msg_cksum;
  if (buf[0] != '$')
    return 0;
  if (buf[buf_len-3]!='*')
    return 0;
  for (i=1;i<buf_len-3;i++)
    cksum ^= buf[i];
  msg_cksum = atoh(nmea_msg_buf[buf_len-2],nmea_msg_buf[buf_len-1]);
  if (cksum == msg_cksum)
    return 1;
  else
    return 0;
}

/**
 * This is the actual parser.
 * It reads one character at a time
 * setting gps_msg_received to TRUE
 * after a full line.
 */
void parse_gps_msg( void ) {
  
  nmea_msg_buf[nmea_msg_len] = 0;
  if (nmea_checksum_valid((uint8_t *)nmea_msg_buf,nmea_msg_len))
  {
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
  }
  else {
    //reported as gps_nb_err in gps message window.
    gps_nb_ovrn++;
  }
    
  nmea_msg_buf[nmea_msg_len]=0;
  // reset message-buffer
  nmea_msg_len = 0;
}

/**
 * This is the actual parser.
 * It reads one character at a time
 * setting gps_msg_received to TRUE
 * after a full line.
 */

void parse_nmea_char( uint8_t c ) {
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

/**
 * Converts date to index of the corresponding weekday (0-6). 
 * Sunday is 0, Monday is 1, ..., Saturday is 6.
 */

uint8_t get_weekday(uint8_t DD, uint8_t MM, uint8_t YY){
  uint8_t CC = 20; // assume 21'st century (20xx) - GPS does not provide this info
                   // CC will need to be updated at the boundary of the 22nd century

  uint8_t months[12] = {0,3,3,6,1,4,6,2,5,0,3,5}; // month lookup table, see 
  // http://en.wikipedia.org/wiki/Calculating_the_day_of_the_week

  uint8_t c = 2 * (3 - (CC % 4));
  uint8_t y = YY + (YY/4);
  uint8_t m = months[MM-1];
  
  // correct for leap year when MM is Jan or Feb
  if (((YY % 4) == 0) && ((MM == 1 || MM == 2))) m -= 1;

  return (DD + c + y + m) % 7;
}
