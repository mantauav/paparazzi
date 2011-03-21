/** \file
    Test the crc module
    $Id$
    ***Warning*** Aurora Flight Sciences, Inc. Proprietary Material
    Author: Max Chtangeev
    Copyright (C) 2010 by Aurora Flight Sciences, Inc. All Rights Reserved.
*/
	
#include <stdio.h>
#include <string.h>
#include <std.h>

#include "tgps_nmea.h"
#include "unittest.h"
#include "stubs.h"
#include "../gps.h"
#include "../gps_nmea.h"

/*
  examples:

  ASSERT_TRUE(1 == 1);
  ASSERT_FALSE(1 == 0);
  ASSERT_EQUAL(1.0, 1.0);
  PASS;
*/

int tgps_nmea_nextField(void){
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t ret;
  char expected[] = {
    '2', //04218.500,
    '4', //221.6043,
    'N',
    '0', //7104.9980,
    'W',
    '2',
    '7',
    '1', //.28,
    '9', //.2,
    'M',
    '-', //33.7,
    'M',
    '0', //000,
    '0', //000*
    '6', //6");
  };

  sprintf(nmea_msg_buf, "$GPGGA,204218.500,4221.6043,N,07104.9980,W,2,7,1.28,9.2,M,-33.7,M,0000,0000*66");
  //sprintf(nmea_msg_buf, "$GPGGA,204218.500,4221.6043,,07104.9980,,,,1.28,9.2,M,-33.7,M,0000,0000*66");
  nmea_msg_len = strlen(nmea_msg_buf);

  // call function under test
  ret = nextField(&i);

  while(ret == 1){ 
    ASSERT_EQUAL(ret, 1);
    ASSERT_EQUAL(nmea_msg_buf[i], expected[j++]);

    // call function under test
    ret = nextField(&i);
  }

  PASS;
}

int tgps_nmea_isEmptySentence(void){
  FAIL("Test not implemented");
}

int tgps_nmea_GPGSA(void){
  sprintf(nmea_msg_buf, "$GPGSA,A,3,29,02,05,10,26,15,08,04,,,,,1.46,1.17,0.87*08");
  nmea_msg_len = strlen(nmea_msg_buf);

  gps_mode = 0;

  // call function under test
  parse_nmea_GPGSA();

#ifdef TESTDEBUGPRINT
  printf("parse_nmea_GPGSA: gps_mode = %d\n", gps_mode);
#endif

  ASSERT_EQUAL(gps_mode, 3);
  PASS;
}

int tgps_nmea_checksum(void){
	  sprintf(nmea_msg_buf, "$GPRMC,203924.750,A,4221.6043,N,07104.9980,W,74.00,16.78,020311,,,A*74");
	  nmea_msg_len = strlen(nmea_msg_buf);
	  uint8_t valid_chksum;

	  // call function under test
	  valid_chksum = nmea_checksum_valid((uint8_t*)nmea_msg_buf, nmea_msg_len);

	  ASSERT_EQUAL(valid_chksum, 1);
	  PASS;
}

int tgps_nmea_GPRMC(void){
  sprintf(nmea_msg_buf, "$GPRMC,203924.750,A,4221.6043,N,07104.9980,W,74.00,16.78,020311,,,A*74");
  nmea_msg_len = strlen(nmea_msg_buf);
  // expected
  uint16_t expected_gspeed = 3806;      //74 knots in m/s
  int16_t  expected_course = 167;
  uint32_t expected_itow   = 333564750; //ms from the beginning of the week

  // initialize to zeros
  gps_gspeed = 0;
  gps_course = 0;
  
  // call function under test
  parse_nmea_GPRMC();

#ifdef TESTDEBUGPRINT
  printf("parse_nmea_GPRMC: gps_gspeed = %d\tgps_course = %d\n", gps_gspeed, gps_course);
#endif 

  ASSERT_EQUAL(gps_gspeed, expected_gspeed);
  ASSERT_EQUAL(gps_course, expected_course);
  ASSERT_EQUAL(gps_itow, expected_itow);
  PASS;
}

int tgps_nmea_GPGGA(void){
  const uint8_t NUM_MSG = 2;
  uint8_t idx = 0;
  char *msg[NUM_MSG];

  struct gpgga {
    uint32_t itow;
    int32_t  lat;
    int32_t  lon;
    bool_t   pos_available;
    int32_t  utm_east;
    int32_t  utm_north;
    uint8_t  utm_zone;
    uint8_t  numSV;
    int32_t  alt;
  };
  
  struct gpgga expected[NUM_MSG];

  // test GPGGA vectors
  idx = 0;
  msg[idx] = (char*)"$GPGGA,204907.250,4221.6044,N,07104.9970,W,2,9,1.02,8.6,M,-33.7,M,0000,0000*6A";
  expected[idx].itow = 74947250; // hhmmss.sss to ms
  expected[idx].lat  = 423600733;
  expected[idx].lon  = -710832833;
  expected[idx].pos_available = TRUE;
  expected[idx].utm_east  = 0;
  expected[idx].utm_north = 0;
  expected[idx].utm_zone  = 0;
  expected[idx].numSV = 9;
  expected[idx].alt   = 859; // (cm)

  idx++;
  msg[idx] = (char*)"$GPGGA,204907.250,4221.6044,S,07104.9970,E,2,9,1.02,8.6,M,-33.7,M,0000,0000*65";
  expected[idx].itow = 74947250; // hhmmss.sss to (ms)
  expected[idx].lat  = -423600733;
  expected[idx].lon  = 710832833;
  expected[idx].pos_available = TRUE;
  expected[idx].utm_east  = 0;
  expected[idx].utm_north = 0;
  expected[idx].utm_zone  = 0;
  expected[idx].numSV = 9;
  expected[idx].alt   = 859; // (cm)


  for(idx=0;idx<NUM_MSG;idx++){    
    sprintf(nmea_msg_buf, "%s", msg[idx]);
    nmea_msg_len = strlen(nmea_msg_buf);
    
    // call function under test
    parse_nmea_GPGGA();
    
#ifdef TESTDEBUGPRINT
    printf("\nparse_nmea_GPGGA: LOOP %d\n", idx);
    printf("parse_nmea_GPGGA: gps_itow = %d\n", gps_itow);
    printf("parse_nmea_GPGGA: gps_lat = %d\n", gps_lat);
    printf("parse_nmea_GPGGA: gps_lon = %d\n", gps_lon);
    printf("parse_nmea_GPGGA: gps_utm_east  = %d\n", gps_utm_east);
    printf("parse_nmea_GPGGA: gps_utm_north = %d\n", gps_utm_north);
    printf("parse_nmea_GPGGA: gps_utm_zone  = %d\n", gps_utm_zone);
    printf("parse_nmea_GPGGA: gps_pos_available = %d\n", gps_pos_available);
    printf("parse_nmea_GPGGA: gps_numSV = %d\n", gps_numSV);
    printf("parse_nmea_GPGGA: gps_alt = %d\n", gps_alt);
#endif

    // MC: itow is no longer set by GPGGA, it is now done in GPRMC message
    //ASSERT_EQUAL(gps_itow, expected[idx].itow);
    ASSERT_EQUAL(gps_lat, expected[idx].lat);
    ASSERT_EQUAL(gps_lon, expected[idx].lon);
    // MC: not sure what to expect for utm_east, utm_north and utm_zone
    //ASSERT_EQUAL(gps_utm_east, expected[idx].utm_east);
    //ASSERT_EQUAL(gps_utm_north, expected[idx].utm_north);
    //ASSERT_EQUAL(gps_utm_zone, expected[idx].utm_zone);
    ASSERT_EQUAL(gps_pos_available, expected[idx].pos_available);
    ASSERT_EQUAL(gps_numSV, expected[idx].numSV);
    ASSERT_EQUAL(gps_alt, expected[idx].alt);
  }

  PASS;
}

int tgps_nmea_GPGSV(void){
  const int LLEN  = 128;
  const int PLEN  = 32;
  const int NUMCH = 16; //assumption: NUMCH = GPS_NB_CHANNELS
  int i;
  int v[NUMCH];
  char msg[LLEN];
  char str[LLEN];
  char param[PLEN];
  FILE *fg;

  struct svinfo e[NUMCH];   

  fg = fopen("gsvvalues.dat", "r");

  // bail out if cannot get golden reference values
  if (fg == NULL){
    FAIL("Cannot open gsvvalues.dat file");
  }
  
  // zero out gps_svinfos[i] array of structures
  for (i=0; i<GPS_NB_CHANNELS; i++){
    gps_svinfos[i].svid  = 0;
    gps_svinfos[i].flags = 0;
    gps_svinfos[i].qi    = 0;
    gps_svinfos[i].cno   = 0;
    gps_svinfos[i].elev  = 0;
    gps_svinfos[i].azim  = 0;
  }

  while(!feof(fg)){
    // NMEA sentence
    fgets(msg, LLEN, fg);

    // svid
    fgets(str, LLEN, fg);
    sscanf(str, "%s %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i",
    	   param, 
    	   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7],
    	   &v[8], &v[9], &v[10], &v[11], &v[12], &v[13], &v[14], &v[15]);
    for (i=0; i<NUMCH; i++) e[i].svid = (uint8_t)v[i];

    // flags
    fgets(str, LLEN, fg);
    sscanf(str, "%s %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i",
    	   param, 
    	   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7],
    	   &v[8], &v[9], &v[10], &v[11], &v[12], &v[13], &v[14], &v[15]);
    for (i=0; i<NUMCH; i++) e[i].flags = (uint8_t)v[i];

    // qi
    fgets(str, LLEN, fg);
    sscanf(str, "%s %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i",
    	   param, 
    	   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7],
    	   &v[8], &v[9], &v[10], &v[11], &v[12], &v[13], &v[14], &v[15]);
    for (i=0; i<NUMCH; i++) e[i].qi = (uint8_t)v[i];

    // cno
    fgets(str, LLEN, fg);
    sscanf(str, "%s %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i",
    	   param, 
    	   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7],
    	   &v[8], &v[9], &v[10], &v[11], &v[12], &v[13], &v[14], &v[15]);
    for (i=0; i<NUMCH; i++) e[i].cno = (uint8_t)v[i];

    // elev
    fgets(str, LLEN, fg);
    sscanf(str, "%s %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i",
    	   param, 
    	   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7],
    	   &v[8], &v[9], &v[10], &v[11], &v[12], &v[13], &v[14], &v[15]);
    for (i=0; i<NUMCH; i++) e[i].elev = (int8_t)v[i];

    // azim
    fgets(str, LLEN, fg);
    sscanf(str, "%s %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i",
	   param, 
    	   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7],
    	   &v[8], &v[9], &v[10], &v[11], &v[12], &v[13], &v[14], &v[15]);
    for (i=0; i<NUMCH; i++) e[i].azim = (int16_t)v[i];    
 
    
    
    // call function under test
    sprintf(nmea_msg_buf, "%s", msg);
    nmea_msg_len = strlen(nmea_msg_buf);
    parse_nmea_GPGSV();

#ifdef TESTDEBUGPRINT
    printf("%s", msg);
    printf("-svid--\n");
    for (i=0; i<NUMCH; i++){
      printf("%i vs %i\n", gps_svinfos[i].svid, e[i].svid);
    }
    printf("-flags-\n");
    for (i=0; i<NUMCH; i++){
      printf("%i vs %i\n", gps_svinfos[i].flags, e[i].flags);
    }
    printf("-qi----\n");
    for (i=0; i<NUMCH; i++){
      printf("%i vs %i\n", gps_svinfos[i].qi, e[i].qi);
    }
    printf("-cno---\n");
    for (i=0; i<NUMCH; i++){
      printf("%i vs %i\n", gps_svinfos[i].cno, e[i].cno);
    }
    printf("-elev--\n");
    for (i=0; i<NUMCH; i++){
      printf("%i vs %i\n", gps_svinfos[i].elev, e[i].elev);
    }
    printf("-azim--\n");
    for (i=0; i<NUMCH; i++){
      printf("%i vs %i\n", gps_svinfos[i].azim, e[i].azim);
    }
    printf("=======\n");
#endif

    // verify results
    for(i=0; i<NUMCH; i++){
    	ASSERT_EQUAL(gps_svinfos[i].svid,  e[i].svid);
    	ASSERT_EQUAL(gps_svinfos[i].flags, e[i].flags);
    	ASSERT_EQUAL(gps_svinfos[i].qi,    e[i].qi);
    	ASSERT_EQUAL(gps_svinfos[i].cno,   e[i].cno);
    	ASSERT_EQUAL(gps_svinfos[i].elev,  e[i].elev);
    	ASSERT_EQUAL(gps_svinfos[i].azim,  e[i].azim);
    }
  }
 
  fclose(fg);

  PASS;
}
