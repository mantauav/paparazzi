/*
 * $Id$
 *
 * Copyright (C) 2010 Aurora Flight Sciences
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

/*
 *    Jpeverill
 *    jpeverill@aurora.aero
 *    8/18/2010
 *
 */

/** \file baro_MS5534A.c
 *  \brief Handling of the MS5803 pressure sensor
 *
 * uses: MOSI, MISO, SCK
 *  code to read pressure and calculate altitude from the measurement specialities MS5803 sensor.
 * 
 *   datasheet for sensor is here:  
 *    MS5803   http://www.meas-spec.com/pressure-sensors/board-level-pressure-sensors/digital-pressure-sensors.aspx#
 */


#include "std.h"
#include "LPC21xx.h"
#include "mcu_periph/spi.h"
#include "sys_time.h"
#include "baro_MS5803.h"

//#include "usb_debug.h"

//--------------------------------------------------
// defines

//calibration values
#define CAL_SENS_T1 1
#define CAL_OFF_T1 2
#define CAL_TCS 3
#define CAL_TCO 4
#define CAL_T_REF 5
#define CAL_TEMPSENS 6

#define VAL_SENS_T1 ms5803_cal_table[CAL_SENS_T1]
#define VAL_OFF_T1 ms5803_cal_table[CAL_OFF_T1]
#define VAL_TCS ms5803_cal_table[CAL_TCS]
#define VAL_TCO ms5803_cal_table[CAL_TCO]
#define VAL_T_REF ms5803_cal_table[CAL_T_REF]
#define VAL_TEMPSENS ms5803_cal_table[CAL_TEMPSENS]


#define CAL_NUM_VALUES 8

//commands
#define COM_RESET 0x1E
#define COM_CONVERT_D1 0x40
#define COM_CONVERT_D2 0x50
#define COM_ADC_READ 0x00
#define COM_PROM_READ 0xA0 //command should be OR'd with cal values above

//D1/D2 OSR setting (Or'd with Com_convert_d1 above)
#define OSR256 0x0
#define OSR512 0x2
#define OSR1024 0x4
#define OSR2048 0x6
#define OSR4096 0x8

//OSR max conversion delays in microseconds
#define DELAY_OSR256 600
#define DELAY_OSR512 1170
#define DELAY_OSR1024 2280
#define DELAY_OSR2048 4540
#define DELAY_OSR4096 9004

#define DELAY_RESET 3000


#define OSR_CONVERSION OSR256
#define OSR_DELAY DELAY_OSR256

// define these to test for conversion delay errors
//#define OSR_CONVERSION 0
//#define OSR_DELAY 0

#define SPI_BUSY (SSPSR & 0x0000010)


//SPI defines
/* SSPCR0 settings */
#define SSP_DDS  0x07 << 0  	/* data size         : 8 bits        */
#define SSP_FRF  0x00 << 4  	/* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  	/* clock polarity    : data captured on first clock transition */  
#define SSP_CPHA 0x00 << 7  	/* clock phase       : SCK idles low */
#define SSP_SCR  0x0F << 8  	/* serial clock rate : divide by 16  */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  	/* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  	/* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  	/* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  	/* slave output disable : don't care when master */

#define SS_PIN   7		//20 // 20 is SSEL on the SPI bus   7 is the external button on the USB bus.  this is GPIO Port 0
#define SS_IODIR IO0DIR
#define SS_IOSET IO0SET
#define SS_IOCLR IO0CLR 	// -->  defined in ./arm7/include/LPC21xx.h


//--------------------------------------------------
// types
typedef enum { uninit, reset, calibration, running } ms5803_sensor_state;


//--------------------------------------------------
// globals
ms5803_sensor_state ms5803_state = uninit;
uint16_t ms5803_cal_table[CAL_NUM_VALUES];   //calibration constant table, read at startup
int32_t ms5803_dT;
float baro_MS5803_last_altitude;
//--------------------------------------------------
// code

//-------------------------
// select/deselect functions
inline void ms5803_select(void) {
  SetBit(SS_IOCLR,SS_PIN);
}
inline void ms5803_unselect() {
  SetBit(SS_IOSET,SS_PIN);
}


//-------------------------
//initializes the MS5803 barometric pressure sensor
void baro_MS5803_init (void) {
  int i;
  unsigned char temp[2];

  //setup SPI
 /* setup pins for SSP (SCK, MISO, MOSI) */
//  PINSEL1 |= 2 << 2 | 2 << 4 | 2 << 6;
  
  /* setup SSP */
//  SSPCR0 = SSP_DDS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
//  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
//  SSPCPSR = 0x20;

  /* configure SS pin */
  SetBit(SS_IODIR, SS_PIN); /* SS pin is output  */
  ms5803_unselect();
  //done with SPI setup

  ms5803_state = reset;
  baro_MS5803_last_altitude=-9999;
}

void baro_dump_debugging(void) {
  int i;
  int32_t temp;
  int32_t pressure;

  #ifdef MS5803_DEBUG
  USB_DEBUG_OUT("%s","MS5803 cal:");
  for(i=1;i<CAL_NUM_VALUES;i++) {
    USB_DEBUG_OUT(" %d",ms5803_cal_table[i]);
  }
  USB_DEBUG_OUT("%s","\n\r");

  //--do sensor test
  /* temp = baro_getTemperature(); */
  /* USB_DEBUG_OUT("MS5803_Temp: %ld\r\n",temp); */

  /* pressure = baro_getPressure(); */
  /* USB_DEBUG_OUT("MS5803_Pressure: %ld\r\n",pressure); */
  #endif

}

//--------------------------------------------------
int check_baro_state(void) {

  int i;
  unsigned char temp[2];

  if (ms5803_state == reset) {

    //call reset function
    baro_reset();
    ms5803_state = calibration;

    return FALSE;

  } else if (ms5803_state == calibration) {
    
    SpiEnable();

  //read calibration table in
  // do not read 0, is a manufacturer reserved value
    for(i=1;i<CAL_NUM_VALUES;i++) {

    sys_time_usleep(100);
    ms5803_select();
    
    //write command
    baro_write_byte (COM_PROM_READ | (i<<1));

    //read result
    temp[0] = baro_read_byte();
    temp[1] = baro_read_byte();

    //store
    ms5803_cal_table[i] = (temp[0]<<8) + temp[1];

    ms5803_unselect();

    } 
  
    ms5803_state = running;
    return FALSE;

  } else if (ms5803_state == running) {

    return TRUE;

  } else {
#ifdef MS5803_DEBUG
    USB_DEBUG_OUT("%s","MS5803_Error\r\n");
#endif
    return FALSE;

  }
  

}

//--------------------------------------------------
// function to write SPI byte
void baro_write_byte (unsigned char byte) {
  volatile int g;

  // wait for SPI to be free
  while (SPI_BUSY) {g=g;}
  // write out command to read prom
  SSPDR= byte;
  while (SPI_BUSY) {g=g;}
  g = SSPDR;
}

//--------------------------------------------------
// function to write SPI byte
unsigned char baro_read_byte (void) {
  volatile int g;

  // wait for SPI to be free
  while (SPI_BUSY) {g=g;}
  // write out command to read prom
  SSPDR= 0;
  while (SPI_BUSY) {g=g;}
  return SSPDR;
}


typedef struct {
  uint32_t a;
  uint32_t b;
} two_int32;

typedef union {
  long long l;
  two_int32 i;
} big_split;


//reset the ms5803
void baro_reset () {

  SpiEnable();

  ms5803_select();

  baro_write_byte( COM_RESET );

  // WAIT A WHILE
  sys_time_usleep ( DELAY_RESET +250 );

  ms5803_unselect();

}



//-------------------------
// function returns pressure in hundredth's of mbar
// ie  100023 = 1000.23 mbar
int32_t baro_getPressure (void) {
  int i;
  unsigned char temp[3];
  int32_t D1;
  double OFF,OFF_int1,OFF_int2,SENS,SENS_int1,SENS_int2,PRESSURE;
  int32_t OFF_int,SENS_int;

  SpiEnable();

  //initiate conversion
  ms5803_select();
  baro_write_byte( COM_CONVERT_D1 | OSR_CONVERSION );
  //we must now wait a little while
  sys_time_usleep( OSR_DELAY+250 );

  ms5803_unselect();

  sys_time_usleep(10);

  //now read back the 24bit result
  ms5803_select();
  baro_write_byte( COM_ADC_READ );
  for(i=0;i<3;i++) {
    temp[i] = baro_read_byte();
  }
  ms5803_unselect();

  //store value and return
  D1 = (temp[0] << 16) + (temp[1] << 8) + temp[2];

  // values from datasheet example case, for testing math
  //D1 = 9085466; 
  //VAL_OFF_T1 = 36924; 
  //VAL_TCO = 23282; 
  //ms5803_dT = 2366;
  //VAL_SENS_T1 = 40127; 
  //VAL_TCS = 23317; 

  OFF_int1 = ((double)VAL_OFF_T1 * 0x10000);
  OFF_int2 = ((double)VAL_TCO * (double)ms5803_dT) / 0x80;
  OFF = OFF_int1 + OFF_int2;

  SENS_int1 = ((double)VAL_SENS_T1 * 0x8000);
  SENS_int2 = ((double)VAL_TCS * (double)ms5803_dT) / 0x100;
  SENS = SENS_int1 + SENS_int2;

  PRESSURE = ( (D1 * SENS / 0x200000) - OFF) / 0x8000;

  return PRESSURE;
}

//-------------------------
// function returns temperature value
//  returns temperature in centidegrees C
//  ie 2007 = 20.07 degrees C
int32_t baro_getTemperature (void) {

  unsigned char temp[3];
  int32_t D2,dT;
  long long int TEMP;
  int i;

  SpiEnable();

  //initiate conversion
  ms5803_select();
  baro_write_byte( COM_CONVERT_D2 | OSR_CONVERSION );
  //we must now wait a little while
  sys_time_usleep(OSR_DELAY+250);
  ms5803_unselect();

  sys_time_usleep(10);

  //now read back the 24bit result
  ms5803_select();
  baro_write_byte( COM_ADC_READ );
  for(i=0;i<3;i++) {
    temp[i] = baro_read_byte();
  }
  ms5803_unselect();

  //store value and return
  D2 = (temp[0] << 16) + (temp[1] << 8) + temp[2];

  dT = D2 - (VAL_T_REF << 8);
  TEMP = 2000 + ((dT * VAL_TEMPSENS) >> 23);

  ms5803_dT = dT; //store for pressure calculation later

  return TEMP;
}

//-------------------------
// function returns altitude in meters
float baro_getAltitude (void) {
  uint32_t pressure;
  uint32_t altitude_i;

  if (check_baro_state() == TRUE) {

    #ifdef MS5803_DEBUG
    USB_DEBUG_OUT("%s","Init Done\r\n");
    #endif

    // get temperature (stores temp compensation factor internally)
    baro_getTemperature();
  
    // fetch pressure
    pressure = baro_getPressure();

    //conversion from noaa.gov
    // dont need it to be wholly accurate.  just that the conversion is consistent.
    double altitude;
    altitude = (1 - pow(((double)pressure/101325),0.190263)) * 44330.762;

    altitude_i = altitude;
    baro_MS5803_last_altitude=(float)altitude;
    #ifdef MS5803_DEBUG
    USB_DEBUG_OUT("alt: %d %d\r\n",altitude_i,pressure);
    #endif

    return altitude;

  } else {
    return 0;
  }

}



