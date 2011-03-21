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
 *
 * By default, chip is hooked to pin P0.07
 */


/* ----------------------------------------------------------------------
 *  This sensor has oversampling control which allows it to function
     at different rates depending on desired resolution
      with 4096 oversampling, max conversion is 9mS, resolution is .012mbar

    It is recommended to use the 4096 oversample, and we can assume a 10mS conversion time.
   ---------------------------------------------------------------------- */


#include "std.h"
#include "LPC21xx.h"
#include "mcu_periph/spi.h"
#include "sys_time.h"
#include "modules/sensors/baro_MS5803.h"

#include "subsystems/nav.h"
#include "estimator.h"

//--------------------------------------------------
// defines

//define this if you want datasheet example values instead of real ones
//
//  This can be used to test the code
//    Temperature returned should be 20.07C
//    Pressure returned should be 1000.09mbar
//#define MS5803_EXAMPLE_VALUES

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
#define DELAY_MARGIN 100

//Constants for floating point math
#define TWO_TOTHE6 64
#define TWO_TOTHE7 128
#define TWO_TOTHE8 256
#define TWO_TOTHE9 512
#define TWO_TOTHE15 32768
#define TWO_TOTHE16 65536
#define TWO_TOTHE17 131072
#define TWO_TOTHE21 2097152
#define TWO_TOTHE23 8388608

//These set the default oversampling used for conversions
//  They are overridden in the module settings
#ifndef OSR_CONVERSION
#define OSR_CONVERSION OSR4096
#endif
#ifndef OSR_DELAY
#define OSR_DELAY DELAY_OSR4096
#endif

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

//--------------------------------------------------
// globals
sensor_state_t ms5803_state = uninit;
uint16_t ms5803_cal_table[MS5803_CAL_NUM_VALUES];   //calibration constant table, read at startup
int32_t ms5803_dT;
float baro_MS5803_last_altitude;
int32_t baro_periodic_state;

float MS5803_last_temperature;
float MS5803_last_pressure;

float baro_MS5803_sealevel_reference_pressure = 1013;
float baro_MS5803_calibrate_reference_altitude;
int baro_MS5803_calibrate_reference_altitude_start = 0;
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
  //unsigned char temp[2];

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
  
  //int32_t temp;
  #ifdef MS5803_DEBUG
  int i;
  int32_t pressure;
  USB_DEBUG_OUT("%s","MS5803 cal:");
  for(i=1;i<MS5803_CAL_NUM_VALUES;i++) {
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
    for(i=0;i<MS5803_CAL_NUM_VALUES;i++) {

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
 
#ifdef MS5803_EXAMPLE_VALUES
    ms5803_cal_table[0]=9999;
    ms5803_cal_table[1]=40127;
    ms5803_cal_table[2]=36924;
    ms5803_cal_table[3]=23317;
    ms5803_cal_table[4]=23282;
    ms5803_cal_table[5]=33464;
    ms5803_cal_table[6]=28312;
    ms5803_cal_table[7]=9999;
#endif
  
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
// function to read SPI byte
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
  sys_time_usleep ( DELAY_RESET + DELAY_MARGIN );

  ms5803_unselect();

}

// start the pressure conversion process
void baro_startPressure (void) {

  SpiEnable();

  //initiate conversion
  ms5803_select();
  baro_write_byte( COM_CONVERT_D1 | OSR_CONVERSION );

  //we must now wait a little while
  sys_time_usleep( 10 );

  ms5803_unselect();

}



//--------------------------------------------------
// read back the pressure result
//
//    returns actual millebars
float baro_readPressure (void) {
  int i;
  unsigned char temp[3];
  int32_t D1;
  double OFF,SENS,PRESSURE;

  //now read back the 24bit result
  ms5803_select();
  baro_write_byte( COM_ADC_READ );
  for(i=0;i<3;i++) {
    temp[i] = baro_read_byte();
  }
  ms5803_unselect();

 #ifdef MS5803_EXAMPLE_VALUES
  temp[0] = 0x8A;
  temp[1] = 0xA2;
  temp[2] = 0x1A;
 #endif

  //store value and return
  D1 = (temp[0] << 16) + (temp[1] << 8) + temp[2];

  // values from datasheet example case, for testing math
  //D1 = 9085466; 
  //VAL_OFF_T1 = 36924; 
  //VAL_TCO = 23282; 
  //ms5803_dT = 2366;
  //VAL_SENS_T1 = 40127; 
  //VAL_TCS = 23317; 

  OFF = ((double)VAL_OFF_T1 * 0x10000) + ((double)VAL_TCO * (double)ms5803_dT) / 0x80;

  SENS = ((double)VAL_SENS_T1 * 0x8000) + ((double)VAL_TCS * (double)ms5803_dT) / 0x100;

  PRESSURE = ( (D1 * SENS / 0x200000) - OFF) / 0x8000;

  return (PRESSURE/100);
}

//--------------------------------------------------
//  Gets a pressure reading from the sensor
float baro_getPressure (void) {

  baro_startPressure();

  //wait the conversion delay time
  sys_time_usleep(OSR_DELAY+DELAY_MARGIN);

  return baro_readPressure();

}


//--------------------------------------------------
// Read back the temperature values from the sensor
float baro_readTemperature (void) {
  unsigned char temp[3];
  int32_t D2,dT;
  float TEMP;
  int i;

  //now read back the 24bit result
  ms5803_select();
  baro_write_byte( COM_ADC_READ );
  for(i=0;i<3;i++) {
    temp[i] = baro_read_byte();
  }
  ms5803_unselect();

 #ifdef MS5803_EXAMPLE_VALUES
  temp[0] = 0x82;
  temp[1] = 0xC1;
  temp[2] = 0x3E;
 #endif

  //store value and return
  D2 = (temp[0] << 16) + (temp[1] << 8) + temp[2];

  dT = D2 - (VAL_T_REF << 8);
  TEMP = 2000 + (((float)dT * (float)VAL_TEMPSENS) / TWO_TOTHE23);

  ms5803_dT = dT; //store for pressure calculation later

  return TEMP;
}


//--------------------------------------------------
// Starts the temperature conversion
void baro_startTemperature (void) {

  SpiEnable();

  //initiate conversion
  ms5803_select();
  baro_write_byte( COM_CONVERT_D2 | OSR_CONVERSION );

  //we must now wait a little while
  sys_time_usleep(10);

  ms5803_unselect();

}


//-------------------------
// function returns temperature value
//  returns temperature in degrees C
float baro_getTemperature (void) {
  baro_startTemperature();

  sys_time_usleep(OSR_DELAY+DELAY_MARGIN);

  return baro_readTemperature()/100;
}



//-------------------------
// function returns altitude in meters
float baro_getAltitude (void) {
  float pressure;
  uint32_t baro_i;

  if (check_baro_state() == TRUE) {

    #ifdef MS5803_DEBUG
    USB_DEBUG_OUT("%s","Init Done\r\n");
    #endif

    // get temperature (stores temp compensation factor internally)
    MS5803_last_temperature = baro_getTemperature();
  
    // fetch pressure
    pressure = baro_getPressure();
    MS5803_last_pressure = pressure;

    if (baro_MS5803_calibrate_reference_altitude_start == 1) {
      baro_MS5803_calibrate_reference_altitude_start = 0;
      
      baro_MS5803_sealevel_reference_pressure = pressure / pow( (1 - ( baro_MS5803_calibrate_reference_altitude / 44307.693)), 5.25588);

    }

    //    pow(1-(altitude/44307),5.25588)=pressure/Pref

    //conversion from noaa.gov
    // dont need it to be wholly accurate.  just that the conversion is consistent.
    double altitude;
    altitude = (1 - pow(((double)pressure/baro_MS5803_sealevel_reference_pressure),0.190263)) * 44307.693;

    baro_i = altitude;
    baro_MS5803_last_altitude=(float)altitude;

    #ifdef MS5803_DEBUG
    USB_DEBUG_OUT("alt: %d %d\r\n",baro_i,pressure);
    #endif

    return altitude;

  } else {
    return 0;
  }

}


//-------------------------
// function sets sealevel reference pressure
void baro_calibrateReferencePressure (float refPressure) {

  baro_MS5803_sealevel_reference_pressure = refPressure;

}



//-------------------------
// function calibrates barometer using input reference altitude in meters
//
//  returns TRUE on error (which means sensor is not yet calibrated)
//  returns FALSE on success
int baro_calibrateReferenceAltitude ( float refAlt ) {

  baro_MS5803_calibrate_reference_altitude = refAlt;
  baro_MS5803_calibrate_reference_altitude_start = 1;

  return FALSE;

}
  

