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

/** \file airspeed_MS5701.c
 *  \brief Handling of the MS5701 pressure sensor
 *
 * uses: MOSI, MISO, SCK
 *  code to read pressure and calculate airspeed from the measurement specialities MS5701 sensor.
 * 
 *   datasheet for sensor is here:  
 *    MS5701  http://www.intersema.ch/index.php?option=com_rubberdoc&view=doc&id=86&format=raw
 *
 * By default, chip is hooked with select to pin P1.21
 */


/* ----------------------------------------------------------------------
 *  This sensor has oversampling control which allows it to function
     at different rates depending on desired resolution
      with 4096 oversampling, max conversion is 9mS, resolution is .001mbar

    It is recommended to use the highest oversampling, and we can assume a 10mS conversion time
   ---------------------------------------------------------------------- */

#include "std.h"
#include "LPC21xx.h"
#include "mcu_periph/spi.h"
#include "sys_time.h"
#include "modules/sensors/airspeed_MS5701.h"

#include "subsystems/nav.h"
#include "estimator.h"

//#include "usb_debug.h"

//--------------------------------------------------
// defines

//define this if you want datasheet example values instead of real ones
//#define MS5701_EXAMPLE_VALUES

//calibration values
#define CAL_SENS_T1 1
#define CAL_OFF_T1 2
#define CAL_TCS 3
#define CAL_TCO 4
#define CAL_T_REF 5
#define CAL_TEMPSENS 6

#define VAL_SENS_T1 ms5701_cal_table[CAL_SENS_T1]
#define VAL_OFF_T1 ms5701_cal_table[CAL_OFF_T1]
#define VAL_TCS ms5701_cal_table[CAL_TCS]
#define VAL_TCO ms5701_cal_table[CAL_TCO]
#define VAL_T_REF ms5701_cal_table[CAL_T_REF]
#define VAL_TEMPSENS ms5701_cal_table[CAL_TEMPSENS]

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

#define DELAY_MARGIN 1000

#define DELAY_RESET 3000

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

#define AIRSPEED_SS_PIN   21		//20 // 20 is SSEL on the SPI bus   7 is the external button on the USB bus.  this is GPIO Port 0
#define AIRSPEED_SS_IODIR IO1DIR
#define AIRSPEED_SS_IOSET IO1SET
#define AIRSPEED_SS_IOCLR IO1CLR 	// -->  defined in ./arm7/include/LPC21xx.h

//--------------------------------------------------
// globals
sensor_state_t ms5701_state = uninit;
uint16_t ms5701_cal_table[MS5701_CAL_NUM_VALUES];   //calibration constant table, read at startup
int32_t ms5701_dT;
float airspeed_MS5701_last_airspeed;
int32_t airspeed_periodic_state;

float MS5701_last_temperature;
float MS5701_last_pressure;

// sets reference pressure for 0 airspeed
float airspeed_MS5701_zero_reference_pressure;
int airspeed_MS5701_zero_reference_calibrate_start = 0;

float airspeed_MS5701_reference_pressure = 1013.25;

//--------------------------------------------------
// code

//-------------------------
// select/deselect functions
inline void ms5701_select(void) {
  SetBit(AIRSPEED_SS_IOCLR,AIRSPEED_SS_PIN);
}
inline void ms5701_unselect() {
  SetBit(AIRSPEED_SS_IOSET,AIRSPEED_SS_PIN);
}



//-------------------------
//initializes the MS5701 barometric pressure sensor
void airspeed_MS5701_init (void) {

  //setup SPI
 /* setup pins for SSP (SCK, MISO, MOSI) */
//  PINSEL1 |= 2 << 2 | 2 << 4 | 2 << 6;
  
  /* setup SSP */
//  SSPCR0 = SSP_DDS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
//  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
//  SSPCPSR = 0x20;

  /* configure SS pin */
  SetBit(AIRSPEED_SS_IODIR, AIRSPEED_SS_PIN); /* SS pin is output  */
  ms5701_unselect();
  //done with SPI setup

  ms5701_state = reset;
  airspeed_periodic_state = 0;
  airspeed_MS5701_last_airspeed=-9999;
}

void airspeed_dump_debugging(void) {

  #ifdef MS5701_DEBUG

  int i;
  int32_t temp;
  int32_t pressure;

  USB_DEBUG_OUT("%s","MS5701 cal:");
  for(i=1;i<MS5701_CAL_NUM_VALUES;i++) {
    USB_DEBUG_OUT(" %d",ms5701_cal_table[i]);
  }
  USB_DEBUG_OUT("%s","\n\r");

  //--do sensor test
  /* temp = airspeed_getTemperature(); */
  /* USB_DEBUG_OUT("MS5701_Temp: %ld\r\n",temp); */

  /* pressure = airspeed_getPressure(); */
  /* USB_DEBUG_OUT("MS5701_Pressure: %ld\r\n",pressure); */
  #endif

}

//--------------------------------------------------
int check_airspeed_state(void) {

  int i;
  unsigned char temp[2];

  if (ms5701_state == reset) {

    //call reset function
    airspeed_reset();
    ms5701_state = calibration;

    return FALSE;

  } else if (ms5701_state == calibration) {
    
    SpiEnable();

  //read calibration table in
  // do not read 0, is a manufacturer reserved value
    for(i=0;i<MS5701_CAL_NUM_VALUES;i++) {

    sys_time_usleep(100);
    ms5701_select();
    
    //write command
    airspeed_write_byte (COM_PROM_READ | (i<<1));

    //read result
    temp[0] = airspeed_read_byte();
    temp[1] = airspeed_read_byte();

    //store
    ms5701_cal_table[i] = (temp[0]<<8) + temp[1];

    ms5701_unselect();

    } 
 
#ifdef MS5701_EXAMPLE_VALUES
    ms5701_cal_table[0]=9999;
    ms5701_cal_table[1]=30343;
    ms5701_cal_table[2]=32322;
    ms5701_cal_table[3]=40838;
    ms5701_cal_table[4]=39805;
    ms5701_cal_table[5]=34295;
    ms5701_cal_table[6]=29546;
    ms5701_cal_table[7]=9999;
#endif

    ms5701_state = running;
    return FALSE;

  } else if (ms5701_state == running) {

    return TRUE;

  } else {
#ifdef MS5701_DEBUG
    USB_DEBUG_OUT("%s","MS5701_Error\r\n");
#endif
    return FALSE;

  }
  

}

//--------------------------------------------------
// function to write SPI byte
void airspeed_write_byte (unsigned char byte) {
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
unsigned char airspeed_read_byte (void) {
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


//reset the ms5701
void airspeed_reset () {

  SpiEnable();

  ms5701_select();

  airspeed_write_byte( COM_RESET );

  // WAIT A WHILE
  sys_time_usleep ( DELAY_RESET + DELAY_MARGIN );

  ms5701_unselect();

}

// start the pressure conversion process
void airspeed_startPressure (void) {
  SpiEnable();

  //initiate conversion
  ms5701_select();
  airspeed_write_byte( COM_CONVERT_D1 | OSR_CONVERSION );
  //we must now wait a little while
  sys_time_usleep( 10 );

  ms5701_unselect();
}

// read back the pressure result
//
//    returns actual millebars
float airspeed_readPressure (void) {
  int i;
  unsigned char temp[3];
  int32_t D1;
  double OFF,SENS,PRESSURE;

  //now read back the 24bit result
  ms5701_select();
  airspeed_write_byte( COM_ADC_READ );
  for(i=0;i<3;i++) {
    temp[i] = airspeed_read_byte();
  }
  ms5701_unselect();

 #ifdef MS5701_EXAMPLE_VALUES
  temp[0] = 0x8A;
  temp[1] = 0x4F;
  temp[2] = 0xFA;
 #endif

  //store value and return
  D1 = (temp[0] << 16) + (temp[1] << 8) + temp[2];

  // values from datasheet example case, for testing math
  //D1 = 9085466; 
  //VAL_OFF_T1 = 36924; 
  //VAL_TCO = 23282; 
  //ms5701_dT = 2366;
  //VAL_SENS_T1 = 40127; 
  //VAL_TCS = 23317; 

  // coefficients from the datasheet
  OFF = ((double)VAL_OFF_T1 * (double)TWO_TOTHE17) + (((double)VAL_TCO * (double)ms5701_dT) / (double)TWO_TOTHE7);
  SENS = ((double)VAL_SENS_T1 * (double)TWO_TOTHE15) + (((double)VAL_TCS * (double)ms5701_dT) / TWO_TOTHE9);

/*   // coefficients from the example code */
/*   OFF = ((double)VAL_OFF_T1 * (double)TWO_TOTHE17) + (((double)VAL_TCO * (double)ms5701_dT) / (double)TWO_TOTHE6); */
/*   SENS = ((double)VAL_SENS_T1 * (double)TWO_TOTHE16) + (((double)VAL_TCS * (double)ms5701_dT) / TWO_TOTHE7); */

  PRESSURE = ( (D1 * SENS / (double)TWO_TOTHE21) - OFF) / (double)TWO_TOTHE15;

  return (PRESSURE/1000);
}


//-------------------------
// function returns pressure in mbar
float airspeed_getPressure (void) {

  airspeed_startPressure();

  sys_time_usleep( OSR_DELAY+DELAY_MARGIN );

  return airspeed_readPressure();
}

// start the temperature conversion process
void airspeed_startTemperature (void) {

  SpiEnable();

  //initiate conversion
  ms5701_select();
  airspeed_write_byte( COM_CONVERT_D2 | OSR_CONVERSION );
  //we must now wait a little while
  sys_time_usleep(10);
  ms5701_unselect();

}

//read back the temperature conversion result
float airspeed_readTemperature (void) {
  unsigned char temp[3];
  int32_t D2,dT;
  float TEMP;
  int i;

  //now read back the 24bit result
  ms5701_select();
  airspeed_write_byte( COM_ADC_READ );
  for(i=0;i<3;i++) {
    temp[i] = airspeed_read_byte();
  }
  ms5701_unselect();

 #ifdef MS5701_EXAMPLE_VALUES
  temp[0] = 0x8F;
  temp[1] = 0x53;
  temp[2] = 0xE6;
 #endif

  //store value and return
  D2 = (temp[0] << 16) + (temp[1] << 8) + temp[2];

  dT = D2 - (VAL_T_REF << 8);
  TEMP = 2000 + (((float)dT * (float)VAL_TEMPSENS) / TWO_TOTHE23);

  ms5701_dT = dT; //store for pressure calculation later

  return TEMP;
}


//-------------------------
// function returns temperature value
//  returns temperature in centidegrees C
//  ie 2007 = 20.07 degrees C
//  this function blocks until read is complete
float airspeed_getTemperature (void) {

  airspeed_startTemperature();

  sys_time_usleep(OSR_DELAY+DELAY_MARGIN);

  return airspeed_readTemperature()/100;
}


//Function to calculate airspeed from pressure differential. Units are millibars and meters/second
float dp2cas(float dp, float Pref)
{	
  float a_sl = 343; // Speed of sound dry air at 20C (m/s)
  float speed = a_sl*sqrt( 5*(pow((dp/Pref + 1),(2./7.)) - 1.) );
  return speed;
}


//-------------------------
// function returns airspeed in meters/second
//   returns negative in the case of invalid reading or sensor not ready
float airspeed_getAirspeed (void) {
  float pressure;
  float airspeed;

  // the check_airspeed_state handles initialization and 
  //    transfer of the calibration information.
  //    this function will not return real data until after a few
  //    calls 
  if (check_airspeed_state() == TRUE) {

    #ifdef MS5701_DEBUG
    USB_DEBUG_OUT("%s"," MS5701 Init Done\r\n");
    #endif

    // get temperature (stores temp compensation factor internally)
    MS5701_last_temperature = airspeed_getTemperature();
  
    // fetch pressure - units are .01mbar
    pressure = airspeed_getPressure();
    MS5701_last_pressure = pressure;

    // calibrate if it is time
    if ( airspeed_MS5701_zero_reference_calibrate_start == 1) {
      airspeed_MS5701_zero_reference_calibrate_start = 0;
      airspeed_MS5701_zero_reference_pressure = pressure;
    }

    pressure = fabs(pressure - airspeed_MS5701_zero_reference_pressure);

    //calculate airspeed using same calc as airspeed_adc.c
    //  probably about as accurate as you can get without temp compensation
    airspeed = dp2cas( (float)pressure, airspeed_MS5701_reference_pressure );

    airspeed_MS5701_last_airspeed = airspeed;

    #ifdef MS5701_DEBUG
    USB_DEBUG_OUT("spd: %f %d\r\n",airspeed,pressure);
    #endif

    return airspeed;

  } else {
    return -9999.0;
  }

}

//----------------------------------------
// function updates the current airspeed
void airspeed_MS5701_update (void) {
  EstimatorSetAirspeed( airspeed_getAirspeed() );
}


//------------------------------
// periodic function for airspeed module
//  should be called at 60hz, and will update airspeed at 10Hz 
void airspeed_MS5701_periodic (void) {
  static int airspeed_getAltitude_periodic = 0;

  float pressure;
  float airspeed;
#ifndef SITL
  // the check_airspeed_state handles initialization and 
  //    transfer of the calibration information.
  //    this function will not return real data until after a few
  //    calls 
  if (check_airspeed_state() == TRUE) {

    switch (airspeed_periodic_state) {
      case 0: //start temperature conversion
	airspeed_startTemperature();
	break;
      case 1: //read temperature
	airspeed_readTemperature();
	break;
      case 2: //start pressure conversion
	airspeed_startPressure();
	break;
      case 3: //read pressure, do calcs and set airspeed
	pressure = airspeed_readPressure();
	airspeed = dp2cas( (float)pressure, airspeed_MS5701_reference_pressure );
	EstimatorSetAirspeed(airspeed);

        #ifdef MS5701_DEBUG
 	USB_DEBUG_OUT("spd: %f %d\r\n",airspeed,pressure); 
        #endif
	break;
      case 4: //nothing
	break;
      case 5: //nothing, reset to first state
	airspeed_periodic_state = -1;
	break;
      }
    airspeed_periodic_state++;
  }
#else //SITL
  extern float sim_air_speed;
  EstimatorSetAirspeed(sim_air_speed);
#endif
}



//----------------------------------------------------------------------
// sets current input pressure to be the zero airspeed reference pressure


// sets reference pressure for 0 airspeed
int airspeed_set_zero_reference() {

  airspeed_MS5701_zero_reference_calibrate_start = 1;

  return FALSE;

}
