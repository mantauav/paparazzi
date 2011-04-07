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

/** \file airspeed_MS5701.h
 *  \brief Handling of the MS5701 pressure sensor
 *
 * uses: MOSI, MISO, SCK
 *  code to read pressure and calculate airspeed from the measurement specialities MS5701 sensor.
 * 
 *   datasheet for sensor is here:  
 *    MS5701  http://www.intersema.ch/index.php?option=com_rubberdoc&view=doc&id=86&format=raw
 */

#ifndef AIRSPEED_MS5701_H
#define AIRSPEED_MS5701_H

//preprocessor definitions


//--------------------------------------------------
// types
#ifndef SENSOR_STATE_T
#define SENSOR_STATE_T
typedef enum { uninit, reset, calibration, running } sensor_state_t;
#endif



// function prototypes

#ifdef USE_AIRSPEED_MS5701

// Cal PROM has 8 values even though only 6 are used
#define MS5701_CAL_NUM_VALUES 8

void airspeed_MS5701_init (void);
float airspeed_getPressure (void);
float airspeed_getTemperature (void);
float airspeed_getAirspeed (void);
void airspeed_write_byte (unsigned char byte);
unsigned char airspeed_read_byte (void);
void airspeed_dump_debugging(void);
void airspeed_reset(void);
int check_airspeed_state(void);
void airspeed_MS5701_update( void );

//calibration function
int airspeed_set_zero_reference( void );

// periodic function, runs state machine and updates airspeed when value is available
//  called at 10x update rate
void airspeed_MS5701_periodic (void);

//external variables
extern float airspeed_MS5701_last_airspeed;

//externs used for telemetry
extern sensor_state_t ms5701_state;
extern uint16_t ms5701_cal_table[MS5701_CAL_NUM_VALUES];
extern int32_t ms5701_dT;
extern int32_t airspeed_periodic_state;
extern float MS5701_last_temperature;
extern float MS5701_last_pressure;
extern float airspeed_MS5701_zero_reference_pressure;

// spi functions
inline void ms5701_select(void);
inline void ms5701_unselect(void);

#endif //USE_AIRSPEED_MS5701
#endif //AIRSPEED_MS5701_H
