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

#ifndef BARO_MS5803_H
#define BARO_MS5803_H

// function prototypes
#ifdef USE_BARO_MS5803

//preprocessor definitions--------------------


// Cal PROM has 8 values even though only 6 are used
#define MS5803_CAL_NUM_VALUES 8


// types ----------------------------------------

#ifndef SENSOR_STATE_T
#define SENSOR_STATE_T
typedef enum { uninit, reset, calibration, running } sensor_state_t;
#endif


void baro_MS5803_init (void);
float baro_getPressure (void);
float baro_getTemperature (void);
float baro_getAltitude (void);
void baro_write_byte (unsigned char byte);
unsigned char baro_read_byte (void);
void baro_dump_debugging(void);
void baro_reset(void);
int check_baro_state(void);
void baro_startTemperature (void);
void baro_startPressure (void);
float baro_readPressure (void);
float baro_readTemperature (void);

//calibration functions
int baro_calibrateReferenceAltitude (float refAlt);
void baro_calibrateReferencePressure (float refPressure);

//external variables
extern float baro_MS5803_last_altitude;

//external variables for telemetry test message
extern sensor_state_t ms5803_state;
extern uint16_t ms5803_cal_table[MS5803_CAL_NUM_VALUES];   //calibration constant table, read at startup
extern int32_t ms5803_dT;
extern float MS5803_last_temperature;
extern float MS5803_last_pressure;
extern int32_t baro_periodic_state;

extern float baro_MS5803_sealevel_reference_pressure;

inline void ms5803_select(void);
inline void ms5803_unselect(void);

#endif //USE_BARO_MS5803

#endif //BARO_MS5803_H
