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

//preprocessor definitions

// function prototypes

#ifdef USE_BARO_MS5803

void baro_MS5803_init (void);
int32_t baro_getPressure (void);
int32_t baro_getTemperature (void);
float baro_getAltitude (void);
void baro_write_byte (unsigned char byte);
unsigned char baro_read_byte (void);
void baro_dump_debugging(void);
void baro_reset(void);
int check_baro_state(void);

//external variables
extern float baro_MS5803_last_altitude;
#endif //USE_BARO_MS5803
#endif //BARO_MS5803_H
