/*
 * Copyright (C) 2010 The Paparazzi Team
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

#ifndef RC_GYRO_H
#define RC_GYRO_H

#include <inttypes.h>
#include "paparazzi.h"
#include "generated/airframe.h"

extern pprz_t	rc_gyro_roll_gain;
extern pprz_t	rc_gyro_pitch_gain;
extern pprz_t	rc_gyro_yaw_gain;

extern float rc_gyro_roll_rate;
extern float rc_gyro_pitch_rate;
extern float rc_gyro_yaw_rate;

extern pprz_t rc_gyro_damped_commands[COMMANDS_NB];

void rc_gyro_init( void );
void rc_gyro_update_rates(float roll_rate,float pitch_rate,float yaw_rate);
void rc_gyro_apply_damping(pprz_t raw_commands[COMMANDS_NB]);

#endif /* RC_GYRO_H */


