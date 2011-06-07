/*
 * $Id$
 *
 * Copyright (C) 2011 - Aurora Flight Sciences   jpeverill@aurora.aero
 * 
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
/** \file rc_trim.h
 *  \brief Simple declaration of some trim constants that can be modified remotely
 *
 */

#ifndef RC_TRIM_H
#define RC_TRIM_H

extern int16_t rc_roll_trim;
extern int16_t rc_pitch_trim;
extern int16_t rc_yaw_trim;

#endif // RC_TRIM_H
