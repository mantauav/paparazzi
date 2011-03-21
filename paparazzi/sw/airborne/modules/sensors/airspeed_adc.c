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

#include "modules/sensors/airspeed_adc.h"
#include "mcu_periph/adc.h"
#include BOARD_CONFIG
#include "generated/airframe.h"
#include "estimator.h"

uint16_t adc_airspeed_val;

#ifndef SITL // Use ADC if not in simulation

#ifndef ADC_CHANNEL_AIRSPEED
#error "ADC_CHANNEL_AIRSPEED needs to be defined to use airspeed_adc module"
#endif

#ifndef ADC_CHANNEL_AIRSPEED_NB_SAMPLES
#define ADC_CHANNEL_AIRSPEED_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

struct adc_buf buf_airspeed;

#endif

void airspeed_adc_init( void ) {
#ifndef SITL
  adc_buf_channel(ADC_CHANNEL_AIRSPEED, &buf_airspeed, ADC_CHANNEL_AIRSPEED_NB_SAMPLES);
#endif
}

void airspeed_adc_update( void ) {
  float airspeed;
#ifndef SITL
  adc_airspeed_val = (buf_airspeed.sum / buf_airspeed.av_nb_sample + AIRSPEED_ADC_OFFSET);
#ifdef AIRSPEED_QUADRATIC_SCALE
  airspeed = (adc_airspeed_val - AIRSPEED_BIAS);
  if (airspeed <= 0.0f)
    airspeed = 0.0f;
  airspeed = sqrtf(airspeed) * AIRSPEED_QUADRATIC_SCALE;
#else 
#ifdef AIRSPEED_ADC_DELTAPRESSURE
  float deltapressure=(float)adc_airspeed_val*AIRSPEED_ADC_SCALE; //convert the adc value into millibars
  if (deltapressure < 0.0)
    deltapressure=0;
  airspeed=dp2cas(deltapressure,1013.25);
  airspeed = AIRSPEED_SCALE * (airspeed - AIRSPEED_BIAS);         //correct the airspeed with constants wind tunnel. should be verified.
#else
  airspeed = AIRSPEED_SCALE * (adc_airspeed_val - AIRSPEED_BIAS);
#endif
#endif
  EstimatorSetAirspeed(airspeed);
#else // SITL
  extern float sim_air_speed;
  EstimatorSetAirspeed(sim_air_speed);
  adc_airspeed_val = 0;
#endif //SITL
}

//Function to calculate airspeed from pressure differential. Units are millibars and meters/second
float dp2cas(float dp, float Pref)
{	
  float a_sl = 343; // Speed of sound dry air at 20C (m/s)
  float speed = a_sl*sqrt( 5*(pow((dp/Pref + 1),(2./7.)) - 1.) );
  return speed;
}

