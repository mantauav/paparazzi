/*
 * Paparazzi $Id: ins_xsens.c 3872 2009-08-05 14:42:41Z mmm $
 *
 * Copyright (C) 2010 ENAC
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

/**
 * \brief driver for the VectorNav VN100
 */

#include "modules/ins/ins_vn100.h"

#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "led.h"

#include "downlink.h"
#include "messages.h"

#include "mcu_periph/uart.h"

#include "gps.h"
#include "autopilot.h"
#include "estimator.h"

#ifdef SITL
float baro_MS5803_last_altitude=0.0;
#include <inttypes.h>
#include "subsystems/sensors/infrared.h"
#include "generated/airframe.h"

#include <caml/mlvalues.h>

float sim_air_speed;

void ir_gain_calib(void) {
}

value set_ir(value roll __attribute__ ((unused)),
	     value front __attribute__ ((unused)),
             value top __attribute__ ((unused)),
	     value air_speed
	     ) {
  // INFRARED_TELEMETRY : Stupid hack to use with modules
/*
#if defined  USE_INFRARED || USE_INFRARED_TELEMETRY
  infrared.roll = Int_val(roll);
  infrared.pitch = Int_val(front);
  infrared.top = Int_val(top);
#endif
  sim_air_speed = Double_val(air_speed);*/
  return Val_unit;
}

/** Required by infrared.c:ir_init() */
void adc_buf_channel(void* a __attribute__ ((unused)),
		     void* b __attribute__ ((unused)),
		     void* c __attribute__ ((unused))) {
}
#endif
float ins_roll_neutral=0.0;
float ins_pitch_neutral=0.0;

bool_t ins_tare=0;

struct FloatEulers ins_eulers;
struct FloatQuat ins_quat;
struct FloatRates ins_rates;
struct FloatRMat ins_rmat;
struct FloatVect3 ins_accel;
struct FloatVect3 ins_mag;
struct FloatVect3 ins_ref_mag; //Magnetic reference vector for local field.
struct FloatVect3 ins_ref_grav; //Gravity reference vector for local field.

 float  ins_ref_mag_declination=-15.4; //Devens
//float  ins_ref_mag_declination=-5.166667; //Florida


struct FloatVect3 df1,df2;

volatile uint8_t ins_msg_received;

/* last received SPI packet */
VN100_Res_Packet last_received_packet;
/* last send packet */
VN100_Req_Packet last_send_packet;

/* output mode */
uint32_t ins_ador;
uint32_t ins_adof;
uint32_t ins_baud;

uint8_t ins_init_status;

/* ins_init and ins_periodic_task to be implemented according to the airframe type : FW or BOOZ */

void parse_ins_msg( void ) {
  #ifndef SITL
  if (last_received_packet.ErrID != VN100_Error_None) {
    //TODO send error
    return;
  }

  // parse message (will work only with read and write register)
  switch (last_received_packet.RegID) {
    case VN100_REG_ADOR :
      ins_ador = last_received_packet.Data[0].UInt;
      break;
    case VN100_REG_ADOF :
      ins_adof = last_received_packet.Data[0].UInt;
      break;
    case VN100_REG_SBAUD :
      ins_baud = last_received_packet.Data[0].UInt;
      break;
    case VN100_REG_REF :
      ins_ref_mag.x = last_received_packet.Data[0].Float;
      ins_ref_mag.y = last_received_packet.Data[1].Float;
      ins_ref_mag.z = last_received_packet.Data[2].Float;
      ins_ref_grav.x = last_received_packet.Data[3].Float;
      ins_ref_grav.y = last_received_packet.Data[4].Float;
      ins_ref_grav.z = last_received_packet.Data[5].Float;    
      break;

    case VN100_REG_YPR :
      ins_eulers.phi   = RadOfDeg(last_received_packet.Data[2].Float);
      ins_eulers.theta = RadOfDeg(last_received_packet.Data[1].Float);
      ins_eulers.psi   = RadOfDeg(last_received_packet.Data[0].Float);
      break;
    case VN100_REG_QTN :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      FLOAT_EULERS_OF_QUAT(ins_eulers, ins_quat);
      break;
    case VN100_REG_QTM :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      FLOAT_EULERS_OF_QUAT(ins_eulers, ins_quat);
      ins_mag.x = last_received_packet.Data[4].Float;
      ins_mag.y = last_received_packet.Data[5].Float;
      ins_mag.z = last_received_packet.Data[6].Float;
      break;
    case VN100_REG_QTA :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      FLOAT_EULERS_OF_QUAT(ins_eulers, ins_quat);
      ins_accel.x = last_received_packet.Data[4].Float;
      ins_accel.y = last_received_packet.Data[5].Float;
      ins_accel.z = last_received_packet.Data[6].Float;
      break;
    case VN100_REG_QTR :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      FLOAT_EULERS_OF_QUAT(ins_eulers, ins_quat);
      ins_rates.p = last_received_packet.Data[4].Float;
      ins_rates.q = last_received_packet.Data[5].Float;
      ins_rates.r = last_received_packet.Data[6].Float;
      break;
    case VN100_REG_QMA :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      FLOAT_EULERS_OF_QUAT(ins_eulers, ins_quat);
      ins_mag.x = last_received_packet.Data[4].Float;
      ins_mag.y = last_received_packet.Data[5].Float;
      ins_mag.z = last_received_packet.Data[6].Float;
      ins_accel.x = last_received_packet.Data[7].Float;
      ins_accel.y = last_received_packet.Data[8].Float;
      ins_accel.z = last_received_packet.Data[9].Float;
      break;
    case VN100_REG_QAR :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      FLOAT_EULERS_OF_QUAT(ins_eulers, ins_quat);
      ins_accel.x = last_received_packet.Data[4].Float;
      ins_accel.y = last_received_packet.Data[5].Float;
      ins_accel.z = last_received_packet.Data[6].Float;
      ins_rates.p = last_received_packet.Data[7].Float;
      ins_rates.q = last_received_packet.Data[8].Float;
      ins_rates.r = last_received_packet.Data[9].Float;
      break;
    case VN100_REG_QMR :
      ins_quat.qi = last_received_packet.Data[0].Float;
      ins_quat.qx = last_received_packet.Data[1].Float;
      ins_quat.qy = last_received_packet.Data[2].Float;
      ins_quat.qz = last_received_packet.Data[3].Float;
      FLOAT_EULERS_OF_QUAT(ins_eulers, ins_quat);
      ins_mag.x = last_received_packet.Data[4].Float;
      ins_mag.y = last_received_packet.Data[5].Float;
      ins_mag.z = last_received_packet.Data[6].Float;
      ins_accel.x = last_received_packet.Data[7].Float;
      ins_accel.y = last_received_packet.Data[8].Float;
      ins_accel.z = last_received_packet.Data[9].Float;
      ins_rates.p = last_received_packet.Data[10].Float;
      ins_rates.q = last_received_packet.Data[11].Float;
      ins_rates.r = last_received_packet.Data[12].Float;
      break;
    case VN100_REG_YMR :
      ins_eulers.phi   = RadOfDeg(last_received_packet.Data[2].Float);
      ins_eulers.theta = RadOfDeg(last_received_packet.Data[1].Float);
      ins_eulers.psi   = RadOfDeg(last_received_packet.Data[0].Float+ins_ref_mag_declination);
      if (ins_eulers.psi < 0.)
        ins_eulers.psi += 2 * M_PI;
      ins_mag.x = last_received_packet.Data[3].Float;
      ins_mag.y = last_received_packet.Data[4].Float;
      ins_mag.z = last_received_packet.Data[5].Float;
      ins_accel.x = last_received_packet.Data[6].Float;
      ins_accel.y = last_received_packet.Data[7].Float;
      ins_accel.z = last_received_packet.Data[8].Float;
      ins_rates.p = last_received_packet.Data[9].Float;
      ins_rates.q = last_received_packet.Data[10].Float;
      ins_rates.r = last_received_packet.Data[11].Float;
      //EstimatorSetAtt(ins_eulers.phi-ins_roll_neutral,ins_eulers.psi,ins_eulers.theta-ins_pitch_neutral);
      if (estimator_heading_source == 0)
      {
        EstimatorSetPhiTheta(ins_eulers.phi-ins_roll_neutral,ins_eulers.theta-ins_pitch_neutral);
      }
      else
      {
        EstimatorSetAtt(ins_eulers.phi-ins_roll_neutral,ins_eulers.psi,ins_eulers.theta-ins_pitch_neutral);
      }
      float cd=(float)gps_course*0.1;
      df1.x=DegOfRad(ins_eulers.psi)-cd;
      if (df1.x < 0)
        df1.x += 360;
      #ifdef USE_RC_GYRO
      rc_gyro_update_rates(ins_rates.p,ins_rates.q,ins_rates.r);
      #endif      

      break;
  }
  #endif
}



extern void ins_report_task( void ) {
/*  DOWNLINK_SEND_AHRS_LKF(DefaultChannel,
      &ins_eulers.phi, &ins_eulers.theta, &ins_eulers.psi,
      &ins_quat.qi, &ins_quat.qx, &ins_quat.qy, &ins_quat.qz,
      &ins_rates.p, &ins_rates.q, &ins_rates.r,
      &ins_accel.x, &ins_accel.y, &ins_accel.z,
      &ins_mag.x, &ins_mag.y, &ins_mag.z);*/
}
