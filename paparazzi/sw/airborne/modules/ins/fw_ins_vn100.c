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
 * \brief driver for the VectorNav VN100 (Fixed-Wing part)
 */

#include "modules/ins/ins_vn100.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

void ins_init( void ) {
#ifndef SITL
  /* SPI polarity = 1 - data sampled on rising edge */
  SpiSetCPOL();
  /* SPI phase = 1 - SCK idle high */
  SpiSetCPHA();

  ins_ador = VN100_ADOR;
  ins_adof = VN100_ADOF;
  ins_baud = VN100_BAUD;

  ins_init_status = INS_VN100_SET_BAUD;
#endif
}

static inline bool_t ins_configure( void ) {
  #ifndef SITL
  switch (ins_init_status) {
    case INS_VN100_SET_BAUD :
      last_send_packet.RegID = VN100_REG_SBAUD;
      spi_buffer_length = 4+VN100_REG_SBAUD_SIZE;
      ins_init_status++;
      break;
    case INS_VN100_SET_ADOR :
      last_send_packet.RegID = VN100_REG_ADOR;
      spi_buffer_length = 4+VN100_REG_ADOR_SIZE;
      ins_init_status++;
      break;
    case INS_VN100_SET_ADOF :
      last_send_packet.RegID = VN100_REG_ADOF;
      spi_buffer_length = 4+VN100_REG_ADOF_SIZE;
      ins_init_status++;
      break;
    case INS_VN100_SET_MAG_REF :
      last_send_packet.RegID = VN100_REG_REF;
      last_send_packet.Data[0].Float=23462.5;
      last_send_packet.Data[1].Float=5611.9;
      last_send_packet.Data[2].Float=41526.8;

//      last_send_packet.Data[0].Float=1.92487;   //Devens
//      last_send_packet.Data[1].Float=-0.51819;  //Devens
//      last_send_packet.Data[2].Float=4.88002;   //Devens
//      last_send_packet.Data[0].Float=2.47652;     //Florida
//      last_send_packet.Data[1].Float=-0.2390;    //Florida
//      last_send_packet.Data[2].Float=3.86421;     //Florida

      last_send_packet.Data[3].Float=0;
      last_send_packet.Data[4].Float=0;
      last_send_packet.Data[5].Float=-9.793746;
      spi_buffer_length = 4+VN100_REG_REF_SIZE;
      ins_init_status++;
      break;

    case INS_VN100_READY :
      return TRUE;
  }
  last_send_packet.CmdID = VN100_CmdID_WriteRegister;
  spi_buffer_input = (uint8_t*)&last_received_packet;
  spi_buffer_output = (uint8_t*)&last_send_packet;
  SpiSelectSlave0();
  SpiStart();
  while (!SpiCheckAvailable()) {
    SpiOverRun();
  }
  #endif
  return FALSE;
}

void ins_periodic_task( void ) {
  #ifndef SITL
  if (!SpiCheckAvailable()) {
    SpiOverRun();
    return;
  }

  if (!ins_configure()) return;

  // Fill request for QMR
  if (ins_tare)
  {
    last_send_packet.CmdID = VN100_CmdID_Tare;
    last_send_packet.RegID = 0x00;
    spi_buffer_length = 4+4;
    last_send_packet.Data[0].byte[0]=0;
    last_send_packet.Data[0].byte[0]=1;
    last_send_packet.Data[0].byte[0]=2;
    last_send_packet.Data[0].byte[0]=3;
    ins_tare = 0;
  }
  else
  if (ins_save)
  {
    last_send_packet.CmdID = VN100_CmdID_WriteSettings;
    last_send_packet.RegID = 0x00;
    spi_buffer_length = 4+4;
    last_send_packet.Data[0].byte[0]=0;
    last_send_packet.Data[0].byte[0]=0;
    last_send_packet.Data[0].byte[0]=0;
    last_send_packet.Data[0].byte[0]=0;
    ins_save = 0;
  }

  else
  {
  last_send_packet.CmdID = VN100_CmdID_ReadRegister;
  last_send_packet.RegID = VN100_REG_YMR;
  spi_buffer_length = 4+VN100_REG_YMR_SIZE;
  }
  // Fill request for REF
/*
  last_send_packet.CmdID = VN100_CmdID_ReadRegister;
  last_send_packet.RegID = VN100_REG_REF;
  spi_buffer_length = 4+VN100_REG_REF_SIZE;*/


  spi_buffer_input = (uint8_t*)&last_received_packet;
  spi_buffer_output = (uint8_t*)&last_send_packet;

  SpiSelectSlave0();
  SpiStart();
  while (!SpiCheckAvailable()) {
    SpiOverRun();
  }
  #endif
}


void ins_event_task( void ) {
  #ifndef SITL
  if (spi_message_received) {
    spi_message_received = FALSE;
    parse_ins_msg();
    //uint8_t s = 4+VN100_REG_QMR_SIZE;
    //DOWNLINK_SEND_DEBUG(DefaultChannel,s,spi_buffer_input);
  }
  #endif
}

