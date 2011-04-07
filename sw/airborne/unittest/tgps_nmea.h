/** \file
    Test the crc module
    $Id$
    ***Warning*** Aurora Flight Sciences, Inc. Proprietary Material
    Author: Max Chtangeev
    Copyright (C) 2010 by Aurora Flight Sciences, Inc. All Rights Reserved.
*/

#ifndef TGPS_NMEA_H
#define TGPS_NMEA_H

int tgps_nmea_nextField(void);
int tgps_nmea_checksum(void);
int tgps_nmea_isEmptySentence(void);
int tgps_nmea_GPGSA(void);
int tgps_nmea_GPRMC(void);
int tgps_nmea_GPGGA(void);
int tgps_nmea_GPGSV(void);

#endif /* TGPS_NMEA_H */
