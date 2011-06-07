/*
 * stubs.h
 */

#ifndef STUBS_H
#define STUBS_H

#include <std.h>

extern uint8_t  nav_utm_zone0;
extern uint8_t  gps_mode;
extern uint16_t gps_gspeed;
extern int16_t  gps_course;
extern uint32_t gps_itow;
extern int32_t  gps_lat;
extern int32_t  gps_lon;
extern bool_t   gps_pos_available;
extern int32_t  gps_utm_east;
extern int32_t  gps_utm_north;
extern uint8_t  gps_utm_zone;
extern uint8_t  gps_numSV;
extern int32_t  gps_alt;

#endif /* STUBS_H */
