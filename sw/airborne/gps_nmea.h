/*
 * gps_nmea.h
 */

#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#include <std.h> // typedefs for inttypes and bool_t

#define NMEA_MAXLEN 255

extern char nmea_msg_buf[NMEA_MAXLEN];
extern int  nmea_msg_len;

/**
 * Fast forward to the next field, modifies current_idx;
 * Return 0 if no next field, 1 otherwise
 * in nmea_msg_buf[].Input argument is the current position in 
 * the nmea_msg_buf[].
 */
uint8_t nextField(uint8_t *current_idx);

/**
 * Attempt to reject empty sentences.
 * Return 1 if sentence contains no data, 0 otherwise.
 */
uint8_t isEmptySentence(uint8_t current_idx);
 
void parse_nmea_GPGSA(void);
void parse_nmea_GPRMC(void);
void parse_nmea_GPGGA(void);
void parse_nmea_GPGSV(void);

/**
 * set of helper query functions
 */
bool_t isGPRMC(void);
bool_t isGPGGA(void);
bool_t isGPGSA(void);
bool_t isGPGSV(void);

#endif /* GPS_NMEA_H */
