/** \file
    Main Unit Test Driver Program
    $Id$
    ***Warning*** Aurora Flight Sciences, Inc. Proprietary Material
    Author: Name
    Copyright (C) 2010 by Aurora Flight Sciences, Inc. All Rights Reserved.
*/

#include "unittest.h"

#include "tgps_nmea.h"

int main()
{
  int retcode = 0;
  
  struct testcase testcases[] = {
    TESTCASE(tgps_nmea_nextField),
    TESTCASE(tgps_nmea_checksum),
    TESTCASE(tgps_nmea_GPGSA),
    TESTCASE(tgps_nmea_GPRMC),
    TESTCASE(tgps_nmea_GPGGA),
    TESTCASE(tgps_nmea_GPGSV),
  };
  
  /* initialize things */
  
  /* run test suite on the airborne code */
  retcode = runSuite((char*)"airborne_test_results.xml",
		     (char*)"airborne",
		     testcases,
		     sizeof(testcases)/sizeof(testcases[0]));
  
  return (retcode == 0) ? 0 : -1;
}
