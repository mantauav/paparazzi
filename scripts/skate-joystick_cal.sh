js#!/bin/bash
# Calibrate skate mini and 4-axis joystick.
# Script works for both as jscal only applys calibration for joysticks with correct number of axes
# mini joystick has 5 axes, 4-axis mapped as a 6 axis. . 

# BU Joystick support
#read joystick serial number
SERIAL=`lsusb -v | grep -A 50 16c0:05ba | grep iSerial | sed 's/.*B//g'`

case $SERIAL in
	"14555" )
		echo "calibrating $SERIAL"
	jscal -s 5,1,0,2026,2026,1073709,1012933,1,0,2064,2064,-1056800,-1052656,1,0,2049,2049,-263422,-262649,1,0,0,0,-2147483648,-2147483648,1,0,0,0,-2147483648,-2147483648 /dev/input/au_js0
        ;;
	"14552" )
		echo "calibrating $SERIAL"
		jscal -s 5,1,0,2138,2138,-1088954,-1034402,1,0,2061,2061,-1127846,-1034402,1,0,2272,2272,236813,295625,1,0,0,0,-2147483648,-2147483648,1,0,0,0,-2147483648,-2147483648 /dev/input/au_js0
	;;
	"14549" )
		echo "calibrating $SERIAL"
		jscal -s 6,1,0,2041,2041,1127846,1095621,1,0,2056,2056,-1030431,-1001594,1,0,2023,2023,1662088,1080190,1,0,2027,2027,1130220,1095621,1,0,0,0,-2147483648,-2147483648,1,0,0,0,-2147483648,-2147483648 /dev/input/au_js0
	;;
	"14553" )
		echo "calibrating $SERIAL"
		jscal -s 6,1,0,2033,2033,1120782,1095621,1,0,2022,2022,-1018699,-1063078,1,0,1975,1975,1034402,969051,1,0,2047,2047,1048544,992337,1,0,0,0,-2147483648,-2147483648,1,0,0,0,-2147483648,-2147483648 /dev/input/au_js0
	;;	
	* )
		echo "generic calibration"
		jscal -s 6,1,0,2033,2033,1120782,1095621,1,0,2022,2022,-1018699,-1063078,1,0,1975,1975,1034402,969051,1,0,2047,2047,1048544,992337,1,0,0,0,-2147483648,-2147483648,1,0,0,0,-2147483648,-2147483648 /dev/input/au_js0
	;;
esac
	
#old calibrations
#4axis
#jscal -s 6,1,0,2033,2033,1120782,1095621,1,0,2022,2022,-1018699,-1063078,1,0,1975,1975,1034402,969051,1,0,2047,2047,1048544,992337,1,0,0,0,-2147483648,-2147483648,1,0,0,0,-2147483648,-2147483648 /dev/input/au_js0

#Mini
#jscal -s 5,1,0,2026,2026,1071566,1005346,1,0,2058,2058,-1030431,-1056800,1,0,2044,2044,-262906,-263422,1,0,0,0,-2147483648,-2147483648,1,0,0,0,-2147483648,-2147483648 /dev/input/au_js0

