##\file
## $Id$
##
## Test Makefile for the Optionally Piloted Vehicle Linear Actuator Controller
##
## ***Warning***  Aurora Flight Sciences, Inc. Proprietary Material
##
## Copyright (C) 2010 by Aurora Flight Sciences, Inc. All rights reserved.
##

## Cheat sheet
# $@ - left side of :
# $^ - right side of :

#airframe = SKATE_GENERIC
airframe = SKATE

top_dir      = ../../..
sw_dir       = $(top_dir)/sw
airborne_dir = $(sw_dir)/airborne
arm7_dir     = $(airborne_dir)/arm7
sw_include_dir   = $(sw_dir)/include
var_include_dir  = $(top_dir)/var/$(airframe)
stub_include_dir = stub_include

CLEANFILES = *.gcov *.gcda *.gcno *.xml

CC = g++ -g
CFLAGS = \
	-I$(stub_include_dir) \
	-I$(airborne_dir) \
	-I$(airborne_dir)/unittest \
	-I$(sw_include_dir) \
	-DUBX \
#	-DTESTDEBUGPRINT \

LDFLAGS = 

# ---- Test Suite
airborne_SOURCES = \
	$(airborne_dir)/gps_nmea.c \
	$(airborne_dir)/latlong.c \

airborne_HEADERS = \
	$(airborne_dir)/gps_nmea.h \
	$(airborne_dir)/latlong.h \

airborne_test_SOURCES = \
	tgps_nmea.c \
	testdriver.c \
	unittest.c \
	stubs.c \

airborne_test_HEADERS = \
	tgps_nmea.h \
	unittest.h \
	stubs.h \

OBJECTS = $(airborne_SOURCES:.c=.o) $(airborne_test_SOURCES:.c=.o)

EXECUTABLE = airborne_testsuite

all: $(airborne_test_SOURCES) $(EXECUTABLE)

build: clean all

$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

%.o: %.c %.h
	$(CC) -c -o $@ $< $(CFLAGS)

.PHONY: clean build

clean:
	rm -f $(OBJECTS) *~ $(EXECUTABLE) $(CLEANFILES)
