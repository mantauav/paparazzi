
# Copyright 2011 Aurora Flight Sciences 
#  James Peverill   jpeverill@aurora.aero

#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.  
# 

# Quiet compilation
Q=@

all: bob4_osd au_osd_setup

bob4_osd: bob4_osd.c
	gcc -o au_osd bob4_osd.c -I /usr/include/Ivy -livy -lm

au_osd_setup: bob4_setup.c
	gcc -o au_osd_setup bob4_setup.c -lm

clean:
	rm -f *~ *.o *.bak .depend au_osd au_osd_setup