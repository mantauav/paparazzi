#!/bin/bash
cd /home/skate/skate/paparazzi
source ../scripts/aliases.txt
export PAPARAZZI_HOME=/home/skate/skate/paparazzi
export PAPARAZZI_SRC=/home/skate/skate/paparazzi
#sudo ifconfig eth1 192.168.0.2/24 up
#skifup
#setpap
./paparazzi -session "Skate"
