#!/bin/bash
cd /home/skate/skate/
source ./scripts/aliases.txt
export PAPARAZZI_HOME=/home/skate/skate/
export PAPARAZZI_SRC=/home/skate/skate/
#sudo ifconfig eth1 192.168.0.2/24 up
#skifup
#setpap
./paparazzi -session "Skate"
