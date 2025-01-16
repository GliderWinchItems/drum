#!/bin/bash
# Compile and load program over CAN
# ./cc <CAN ID> <hub-server> <port>
# E.g. ./cc 80000000 localhost 32123
# E.g. ./cc 80000000 192.168.2.50 32125

./mm $1

cd ../../GliderWinchCommons/embed/svn_discoveryf4/PC/sensor/CANldr1/trunk
echo "Start CANldr"
pwd
./CANldr $2 $3 $1 ~/GliderWinchItems/drum/build/drum.xbin
echo $?

cd -

