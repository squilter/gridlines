#!/bin/bash
invalue="$1"
rootvalue="$2"
if [[ "$invalue" = "" ]]; then
	invalue="badvalue"
	# print "$invalue"
fi
if [[ "$rootvalue" = "" ]]; then
	rootvalue="badroot"
	# print "$invalue"
fi

quadlog='/logs/quadlog'
quadfull=$rootvalue$quadlog
dottxt='.txt'
decode='_decode'

myLog=$quadfull$invalue$dottxt
myLogDecode=$quadfull$invalue$decode$dottxt

#echo $myLog
mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600 --logfile=$myLog --cmd='status GPS_RAW_INT;exit'
mavlogdump.py $myLog > $myLogDecode

echo 'Got to the end of the shell script'
exit 0
