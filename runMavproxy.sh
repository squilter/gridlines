#!/bin/bash
invalue="$1"
if [[ "$invalue" = "" ]]; then
	invalue="badvalue"
	# print "$invalue"
fi

quadlog='../comm/logs/quadlog'
dottxt='.txt'
decode='_decode'

myLog=$quadlog$invalue$dottxt
myLogDecode=$quadlog$invalue$decode$dottxt

#echo $myLog
mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600 --logfile=$myLog --cmd='status GPS_RAW_INT;exit'
mavlogdump.py $myLog > $myLogDecode

echo 'Got to the end of the shell script'
exit 0
