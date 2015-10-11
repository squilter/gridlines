chmod 777 getVelocity.py
new terminal tab -> roscore
rosrun comm getVelocity.py


mavplayback.py mav.tlog
mavproxy.py --master=127.0.0.1:14550 --baudrate=57600 --logfile=/home/sam/Desktop/pixhawk/src/comm/logs/quadlogNEW.tlog --cmd='status GPS_RAW_INT'

