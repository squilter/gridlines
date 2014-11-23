#!/usr/bin/env python

import mavproxy
from subprocess import Popen
import os
import time
import rospy
from uav_msgs.msg import OpticalFlowPost
from uav_msgs.msg import GridLinePosPost
# probably need to follow http://wiki.ros.org/ROSNodeTutorialPython to get these msgs to work

#geo_reference.py will be useful too

## global mavproxy state
#mpstate = mavproxy.MPState()
#mpstate.status.exit = False
#mpstate.command_map = command_map
#mpstate.continue_mode = False
#mavproxy.process_stdin('status GPS_RAW_INT')


date = str(time.time()) #so we make a unique log each run
date = date[:-2] #substringing the last two decimal places off

os.system('/home/sam/Desktop/pixhawk2/test.sh '+date) #passing in the unique label as input

#myFile = '/home/sam/Desktop/pixhawk/logs/quadlog'+date+'_decode.txt'
myFile = '/home/sam/Desktop/pixhawk2/logs/quadlog_decode.txt'

print "Preparing to parse "+myFile
with open(myFile) as f:
    content = f.read().splitlines()

for line in content:
    if "LAND_SPEED" in line:
        #print line
	myDate = line[:]
	param_values = line.split('{')[1]
	params = param_values.split(',')
	value = params[1]
	velocity = value.split(':')[1]
	print "velocity: "+str(velocity)
	heading = 10

	
#	heading = line[20:25]
#	altSpeed = line[30:40]

optFlowFrequency = 20
opticalFlowPub = rospy.Publisher('of_meas_post', OpticalFlowPost)
rospy.init_node('simulator', anonymous=True)
rate_velocity = rospy.Rate(optFlowFrequency)


omsg = OpticalFlowPost()
omsg.x_vel=velocity*cos(heading)
omsg.y_vel=velocity*sin(heading)
omsg.z_vel=altSpeed

#rospy.loginfo(omsg)
#optPub.publish(omsg)



#this loop executes after leaving the above loop and is for cleanup on exit
#for (m,pm) in mpstate.modules:
#if hasattr(m, 'unload'):
#    print("Unloading module %s" % m.name)
#    m.unload()

#sys.exit(1)


