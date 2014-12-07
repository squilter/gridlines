#!/usr/bin/env python

import mavproxy
from subprocess import Popen
import os
import time
import rospy
import sys
from uav_msgs.msg import OpticalFlowPost
from uav_msgs.msg import GridLinePosPost
from droneapi.lib import *
from pymavlink import mavutil


#ROOT = '/home/sam/Desktop/pixhawk/src/comm'
try:
	ROOT = os.path.dirname(os.path.abspath(__file__))
except:
	ROOT = '/home/sam/Desktop/pixhawk/src/comm'

date = str(time.time()) #so we make a unique log each run
date = date[:-3] #substringing the last two decimal places and decimal off

#os.system(ROOT+'/runMavproxy.sh '+date+' '+ROOT) #passing in the unique label as input, and the root
#myFile = ROOT+'/logs/quadlog'+date+'_decode.txt'

api = local_connect()
vehicles = api.get_vehicles()
v = vehicles[0]

print v.location
print v.attitude
print v.airspeed
print v.groundspeed


sys.exit()

#print "Preparing to parse "+myFile
#with open(myFile) as f:
#    content = f.read().splitlines()

#for line in content:
#    if "LAND_SPEED" in line:
        #print line
#	myDate = line[:]
#	param_values = line.split('{')[1]
#	params = param_values.split(',')
#	value = params[1]
#	velocity = value.split(':')[1]
#	print "velocity: "+str(velocity)
#	heading = 10	
#	heading = line[20:25]
#	altSpeed = line[30:40]

optFlowFrequency = 20
opticalFlowPub = rospy.Publisher('of_meas_post', OpticalFlowPost)
rospy.init_node('simulator', anonymous=True)
rate_velocity = rospy.Rate(optFlowFrequency)


omsg = OpticalFlowPost()
omsg.x_vel=velocity*Math.cos(heading)
omsg.y_vel=velocity*Math.sin(heading)
omsg.z_vel=altSpeed

#rospy.loginfo(omsg)
#optPub.publish(omsg)



#this loop executes after leaving the above loop and is for cleanup on exit
#for (m,pm) in mpstate.modules:
#if hasattr(m, 'unload'):
#    print("Unloading module %s" % m.name)
#    m.unload()

#sys.exit(1)


