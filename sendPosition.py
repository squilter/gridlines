#!/usr/bin/env python

import mavproxy
from subprocess import Popen
import os
import time
import rospy
import sys
from uav_msgs.msg import WayPointPost.msg
from droneapi.lib import *
from pymavlink import mavutil


velocityFreq=20

class UAV():

	def __init__(self):
		self.api = local_connect()


	def initialze(self):
	 	#api = local_connect()
		v = api.get_vehicles()[0]
		if v.mode.name == "INITIALISING":
		    print "Vehicle still booting, try again later"
		    return
		rospy.Subscriber('way_point_post',WayPointPost,next_cmd)



	def next_cmd(self, a):#format of these coordinates with respect to gps layout not certain, are we assuming drone has established 	 interface so apiconnect is needed?

		if not a instanceof WayPointPost:
			print('bad parameter')
			return

		# Use the python gps package to access the laptop GPS
		# gpsd = gps.gps(mode=gps.WATCH_ENABLE)
		cmds = v.commands
		while not self.api.exit:
		 
	
		   # if is_guided and v.mode.name != "GUIDED":
		       # print "User has changed flight modes - aborting follow-me"
		       # break

	 	    #if (gpsd.valid & gps.LATLON_SET) != 0:
		        
		 	dest = Location(a.x,a.y,a.z, is_relative=True)
		        print "Going to: %s" % dest
			cmd.add(dest)
		        cmds.goto(dest)
		        is_guided = True
		        v.flush()

		        # Send a new target every ___ seconds
		        
		        time.sleep(.1)

uav=UAV()
uav.initialize()
