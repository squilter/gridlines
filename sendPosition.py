#!/usr/bin/env python
import mavproxy
from subprocess import Popen	
import os
import time
import rospy
import sys
#from uav_msgs.msg import WayPointPost.msg
import droneapi.lib
from pymavlink import mavutil


velocityFreq=20

class fakeLoc():
	def __init__(self,x,y,z):
		self.x = x
		self.y = y
		self.z = z

class UAV():

	def __init__(self):
		self.api = local_connect()


	def initialize(self):
	 	#api = local_connect()
		self.v = self.api.get_vehicles()[0]
		if self.v.mode.name == "INITIALISING":
		    print "Vehicle still booting, try again later"
		    return
		#rospy.Subscriber('way_point_post',WayPointPost,next_cmd)



	def next_cmd(self, a):#format of these coordinates with respect to gps layout not certain, are we assuming drone has established 	 interface so apiconnect is needed?

		#if not a instanceof WayPointPost:
		#	print('bad parameter')
		#	return

		# Use the python gps package to access the laptop GPS
		# gpsd = gps.gps(mode=gps.WATCH_ENABLE)
		cmds = self.v.commands
		#while not self.api.exit:
		 
	
		   # if is_guided and v.mode.name != "GUIDED":
		       # print "User has changed flight modes - aborting follow-me"
		       # break

	 	    #if (gpsd.valid & gps.LATLON_SET) != 0:
		        
	 	dest = droneapi.lib.Location(a.x,a.y,a.z, is_relative=True)
	        print "Going to: %s" % dest
		#cmds.add(dest)
	        cmds.goto(dest)
	        is_guided = True
	        self.v.flush()

	        # Send a new target every ___ seconds
	        
	        time.sleep(.1)

uav=UAV()
uav.initialize()



uav.next_cmd(fakeLoc(100,50,10))
