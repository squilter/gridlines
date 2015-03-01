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
import math


velocityFreq=20

class fakeLoc():
	def __init__(self,x,y,z):
		self.x = x
		self.y = y
		self.z = z

class UAV():

	def __init__(self,current_altitude):
		self.api = local_connect()
		self.v = None
		self.current_altitude=current_altitude

	def initialize(self):
	 	#api = local_connect()
		self.v = self.api.get_vehicles()[0]
		if self.v.mode.name == "INITIALISING":
		    print "Vehicle still booting, try again later"
		    return
		rospy.Subscriber('simple_uav_cmd',SimpleUavCmd,next_cmd)

	def convert_to_gps(x,y):
		theta=math.arctan(y/x)
		r=math.sqrt(x*x+y*y)
		#twist so angle is relative to front of camera frame
		if self.v.attitude.yaw+theta >= math.pi:
			dif=theta-math.pi-self.v.attitude
			yaw=-math.pi+dif
		
		elif:
			yaw=self.attitude.yaw+theta <= -math.pi
			dif=theta+math.pi+self.v.attitude.yaw
			yaw=math.pi-dif

		lat = self.v.location.lat + (r*math.sin(theta+yaw)/6378137)
 		lon = self.v.location.lon + (r*math.cos(theta+yaw)/6378137)/math.cos(lat)
 		self.lat_go_to=lat
 		self.lon_go_to=lon

	def next_cmd(self, a):#format of these coordinates with respect to gps layout not certain, are we assuming drone has established 	 interface so apiconnect is needed?
		#if not a instanceof WayPointPost:
		#	print('bad parameter')
		#	return
		# Use the python gps package to access the laptop GPS
		# gpsd = gps.gps(mode=gps.WATCH_ENABLE)
		convert_to_gps(a.x,a.y)
		cmds = self.v.commands
		#while not self.api.exit:
		   # if is_guided and v.mode.name != "GUIDED":
		       # print "User has changed flight modes - aborting follow-me"
		       # break
	 	    #if (gpsd.valid & gps.LATLON_SET) != 0:
	 	dest = droneapi.lib.Location(self.lat_go_to,self.lon_go_to,self.current_altitude, is_relative=False)
	    print "Going to: %s" % dest
		#cmds.add(dest)
        cmds.goto(dest)
        is_guided = True
        self.v.flush()
        # Send a new target every ___ seconds
 #        time.sleep(.1)

	#def testAttitude(self):
		#if (self.v != None):
			#while(True):
				#time.sleep(.1)
				#print self.v.attitude.yaw;

uav=UAV(2)
uav.initialize()
#uav.testAttitude()


# uav.next_cmd(fakeLoc(100,50,10))
