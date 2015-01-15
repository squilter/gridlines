#!/usr/bin/env python

import mavproxy
from subprocess import Popen
import os
import time
import rospy
import sys
import inspect
currentDir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentDir = os.path.dirname(currentDir)
sys.path.insert(0,parentDir)

#from uav_msgs.msg import OpticalFlowPost
#from uav_msgs.msg import UavEstState
#from droneapi.lib import *

import droneapi.lib
from pymavlink import mavutil


#ROOT = '/home/sam/Desktop/pixhawk/src/comm'
#try:
#	ROOT = os.path.dirname(os.path.abspath(__file__))
#except:
#	ROOT = '/home/sam/Desktop/pixhawk/src/comm'
#os.system(ROOT+'/runMavproxy.sh '+date+' '+ROOT) #passing in the unique label as input, and the root
#myFile = ROOT+'/logs/quadlog'+date+'_decode.txt'
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
#date = str(time.time()) #so we make a unique log each run
#date = date[:-3] #substringing the last two decimal places and decimal off

velocityFrequency = 20
api = local_connect()
vehicles = api.get_vehicles()
v = vehicles[0]
def getVelocity():
	print 'Initializing getVelocity....'
	

	rospy.init_node('testing123') #to start time	

	print str(v.location)	
	print v.attitude
	print 'airspeed: '+str(v.airspeed)
	print 'groundspeed: '+str(v.groundspeed)

	#rate2 = float(rospy.get_param('~rate','1.0'))
	#topic = rospy.get_param('~topic','of_meas_post')
	#rospy.loginfo('rate = %d',rate2)
	#rospy.loginfo('topic = %s',topic)
	#opticalFlowPub = rospy.Publisher('of_meas_post', OpticalFlowPost)

	#while not api.exit: 
	for x in range(10):
		vx_vy_vz = v.velocity
		attitude = v.attitude
		#time = rospy.now() todo
		#quality = 155 #has to be in range [0 255]

		print "velocity: "+str(vx_vy_vz)
		print "attitude: "+str(attitude)
		time.sleep(.5)
	#	omsg = OpticalFlowPost()
	#	omsg.x_vel = vx_vy_vz[0]
	#	omsg.y_vel = vx_vy_vz[1]
		#omsg.z_vel = vx_vy_vz[2]
		#omsg.quality = quality
		#omsg.uncertainty = 1 - (quality/255)
	#	opticalFlowPub.publish(omsg)

		#rate1.sleep()
	print getParameters().__getitem__('INS_ACC2OFFS_X')

	setOrigin(8,8,7)
	#sys.exit()



	#optFlowFrequency = 20
	#rospy.init_node('simulator', anonymous=True)
	#rate_velocity = rospy.Rate(optFlowFrequency)

	#this loop executes after leaving the above loop and is for cleanup on exit
	#for (m,pm) in mpstate.modules:
	#if hasattr(m, 'unload'):
	#    print("Unloading module %s" % m.name)
	#    m.unload()

	#sys.exit(1)

def setPosition(x,y,z,yaw="none"):
	#https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/pixhawk.xml

	#construct a message to set a new offset from the currently controlled position
	if yaw=="none":
		msg = v.message_factory.set_position_control_offset_encode(x,y,z)
	else:
		msg = v.message_factory.set_position_control_offset_encode(x,y,z,yaw)
	v.send_mavlink(msg)
	#Bitmask to indicate which dimensions should be ignored by the vehicle: 
	#a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. 
	#If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. 
	#Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, 
	#bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
	msg2 = v.message_factory.position_target_local_ned_encode(x,y,z,vx,vy,vz,afx,afy,afz,yaw,yaw_rate)
	response2 = v.send_mavlink(msg2)

def setAttitude(roll,pitch,yaw):
	msg = v.message_factory.attitude_control_encode(roll,pitch,yaw)
	response = v.send_mavlink(msg)
	msg2 = v.message_factory.attitude_target_encode(roll,pitch,yaw)
	response2 = v.send_mavlink(msg2)

def getParameters():
	return v.parameters
	
def getIMU():
	msg = v.message_factory.scaled_imu_encode()#takes 11 args 
	#timestamp (millisec since system boot)
	# xacc (mg), yacc (mg), zacc (mg)
	# w_x (mrad/sec), w_y, w_z
	# magneticfield_x, B_y, B_z (milli tesla)
	response = v.send_mavlink(msg)	

def getLocalPos():
	msg = v.message_factory.local_position_ned_encode()
	# filtered local position based off accelerometers.
	#right handed coord frame, z-axis down. north-east-down convention
	# x y z vx vy vz
	msg2 = v.message_factory.local_position_ned_cov_encode()
	# plus covariance	
	response = v.send_mavlink(msg)	
	response2 = v.send_mavlink(msg2)

def setOrigin(mylat,mylong,myalt):
	sysID = 0 #target system and target componenet can always be set to 0
	msg = v.message_factory.set_gps_global_origin_encode(sysID,mylat,mylong,myalt)
	print msg
	#sets the global mission (0,0,0) point. mavlink supports indoor navigation
	response = v.send_mavlink(msg)
	v.set_mavlink_callback(lambda x: x )
	response = v.flush()
	print 'Response: '+str(response)

def getOpticalFlow():
	msg = v.message_factory.optical_flow_rad_encode()
	#integration time, integrated_x, integrated)y, integrated_xgyro, ygyro, zgryo,temp, quality, time_delta_distance,distance to center of flow field
	response = v.send_mavlink(msg)
	v.flush()

def injectGPS():
	raw_data = [110] #i think max length is 110? enough for 12 satellites
	msg = v.message_factory.gps_inject_data_encode(raw_data.length,raw_data)
	response = v.send_mavlink(msg)

#if __name__=='__main__':

try:	
	getVelocity()
	print "Finished"
except rospy.ROSInterruptException:
	print "exception: ROSInterruptException" 
	pass


