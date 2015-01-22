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

from uav_msgs.msg import IMUSample
from uav_msgs.msg import OptFlowSample

import droneapi.lib
from pymavlink import mavutil
#from comm_node import MavCommNode


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

class UAV():

	velocityFrequency = 20
	api = None
	v = None
	pub_imu = None

	def __init__(self):
		self.api = local_connect()		
		rospy.init_node('comm_node',disable_signals=True) #to start time
		self.pub_imu = rospy.Publisher('uav_telemetry/imu', IMUSample, queue_size=10)
		self.of_pub = rospy.Publisher('uav_telemetry/opt_flow', OptFlowSample, queue_size=5)
		vehicles = self.api.get_vehicles()
		self.v = vehicles[0]	
		
	def handleMavlink(self):
		print 'Setting mavlink callback'
		self.v.set_mavlink_callback(self.mavlinkCallback)
		print 'Mavlink callback successfully set'

	def spin(self):
		rospy.spin()
		#while not self.api.exit:
			#time.sleep(3)	

	def getVelocity(self):
		print 'Initializing getVelocity....'		
		
		print str(self.v.location)	
		print self.v.attitude
		print 'airspeed: '+str(self.v.airspeed)
		print 'groundspeed: '+str(self.v.groundspeed)								

		vx_vy_vz = self.v.velocity
		attitude = self.v.attitude
		#time = rospy.now() todo
		#rate1.sleep()
	#	setOrigin(8,8,7)
		#sys.exit()
		#optFlowFrequency = 20
		#rospy.init_node('simulator', anonymous=True)
		#rate_velocity = rospy.Rate(optFlowFrequency)

		#this loop executes after leaving the above loop and is for cleanup on exit
		#for (m,pm) in mpstate.modules:
		#if hasattr(m, 'unload'):
		#    print("Unloading module %s" % m.name)
		#    m.unload()

	def setPosition(self,x,y,z,yaw="none"):
		#https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/pixhawk.xml

		#construct a message to set a new offset from the currently controlled position
		if yaw=="none":
			msg = self.v.message_factory.set_position_control_offset_encode(x,y,z)
		else:
			msg = self.v.message_factory.set_position_control_offset_encode(x,y,z,yaw)
		self.v.send_mavlink(msg)
		#Bitmask to indicate which dimensions should be ignored by the vehicle: 
		#a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. 
		#If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. 
		#Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, 
		#bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
		msg2 = self.v.message_factory.position_target_local_ned_encode(x,y,z,vx,vy,vz,afx,afy,afz,yaw,yaw_rate)
		response2 = self.v.send_mavlink(msg2)

	def setAttitude(self,roll,pitch,yaw):
		msg = self.v.message_factory.attitude_control_encode(roll,pitch,yaw)
		response = self.v.send_mavlink(msg)
		msg2 = self.v.message_factory.attitude_target_encode(roll,pitch,yaw)
		response2 = self.v.send_mavlink(msg2)

	def getParameters(self):
		return self.v.parameters
		
	def getIMU(self):
		msg = self.v.message_factory.scaled_imu_encode()#takes 11 args 
		#timestamp (millisec since system boot)
		# xacc (mg), yacc (mg), zacc (mg)
		# w_x (mrad/sec), w_y, w_z
		# magneticfield_x, B_y, B_z (milli tesla)
		response = self.v.send_mavlink(msg)	

	def getLocalPos(self):
		msg = self.v.message_factory.local_position_ned_encode()
		# filtered local position based off accelerometers.
		#right handed coord frame, z-axis down. north-east-down convention
		# x y z vx vy vz
		msg2 = self.v.message_factory.local_position_ned_cov_encode()
		# plus covariance	
		response = self.v.send_mavlink(msg)	
		response2 = self.v.send_mavlink(msg2)

	def setOrigin(self,mylat,mylong,myalt):
		sysID = 0 #target system and target componenet can always be set to 0
		msg = self.v.message_factory.set_gps_global_origin_encode(sysID,mylat,mylong,myalt)
		print msg
		#sets the global mission (0,0,0) point. mavlink supports indoor navigation
		response = self.v.send_mavlink(msg)		
		response = self.v.flush()
		print 'Response: '+str(response)

	def getOpticalFlow(self):
		msg = self.v.message_factory.optical_flow_rad_encode()
		#integration time, integrated_x, integrated)y, integrated_xgyro, ygyro, zgryo,temp, quality, time_delta_distance,distance to center of flow field
		response = self.v.send_mavlink(msg)
		self.v.flush()

	def injectGPS(self):
		raw_data = [110] #i think max length is 110? enough for 12 satellites
		msg = self.v.message_factory.gps_inject_data_encode(raw_data.length,raw_data)
		response = self.v.send_mavlink(msg)

	def mavlinkCallback(self,packet):
		typ = packet.get_type()
		#print typ

		if str(typ) == "RAW_IMU":		
			imu = IMUSample()
			imu.gyro_x = packet.xgyro
			imu.gyro_y = packet.ygyro
			imu.gyro_z = packet.zgyro
			imu.acc_x = packet.xacc
			imu.acc_y = packet.yacc
			imu.acc_z = packet.zacc
			imu.mag_x = packet.xmag
			imu.mag_y = packet.ymag
			imu.mag_z = packet.zmag
			#imu.timestamp = MavCommNode.uavTimetoRosTime(packet.time_usec)
			try:
				self.pub_imu.publish(imu)
				rospy.loginfo('Published imu message')
			except:
				print "error publishing imu."			
			
			#print "packet time: "+str(packet.time_usec)
			#print "xacc: "+str(packet.xacc)+" yacc: "+str(packet.yacc)+" zacc: "+str(packet.zacc)
			#print "xgyro: "+str(packet.xgyro)+" ygyro: "+str(packet.ygyro)+" zgyro: "+str(packet.zgyro)
			#print "xmag: "+str(packet.xmag)+" ymag: "+str(packet.ymag)+" zmag: "+str(packet.zmag)
		elif str(typ) == "OPTICAL_FLOW":
			of = OptFlowSample()
			of.x_vel = packet.flow_comp_m_x
			of.y_vel = packet.flow_comp_m_y
			of.ground_distance = packet.ground_distance
			of.quality = packet.quality

			#of.timestamp = packet.time_usec #uavTimeToRosTime(msg.time_us, rx_time_ros)
			try:
				self.of_pub.publish(of)
				rospy.loginfo('Published flow message')
			except:
				print "error publishing flow"

			print "flow_x: "+str(packet.flow_x)+" flow_y: "+str(packet.flow_y)+" quality: "+str(packet.quality)
			print "ground_distance: "+str(packet.ground_distance)

		elif str(typ) == "OPTICAL_FLOW_RAD" or str(typ) == "HIL_OPTICAL_FLOW" or str(typ) == "MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW":
			print "more flow data!!!"
		else:
			pass 	     

try:	
	uav = UAV()
	uav.handleMavlink()
	uav.getVelocity()
	uav.spin()	
	print "Finished"
except rospy.ROSInterruptException:
	print "exception: ROSInterruptException" 
	pass


