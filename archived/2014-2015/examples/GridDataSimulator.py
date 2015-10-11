#!/usr/bin/env python
from random import randint
import rospy
#make seperate message type for GridData??????????????????????????
from uav_msgs.msg import OpticalFlowPost
from uav_msgs.msg import GridLinePosPost

#set global variables here
optFlowFrequency=20
gridLineFrequency=5


def simulator():
        

        frequency=raw_input('Enter frequency: ')
        frequency=float(frequency)
        timetorun=raw_input('Enter run time: ')
        timetorun=float(timetorun)
	timestep=1/frequency
	timestepcount=0
        
	#Initialize grid line node
        gridPub=rospy.Publisher('gridline_pos_post', GridLinePosPost)
        rospy.init_node('simulator', anonymous=True)
        r1=rospy.Rate(gridLineFrequency)


	#Initialize opt flow node
	optPub=rospy.Publisher('of_meas_post', OpticalFlowPost)
        rospy.init_node('simulator', anonymous=True)
        r2=rospy.Rate(optFlowFrequency)


        #starts quad at the upper left corner on the floor. The grid is a 20x20 with the origin in the center
        xPos=-9.0
        yPos=9.0
        zPos=0.0

        #These are all chosen arbitrarily
        xV=0.40
        yV=0.40
        zV=0.1

        while (timestepcount < timetorun/timestep) and not rospy.is_shutdown():
                #This moves the quad
                print 'moved'
                xPos=xPos+xV*(timestep)
                yPos=yPos+yV*(timestep)
                zPos=zPos+zV*(timestep)


                #This makes sure the quad doesn't leave the arena 
                if abs(xPos)>9.5:
                        xV=-xV
                if abs(yPos)>9.5:
                        yV=-yV

                #and so if it reaches the top, it goes slowly
                if zPos>2.7:
                        zV=-0.05
                if zPos<0.2:
                        zV=0.1


                #generates noise for position
                noiseConstant=0.05
                noise=randint(0,9)*(noiseConstant)

                #creates messages
                
                gmsg = GridLinePosPost()
                gmsg.vel=[xV, yV, zV]
                gmsg.position.x=xPos+noise
                gmsg.position.y=yPos+noise
                gmsg.position.z=zPos+noise

		omsg = OpticalFlowPost()
		k=0.1
                omsg.x_vel=xV-k*noise
		omsg.y_vel=yV-k*noise
                

                #publishes
                rospy.loginfo(gmsg)
                gridPub.publish(gmsg)

		rospy.loginfo(omsg)
                optPub.publish(omsg)
                r1.sleep()
		r2.sleep()

                timestepcount=timestepcount+1
        

        if __name__=='__main__':
               try:
                       simulator()
               except rospy.ROSInterruptException: pass
                
simulator()

