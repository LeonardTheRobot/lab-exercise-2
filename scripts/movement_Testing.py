#!/usr/bin/env python
import rospy
import math

from nav_msgs.msg import Odometry
#from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import (PoseWithCovarianceStamped, PoseArray,
                               Quaternion,  Transform,  TransformStamped )

startTime = 0
init = True
distance = 0

class Testing:

    def __init__(self):
        rospy.Subscriber('/odom', Odometry, self.callback)
        self.movement_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        
        INIT_X = 0 		# Initial x location of robot (metres)
        INIT_Y = 0			# Initial y location of robot (metres)
        INIT_Z = 0 			# Initial z location of robot (metres)
        INIT_HEADING = 0 	# Initial orientation of robot (radians)
        distance = 0
        #global startTime
        #startTime = rospy.get_rostime()
        #ospy.loginfo("Start Time: %i %i", startTime.secs, startTime.nsecs)

        self.lin_speed = 0.5
        self.ang_speed = 0

    def callback(self, odom_msg):
        global init
        new_speed = Twist()
        new_speed.angular.z = self.ang_speed
        new_speed.linear.x = self.lin_speed
	stop = Twist()
	stop.linear.x = 0
        if init:
            print(odom_msg.pose.pose.position.x)
            print(odom_msg.pose.pose.position.y)
            self.INIT_X = odom_msg.pose.pose.position.x
            self.INIT_Y = odom_msg.pose.pose.position.y
            print("got initial")
            init = False
            self.movement_publisher.publish(new_speed)
        else:
            distance = math.sqrt(pow(odom_msg.pose.pose.position.x - self.INIT_X,2) + pow(odom_msg.pose.pose.position.y - self.INIT_Y,2))
            
            #global startTime
            #timeLimit = rospy.Duration(secs=5)
            #while (rospy.get_rostime() - startTime) < timeLimit:
        	    #rospy.loginfo('lin: {}, ang: {}'.format(new_speed.linear.x, new_speed.angular.z))
        	    #self.movement_publisher.publish(new_speed)
            global distance
            if distance < 1:
                print(distance)
                self.movement_publisher.publish(new_speed)
            else:
		self.movement_publisher.publish(stop)
                print "stop ({})".format(distance)
            



if __name__ == '__main__':
    rospy.init_node(name='movement_Testing')
    raw_input('Switch on motors, then press Enter to start...')
    movement_Testing = Testing()
    rospy.spin()
