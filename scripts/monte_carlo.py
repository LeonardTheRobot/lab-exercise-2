#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray


class MonteCarloLocalisation:
    def __init__(self):
        rospy.Subscriber('base_scan', LaserScan, self.laser_callback)
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        # self.movement_publisher = 

    def laser_callback(self, laser_msg):
        # sensor_msgs/LaserScan - for description run: rosmsg show sensor_msgs/LaserScan
        laser_readings = np.array(laser_msg.ranges)
        rospy.loginfo('Laser Readings:\n{}'.format(laser_readings))
    
    def map_callback(self, map_msg):
        # nav_msgs/OccupancyGrid - for description run: rosmsg show nav_msgs/OccupancyGrid
        rospy.loginfo('map_msg:\n{}'.format(map_msg))


if __name__ == '__main__':
    rospy.init_node(name='monte_carlo')
    raw_input('Switch on motors, then press Enter to start...')
    mcl = MonteCarloLocalisation()
    rospy.spin()
