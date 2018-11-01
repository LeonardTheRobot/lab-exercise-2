from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
 
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20 	# Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        # Distribute particles randomly across the map
        number_of_particles = 300
	    map_width = self.occupancy_map.info.width
	    map_height = self.occupancy_map.info.height
	

        for i in range(number_of_particles):
            pose = Pose()
            x = random.randint(0, 20 * map_width) / 20.0
            y = random.randint(0, 20 * map_height) / 20.0
            z = random.uniform(0, 6.283)
            pose.position.x = x
            pose.position.y = y
            pose.orientation = rotateQuaternion(Quaternion(w=1.0),z)

            # Remove particles that are outside map boundaries
            x_index = x / 0.05
            y_index = y / 0.05

            major_index = x_index + (y_index * map.width)
            valid = self.occupancy_map.data[major_index]

            if valid == 0 or valid == -1:
                continue
            else:
                self.particlecloud.poses.add(pose)


    def update_particle_cloud(self, scan):
        # Update particlecloud, given map and laser scan

    def estimate_pose(self):
        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.
        
        # Better approximations could be made by doing some simple clustering,
        # e.g. taking the average location of half the particles after 
        # throwing away any which are outliers

