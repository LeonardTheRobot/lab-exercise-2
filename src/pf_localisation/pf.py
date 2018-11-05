from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy

from util import rotateQuaternion, getHeading
import random

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
        particlecloud = PoseArray()
        number_of_particles = 100
        map_width = self.occupancy_map.info.width
        map_height = self.occupancy_map.info.height

        accepted_particles = 0

        while accepted_particles < number_of_particles:
            x = random.randint(0, map_width - 1)
            y = random.randint(0, map_height - 1)
            theta = random.uniform(0, 2 * math.pi)

            pose = Pose()
            pose.position.x = x * 0.05
            pose.position.y = y * 0.05
            pose.orientation = rotateQuaternion(Quaternion(w=1.0), theta)

            map_index = x + y * map_width
            if self.occupancy_map.data[map_index] == 0:
                particlecloud.poses.append(pose)
                accepted_particles += 1

        return particlecloud
 
    def update_particle_cloud(self, scan):
        # Update particlecloud, given map and laser scan
        pass

    def estimate_pose(self):
        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.
        
        # Better approximations could be made by doing some simple clustering,
        # e.g. taking the average location of half the particles after 
        # throwing away any which are outliers
        est_pose = self.particlecloud.poses[0]
        return est_pose
