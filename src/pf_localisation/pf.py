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
        self.ODOM_ROTATION_NOISE = 0.05 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.1 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.05 # Odometry model y axis (side-to-side) noise
 
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20 # Number of readings to predict

    def gen_random_particles(self, number_of_particles):
        particles = PoseArray()

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
                particles.poses.append(pose)
                accepted_particles += 1
            
        return particles

    def initialise_particle_cloud(self, initialpose):
        return self.gen_random_particles(200)
 
    def update_particle_cloud(self, scan):
        # Update particlecloud, given map and laser scan
        particles = self.particlecloud.poses
        # Work out weights of particles from sensor readings
        weighted_particles = []
        weight_sum = 0.0
        for p in particles:
            pw = self.sensor_model.get_weight(scan, p)
            weighted_particles.append((p, pw))

        # Sort the particles by weight in descending order
        sorted_weighted_particles = sorted(weighted_particles, key=lambda p: p[1], reverse=True)
        # Resample the 150 particles with the highest weights
        pred_weighted_particles = sorted_weighted_particles[:150]
        weight_sum = sum([p[1] for p in pred_weighted_particles])
        # Random distribute the remaining 50 particles across the map
        rand_weighted_particles = self.gen_random_particles(50)
        
        # Resample particles according to weights
        cdf_aux = 0.0
        cdf = []
        for (p, w) in pred_weighted_particles:
            cdf.append((p, cdf_aux + w / weight_sum))
            cdf_aux += w / weight_sum

        u = random.uniform(0.0, 1.0 / len(pred_weighted_particles))
        i = 0
        new_particles = PoseArray()

        for j in range(0, len(pred_weighted_particles)):
            while(u > cdf[i][1]):
                i += 1
            new_particle = Pose()
            new_particle.position.x = cdf[i][0].position.x + random.gauss(0.0, 0.2)
            new_particle.position.y = cdf[i][0].position.y + random.gauss(0.0, 0.2)
            new_particle.orientation = rotateQuaternion(Quaternion(w=1.0), getHeading(cdf[i][0].orientation) + random.gauss(0, 0.05))
            new_particles.poses.append(new_particle)
            
            u += (1.0 / len(pred_weighted_particles))
        
        new_particles.poses += rand_weighted_particles.poses
        self.particlecloud = new_particles
    
    def estimate_pose(self):
        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.
        
        # Better approximations could be made by doing some simple clustering,
        # e.g. taking the average location of half the particles after 
        # throwing away any which are outliers
        num_particles = len(self.particlecloud.poses)
        sum_x = 0.0
        sum_y = 0.0
        sum_theta = 0.0

        for pose in self.particlecloud.poses:
            sum_x += pose.position.x
            sum_y += pose.position.y
            sum_theta += getHeading(pose.orientation)
        
        est_pose = Pose()
        est_pose.position.x = sum_x / num_particles
        est_pose.position.y = sum_y / num_particles
        est_pose.orientation = rotateQuaternion(Quaternion(w=1.0), sum_theta / num_particles)

        return est_pose
