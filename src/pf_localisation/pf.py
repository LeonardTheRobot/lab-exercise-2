from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import rospy
import numpy as np

from util import rotateQuaternion, getHeading
import random

from time import time

from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN


class PFLocaliser(PFLocaliserBase):
    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        self.NUMBER_PARTICLES = 400
        self.REDIST_PERCENTAGE = 0.25
        
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.05 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.1 # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.05 # Odometry model y axis (side-to-side) noise
 
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 30 # Number of readings to predict

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
        # Generate all the particles randomly distributed throghout the world
        return self.gen_random_particles(self.NUMBER_PARTICLES)
 
    def update_particle_cloud(self, scan):
        # Update particlecloud, given map and laser scan
        particles = self.particlecloud.poses
        # Work out weights of particles from sensor readings
        weighted_particles = []

        for p in particles:
            pw = self.sensor_model.get_weight(scan, p)
            weighted_particles.append((p, pw))

        # Sort the particles by weight in descending order
        sorted_weighted_particles = sorted(weighted_particles, key=lambda p: p[1], reverse=True)

        # Keep the 3/4 of particles with the highest weights for resampling
        pred_weighted_particles = sorted_weighted_particles[:int((1 - self.REDIST_PERCENTAGE) * self.NUMBER_PARTICLES)]
        weight_sum = sum([p[1] for p in pred_weighted_particles])
        # Discard and randomly distribute the remaining 1/4 of particles across the map
        rand_weighted_particles = self.gen_random_particles(int(self.REDIST_PERCENTAGE * self.NUMBER_PARTICLES))
        
        # Resample particles according to weights
        cdf_aux = 0.0
        cdf = []
        for (p, w) in pred_weighted_particles:
            cdf.append((p, cdf_aux + w / weight_sum))
            cdf_aux += w / weight_sum

        u = random.uniform(0.0, 1.0 / len(pred_weighted_particles))
        i = 0
        new_particles = PoseArray()
        rejected_particles = 0

        for j in range(0, len(pred_weighted_particles)):
            while(u > cdf[i][1]):
                i += 1
            new_particle = Pose()
            new_particle.position.x = cdf[i][0].position.x + random.gauss(0.0, 0.2)
            new_particle.position.y = cdf[i][0].position.y + random.gauss(0.0, 0.2)
            new_particle.orientation = rotateQuaternion(Quaternion(w=1.0), getHeading(cdf[i][0].orientation) + random.gauss(0, 0.05))
            
            map_index = (new_particle.position.x/0.05) + (new_particle.position.y/0.05) * self.occupancy_map.info.width
            # print(int(map_index))
            if self.occupancy_map.data[int(map_index)] == 0:
                new_particles.poses.append(new_particle)
                u += (1.0 / len(pred_weighted_particles))
            else:
                rejected_particles += 1
            
        rand_weighted_particles = self.gen_random_particles(int(self.REDIST_PERCENTAGE * self.NUMBER_PARTICLES) + rejected_particles)  
        new_particles.poses += rand_weighted_particles.poses
        self.particlecloud = new_particles

    
    def get_neighbours(self, center, particles, limit):
        temp = []
        for p in particles:
            temp_p = np.array((p.position.x, p.position.y))
            dist = np.linalg.norm(center-temp_p)
            if(dist < limit and dist > -limit):
                temp.append(p)
        return temp
    
    def estimate_pose(self):
        # Create new estimated pose, given particle cloud
        # E.g. just average the location and orientation values of each of
        # the particles and return this.
        
        # Better approximations could be made by doing some simple clustering,
        # e.g. taking the average location of half the particles after 
        # throwing away any which are outliers
        particles = self.particlecloud.poses
        arr = []
        cluster_radius = 1

        for p in particles:
            arr.append((p.position.x, p.position.y))

        X = np.array(arr, ndmin=2)

        min_samples = 80
        x_dbs = DBSCAN(eps=cluster_radius, min_samples=min_samples).fit(X)
        while len(x_dbs.components_) == 0:
            min_samples -= 20
            x_dbs = DBSCAN(eps=cluster_radius, min_samples=min_samples).fit(X)
        center = x_dbs.components_[0]

        neighbours = self.get_neighbours(center, particles, cluster_radius)

        totalX = 0.0
        totalY = 0.0
        totalZ = 0.0
        totalW = 0.0
        total = 0

        for p in neighbours:
            total += 1
            totalX += p.orientation.x
            totalY += p.orientation.y
            totalZ += p.orientation.z
            totalW += p.orientation.w
        new_orientation = Quaternion(totalX / total, totalY / total, totalZ  /total, totalW / total)
       
        est_pose = Pose()
        est_pose.position.x = center[0]
        est_pose.position.y = center[1]
        est_pose.orientation = new_orientation
        return est_pose
