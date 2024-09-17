#!/usr/bin/env python3

import numpy as np

class ForwardKinematics:
    def __init__(self, kinematic_model):
        self.kinematic_model = kinematic_model
        self.integrated_distances = np.array([0.,0.,0.])

    def calc_speed(self,wheels_speeds):
        self.wheels_speeds = wheels_speeds
        base_link_speeds = np.matmul(self.kinematic_model, self.wheels_speeds)
        return base_link_speeds
    
    def calc_distance(self, dt, base_link_speeds, theta):
        odom_speeds = self.apply_rotational_matrix(base_link_speeds, theta)
        self.integrated_distances += odom_speeds * dt
        return self.integrated_distances, odom_speeds
    
    def apply_rotational_matrix(self, input, theta):
        rotational_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                    [np.sin(theta), np.cos(theta), 0],
                                    [0 , 0 , 1]])
        map_reference_matrix = np.matmul(rotational_matrix,  input)
        return map_reference_matrix
class Mecanum(ForwardKinematics):
    def __init__(self, lx, ly, wheel_radius):
        constant = wheel_radius / 4
        kinematic_model = constant * np.array(
                                  [[1 , 1, 1 , 1],
                                   [-1, 1 , 1 , -1],
                                   [-1/(lx+ly), 1/(lx+ly), -1/(lx+ly), 1/(lx+ly)]])
        super().__init__(kinematic_model)