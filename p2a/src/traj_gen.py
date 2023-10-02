import math
import numpy as np
import scipy

# This code is for minimum jerk trajectory (MIGHT HAVE TO CHANGE)

class traj_gen:
    # Constructor
    def _init_(self, envi,waypoints):
        self.waypoints = waypoints
        num_wps,point_size = waypoints.shape #no of pooints,3
        self.num_wps = num_wps
        self.point_size = point_size
        self.order = 3 # Quintic for minimum jerk path 2*3-1

        self.yaw = 0
        self.ts = []
    def dist(self,p1,p2):
        return np.linalg.norm(p1-p2)
    def timeinterval_calc(self):
        # 5m/s
        avg_speed = 5
        dist_list = []
        total_dist =0
        for i in range(self.num_wps-1):
            wp_dist = self.dist(self.waypoints[i,:],self.waypoints[i+1,:])
            dist_list.append(wp_dist)
            total_dist+=wp_dist
        total_time = total_dist/avg_speed
        self.ts.append(0)
        prev_time = 0
        for i in range(self.num_wps):
            prev_time += (dist_list[i]/total_dist)*total_time
            self.ts.append(prev_time)
        return dist_list
    def matrix_default(self, r, q):
        
        return None
    def get_coeffs(self):
        num_eqns = self.num_wps - 1
        coeff_per_eqn = self.order + 1
        total_coeffs = num_eqns*coeff_per_eqn
        num_constraints = 4*(self.num_wps-2) + 6
        coeff_array = np.zeros((num_eqns*coeff_per_eqn,))
        big_mat = np.zeros((num_constraints,total_coeffs))
        constraint_mat = np.zeros((num_constraints,))
        for i in range(self.num_wps):
            big_mat[1,0:4] =