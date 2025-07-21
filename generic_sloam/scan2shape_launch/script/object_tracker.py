#!/usr/bin/python
#title			:exploration.py
#description	:
#author			:Xu Liu and Ankit Prabhu

import numpy as np
import math
from visualization_msgs.msg import MarkerArray,Marker
import rospy
from assignment import hungarian_assignment
from statistics import mean
import open3d as o3d


class ObjectTrack(object):
    def __init__(self, x, y, l, w, raw_points, last_update_scan_idx, track_idx, downsample_res = 0.3, num_instance_point_lim = 50000):
        # x, y position of objects in global frame
        self.x = x
        self.y = y
        self.l = l
        self.w = w
        self.age = 1
        self.pos_update_rate = 0.1
        self.last_update_scan_idx = last_update_scan_idx
        self.track_idx = track_idx
        self.o_pcd = o3d.geometry.PointCloud()
        # Resolution to perform voxel downsampling for instance point cloud accumulation
        self.downsample_res = downsample_res # in meters, -1 means no downsample
        # only keep the most recent num_points_limit_per_instance for any instance
        self.num_points_limit_per_instance = num_instance_point_lim# 50000
        print(f"Downsample the point cloud with a voxel of: ", self.downsample_res)
        print('Max number of points per instance is set as: ',  self.num_points_limit_per_instance)

        # initial covariance and standard devation
        self.xy_cov = 3*np.ones((2,2))

        # N*2 history positions of the object
        self.xy_hist = np.array([[x, y]])
        self.all_raw_points = self.downsample_point_cloud(raw_points)


    # update once it's assigned to a new detection, update all properties
    def update(self, x_new, y_new, l_new, w_new, raw_points_new, scan_idx):
        self.xy_hist = np.append(self.xy_hist, np.array([[x_new, y_new]]), axis=0)
        self.age = self.age+1

        self.x = self.pos_update_rate * x_new + (1 - self.pos_update_rate) *  self.x
        self.y = self.pos_update_rate * y_new + (1 - self.pos_update_rate) *  self.y
        self.l = self.pos_update_rate * l_new + (1 - self.pos_update_rate) *  self.l
        self.w = self.pos_update_rate * w_new + (1 - self.pos_update_rate) *  self.w

        self.xy_cov = np.cov(np.transpose(self.xy_hist))

        if self.downsample_res > 0:
            self.all_raw_points = np.vstack((self.all_raw_points, self.downsample_point_cloud(raw_points_new))) 
        else:
            self.all_raw_points = np.vstack((self.all_raw_points, raw_points_new)) 

        # limit the number of points to less than num_points_limit per instance
        if self.all_raw_points.shape[0] >  self.num_points_limit_per_instance:
            print('max number of points per instance reached! Limiting the number of points to: ',  self.num_points_limit_per_instance)
            # take the most recent num_points_limit points
            self.all_raw_points = self.all_raw_points[- self.num_points_limit_per_instance:, :]
        self.last_update_scan_idx = scan_idx
    
    def downsample_point_cloud(self, points_xyz):
        self.o_pcd.points = o3d.utility.Vector3dVector(points_xyz)
        downsampled_pcd = self.o_pcd.voxel_down_sample(voxel_size=max(self.downsample_res,0.01))
        print(f"before downsample: ", points_xyz.shape[0])
        print(f"after downsample: ", np.asarray(downsampled_pcd.points).shape[0])
        return np.asarray(downsampled_pcd.points)

