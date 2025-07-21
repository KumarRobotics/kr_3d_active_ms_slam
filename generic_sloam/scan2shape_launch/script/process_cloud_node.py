#! /usr/bin/env python3
# title			:
# description	:
# author		:Xu Liu and Ankit Prabhu

# type: sensor_msgs/PointCloud2
# topic of interest: /os_node/llol_odom/sweep
# field of interest: header/stamp (secs and nsecs)
# in the segmentation result folder, the pcds are named by point_cloud_secs+nsecs.pcd
# goal: find the point cloud in /os_node/llol_odom/sweep topic that has matched timestamp of the pcds, and save them as world_frame_point_cloud_secs+nsecs.pcd

import time
from matplotlib import use
import argparse
import tf
from tf2_ros import TransformListener, Buffer
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sklearn.cluster import DBSCAN
import copy
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray
import message_filters
import open3d as o3d
from sloam_msgs.msg import ROSRangeBearing
from sem_detection_msgs.msg import syncPcOdom

import rospy
from scipy.spatial.transform import Rotation as R
import glob
import numpy as np
np.float = np.float64 # This is for ros_numpy since np.float is deprecated since 1.20.
import ros_numpy
import os
from utils import show_clusters, publish_tree_cloud, publish_ground_cloud, transform_publish_pc,send_tfs, publish_accumulated_cloud, make_fields, threshold_by_range, calc_dist_to_ground, publish_ground_cloud_and_fake_cubes
from cuboid_utils import fit_cuboid, publish_cuboid_markers, publish_range_bearing_measurements, cuboid_detection, generate_publish_instance_cloud
from object_tracker_utils import track_objects, publish_markers
from nav_msgs.msg import Odometry
import sys


class ProcessCloudNode:
    def __init__(self):
        
        ################################## IMPORTANT PARAMS ##################################
        # the label corresponds to ground class
        self.ground_class_label = rospy.get_param('~ground_class_label', 0) 

        # the label corresponds to vehicle class
        self.car_class_label = rospy.get_param('~car_class_label', 1) 

        # the label corresponds to tree class
        self.tree_trunk_class_label = rospy.get_param('~tree_trunk_class_label', 8) 

        # the label corresponds to light pole class
        self.light_pole_class_label = rospy.get_param('~light_pole_class_label', 9) 

        self.desired_acc_obj_pub_rate = rospy.get_param('~desired_acc_obj_pub_rate', 1) # in Hz
        
        self.prev_acc_obj_pub_time = None

        # the expected lidar frequency for calculating when to place or remove a track from object assignment
        self.expected_segmentation_rate = rospy.get_param('~expected_segmentation_rate', 3) # in Hz
        
        # use simulator or not
        self.use_sim = rospy.get_param('~sim', False)

        # save figures of car clustering results
        self.visualize = False

        # process yolo bag
        self.process_yolo_bag = False
        

        # threshold for lidar segmentation range        
        self.valid_range_threshold = rospy.get_param('~valid_range_threshold', 6.0)
        # TODO currently we are using pano frame which is not really body centered
        
        # the height of considering a point as a ground point 
        # TODO use actual local ground patches, instead of all ground points in the scan
        # self.ground_median_increment = 0.4


        # epsilon_scan: determines the radius of clustering when considering neighboring points as single cluster.
        # Higher value results in more bigger cluster but also nosier clusters. Best value for now is between 0.5-1.0
        self.epsilon_scan = rospy.get_param('~epsilon_scan', 0.5)

        # min_samples_scan : determines the minimum samples needed inside the epsilon radius of the cluster to be clustered together
        # Higher values result in more sparser clusters with less noise
        self.min_samples_scan = rospy.get_param('~min_samples_scan', 100)
                 
        # Cuboid length and width threshold used to start initial object tracking process
        self.fit_cuboid_length_thresh = 0.05
        self.fit_cuboid_width_thresh = 0.05

        # visualize all car instances (keep track of all and never remove tracks), should be set as False unless you know what you are doing
        self.car_inst_visualize = False
        
        # depth percentile for filtering out noisy points in depth camera
        depth_percentile_lower = rospy.get_param('~depth_percentile_lower', 30)
        depth_percentile_uppper = rospy.get_param('~depth_percentile_uppper', 50)
        self.depth_percentile = (depth_percentile_lower, depth_percentile_uppper)

        # confidence_threshold
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)


        # cutoff for filtering out the FINAL false positive cuboids (lower_limit, upper_limit)
        cuboid_length_cutoff_upper = rospy.get_param('~cuboid_length_cutoff_upper', 1.5)
        cuboid_width_cutoff_upper = rospy.get_param('~cuboid_width_cutoff_upper', 1.5)
        cuboid_height_cutoff_upper = rospy.get_param('~cuboid_height_cutoff_upper', 5)
        self.cuboid_length_cutoff = (0.1, cuboid_length_cutoff_upper)
        self.cuboid_width_cutoff = (0.1, cuboid_width_cutoff_upper)
        self.cuboid_height_cutoff = (0.05, cuboid_height_cutoff_upper)
        
        # Minimum time required to track a cuboid per lidar scan 
        time_to_initialize_cuboid = rospy.get_param('~time_to_initialize_cuboid', 0.3) # in seconds
        self.tracker_age_thresh_lower = self.expected_segmentation_rate * time_to_initialize_cuboid   

        # Lower will delete tracks faster and will cause new tracks to be initiated at the same location
        time_to_delete_lost_track_cuboid = rospy.get_param('~time_to_delete_lost_track_cuboid', 5) # in seconds
        self.num_lost_track_times_thresh = self.expected_segmentation_rate * time_to_delete_lost_track_cuboid # running 10hz:20 #20

        # Hungarian assignment cost to associate 2 cuboid centroids as one
        self.assignment_threshold = rospy.get_param('~assignment_threshold', 1.0) 
        # Resolution to perform voxel downsampling for instance point cloud accumulation
        self.downsample_res = rospy.get_param('~downsample_res', 0.1) 
        # only keep the most recent num_points_limit_per_instance for any instance
        self.num_instance_point_lim = rospy.get_param('~num_instance_point_lim', 30000) 
        # run forestry data or not
        self.for_forestry_bags = rospy.get_param('~for_forestry_bags', False) 
        ################################## IMPORTANT PARAMS ENDS ##################################
        
            
        
        # PRINT ALL VARIABLES INIITIALIZED ABOVE USING GET_PARAM
        print("######################################################################################################")
        print("ground_class_label: ", self.ground_class_label)
        print("car_class_label: ", self.car_class_label)
        print("tree_trunk_class_label: ", self.tree_trunk_class_label)
        print("light_pole_class_label: ", self.light_pole_class_label)
        print("desired_acc_obj_pub_rate: ", self.desired_acc_obj_pub_rate)
        print("expected_segmentation_rate: ", self.expected_segmentation_rate)
        print("use_sim: ", self.use_sim)
        print("visualize: ", self.visualize)
        print("process_yolo_bag: ", self.process_yolo_bag)
        print("valid_range_threshold: ", self.valid_range_threshold)
        print("epsilon_scan: ", self.epsilon_scan)
        print("min_samples_scan: ", self.min_samples_scan)
        print("fit_cuboid_length_thresh: ", self.fit_cuboid_length_thresh)
        print("fit_cuboid_width_thresh: ", self.fit_cuboid_width_thresh)
        print("car_inst_visualize: ", self.car_inst_visualize)
        print("depth_percentile: ", self.depth_percentile)
        print("confidence_threshold: ", self.confidence_threshold)
        print("cuboid_length_cutoff: ", self.cuboid_length_cutoff)
        print("cuboid_width_cutoff: ", self.cuboid_width_cutoff)
        print("cuboid_height_cutoff: ", self.cuboid_height_cutoff)
        print("tracker_age_thresh_lower: ", self.tracker_age_thresh_lower)
        print("num_lost_track_times_thresh: ", self.num_lost_track_times_thresh)
        print("assignment_threshold: ", self.assignment_threshold)
        print("downsample_res: ", self.downsample_res)
        print("num_instance_point_lim: ", self.num_instance_point_lim)
        print("for_forestry_bags: ", self.for_forestry_bags)
        print("time_to_initialize_cuboid: ", time_to_initialize_cuboid)
        print("time_to_delete_lost_track_cuboid: ", time_to_delete_lost_track_cuboid)
        print("######################################################################################################")

        self.o_pcd = o3d.geometry.PointCloud()

        # Leave this as False, it will be automatically set according to callbacks
        self.use_faster_lio_or_depth = False

        self.tf_listener2 = tf.TransformListener()
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.pc_fields_ = make_fields()
        self.pc_fields_depth_seg_ = make_fields()


        # IMPORTANT: THESE TWO NEED TO BE UPDATED SIMULTANEOUSLY, THEIR LENGTH SHOULD MATCH!
        # the x, y, length and width of each object
        self.all_objects = []
        # the ObjectTrack instance of each object
        self.all_tracks = []
        self.save_fig_idx = 0
        self.save_fig_counter = 0
        self.processed_scan_idx = -1
        self.last_stamp = 0
        self.raw_cloud_dict = {}
        self.thresh_by_range_time = np.array([])
        self.transform_pano_to_world_time = np.array([])
        self.clustering_time = np.array([])
        self.fit_cuboid_time = np.array([])
        self.track_objects_time = np.array([])
        self.deleting_tracks_time = np.array([])
        self.accumulated_cloud_time = np.array([])
        self.final_cuboid_detection_time = np.array([])
        self.callback_time = np.array([])


        self.segmented_pc_pub = rospy.Publisher("/process_cloud_node/filtered_semantic_segmentation", PointCloud2, queue_size=1)
        self.car_convex_hull_pub = rospy.Publisher("/car_convex_hull", PointCloud2, queue_size=100)


        self.cuboid_center_marker_pub = rospy.Publisher(
            "cuboid_centers", MarkerArray, queue_size=1)
        self.cuboid_center_cov_pub = rospy.Publisher(
            "cuboid_centers_covariance", MarkerArray, queue_size=1)

        self.instance_cloud_pub = rospy.Publisher(
            "/car_instance_segmentation_accumulated", PointCloud2, queue_size=1)

        self.cuboid_marker_pub = rospy.Publisher(
            "/car_cuboids", MarkerArray, queue_size=5)
        
        self.range_bearing_pub = rospy.Publisher(
            "/semantic_range_bearing_measurements", ROSRangeBearing, queue_size=5)
        
        self.cuboid_marker_body_pub = rospy.Publisher(
            "/car_cuboids_body", MarkerArray, queue_size=5)

        self.odom_received = False

        self.tree_cloud_pub = rospy.Publisher(
            "/tree_cloud", PointCloud2, queue_size=1)

        self.ground_cloud_pub = rospy.Publisher(
            "/ground_cloud", PointCloud2, queue_size=1)

        self.odom_pub = rospy.Publisher(
            "/quadrotor/lidar_odom", Odometry, queue_size=100)


        # frame ids
        if self.use_sim  == False:
            # range image frame
            self.range_image_frame = "body"
            self.reference_frame = "dragonfly67/odom"
            # undistorted point cloud frame
            self.undistorted_cloud_frame = "camera" # pano for Chao LLOL
        else:
            # Note: interface with exploration
            self.range_image_frame = "body"
            self.reference_frame = "world"
            self.undistorted_cloud_frame = "camera" # pano for Chao LLOL
            # self.range_image_frame = "quadrotor"
            # self.undistorted_cloud_frame = "quadrotor" 
            # self.reference_frame = "quadrotor/odom"

        # subscriber and publisher
        if self.use_sim == False:
            print("Running real-world experiments...")
            time.sleep(1)
            
            self.segmented_pc_sub = rospy.Subscriber(
                "/sem_detection/sync_pc_odom", syncPcOdom, callback=self.segmented_pc_cb, queue_size=1)
            # NOTE: IMPORTANT - it is the ideal approach to use raw odom
            # this allows us to accumulate the observation WITHOUT sloam optimization to avoid cyclic dependency
            # we just publish everything including positions of landmarks in body frame
            # publish odom as tf, then use tf to transform point clouds, etc...
            self.odom_sub = rospy.Subscriber("/dragonfly67/quadrotor_ukf/control_odom", Odometry, callback=self.odom_callback, queue_size=100)

        else:
            print("Running simulation experiments...")
            time.sleep(1)
            self.segmented_pc_sub = rospy.Subscriber(
                "/sem_detection/sync_pc_odom", syncPcOdom, callback=self.segmented_pc_cb, queue_size=1)
            self.odom_sub = rospy.Subscriber("/dragonfly67/quadrotor_ukf/control_odom", Odometry, callback=self.odom_callback, queue_size=100)


    def sim_segmented_synced_pc_cb(self, car_cloud_msg, odom_msg):
        self.segmented_synced_pc_cb(car_cloud_msg, None)

    def segmented_pc_cb(self, seg_cloud_msg):
        # Now, this seg_cloud_msg has three parts, 
        # Header, cloud, odom (synced with cloud)
        # this callback will only be used for faster lio
        self.use_faster_lio_or_depth = True
        self.odom_from_cloud_msg = seg_cloud_msg.odom
        self.segmented_synced_pc_cb(seg_cloud_msg.cloud, None)

    # synced version
    def segmented_pc_with_odom_cb(self, seg_cloud_msg, odom_msg):
        # this callback will only be used for faster lio
        self.use_faster_lio_or_depth = True
        self.odom_callback(odom_msg)
        self.segmented_synced_pc_cb(seg_cloud_msg, None)

    def segmented_synced_pc_cb(self, segmented_cloud_msg, undistorted_cloud_msg):
        cb_start_time = time.time()
        self.processed_scan_idx = self.processed_scan_idx + 1
        current_raw_timestamp = segmented_cloud_msg.header.stamp
        # create pc from the undistorted_cloud
        segmented_pc = ros_numpy.numpify(segmented_cloud_msg)
        if self.use_sim or self.use_faster_lio_or_depth:
            undistorted_pc = segmented_pc
        else:
            undistorted_pc = ros_numpy.numpify(undistorted_cloud_msg)
        # update the x y z to those in the bag
        x_coords = np.nan_to_num(
            undistorted_pc['x'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        y_coords = np.nan_to_num(
            undistorted_pc['y'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        z_coords = np.nan_to_num(
            undistorted_pc['z'].flatten(), copy=True, nan=0.0, posinf=None, neginf=None)
        print(z_coords)
        # fill in the intensity values that represent the class labels
        if self.use_sim:
            intensities = (segmented_pc['intensity']).flatten()
            ids = (segmented_pc['id']).flatten()
            confs = (segmented_pc['confidence']).flatten()
        else:
            intensities = (segmented_pc['intensity']).flatten()
            ids = (segmented_pc['id']).flatten()
            confs = (segmented_pc['confidence']).flatten()

        pc_xyzi_id_conf = np.zeros((x_coords.shape[0], 6))
        pc_xyzi_id_conf[:, 0] = x_coords
        pc_xyzi_id_conf[:, 1] = y_coords
        pc_xyzi_id_conf[:, 2] = z_coords
        pc_xyzi_id_conf[:, 3] = intensities
        pc_xyzi_id_conf[:, 4] = ids
        pc_xyzi_id_conf[:, 5] = confs


        # threshold the point cloud according to the range -- throw away too far to 0-range points
        # print("\n-----------Benchmarking Starts-----------")
        start_time = time.time()
        valid_indices = threshold_by_range(self.valid_range_threshold, pc_xyzi_id_conf)
        self.thresh_by_range_time = np.append(self.thresh_by_range_time, [time.time() - start_time])
        print("\n************")
        print("The instant threshold by range time is {:.7f}".format(self.thresh_by_range_time[-1]))
        print("The average threshold by range time is {:.7f}".format(self.thresh_by_range_time.mean()))
        print("************\n")

        if np.sum(valid_indices) == 0:
            print("no valid points found!!!")
            return

        # apply thresholding
        pc_xyzi_id_conf_thresholded = pc_xyzi_id_conf[valid_indices, :]

        indices_mask = np.cumsum(valid_indices) - 1
        # indices_original = np.arange(valid_indices.shape[0])

        # apply transform and publish point cloud in the world frame

        start_time = time.time()
        points_world_xyzi_id_conf_depth, points_body_xyzi_id_conf = transform_publish_pc(self,
            current_raw_timestamp, pc_xyzi_id_conf_thresholded)
        self.transform_pano_to_world_time = np.append(self.transform_pano_to_world_time, [time.time() - start_time])
        print("\n************")
        print("The instant transform pano to world time is {:.7f}".format(self.transform_pano_to_world_time[-1]))
        print("The average transform pano to world time is {:.7f}".format(self.transform_pano_to_world_time.mean()))
        print("************\n")


        start_time = time.time()
        if points_world_xyzi_id_conf_depth is None or points_body_xyzi_id_conf is None:
            print("failed to find transform, skipping this cloud (see error msg above for exact failed tf)... ")
            print("if you are replying bags, try setting /use_sim_time to true and add --clock flag to rosbag play")
            print("it may also be caused by that your laptop struggles to run real time, play bag with slower rate")

        else:
            # intensity is the class label, which is: 1-road, 5-car, 8-tree-trunk, 9-light-pole, 0-unlabeled(background)
            points_world_xyzi_id_conf_depth_cars = points_world_xyzi_id_conf_depth[points_world_xyzi_id_conf_depth[:, 3] == self.car_class_label, :]
            points_world_xyzi_ground = points_world_xyzi_id_conf_depth[points_world_xyzi_id_conf_depth[:, 3] == self.ground_class_label, :3]

            ##################################### Ground Plane Fitting ##################################

            # [a, b, c, d] = plane_eq
            self.ground_plane_coeff = None# np.array([a,b,c,d])

            ############################################################################### prepare point clouds for SLOAM tree stuff ###############################################################
            points_body_xyzi_id_conf_organized = points_body_xyzi_id_conf[indices_mask, :]


            points_body_xyzi_id_conf_organized[valid_indices == 0, :] = np.NaN

            points_body_xyzi_tree_pole_masked = points_body_xyzi_id_conf_organized[:,:4].copy()
            points_body_xyzi_ground_masked = points_body_xyzi_id_conf_organized[:,:4].copy()
            non_tree_idx = points_body_xyzi_id_conf_organized[:, 3] != self.tree_trunk_class_label
            non_light_pole_idx = points_body_xyzi_id_conf_organized[:, 3] != self.light_pole_class_label
            non_tree_or_light_pole_idx = np.logical_and(
                non_tree_idx, non_light_pole_idx)
            points_body_xyzi_tree_pole_masked[non_tree_or_light_pole_idx, :] = np.NaN
            points_body_xyzi_ground_masked[points_body_xyzi_id_conf_organized[:, 3]
                                           != self.ground_class_label, :] = np.NaN
            
            # destagger
            points_body_xyzi_tree_pole_masked_reshaped = points_body_xyzi_tree_pole_masked.reshape(
                (480, 640, 4))

            pc_msg_tree = publish_tree_cloud(self.pc_fields_, points_body_xyzi_tree_pole_masked_reshaped, current_raw_timestamp, self.range_image_frame)
            self.tree_cloud_pub.publish(pc_msg_tree)

            print("Running on forestry bags is set as: ", self.for_forestry_bags, "(will publish fake cuboids if true).")
            if self.for_forestry_bags:
                pc_msg_ground, fake_cube_body = publish_ground_cloud_and_fake_cubes(self.pc_fields_, points_body_xyzi_ground_masked.reshape(
                    (480, 640, 4)), current_raw_timestamp, self.range_image_frame)
                self.cuboid_marker_body_pub.publish(fake_cube_body)
            else:
                pc_msg_ground = publish_ground_cloud(self.pc_fields_, points_body_xyzi_ground_masked.reshape(
                    (480, 640, 4)), current_raw_timestamp, self.range_image_frame)

            self.ground_cloud_pub.publish(pc_msg_ground)
            #############################################################################################################################################################################################


            # find object instances
            if points_world_xyzi_id_conf_depth_cars.shape[0] > 0:
                
                start_time = time.time()
                xcs, ycs, lengths, widths, raw_points = fit_cuboid(self.fit_cuboid_length_thresh, 
                    self.fit_cuboid_width_thresh, points_world_xyzi_id_conf_depth_cars, self.depth_percentile, self.confidence_threshold, self.epsilon_scan, self.min_samples_scan)
                                
                self.fit_cuboid_time = np.append(self.fit_cuboid_time, [time.time() - start_time])
                print("\n************")
                print("The instant fit cuboid time is {:.7f}".format(self.fit_cuboid_time[-1]))
                print("The average fit cuboid time is {:.7f}".format(self.fit_cuboid_time.mean()))
                print("************\n")

                # N*4 objects, first two columns are x and y coordinates, third column is length (x-range), and fourth colum is width (y-range)
                cur_objects = np.transpose(
                    np.asarray([xcs, ycs, lengths, widths]))

                no_detections = False
                if cur_objects.shape[0] == 0:
                    no_detections = True

                if no_detections == False:

                    start_time = time.time()
                    self.all_objects, self.all_tracks = track_objects(
                        cur_objects, self.all_objects, self.all_tracks, self.processed_scan_idx, copy.deepcopy(raw_points), self.assignment_threshold, self.downsample_res, self.num_instance_point_lim)
                    self.track_objects_time = np.append(self.track_objects_time, [time.time() - start_time])
                    print("\n************")
                    print("The instant track objects time is {:.7f}".format(self.track_objects_time[-1]))
                    print("The average track objects time is {:.7f}".format(self.track_objects_time.mean()))
                    print("************\n")

                    publish_markers(self, self.all_tracks)

                    if self.car_inst_visualize:
                        for track in self.all_tracks:

                            self.raw_cloud_dict[track.track_idx] = (
                                track.age, track.all_raw_points)
            else:
                print("skipping the car fitting no valid points above ground")

            # get rid of too old tracks to bound computation and make sure our cuboid measurements are local and do not incorporate too much odom noise
            start_time = time.time()
            idx_to_delete = []
            for idx, track in enumerate(self.all_tracks):
                num_lost_track_times = self.processed_scan_idx - track.last_update_scan_idx
                if num_lost_track_times > self.num_lost_track_times_thresh:
                    idx_to_delete.append(idx)

            # delete in descending order so that it does not get messed up
            for idx in sorted(idx_to_delete, reverse=True):
                del self.all_tracks[idx]
                del self.all_objects[idx]
            self.deleting_tracks_time = np.append(self.deleting_tracks_time, [time.time() - start_time])
            print("\n************")
            print("The instant track deletion time is {:.7f}".format(self.deleting_tracks_time[-1]))
            print("The average track deletion time is {:.7f}".format(self.deleting_tracks_time.mean()))
            print("************\n")

            start_time = time.time()
            extracted_instances_xyz, instance_global_ids = generate_publish_instance_cloud(self,
                current_raw_timestamp)
            self.accumulated_cloud_time = np.append(self.accumulated_cloud_time, [time.time() - start_time])
            print("\n************")
            print("The instant accumulated cloud time is {:.7f}".format(self.accumulated_cloud_time[-1]))
            print("The average accumulated cloud time is {:.7f}".format(self.accumulated_cloud_time.mean()))
            print("************\n")

            if extracted_instances_xyz is not None:
                start_time = time.time()
                cuboids = cuboid_detection(self, 
                    extracted_instances_xyz, current_raw_timestamp, instance_global_ids)
                
                if len(cuboids) > 0:

                    if (self.prev_acc_obj_pub_time is not None) and ((rospy.Time.now() - self.prev_acc_obj_pub_time).to_sec() < 1.0/self.desired_acc_obj_pub_rate):
                        pass
                    else:
                        print ("publishing range-bearing measurements at desired rate of ", self.desired_acc_obj_pub_rate, " Hz")
                        self.prev_acc_obj_pub_time = rospy.Time.now()
                        publish_range_bearing_measurements(self, 
                            copy.deepcopy(cuboids), current_raw_timestamp, self.odom_from_cloud_msg)
                        
                self.final_cuboid_detection_time = np.append(self.final_cuboid_detection_time, [time.time() - start_time])
                print("\n************")
                print("The instant final cuboid formation time is {:.7f}".format(self.final_cuboid_detection_time[-1]))
                print("The average final cuboid formation time is {:.7f}".format(self.final_cuboid_detection_time.mean()))
                print("************\n")
    
        self.callback_time = np.append(self.callback_time, [time.time() - cb_start_time])
        print("\n************")
        print("The TOTAL CALLBACK time is {:.7f}".format(self.callback_time[-1]))
        print("The average TOTAL CALLBACK time is {:.7f}".format(self.callback_time.mean()))
        print("************\n")


    # TODO add the ground fitting from the stats branch

    def odom_callback(self, msg):
        send_tfs(self, msg)
        if self.odom_received == False:
            self.odom_received = True

if __name__ == '__main__':

    rospy.init_node("process_cloud_node")
    r = rospy.Rate(30)
    my_node = ProcessCloudNode()
    while not rospy.is_shutdown():
        print("node started!")
        rospy.spin()
    # r.sleep()
    print("node killed!")
