#! /usr/bin/env python3
# title			:
# description	:
# author		:Yuezhan Tao and Xu Liu

import rospy
import numpy as np
# import rviz marker
from visualization_msgs.msg import Marker, MarkerArray
import time 
from sklearn.cluster import DBSCAN
import copy
# import Point
from geometry_msgs.msg import Point
import matplotlib.colors as mcolors
# important rotation R from scipy
from scipy.spatial.transform import Rotation as R
class LoopClosureSubmapNode(object):
    def __init__(self):
        # initialize the node

        # initialize random seed for numpy
        np.random.seed(0)

        # record the history robot positions
        self.robot_history_positions_with_inds_array = None


        # multi floor information
        self.lowest_reached_altitude = None
        self.highest_reached_altitude = None
        self.num_floors = None

        ###################################### MOST IMPORANT PARAMETERS ######################################
        # assumed floor height (should be similar to the actual floor height)
        self.floor_height = 3.5
        # dilation of floor height (so that even if the robot drifted the landmark and robot pose can be considered to be on the right floor)
        self.floor_height_dilation = 0.5 # up and down, each side

        # most important DBSCAN parameters
        self.cluster_distance = 3.5 # eps
        # number of landmarks to be regarded as a candidate loop closure cluster
        self.cluster_min_samples = 4
        # For the following two params, do not worry about landmarks from different floor, they should already be filtered out
        # max distance (in XYZ) between robot key pose and loop closure candidate clustould be similar to sensor perception range
        self.max_key_pose_cluster_distance = 7.0
        # min_key_pose_to_landmark: minimum distance (in XYZ) from the sampled loop closure key pose to ANY landmark in the cluster 
        self.min_key_pose_to_landmark = 1.5
        # additional distance from closest robot key pose so as to find a better (older) key pose
        self.allowed_distance_to_closest_robot_key_pose = 3.0
        #################################### MOST IMPORANT PARAMETERS ENDS ###################################

        # create random but distinct color options for loop closure submaps within 0-1 range
        colors = ["red", "green", "blue", "yellow", "orange", "purple", "pink", "brown", "gray", "olive", "cyan", "magenta", "lime", "teal", "lavender", "maroon", "navy", "black", "white"]
        color_lookup_table = mcolors.CSS4_COLORS
        self.loop_closure_submap_colors = []
        for cur_color in colors:
            self.loop_closure_submap_colors.append(mcolors.hex2color(color_lookup_table[cur_color]))
        
        # reference frame
        self.reference_frame = rospy.get_param('~reference_frame', "quadrotor/map")

        # number of cpu threads for clustering
        self.cpu_thread_clutering = 2


        # subscribe to robot trajectory
        self.robot_trajectory_sub = rospy.Subscriber(
            "/factor_graph_atl/optimized_trajectory_with_pose_inds", MarkerArray, callback=self.robot_trajectory_cb, queue_size=1)

        # subscribe to the semantic map
        self.semantic_map_sub = rospy.Subscriber("/factor_graph_atl/optimized_point_landmarks", MarkerArray, callback=self.semantic_map_cb, queue_size=1)

        # publish the loop closure submap as a MarkerArray
        self.loop_closure_submap_pub = rospy.Publisher("/factor_graph_atl/loop_closure_submap", MarkerArray, queue_size=1)

    def robot_trajectory_cb(self, msg):
        # the input is a MarkerArray of point-line markers, parse it to get the points that represent the history robot positions 
        all_markers = msg.markers
        print("number of raw points on the latest robot trajectory is: ", len(all_markers))
        # add each raw point to the set of robot history positions as a tuple (x, y, z)
        
        # robot_history_positions_array will be Nx4, where N is number of poses (in msg.markers), and 4 is (x, y, z, key_pose_ind)
        self.robot_history_positions_with_inds_array = np.zeros((len(all_markers), 4))

        # print("number of robot history positions before adding current trajectory is: ", len(self.robot_history_positions))
        # time this operation 
        start_time = time.time()

        # iterate through all markers, extract the position and key_pose_ind
        for i in np.arange(len(all_markers)):
            cur_marker = all_markers[i]
            # each marker is a RVIZ SPHERE marker, where the position is the robot position, and the id is the key pose index
            cur_position = cur_marker.pose.position
            cur_key_pose_ind = cur_marker.id
            # add the current position to the robot history positions
            self.robot_history_positions_with_inds_array[i, 0] = cur_position.x
            self.robot_history_positions_with_inds_array[i, 1] = cur_position.y
            self.robot_history_positions_with_inds_array[i, 2] = cur_position.z
            self.robot_history_positions_with_inds_array[i, 3] = cur_key_pose_ind

        
        # update the multi floor information, estimate the number of floors in the environment
        self.lowest_reached_altitude = np.min(self.robot_history_positions_with_inds_array[:, 2])
        self.lowest_reached_altitude = min(self.lowest_reached_altitude, 0.0) # floor should be lower than or equal to 0
        self.highest_reached_altitude = np.max(self.robot_history_positions_with_inds_array[:, 2])
        self.num_floors = 1.0 + np.floor((self.highest_reached_altitude - self.lowest_reached_altitude) / self.floor_height)
        print("lowest reached altitude is: ", self.lowest_reached_altitude)
        print("highest reached altitude is: ", self.highest_reached_altitude)
        print("estimated number of floors in the environment is: ", self.num_floors)


        # timer stuff
        end_time = time.time()
        time_elapsed = end_time - start_time
        # print time elapsed with 5 decimal places
        print("time to add current trajectory to robot history positions is: {:.5f} (seconds)".format(time_elapsed))
        print("number of robot history positions AFTER adding current trajectory is: ", self.robot_history_positions_with_inds_array.shape[0])
        
    
    def semantic_map_cb(self, msg):
        # the input is a MarkerArray of cylinder markers, where the center of each cylinder is a landmark position
        # parse the marker array to get the landmark positions
        raw_landmark_positions = set()
        for marker in msg.markers:
            raw_landmark_positions.add((marker.pose.position.x, marker.pose.position.y, marker.pose.position.z))
        print("number of raw landmark positions is: ", len(raw_landmark_positions))
        # cluster the landmark positions
        self.cluster_multi_floor_landmarks(raw_landmark_positions)


    def cluster_multi_floor_landmarks(self, landmark_positions):
        # check if the robot history positions is empty
        if self.robot_history_positions_with_inds_array is None:
            print("robot history positions is empty!")
            return
        elif len(landmark_positions) == 0:
            print("landmark positions is empty!")
            return
        else:
            # convert landmark_positions into a numpy array
            landmark_positions_array = np.array(list(landmark_positions))

            # separate landmarks into different floors
            for floor in range(int(self.num_floors)):
                floor_height_range = [self.lowest_reached_altitude + floor * self.floor_height, self.lowest_reached_altitude + (floor + 1) * self.floor_height]
                # dilate the floor height range
                floor_height_range[0] -= self.floor_height_dilation
                floor_height_range[1] += self.floor_height_dilation
                # get all landmarks in the current floor
                landmarks_in_current_floor = landmark_positions_array[(landmark_positions_array[:, 2] >= floor_height_range[0]) & (landmark_positions_array[:, 2] < floor_height_range[1])]
                # get all robot poses in the current floor
                robot_poses_with_inds_in_current_floor = self.robot_history_positions_with_inds_array[(self.robot_history_positions_with_inds_array[:, 2] >= floor_height_range[0]) & (self.robot_history_positions_with_inds_array[:, 2] < floor_height_range[1])]
                # print the number of landmarks in the current floor
                print("number of landmarks in floor {} is: ".format(floor), len(landmarks_in_current_floor))
                # print the number of robot poses in the current floor
                print("number of robot poses in floor {} is: ".format(floor), len(robot_poses_with_inds_in_current_floor))
                # cluster the landmarks in the current floor using X and Y positions
                loop_closure_candidate_clusters = self.cluster_landmarks_in_floor(landmarks_in_current_floor)
                key_pose_submap_pairs = self.assemble_key_pose_submap_pairs(loop_closure_candidate_clusters, robot_poses_with_inds_in_current_floor)
                # print key_pose_submap_pairs size

                self.publish_results(key_pose_submap_pairs)

    def cluster_landmarks_in_floor(self, landmarks_in_current_floor):
        if len(landmarks_in_current_floor) == 0:
            print("landmarks_in_current_floor is empty!")
            return None
        loop_closure_candidate_clusters = []
        # ref[https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html#sklearn.cluster.DBSCAN]
        xy_positions_in_current_floor = landmarks_in_current_floor[:, 0:2]
        object_clusters = DBSCAN(eps=self.cluster_distance, min_samples=self.cluster_min_samples, metric='euclidean', n_jobs=self.cpu_thread_clutering).fit(xy_positions_in_current_floor)
        labels = object_clusters.labels_
        # get unique labels
        unique_labels = set(labels)
        # get each cluster
        for label in unique_labels:
            if label == -1:
                # label -1 means this is noise
                continue
            else:
                # get all landmarks in this cluster
                landmarks_in_current_cluster = landmarks_in_current_floor[labels == label]
                if len(landmarks_in_current_cluster) >= self.cluster_min_samples:
                    # print the number of landmarks in this cluster
                    print("number of landmarks in current loop closure candidate is: ", len(landmarks_in_current_cluster))
                    # add this cluster to the list of loop closure candidate clusters
                    loop_closure_candidate_clusters.append(landmarks_in_current_cluster)
        # print the number of loop closure candidate clusters
        print("number of loop closure candidate clusters is: ", len(loop_closure_candidate_clusters))
        return copy.deepcopy(loop_closure_candidate_clusters)



    def assemble_key_pose_submap_pairs(self, loop_closure_candidate_clusters, robot_poses_with_inds_in_current_floor):
        if (loop_closure_candidate_clusters == None) or (len(loop_closure_candidate_clusters) == 0):
            print("loop_closure_candidate_clusters is empty!")
            return None
        if len(robot_poses_with_inds_in_current_floor) == 0:
            print("robot_poses_with_inds_in_current_floor is empty!")
            return None
        # list of key poses and clusters, each element of the list is another list, where the first element is the key pose XYZ positions, and the second element is the landmarks XYZ positions in the cluster 
        key_pose_submap_pairs = []
        # find the key pose for each loop closure candidate cluster in the current floor
        # the distance between the chosen key pose and the cluster center should be the smallest and smaller than max_key_pose_cluster_distance
        for cluster in loop_closure_candidate_clusters:
            # center of the cluster
            cluster_center = np.mean(cluster, axis=0)
            # distances to the cluster center
            distances = (np.linalg.norm(robot_poses_with_inds_in_current_floor[:, 0:3] - cluster_center, axis=1))
            # closest robot pose to the cluster center 
            # closest_robot_pose_with_index = robot_poses_with_inds_in_current_floor[np.argmin(xy_distances)]

            # first threshold out the robot poses that are too far away from the cluster center using self.max_key_pose_cluster_distance
            candidate_robot_poses_with_index = robot_poses_with_inds_in_current_floor[distances <= self.max_key_pose_cluster_distance]

            # check and select the poses in candidate_robot_poses_with_index that has more than min_key_pose_to_landmark distance to ANY landmark in the cluster 
            for cluster_landmark in cluster:
                # distance between the candidate robot poses and the current cluster landmark
                candidate_robot_poses_to_landmark_distances = np.linalg.norm(candidate_robot_poses_with_index[:, 0:3] - cluster_landmark, axis=1)
                # threshold out the candidate robot poses that are too close to the current cluster landmark
                candidate_robot_poses_with_index = candidate_robot_poses_with_index[candidate_robot_poses_to_landmark_distances >= self.min_key_pose_to_landmark]
                # check if there are still candidate robot poses left
                if len(candidate_robot_poses_with_index) == 0:
                    print("ERROR: no candidate robot poses left! discard this cluster! unless the robot has not moved much, this should be avoided by decreasing min_key_pose_to_landmark!")
                    print("ERROR: no candidate robot poses left! discard this cluster! unless the robot has not moved much, this should be avoided by decreasing min_key_pose_to_landmark!")
                    break
            
            if len(candidate_robot_poses_with_index) != 0:
                print("number of candidate robot poses satisfying not too close and not too far rules is: ", len(candidate_robot_poses_with_index))
                # find the closest robot pose to the cluster center (cluster_center) among the candidate robot poses
                # distances to the cluster center
                distances_candidate = (np.linalg.norm(candidate_robot_poses_with_index[:, 0:3] - cluster_center, axis=1))
                closest_robot_pose_with_index = candidate_robot_poses_with_index[np.argmin(distances_candidate)]

                try_older_pose_as_key_pose = True
                if try_older_pose_as_key_pose: 
                    # for better information gain, try to find an older robot pose 
                    # refine step: find the oldest robot pose among candidate_robot_poses_with_index with no more than self.allowed_distance_to_closest_robot_key_pose from closest_robot_pose_with_index
                    # distances to the closest robot pose
                    distances_to_closest = (np.linalg.norm(candidate_robot_poses_with_index[:, 0:3] - closest_robot_pose_with_index[0:3], axis=1))
                    # find the oldest robot pose among candidate_robot_poses_with_index with no more than self.allowed_distance_to_closest_robot_key_pose from closest_robot_pose_with_index
                    final_candidate_robot_pose_with_index = candidate_robot_poses_with_index[distances_to_closest <= self.allowed_distance_to_closest_robot_key_pose]
                    if len(final_candidate_robot_pose_with_index) != 0:
                        # find the oldest pose (i.e. the one with smallest index)
                        best_robot_pose_with_index = final_candidate_robot_pose_with_index[np.argmin(final_candidate_robot_pose_with_index[:, 3])]
                        print("found a better (older) key pose with index: ", best_robot_pose_with_index[3])
                    else:
                        best_robot_pose_with_index = closest_robot_pose_with_index
                        print("Error: this should not happen! no final_candidate_robot_pose_with_index found! check if self.allowed_distance_to_closest_robot_key_pose is set as negative!")
                else:
                    best_robot_pose_with_index = closest_robot_pose_with_index

                
                # add the key pose and the cluster to the list of key pose submap pairs
                key_pose_submap_pairs.append([best_robot_pose_with_index, cluster])
                

                
        # print the number of key pose submap pairs
        print("number of key pose submap pairs is: ", len(key_pose_submap_pairs))
        return copy.deepcopy(key_pose_submap_pairs)

    def publish_results(self, key_pose_submap_pairs_input):
        if key_pose_submap_pairs_input is None:
            print("no valid key pose submap pairs to publish!")
            return
        
        # sort key_pose_submap_pairs_input by the distance of the key poses to the origin (according to X and Y positions only)
        key_pose_submap_pairs = sorted(key_pose_submap_pairs_input, key=lambda x: np.linalg.norm(x[0][0:2]))
        
        marker_array_msg = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.ns = "delete"
        marker.action = Marker.DELETEALL
        marker_array_msg.markers.append(marker)
        self.loop_closure_submap_pub.publish(marker_array_msg)

        # create colors for each cluster
        num_clusters = len(key_pose_submap_pairs)

        # publish each key pose - submap pair as RVIZ markers
        marker_array = MarkerArray()
        for i in range(num_clusters):
            # print the key pose and submap of the current cluster
            print("key pose of index {} is: ".format(key_pose_submap_pairs[i][0][3]), key_pose_submap_pairs[i][0])
            # key pose
            cur_key_pose_with_id = key_pose_submap_pairs[i][0]
            # submap
            submap = key_pose_submap_pairs[i][1]
            # create an arrow marker for the key pose
            key_pose_marker = Marker()
            key_pose_marker.header.frame_id = self.reference_frame
            key_pose_marker.header.stamp = rospy.Time.now()
            # get the key pose index associated with this cluster
            key_pose_index = int(cur_key_pose_with_id[3])
            key_pose_marker.ns = "key_pose_" + str(key_pose_index)
            key_pose_marker.id = key_pose_index
            key_pose_marker.type = Marker.ARROW
            key_pose_marker.action = Marker.ADD
            # point along z axis of length 1 meter, bottom at key_pose
            key_pose_marker.points.append(Point(cur_key_pose_with_id[0], cur_key_pose_with_id[1], cur_key_pose_with_id[2]))
            key_pose_marker.points.append(Point(cur_key_pose_with_id[0], cur_key_pose_with_id[1], cur_key_pose_with_id[2] + 2.0))
            key_pose_marker.scale.x = 0.2
            key_pose_marker.scale.y = 0.4
            key_pose_marker.scale.z = 0.0

            key_pose_marker.color.a = 1.0

            color_id = i % len(self.loop_closure_submap_colors)
            key_pose_marker.color.r = self.loop_closure_submap_colors[color_id][0]
            key_pose_marker.color.g = self.loop_closure_submap_colors[color_id][1]
            key_pose_marker.color.b = self.loop_closure_submap_colors[color_id][2]
            
            marker_array.markers.append(key_pose_marker)


            # create a cube marker for centroid of all landmarks in the submap
            centroid_marker = Marker()
            centroid_marker.header.frame_id = self.reference_frame
            centroid_marker.header.stamp = rospy.Time.now()
            centroid_marker.ns = "submap_centroid_" + str(key_pose_index)
            centroid_marker.id = 0
            centroid_marker.type = Marker.CUBE
            centroid_marker.action = Marker.ADD
            centroid_marker.pose.position.x = submap[:,0].mean()
            centroid_marker.pose.position.y = submap[:,1].mean()
            centroid_marker.pose.position.z = submap[:,2].mean()
            centroid_marker.pose.orientation.x = 0.0
            centroid_marker.pose.orientation.y = 0.0
            centroid_marker.pose.orientation.z = 0.0
            centroid_marker.pose.orientation.w = 1.0
            centroid_marker.scale.x = 0.1
            centroid_marker.scale.y = 0.1
            centroid_marker.scale.z = 0.1
            centroid_marker.color.a = 0.0
            centroid_marker.color.r = self.loop_closure_submap_colors[color_id][0]
            centroid_marker.color.g = self.loop_closure_submap_colors[color_id][1]
            centroid_marker.color.b = self.loop_closure_submap_colors[color_id][2]
            marker_array.markers.append(centroid_marker)

            # create a sphere marker for each landmark in the submap
            for j in range(len(submap)):
                landmark = submap[j]
                landmark_marker = Marker()
                landmark_marker.header.frame_id = self.reference_frame
                landmark_marker.header.stamp = rospy.Time.now()
                landmark_marker.ns = "submap_landmarks_" + str(key_pose_index)
                landmark_marker.id = j
                # landmark_marker.type = Marker.SPHERE
                # use the chair model at file:///home/sam/Desktop/Chair.dae
                landmark_marker.type = Marker.MESH_RESOURCE
                landmark_marker.mesh_resource = "package://sloam/resource/chair.stl"
                
                landmark_marker.action = Marker.ADD
                landmark_marker.pose.position.x = landmark[0]
                landmark_marker.pose.position.y = landmark[1]
                landmark_marker.pose.position.z = landmark[2]
                # landmark_marker.pose.orientation.x = 0.0
                # landmark_marker.pose.orientation.y = 0.0
                # landmark_marker.pose.orientation.z = 0.0
                # landmark_marker.pose.orientation.w = 1.0
                # assign yaw according to x position, every 1 meter of x add 20 degree of yaw
                yaw = landmark[0] * 0.34906585               
                # limit yaw to be within 0-2pi
                yaw = yaw % (2 * np.pi)
                rot = R.from_euler('z', yaw).as_quat()
                landmark_marker.pose.orientation.x = rot[0]
                landmark_marker.pose.orientation.y = rot[1]
                landmark_marker.pose.orientation.z = rot[2]
                landmark_marker.pose.orientation.w = rot[3]
                scale_factor = 1 #0.001 for detailed
                landmark_marker.scale.x = scale_factor
                landmark_marker.scale.y = scale_factor
                landmark_marker.scale.z = scale_factor
                landmark_marker.color.a = 0.2
                landmark_marker.color.r = self.loop_closure_submap_colors[color_id][0]
                landmark_marker.color.g = self.loop_closure_submap_colors[color_id][1]
                landmark_marker.color.b = self.loop_closure_submap_colors[color_id][2]

                marker_array.markers.append(landmark_marker)

        

            
        self.loop_closure_submap_pub.publish(marker_array)

            

if __name__ == '__main__':

    rospy.init_node('loop_closure_submap_node')
    r = rospy.Rate(30)
    my_node = LoopClosureSubmapNode()
    while not rospy.is_shutdown():
        print("node started!")
        rospy.spin()
    # r.sleep()
    print("node killed!")
