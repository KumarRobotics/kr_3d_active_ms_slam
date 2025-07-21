#! /usr/bin/env python3
# title			:
# description	:
# author		:Xu Liu  and Ankit Prabhu

#!/usr/bin/env python
import rospy
from ros_numpy.point_cloud2 import fields_to_dtype, get_xyz_points, DUMMY_FIELD_PREFIX
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from scipy.spatial.transform import Rotation as R
import tf
from sklearn.decomposition import PCA
import open3d as o3d
import copy
from sklearn.cluster import DBSCAN

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, UInt64
from sloam_msgs.msg import ROSRangeBearing
from geometry_msgs.msg import Point



def generate_publish_instance_cloud(process_cloud_node_object, timestamp):
    instances_xyz = []
    global_track_ids = []
    current_label = -1
    safe_to_publish = False
    time_taken = rospy.Time.now().to_sec() - process_cloud_node_object.last_stamp
    # print("time_taken is ", time_taken)
    process_cloud_node_object.last_stamp = rospy.Time.now().to_sec()

    if process_cloud_node_object.car_inst_visualize:

        if len(process_cloud_node_object.raw_cloud_dict.keys()) > 0:

            for track_inst in process_cloud_node_object.raw_cloud_dict.values():

                if track_inst[0] > process_cloud_node_object.tracker_age_thresh_lower:

                    current_label += 1
                    current_xyz = track_inst[1]
                    current_i = np.ones(
                        (track_inst[1].shape[0], 1)) * current_label
                    current_xyzi = np.hstack((current_xyz, current_i))
                    instances_xyz.append(current_xyz)
                    global_track_ids.append(track.track_idx)

                    if current_label == 0:

                        valid_points_labels = current_xyzi
                        safe_to_publish = True

                    else:

                        valid_points_labels = np.vstack(
                            (valid_points_labels, current_xyzi))

        else:
            print(
                "no valid car cluster found in current accumulated point cloud!")
            return None, None

    else:

        if len(process_cloud_node_object.all_tracks) > 0:
            for track in process_cloud_node_object.all_tracks:
                if track.age > process_cloud_node_object.tracker_age_thresh_lower:
                    current_label += 1
                    current_xyz = track.all_raw_points
                    current_i = np.ones(
                        (track.all_raw_points.shape[0], 1)) * current_label
                    current_xyzi = np.hstack((current_xyz, current_i))
                    instances_xyz.append(current_xyz)
                    global_track_ids.append(track.track_idx)
                    
                    if current_label == 0:
                        valid_points_labels = current_xyzi
                        safe_to_publish = True
                    else:
                        valid_points_labels = np.vstack(
                            (valid_points_labels, current_xyzi))
                    

        else:
            print(
                "no valid car cluster found in current accumulated point cloud!")
            return None, None

        # publish
    if safe_to_publish:
        pc_msg_2 = PointCloud2()
        # define our own header to publish in the world frame
        header = Header()
        header.stamp = timestamp  # rospy.Time.now()
        header.frame_id = process_cloud_node_object.reference_frame
        pc_msg_2.header = header
        pc_msg_2.width = valid_points_labels.shape[0]
        pc_msg_2.height = 1
        pc_msg_2.point_step = 16
        pc_msg_2.row_step = pc_msg_2.width * pc_msg_2.point_step
        pc_msg_2.fields = process_cloud_node_object.pc_fields_
        full_data_2 = valid_points_labels.astype(np.float32)
        pc_msg_2.data = full_data_2.tobytes()
        print('accumulated semantic segmentation point clouds published!')
        process_cloud_node_object.instance_cloud_pub.publish(pc_msg_2)
        instances_xyz_copied = copy.deepcopy(instances_xyz)
        global_track_ids_copied = copy.deepcopy(global_track_ids)
        return instances_xyz_copied, global_track_ids_copied
    else:
        return None, None



def cuboid_detection(process_cloud_node_object, instances_xyz, current_raw_timestamp, instance_global_ids, use_convex=False):
    # TODO: project to ground plane, instead of project to XY plane
    cuboids = []
    assert len(instances_xyz) == len(instance_global_ids)
    for instance_id, instance_xyz in enumerate(instances_xyz):
        pca = PCA(n_components=2)
        # threshold the number of points to be used for PCA to avoid degenerate cases
        if use_convex and instance_xyz[:, :2].shape[0] > 10: 
            # TODO: this has runtime error: RuntimeError: QH6114 qhull precision error: initial simplex is not convex. Distance=-9.2e-15
            o_pcd = o3d.geometry.PointCloud()
            o_pcd.points = o3d.utility.Vector3dVector(instance_xyz)
            hull_points = o_pcd.compute_convex_hull()[1]
            instance_hull_points = instance_xyz[hull_points, :]
            pca.fit(instance_hull_points[:, :2])
        else:
            pca.fit(instance_xyz[:, :2])
        components = pca.components_
        x = np.array([components[0, 0], components[0, 1], 0])
        x = x / np.linalg.norm(x)
        z = np.array([0, 0, 1])
        y = np.cross(z, x)
        y = y / np.linalg.norm(y)
        yaw = np.arctan2(x[1], x[0])
        raw_yaw = yaw

        if yaw < 0:
            yaw+=np.pi
        if yaw == np.pi:
            yaw-=np.pi
        
        # rotation from PCA frame to world frame -- will be used to find centroid as well
        r_pca_world_raw =  R.from_rotvec(raw_yaw * z)

        x_projections = instance_xyz @ x
        length = np.percentile(x_projections, 99) - np.percentile(x_projections,1)          
        y_projections = instance_xyz @ y
        width = np.percentile(y_projections, 95) - np.percentile(y_projections,5)   
        z_projections = instance_xyz @ z
        height = np.percentile(z_projections, 95) - np.percentile(z_projections,5)   

        # estimate the heading direction of the car
        able_to_estimate_facing_dir = False
        if able_to_estimate_facing_dir: 
            rear_cut_off = np.percentile(x_projections, 5) # 5 percentile
            front_cut_off = np.percentile(x_projections, 95) # 95 percentile
            idx_rear = x_projections <= rear_cut_off
            idx_front = x_projections >= front_cut_off

            front_part_height = z_projections[idx_front]
            rear_part_height =  z_projections[idx_rear]
            # highest point, take 75 percentile to be robust to the noise
            front_median_height = np.percentile(front_part_height, 70)
            rear_median_height =  np.percentile(rear_part_height, 70)


        # Calculate the centroid found based on body frame coords
        x_centroid_pca = 0.5*(np.percentile(x_projections, 99) + np.percentile(x_projections,1))
        y_centroid_pca = 0.5*(np.percentile(y_projections, 95) + np.percentile(y_projections,5))
        z_centroid_pca = 0.5*(np.percentile(z_projections, 95) + np.percentile(z_projections,5))
        centroid_pca = np.array([x_centroid_pca, y_centroid_pca, z_centroid_pca])
        centroid_world = r_pca_world_raw .as_matrix() @ centroid_pca


        flag = False
        flag = (width > process_cloud_node_object.cuboid_width_cutoff[0] and width < process_cloud_node_object.cuboid_width_cutoff[1] and length > process_cloud_node_object.cuboid_length_cutoff[0] and length < process_cloud_node_object.cuboid_length_cutoff[1] and height > process_cloud_node_object.cuboid_height_cutoff[0] and height < process_cloud_node_object.cuboid_height_cutoff[1])
        if flag:
            if able_to_estimate_facing_dir: 
                if  rear_median_height < front_median_height:
                    # flip the orientation since front should have less points than back
                    yaw = yaw+np.pi
                                
            r_pca_world = R.from_rotvec(yaw * z)
            quaternion = r_pca_world.as_quat()

            current_cube = {}
            current_cube['dimensions'] = np.array([length, width, height])
            current_cube['centroid'] = centroid_world
            current_cube['orientation'] = quaternion
            current_cube['global_id'] = instance_global_ids[instance_id]
            # TODO: only add and publish cuboids observed in current frame
            cuboids.append(current_cube)
    return(cuboids)



def fit_cuboid(fit_cuboid_length_thresh, fit_cuboid_width_thresh, points_world_xyzi_id_conf_depth_cars, depth_percentile, confidence_threshold, epsilon_scan, min_samples_scan):
    cloud_mat_3d = points_world_xyzi_id_conf_depth_cars[:,:3]
    instance_ids = points_world_xyzi_id_conf_depth_cars[:,4]
    confidence_values = points_world_xyzi_id_conf_depth_cars[:,5]
    depth_values = points_world_xyzi_id_conf_depth_cars[:,6]
    unique_labels = np.unique(instance_ids)
    xcs = []
    ycs = []
    lengths = []
    widths = []
    raw_points = []
    for k in unique_labels:
        if k == 0:
            continue  # background points or other instances
        class_member_mask = (instance_ids == k)
        if np.sum(class_member_mask) >0:
            cur_depth_values = depth_values[class_member_mask]
            depth_thres_low = np.percentile(cur_depth_values, depth_percentile[0])
            depth_thres_high = np.percentile(cur_depth_values, depth_percentile[1])
            print(depth_thres_low, depth_thres_high)
            confidence = np.median(confidence_values[class_member_mask])
            if confidence > confidence_threshold:
                valid_pts_mask = np.logical_and((cur_depth_values > depth_thres_low), (cur_depth_values < depth_thres_high))
                if np.sum(valid_pts_mask) >0:
                    xyzs_instance = cloud_mat_3d[class_member_mask, :]
                    xyzs = xyzs_instance[valid_pts_mask, :]
                    
                    if True:
                        xyzs_valid = xyzs
                        xs = xyzs_valid[:, 0]
                        ys = xyzs_valid[:, 1]
                        x_max = np.max(xs)
                        y_max = np.max(ys)
                        x_min = np.min(xs)
                        y_min = np.min(ys)
                        xc = 0.5*(x_max + x_min)
                        yc = 0.5*(y_max + y_min)
                        length = x_max - x_min
                        width = y_max - y_min
                        if length > fit_cuboid_length_thresh and width > fit_cuboid_width_thresh:
                            xcs.append(xc)
                            ycs.append(yc)
                            lengths.append(length)
                            widths.append(width)
                            raw_points.append(xyzs_valid)
                            
    assert(len(xcs) == len(ycs) == len(lengths) == len(widths) == len(raw_points))
    return xcs, ycs, lengths, widths, raw_points

def cluster(xyzi, epsilon, min_samples, use_2d, cpu_thread_clutering_ = 2):
    # reduce to 2d
    if use_2d:
        cloud = xyzi[:, :2]
    else:
        cloud = xyzi[:, :3]
    object_clusters = DBSCAN(eps=epsilon, min_samples=min_samples, metric='euclidean', metric_params=None,
                                algorithm='auto', leaf_size=30, p=None, n_jobs=cpu_thread_clutering_).fit(cloud)
    labels = object_clusters.labels_

    return labels

def publish_cuboid_markers(process_cloud_node_object, cuboids, current_raw_timestamp):
    car_markers = MarkerArray()
    car_markers.markers = []
    car_markers_body = MarkerArray()
    car_markers_body.markers = []
    stamp = current_raw_timestamp

    # Used for deleting markers which are no longer used
    if not process_cloud_node_object.car_inst_visualize:
        marker_del_arr = MarkerArray()
        marker_del = Marker()
        marker_del.header.frame_id = process_cloud_node_object.reference_frame
        marker_del.header.stamp = stamp
        marker_del.action = Marker.DELETEALL
        marker_del_arr.markers.append(marker_del)
        process_cloud_node_object.cuboid_marker_pub.publish(marker_del_arr)

    H_body_world = np.zeros((4, 4), dtype=np.float32)
    try:
        # transform data in the source_frame into the target_frame
        (t_body_world, quat_body_world) = process_cloud_node_object.tf_listener2.lookupTransform(
            process_cloud_node_object.reference_frame, process_cloud_node_object.range_image_frame, current_raw_timestamp)
        r_body_world = R.from_quat(quat_body_world)
        H_body_world_rot = r_body_world.as_matrix()
        H_body_world_trans = np.array(t_body_world)
        H_body_world[:3, :3] = H_body_world_rot
        H_body_world[:3, 3] = H_body_world_trans
        # body to world transformation
        H_body_world[3, 3] = 1

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("\n cannot find tf1 from " + process_cloud_node_object.range_image_frame +
                " to " + process_cloud_node_object.reference_frame + "!!! \n")
        return

    for idx, cuboid in enumerate(cuboids):
        marker = Marker()
        marker_body = Marker()
        marker.header.frame_id = process_cloud_node_object.reference_frame
        marker.header.stamp = stamp
        marker.ns = "car"
        marker.id = idx
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker_body = Marker()
        marker_body.header.frame_id = process_cloud_node_object.range_image_frame
        marker_body.header.stamp = stamp
        marker_body.ns = "car_body"
        marker_body.id = idx
        marker_body.type = Marker.CUBE
        marker_body.action = Marker.ADD

        cuboid_center_homo = np.ones((4,), dtype=float)
        cuboid_center_homo[:3] = cuboid['centroid']
        cuboid_center_body_homo = np.linalg.pinv(
            H_body_world) @ cuboid_center_homo
        cuboid_center_body = cuboid_center_body_homo[:3] / \
            cuboid_center_body_homo[3]

        marker.pose.position.x = cuboid['centroid'][0]
        marker.pose.position.y = cuboid['centroid'][1]
        marker.pose.position.z = cuboid['centroid'][2]

        marker_body.pose.position.x = cuboid_center_body[0]
        marker_body.pose.position.y = cuboid_center_body[1]
        marker_body.pose.position.z = cuboid_center_body[2]

        marker.scale.x = cuboid['dimensions'][0]
        marker.scale.y = cuboid['dimensions'][1]
        marker.scale.z = cuboid['dimensions'][2]
        marker_body.scale.x = cuboid['dimensions'][0] + 0.1
        marker_body.scale.y = cuboid['dimensions'][1]
        marker_body.scale.z = cuboid['dimensions'][2]

        marker.pose.orientation.x = cuboid['orientation'][0]
        marker.pose.orientation.y = cuboid['orientation'][1]
        marker.pose.orientation.z = cuboid['orientation'][2]
        marker.pose.orientation.w = cuboid['orientation'][3]

        r_cube_world = R.from_quat(cuboid['orientation'])
        H_cube_world_rot = r_cube_world.as_matrix()
        H_cube_body_rot = H_body_world_rot.T @ H_cube_world_rot
        quat_body_cube = R.from_matrix(H_cube_body_rot).as_quat()
        marker_body.pose.orientation.x = quat_body_cube[0]
        marker_body.pose.orientation.y = quat_body_cube[1]
        marker_body.pose.orientation.z = quat_body_cube[2]
        marker_body.pose.orientation.w = quat_body_cube[3]

        marker.color.a = 0.5  # alpha = 1 means not transparent at all
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        car_markers.markers.append(marker)

        marker_body.color.a = 0.7  # alpha = 1 means not transparent at all
        marker_body.color.r = 0.0
        marker_body.color.g = 0.0
        marker_body.color.b = 1.0
        car_markers_body.markers.append(marker_body)

    process_cloud_node_object.cuboid_marker_pub.publish(car_markers)
    process_cloud_node_object.cuboid_marker_body_pub.publish(car_markers_body)





def publish_range_bearing_measurements(process_cloud_node_object, cuboids, current_raw_timestamp, synced_odom):
    car_markers = MarkerArray()
    car_markers.markers = []
    car_markers_body = MarkerArray()
    car_markers_body.markers = []
    stamp = current_raw_timestamp


    bearing_measurements = []
    range_measurements = []
    landmark_body_positions = []
    meas_ids = []

    
    # Used for deleting markers which are no longer used
    if not process_cloud_node_object.car_inst_visualize:
        marker_del_arr = MarkerArray()
        marker_del = Marker()
        marker_del.header.frame_id = process_cloud_node_object.reference_frame
        marker_del.header.stamp = stamp
        marker_del.action = Marker.DELETEALL
        marker_del_arr.markers.append(marker_del)
        process_cloud_node_object.cuboid_marker_pub.publish(marker_del_arr)

    H_body_world = np.zeros((4, 4), dtype=np.float32)
    try:
        # transform data in the source_frame into the target_frame
        (t_body_world, quat_body_world) = process_cloud_node_object.tf_listener2.lookupTransform(
            process_cloud_node_object.reference_frame, process_cloud_node_object.range_image_frame, current_raw_timestamp)
        r_body_world = R.from_quat(quat_body_world)
        H_body_world_rot = r_body_world.as_matrix()
        H_body_world_trans = np.array(t_body_world)
        H_body_world[:3, :3] = H_body_world_rot
        H_body_world[:3, 3] = H_body_world_trans
        # body to world transformation
        H_body_world[3, 3] = 1
        # points_body_xyz_homo = (np.linalg.pinv(H_body_pano) @ points_pano_xyz_homo.T).T

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("\n cannot find tf1 from " + process_cloud_node_object.range_image_frame +
                " to " + process_cloud_node_object.reference_frame + "!!! \n")
        return

    for idx, cuboid in enumerate(cuboids):
        marker = Marker()
        marker_body = Marker()
        marker.header.frame_id = process_cloud_node_object.reference_frame
        marker.header.stamp = stamp
        marker.ns = "car"
        marker.id = idx
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker_body = Marker()
        marker_body.header.frame_id = process_cloud_node_object.range_image_frame
        marker_body.header.stamp = stamp
        marker_body.ns = "car_body"
        marker_body.id = idx
        marker_body.type = Marker.CUBE
        marker_body.action = Marker.ADD

        cuboid_center_homo = np.ones((4,), dtype=float)
        cuboid_center_homo[:3] = cuboid['centroid']
        cuboid_center_body_homo = np.linalg.pinv(
            H_body_world) @ cuboid_center_homo
        cuboid_center_body = cuboid_center_body_homo[:3] / \
            cuboid_center_body_homo[3]
        

        marker.pose.position.x = cuboid['centroid'][0]
        marker.pose.position.y = cuboid['centroid'][1]
        marker.pose.position.z = cuboid['centroid'][2]        


        cuboid_center =  cuboid['centroid']
        odom_position = H_body_world[:3, 3] 
        # VERY IMPORTANT: bearing vector should be expressed in body frame!!!!!!!!!!!!!!!!!!!
        cuboid_center_body_est = H_body_world[:3, :3].T@(cuboid_center - odom_position)
        print('(cuboid_center_body_est - cuboid_center_body) is: ', (cuboid_center_body_est - cuboid_center_body))
        assert(np.linalg.norm(cuboid_center_body_est - cuboid_center_body) < 1e-3)

        marker_body.pose.position.x = cuboid_center_body[0]
        marker_body.pose.position.y = cuboid_center_body[1]
        marker_body.pose.position.z = cuboid_center_body[2]

        marker.scale.x = cuboid['dimensions'][0]
        marker.scale.y = cuboid['dimensions'][1]
        marker.scale.z = cuboid['dimensions'][2]
        marker_body.scale.x = cuboid['dimensions'][0] + 0.1
        marker_body.scale.y = cuboid['dimensions'][1]
        marker_body.scale.z = cuboid['dimensions'][2]

        marker.pose.orientation.x = cuboid['orientation'][0]
        marker.pose.orientation.y = cuboid['orientation'][1]
        marker.pose.orientation.z = cuboid['orientation'][2]
        marker.pose.orientation.w = cuboid['orientation'][3]

        r_cube_world = R.from_quat(cuboid['orientation'])
        H_cube_world_rot = r_cube_world.as_matrix()
        H_cube_body_rot = H_body_world_rot.T @ H_cube_world_rot
        quat_body_cube = R.from_matrix(H_cube_body_rot).as_quat()
        marker_body.pose.orientation.x = quat_body_cube[0]
        marker_body.pose.orientation.y = quat_body_cube[1]
        marker_body.pose.orientation.z = quat_body_cube[2]
        marker_body.pose.orientation.w = quat_body_cube[3]

        marker.color.a = 0.5  # alpha = 1 means not transparent at all
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        car_markers.markers.append(marker)

        marker_body.color.a = 0.7  # alpha = 1 means not transparent at all
        marker_body.color.r = 0.0
        marker_body.color.g = 0.0
        marker_body.color.b = 1.0
        car_markers_body.markers.append(marker_body)




        range_bearing = cuboid_center_body#cuboid_center - odom_position

        bearing_vec_world = range_bearing / np.linalg.norm(range_bearing)
        range_vec_world =  np.linalg.norm(range_bearing)

        bearing_vec_world_point = Point()
        bearing_vec_world_point.x = bearing_vec_world[0]
        bearing_vec_world_point.y = bearing_vec_world[1]
        bearing_vec_world_point.z = bearing_vec_world[2]

        body_position_point = Point()
        body_position_point.x = cuboid_center_body[0]
        body_position_point.y = cuboid_center_body[1]
        body_position_point.z = cuboid_center_body[2]

        bearing_measurements.append(bearing_vec_world_point)
        range_measurements.append(range_vec_world)
        landmark_body_positions.append(body_position_point)
        current_meas_id = UInt64()
        current_meas_id.data = cuboid['global_id']
        meas_ids.append(current_meas_id)


    process_cloud_node_object.cuboid_marker_pub.publish(car_markers)
    process_cloud_node_object.cuboid_marker_body_pub.publish(car_markers_body)

    range_bearing_position_msg = ROSRangeBearing()
    range_bearing_position_msg.header.stamp = stamp
    range_bearing_position_msg.header.frame_id = process_cloud_node_object.reference_frame
    range_bearing_position_msg.bearing_factors = copy.deepcopy(bearing_measurements)
    range_bearing_position_msg.range_factors = copy.deepcopy(range_measurements)
    range_bearing_position_msg.landmark_body_positions = copy.deepcopy(landmark_body_positions)
    range_bearing_position_msg.meas_ids = meas_ids
    range_bearing_position_msg.odom = synced_odom
    process_cloud_node_object.range_bearing_pub.publish(range_bearing_position_msg)