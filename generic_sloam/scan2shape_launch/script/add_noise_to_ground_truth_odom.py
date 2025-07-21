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
# import nav_msgs/Odometry
from nav_msgs.msg import Odometry
# scipy import R
from scipy.spatial.transform import Rotation as R

import tf

class SimulateNoiseOdometry(object):
    def __init__(self):
        # initialize random seed 
        np.random.seed(0)

        # noise added in x, y, z, roll, pitch, yaw per 1 meter of movement
        self.noise_level = np.array([0.005, 0.005, 0.002, 0, 0, self.degree_to_rad(0)]) # gaussian noise std
        self.noise_bias = np.array([0.02, 0.02, 0.001, 0, 0, self.degree_to_rad(0)]) # gaussian noise mean
        self.noise_type = "gaussian" # "gaussian" or "fixed"
        # self.noise_level = np.array([0.01, 0.01, 0.02, 0, 0, self.degree_to_rad(0)]) # fixed noise magnitude
        # self.noise_type = "fixed" # "gaussian" or "fixed"

        self.odom_sub = rospy.Subscriber("/gt_odom", Odometry, callback=self.odom_cb, queue_size=1)
        self.perturb_odom_pub = rospy.Publisher("/gt_odom_perturbed", Odometry, queue_size=1)
        # self.sloam_to_vio_tf_pub = rospy.Publisher("/factor_graph_atl/quadrotor/sloam_to_vio_odom_fake", Odometry, queue_size=1)

        
        self.tf_listener2 = tf.TransformListener()
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.prev_odom_msg = None
        self.prev_odom_perturbed = None
        self.prev_odom_gt = None
        self.total_perturb_gt2vio = np.eye(4)
        self.total_perturb_gt2vio_sanity = np.eye(4)
        self.total_perturb_gt2vio_rotation = np.eye(3)
        self.total_perturb_gt2vio_rotation_sanity = np.eye(3)
        self.total_perturb_gt2vio_translation = np.zeros((1,3))
        self.total_perturb_gt2vio_translation_sanity = np.zeros((1,3))
        self.sanity_check = False

    def odom_cb(self, msg):
        # scale the noise by the distance traveled
        if self.prev_odom_msg is None:
            self.prev_odom_msg = msg
            ground_truth_odom = self.ros_msg_to_se3_matrix(msg)
            self.prev_odom_perturbed = ground_truth_odom
            self.prev_odom_gt = ground_truth_odom
            return
        dist = self.distance(self.prev_odom_msg, msg)
        print("current gt distance is: ", dist)
        self.prev_odom_msg = msg
        scaled_noise_level = self.noise_level * dist
        # sample gaussian noise from the scaled noise std

        if self.noise_type == "gaussian":
            # create biased sampled noise
            noise_sampled = np.random.normal(self.noise_bias*dist, scaled_noise_level) 
            # clip within two sigma
            noise_sampled = np.clip(noise_sampled, self.noise_bias*dist-scaled_noise_level*2, self.noise_bias*dist+scaled_noise_level*2)
        elif self.noise_type == "fixed":
            noise_sampled = scaled_noise_level
        print("noise_sampled: ", noise_sampled)


        # create an SE3 perturbation matrix based on scaled_noise_level
        odom_perturbation = np.eye(4)
        odom_perturbation[0:3, 0:3] = self.rotation_matrix(noise_sampled[3], noise_sampled[4], noise_sampled[5])
        odom_perturbation[0:3, 3] = noise_sampled[0:3]

        print("current sampled perturbation translation: ", odom_perturbation[:,3])


        ground_truth_odom = self.ros_msg_to_se3_matrix(msg)

        # calculate the relative motion from the previous odom to the current odom
        # relative motion from 1 to 2, from 2 to 3 ... from n-1 to n coupled should be the same as from 1 to n
        # gt_0 * gt_0.inverse() * gt_1 * gt_1.inverse() * ... * gt_n-1 * gt_n-1.inverse() * gt_n
        relative_motion = np.linalg.inv(self.prev_odom_gt) @ ground_truth_odom


        # accumulate the perturbation, calculate the perturbed odom
        perturbed_odom_predicted = self.prev_odom_perturbed @ relative_motion
        perturbed_odom =  perturbed_odom_predicted @ odom_perturbation        


        # update the prev_odom_perturbed
        self.prev_odom_perturbed = perturbed_odom
        self.prev_odom_gt = ground_truth_odom

        # H^vio_sloam = H^vio_body * H^body_sloam 
        self.total_perturb_gt2vio = perturbed_odom @ np.linalg.inv(ground_truth_odom)
        
        if self.sanity_check:
            # old method, only for sanity check
            # self.total_perturb_gt2vio = self.total_perturb_gt2vio @ odom_perturbation
            # separate rotation and translation here
            self.total_perturb_gt2vio_rotation_sanity = self.total_perturb_gt2vio_rotation_sanity @ odom_perturbation[:3,:3]
            self.total_perturb_gt2vio_translation_sanity = self.total_perturb_gt2vio_translation_sanity + odom_perturbation[:3,3]
            # compose rotation and translation
            self.total_perturb_gt2vio_sanity[0:3, 0:3] = self.total_perturb_gt2vio_rotation_sanity
            self.total_perturb_gt2vio_sanity[0:3, 3] = self.total_perturb_gt2vio_translation_sanity

            # apply the perturbation to the ground truth odometry
            perturbation_translation_sanity = np.eye(4)
            perturbation_translation_sanity[0:3, 3] = self.total_perturb_gt2vio_sanity[0:3, 3]
            perturbation_rotation_sanity = np.eye(4)
            perturbation_rotation_sanity[0:3, 0:3] = self.total_perturb_gt2vio_sanity[0:3, 0:3]

            # premultiply with translation and postmultiply with rotation to make sure that we are applying the perturbation as desired 
            # (i.e. so that translation does not affect rotation and vice versa)
            perturbed_odom_sanity = perturbation_translation_sanity @ ground_truth_odom @ perturbation_rotation_sanity

            # check the difference between the perturbed odom sanity and the perturbed_odom
            print("==========SNITY CHECK===========")
            rot_diff = perturbed_odom_sanity[0:3, 0:3] @ perturbed_odom[0:3, 0:3].transpose()
            trans_diff = perturbed_odom_sanity[0:3, 3] - perturbed_odom[0:3, 3]
            # convert rot_diff to euler angles using scipy 
            rot_diff_euler = R.from_matrix(rot_diff).as_euler('xyz', degrees=True)          
            # print rot_diff_euler and trans_diff in 2 digits
            print("rot_diff in pose euler degrees: ", np.around(rot_diff_euler, 2))
            print("trans_diff in pose: ", np.around(trans_diff, 2))
            # check the difference between the total_perturbation_sanity and self.total_perturb_gt2vio
            rot_diff_perturbation = self.total_perturb_gt2vio_sanity[0:3, 0:3] @ self.total_perturb_gt2vio[0:3, 0:3].transpose()
            trans_diff_perturbation = self.total_perturb_gt2vio_sanity[0:3, 3] - self.total_perturb_gt2vio[0:3, 3]
            # convert rot_diff_perturbation to euler angles using scipy 
            rot_diff_perturbation_euler = R.from_matrix(rot_diff_perturbation).as_euler('xyz', degrees=True)          
            # print rot_diff_euler and trans_diff in 2 digits
            print("rot_diff in total perturbation euler degrees: ", np.around(rot_diff_perturbation_euler, 2))
            print("trans_diff in total perturbation: ", np.around(trans_diff_perturbation, 2))
            print("========SNITY CHECK DONE=========")



        # publish the perturbed odometry
        perturbed_odom_msg = self.se3_matrix_to_ros_msg(perturbed_odom, msg)
        # print total perturbation
        print("total perturbation: ", self.total_perturb_gt2vio)
        self.perturb_odom_pub.publish(perturbed_odom_msg)

    def skew_symmetric_matrix(self, v):
        # convert a vector to a skew symmetric matrix
        return np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]])


    def se3_matrix_to_ros_msg(self, T, odom_msg):
        # convert an SE3 matrix to a ROS message
        # create a copy of odom_msg
        msg = Odometry()
        msg.header = odom_msg.header
        msg.child_frame_id = "base_link_perturbed"
        msg.twist = odom_msg.twist

        msg.pose.pose.position.x = T[0, 3]
        msg.pose.pose.position.y = T[1, 3]
        msg.pose.pose.position.z = T[2, 3]
        q = R.from_matrix(T[0:3, 0:3]).as_quat()
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        return msg
    
    def ros_msg_to_se3_matrix(self, msg):
        # convert a ROS message to an SE3 matrix
        T = np.eye(4)
        T[0:3, 0:3] = self.quaternion_to_rotation_matrix(msg.pose.pose.orientation)
        T[0:3, 3] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        return T
    
    def quaternion_to_rotation_matrix(self, q):
        # use scipy
        return R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    
    def rotation_matrix(self, roll, pitch, yaw):
        # roll, pitch, yaw are in radians
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
        R = np.dot(R_z, np.dot(R_y, R_x))
        return R
    

    def distance(self, odom1, odom2):
        return np.sqrt((odom1.pose.pose.position.x - odom2.pose.pose.position.x)**2 + (odom1.pose.pose.position.y - odom2.pose.pose.position.y)**2 + (odom1.pose.pose.position.z - odom2.pose.pose.position.z)**2)
    
    def degree_to_rad(self, degree):
        return degree * np.pi / 180

if __name__ == '__main__':

    rospy.init_node('simulate_noise_odometry_node')
    r = rospy.Rate(200)
    my_node = SimulateNoiseOdometry()
    while not rospy.is_shutdown():
        print("node started!")
        rospy.spin()
    # r.sleep()
    print("node killed!")
