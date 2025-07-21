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

class LoopClosureSubmapNodeParser(object):
    def __init__(self):

        # publish the loop closure submap as a MarkerArray
        self.loop_closure_submap_sub = rospy.Subscriber("/factor_graph_atl/loop_closure_submap", MarkerArray,  callback=self.cb, queue_size=1)


    def cb(self, msg):
        # iterate through the markers in the MarkerArray
        key_pose_submap_pairs = {}
        for marker in msg.markers:
            if "delete" in marker.ns:
                return
            if "key_pose" in marker.ns:
                # the last one in the ns is the id
                key_pose_id = marker.id
                if key_pose_id not in key_pose_submap_pairs:
                    key_pose_submap_pairs[key_pose_id] = {}
                key_pose_submap_pairs[key_pose_id]["key_pose"] = np.array([marker.points[0].x, marker.points[0].y, marker.points[0].z])
            elif "submap_landmarks" in marker.ns:
                # the last one in the ns is the id
                key_pose_id = int(marker.ns.split("_")[-1])
                if key_pose_id not in key_pose_submap_pairs:
                    key_pose_submap_pairs[key_pose_id] = {}
                if "submap_landmarks" not in key_pose_submap_pairs[key_pose_id]:
                    key_pose_submap_pairs[key_pose_id]["submap_landmarks"] = []
                key_pose_submap_pairs[key_pose_id]["submap_landmarks"].append(np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]))
            elif "submap_centroid_" in marker.ns:
                # the last one in the ns is the id
                key_pose_id = int(marker.ns.split("_")[-1])
                print(key_pose_id)
                key_pose_submap_pairs[key_pose_id]["submap_centroid"] = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
            

        # print all the key_pose_submap_pairs
        for key_pose_id in key_pose_submap_pairs:
            print("key_pose_id: ", key_pose_id)
            print("key_pose: ", key_pose_submap_pairs[key_pose_id]["key_pose"])
            print("submap_centroid: ", key_pose_submap_pairs[key_pose_id]["submap_centroid"])
            print("submap_landmarks: ", key_pose_submap_pairs[key_pose_id]["submap_landmarks"])
            print("")

if __name__ == '__main__':

    rospy.init_node('loop_closure_submap_node_parser')
    r = rospy.Rate(30)
    my_node = LoopClosureSubmapNodeParser()
    while not rospy.is_shutdown():
        print("node started!")
        rospy.spin()
    # r.sleep()
    print("node killed!")
