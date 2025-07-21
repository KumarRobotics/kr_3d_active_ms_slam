#! /usr/bin/env python3

import rospy
import rospkg
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import message_filters
import numpy as np
import torch
import matplotlib.pyplot as plt
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, PointField
import cv2
from sem_detection_msgs.msg import syncPcOdom

bridge = CvBridge()

# observation 1: in the bag file, color and depth images can go out of sync
# observation 2: might be able to use aligned depth?



class sem_detection:
    def __init__(self) -> None:
        # Initialize the ROS node
        rospy.init_node('sem_detection')

        self.prev_time = rospy.Time.now()
        rospack = rospkg.RosPack()
        self.model_path = rospack.get_path('rgb_sem_segmentation') + '/models/yolov8x-seg.pt'
        self.yolo = YOLO(self.model_path)

        # Subscribe to the rgb image
        # ros params
        self.sim = rospy.get_param('~sim', False)
        self.color_fx = rospy.get_param('~fx', 603.7166748046875)
        self.color_fy = rospy.get_param('~fy', 603.9064331054688)
        self.color_cx = rospy.get_param('~cx', 314.62518310546875)
        self.color_cy = rospy.get_param('~cy', 244.9166717529297)
        if self.sim:
            # Note: Somehow the cv bridge change the raw depth value to meter
            # So we should not apply depth scale to the depth image again
            self.depth_scale = 1
        else:
            self.depth_scale = 1 / rospy.get_param('~k_depth_scaling_factor', 1000)
        print("Depth scale: ", self.depth_scale)
        print("fx: ", self.color_fx)
        print("fy: ", self.color_fy)
        print("cx: ", self.color_cx)
        print("cy: ", self.color_cy)

        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/color/image_raw/')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_rect_raw/')
        self.aligned_depth_topic = rospy.get_param('~aligned_depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.odom_topic = rospy.get_param('~odom_topic', '/dragonfly67/quadrotor_ukf/control_odom')
        self.sync_odom_measurements = rospy.get_param('~sync_odom_measurements', True)
        self.sync_pc_odom_topic = rospy.get_param('~sync_pc_odom_topic', '/sem_detection/sync_pc_odom')
        self.pc_topic = rospy.get_param('~pc_topic', '/sem_detection/point_cloud')

        self.desired_rate = rospy.get_param('~desired_rate', 4.0)
        # Subscriber and publisher
        self.rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)

        # Subscribe to the depth image
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.aligned_depth_sub = message_filters.Subscriber(self.aligned_depth_topic, Image)
        self.odom_sub = message_filters.Subscriber(self.odom_topic, Odometry)

        self.pc_pub_ = rospy.Publisher(self.pc_topic, PointCloud2, queue_size=1)
        self.synced_pc_pub_ = rospy.Publisher(self.sync_pc_odom_topic, syncPcOdom, queue_size=1)


        # Synchronize the two image topics with a time delay of 0.1 seconds
        # ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        if (self.sync_odom_measurements):
            rospy.loginfo("syncing rgb, aligned depth and odom")
            # ApproximateTimeSynchronizer to allow for 0.01s time difference
            ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.aligned_depth_sub, self.odom_sub], 10, 0.05)
            ts.registerCallback(self.rgb_aligned_depth_odom_callback)
        else:
            ts = message_filters.TimeSynchronizer([self.rgb_sub, self.aligned_depth_sub], 5)
            ts.registerCallback(self.rgb_aligned_depth_callback)


        self.pc_fields_ = self.make_fields()

        self.cls = {"chair": 1, "dining table": 2, "tv": 3}


    def rgb_aligned_depth_odom_callback(self, rgb, aligned_depth, odom):
        if (rospy.Time.now() - self.prev_time).to_sec() < 1.0/self.desired_rate:
            print("time elapsed since last depth rgb callback is: ", (rospy.Time.now() - self.prev_time).to_sec())
            print("skipping current depth image to get desired rate of ", self.desired_rate)
            return
        else:
            self.prev_time = rospy.Time.now()

        rospy.loginfo("depth rgb odom callback")
        try:
            # Convert ROS image message to OpenCV image
            # Note: Somehow the cv bridge change the raw depth value to meter
            # So we should not apply depth scale to the depth image again
            color_img = bridge.imgmsg_to_cv2(rgb, "bgr8") 
            depth_img = bridge.imgmsg_to_cv2(aligned_depth, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(e)
            return
        
        # 1. detect semantics
        # Perform instance segmentation using YOLOv8
        detections = self.yolo.predict(color_img,  show=False)

        # 2. open img_size * 2 array save class and id
        print("color shape:", color_img.shape)
        label = np.zeros([color_img.shape[0], color_img.shape[1]])
        id = np.zeros([color_img.shape[0], color_img.shape[1]])
        conf = np.zeros([color_img.shape[0], color_img.shape[1]])
        
        # 3. go though all masks, fill them in class and id
        if len(detections[0]) != 0:
            # detections is of size 1
            # detections[0] contains everything; masks is of shape (N obj, H, W)
            for detection in detections:
                # segmentation
                num_obj = detection.masks.shape[0]
                for i in range(num_obj):
                    cls_int = int(detection.boxes.cls[i])
                    cls_str = self.yolo.names[cls_int]
                    cur_mask = detection.masks.masks[i, :, :].cpu().numpy()
                    
                    # reduce the size of mask to remove outlies at the boundary -> doing this in process_cloud_node instead now
                    mask_pos = np.where(cur_mask != 0)
                    label[mask_pos] = self.get_cls_label(cls_str)
                    id[mask_pos] = i+1
                    conf[mask_pos] = float(detection.boxes.conf[i])

        # Create a grid of pixel coordinates
        u, v = np.meshgrid(np.arange(depth_img.shape[1]), np.arange(depth_img.shape[0]))
        u = u.astype(np.float32)
        v = v.astype(np.float32)
        d = depth_img.flatten()
        d = d * self.depth_scale
        x = (u.flatten() - self.color_cx) * d / self.color_fx
        y = (v.flatten() - self.color_cy) * d / self.color_fy
        z = d
        x_pt = x.reshape(-1, depth_img.shape[1])
        y_pt = y.reshape(-1, depth_img.shape[1])
        z_pt = z.reshape(-1, depth_img.shape[1])
        x_pt = x_pt[..., None]
        y_pt = y_pt[..., None]
        z_pt = z_pt[..., None]
        points = np.concatenate((x_pt, y_pt, z_pt), axis=2)
        # 5. Stack labels, id, conf
        label = label[..., None]
        id = id[..., None]
        conf = conf[..., None]
        pc_data = np.concatenate((points, label, id, conf), axis=2).astype(np.float32)

        self.make_fields()
        
        # 6. publish point cloud
        sync_pc_odom_msg = syncPcOdom()
        sync_pc_odom_msg.header = Header()
        sync_pc_odom_msg.header.stamp = odom.header.stamp
        sync_pc_odom_msg.header.frame_id = "camera_color_optical_frame"
        pc_msg = PointCloud2()
        header = Header()
        pc_msg.header = header
        pc_msg.header.stamp = odom.header.stamp
        pc_msg.header.frame_id = "camera_color_optical_frame"
        print('hard coding seg pc frame_id to camera')
        pc_msg.width = color_img.shape[1]
        pc_msg.height = color_img.shape[0]
        pc_msg.point_step = 24
        pc_msg.row_step = pc_msg.width * pc_msg.point_step
        pc_msg.fields = self.pc_fields_
        pc_msg.data = pc_data.tobytes()
        sync_pc_odom_msg.cloud = pc_msg
        sync_pc_odom_msg.odom = odom
        print("Pub synced pc odom msg")
        self.synced_pc_pub_.publish(sync_pc_odom_msg)
        self.pc_pub_.publish(pc_msg)

                

    def rgb_aligned_depth_callback(self, rgb, aligned_depth):
        if (rospy.Time.now() - self.prev_time).to_sec() < 1.0/self.desired_rate:
            print("time elapsed since last depth rgb callback is: ", (rospy.Time.now() - self.prev_time).to_sec())
            print("skipping current depth image to get desired rate of ", self.desired_rate)
            return
        else:
            self.prev_time = rospy.Time.now()

        rospy.loginfo("depth rgb callback")
        try:
            # Convert ROS image message to OpenCV image
            color_img = bridge.imgmsg_to_cv2(rgb, "bgr8") 
            depth_img = bridge.imgmsg_to_cv2(aligned_depth, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(e)
            return
        
        # 1. detect semantics
        # Perform instance segmentation using YOLOv8
        detections = self.yolo.predict(color_img,  show=False)

        # 2. open img_size * 2 array save class and id
        print("color shape:", color_img.shape)
        # pc_pos = np.zeros([color_img.shape[0], color_img.shape[1], 3])
        label = np.zeros([color_img.shape[0], color_img.shape[1]])
        id = np.zeros([color_img.shape[0], color_img.shape[1]])
        conf = np.zeros([color_img.shape[0], color_img.shape[1]])
        
        # 3. go though all masks, fill them in class and id
        if len(detections[0]) != 0:
            # detections[0] contains everything
            # masks is of shape (N obj, H, W)
            for detection in detections:
                # segmentation
                num_obj = detection.masks.shape[0]
                for i in range(num_obj):
                    cls_int = int(detection.boxes.cls[i])
                    cls_str = self.yolo.names[cls_int]
                    cur_mask = detection.masks.masks[i, :, :].cpu().numpy()
                    kernel = np.ones((20,20),np.uint8)
                    mask_pos = np.where(cur_mask != 0)
                    label[mask_pos] = self.get_cls_label(cls_str)
                    id[mask_pos] = i+1
                    conf[mask_pos] = float(detection.boxes.conf[i])
        
        # 4. go through depth, project them to 3D
        # Create a grid of pixel coordinates
        u, v = np.meshgrid(np.arange(depth_img.shape[1]), np.arange(depth_img.shape[0]))
        u = u.astype(np.float32)
        v = v.astype(np.float32)
        d = depth_img.flatten()
        d = d * self.depth_scale
        x = (u.flatten() - self.color_cx) * d / self.color_fx
        y = (v.flatten() - self.color_cy) * d / self.color_fy
        z = d
        x_pt = x.reshape(-1, depth_img.shape[1])
        y_pt = y.reshape(-1, depth_img.shape[1])
        z_pt = z.reshape(-1, depth_img.shape[1])
        x_pt = x_pt[..., None]
        y_pt = y_pt[..., None]
        z_pt = z_pt[..., None]
        points = np.concatenate((x_pt, y_pt, z_pt), axis=2)

        # 5. Stack labels, id, conf
        label = label[..., None]
        id = id[..., None]
        conf = conf[..., None]
        pc_data = np.concatenate((points, label, id, conf), axis=2).astype(np.float32)

        self.make_fields()

        pc_msg = PointCloud2()
        header = Header()
        pc_msg.header = header
        pc_msg.header.stamp = aligned_depth.header.stamp
        pc_msg.header.frame_id = "camera"
        pc_msg.width = color_img.shape[1]
        pc_msg.height = color_img.shape[0]
        pc_msg.point_step = 24
        pc_msg.row_step = pc_msg.width * pc_msg.point_step
        pc_msg.fields = self.pc_fields_
        pc_msg.data = pc_data.tobytes()
        self.pc_pub_.publish(pc_msg)
                

    def get_cls_label(self, cls_str):
        if cls_str in self.cls.keys():
            return self.cls[cls_str]
        else:
            return 0#4

    def run(self):
        # Spin the ROS node
        rospy.loginfo("Semantic detection node init.")
        rospy.spin()

    def make_fields(self):
        # manually add fiels by Ian
        fields = []
        field = PointField()
        field.name = 'x'
        field.count = 1
        field.offset = 0
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'y'
        field.count = 1
        field.offset = 4
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'z'
        field.count = 1
        field.offset = 8
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'intensity'
        field.count = 1
        field.offset = 12
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'id'
        field.count = 1
        field.offset = 16
        field.datatype = PointField.FLOAT32
        fields.append(field)

        field = PointField()
        field.name = 'confidence'
        field.count = 1
        field.offset = 20
        field.datatype = PointField.FLOAT32
        fields.append(field)
        return fields


if __name__ == '__main__':
    node = sem_detection()
    node.run()

