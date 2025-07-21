#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "plan_env/sdf_map.h"

#include <memory>
#include <random>

using std::shared_ptr;
using std::normal_distribution;
using std::default_random_engine;

namespace ms_planner {
class SDFMap;

class MapROS {
public:
  MapROS();
  ~MapROS();
  void setMap(SDFMap* map);
  void init(ros::NodeHandle& nh);
  void initGlobalMap();
  void initStorageMap();
  void initLocalMap();

  void cropLocalMap(
    const Eigen::Vector3d& center_position_map,
    const Eigen::Vector3d& center_position_vio);
  // void getPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud);



  shared_ptr<SDFMap> global_map_;
  shared_ptr<SDFMap> storage_map_;
  // shared_ptr<SDFMap> local_map_;
  shared_ptr<MapParam> global_map_param_;
  shared_ptr<MapParam> storage_map_param_;
  shared_ptr<MapParam> local_map_param_;
  shared_ptr<MapData> global_map_data_;
  shared_ptr<MapData> storage_map_data_;
  shared_ptr<MapData> local_map_data_;



private:
  void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                         const geometry_msgs::PoseStampedConstPtr& pose);
  void cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                         const geometry_msgs::PoseStampedConstPtr& pose);
  void updateESDFCallback(const ros::TimerEvent& /*event*/);
  void visCallback(const ros::TimerEvent& /*event*/);

  void publishMapGlobal();
  void publishMapStorage();
  void publishMapInflateGlobal();  
  void publishMapInflateStorage();
  void publishMapInflateLocal();
  void publishESDF();
  void publishUpdateRange();
  void publishUnknown();
  void publishDepth();

  void proessDepthImage();

  // may use ExactTime?
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                          geometry_msgs::PoseStamped>
      SyncPolicyCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;


  ros::NodeHandle nh_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> slam_odom_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> vio_odom_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerCloudPose sync_cloud_pose_;
  SynchronizerImageOdom sync_image_slam_odom_;
  SynchronizerImageOdom sync_image_vio_odom_;


  void depthSlamOdomCallback(const sensor_msgs::ImageConstPtr &img,
                         const nav_msgs::OdometryConstPtr &odom);
  void depthVioOdomCallback(const sensor_msgs::ImageConstPtr &img,
                         const nav_msgs::OdometryConstPtr &odom);
 
  Eigen::Matrix4d cam2body_;


  // ros::Publisher map_local_pub_, map_local_inflate_pub_, esdf_pub_, map_all_pub_, map_inflate_all_pub_, 
  //     unknown_pub_, update_range_pub_, depth_pub_, map_storage_pub_;
  ros::Publisher map_global_pub_, map_storage_pub_;
  ros::Publisher map_inflate_global_pub_, map_inflate_storage_pub_;
  ros::Publisher map_inflate_local_pub_;
  ros::Publisher unknown_pub_, esdf_pub_, update_range_pub_, depth_pub_;

  ros::Timer esdf_timer_, vis_timer_;

  // Local map stuff
  Eigen::Vector3d local_ori_offset_;
  

  // params, depth projection
  double cx_, cy_, fx_, fy_;
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;
  string frame_id_;
  // msg publication
  double esdf_slice_height_;
  double visualization_truncate_high_, visualization_truncate_low_;
  bool show_esdf_time_, show_occ_time_;
  bool show_all_map_;
  string out_fname_;

  // data
  // flags of map state
  bool local_updated_, esdf_need_update_;
  // input
  Eigen::Vector3d camera_pos_;
  Eigen::Vector3d vio_pos_;
  Eigen::Vector3d optimized_pos_;

  Eigen::Quaterniond camera_q_;
  unique_ptr<cv::Mat> depth_image_;
  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int fuse_num_, esdf_num_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;

  normal_distribution<double> rand_noise_;
  default_random_engine eng_;

  ros::Time map_start_time_;

  bool init_time_;

  friend SDFMap;
};
}

#endif