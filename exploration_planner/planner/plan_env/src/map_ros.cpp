#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <fstream>

namespace ms_planner {
MapROS::MapROS() {
}

MapROS::~MapROS() {
}


void MapROS::init(ros::NodeHandle& nh) {
  nh_ = nh;

  // ########## Initialize global_map and local_map ##########
  global_map_param_.reset(new MapParam);
  storage_map_param_.reset(new MapParam);
  local_map_param_.reset(new MapParam);
  global_map_data_.reset(new MapData);
  storage_map_data_.reset(new MapData);
  local_map_data_.reset(new MapData); // May not need?
  // #### Params of global map ####
  double global_x_max, global_y_max, global_z_max;
  double global_x_min, global_y_min, global_z_min;
  nh_.param("map_ros/global/resolution", global_map_param_->resolution_, -1.0);

  nh_.param("map_ros/global/map_min_boundary_x", global_x_min, -1.0);
  nh_.param("map_ros/global/map_min_boundary_y", global_y_min, -1.0);
  nh_.param("map_ros/global/map_min_boundary_z", global_z_min, -1.0);
  nh_.param("map_ros/global/map_max_boundary_x", global_x_max, -1.0);
  nh_.param("map_ros/global/map_max_boundary_y", global_y_max, -1.0);
  nh_.param("map_ros/global/map_max_boundary_z", global_z_max, -1.0);

  global_map_param_->map_min_boundary_ = Eigen::Vector3d(global_x_min, global_y_min, global_z_min);
  global_map_param_->map_max_boundary_ = Eigen::Vector3d(global_x_max, global_y_max, global_z_max);
  global_map_param_->map_origin_ = Eigen::Vector3d(global_x_min, global_y_min, global_z_min);
  global_map_param_->map_size_ = Eigen::Vector3d(global_x_max - global_x_min, global_y_max - global_y_min, global_z_max - global_z_min);
  ROS_WARN_STREAM("Global Map Size: " << global_map_param_->map_size_);

  nh_.param("map_ros/global/inflation_xy", global_map_param_->inflation_xy_, -1.0);
  nh_.param("map_ros/global/inflation_z", global_map_param_->inflation_z_, -1.0);
  nh_.param("map_ros/global/max_ray_length", global_map_param_->max_ray_length_, -0.1);
  nh_.param("map_ros/global/decay_times_to_empty", global_map_param_->decay_times_to_empty_, -0.1);
  nh_.param("map_ros/global/decay_amount_lower_than_occ", global_map_param_->decay_amount_lower_than_occ_, -0.1);


  nh_.param("map_ros/map_update_bound_inflate", global_map_param_->map_update_bound_inflate_, 1.0);
  // Try retriving bounding box of map, set box to map size if not specified
  vector<string> axis = { "x", "y", "z" };
  for (int i = 0; i < 3; ++i) {
    nh_.param("map_ros/global/box_min_" + axis[i], global_map_param_->box_mind_[i], global_map_param_->map_min_boundary_[i]);
    nh_.param("map_ros/global/box_max_" + axis[i], global_map_param_->box_maxd_[i], global_map_param_->map_max_boundary_[i]);
  }

  // Satefy check of box size and map size, map size should be larger than box_maxd_ - box_mind_, if not, set box_maxd_ according to mind_ + size
  for (int i = 0; i < 3; ++i) {    
    if (global_map_param_->box_mind_[i] < global_map_param_->map_min_boundary_[i]) {
      ROS_WARN_STREAM("[MAP ROS: global map] Box min is smaller than min boundary, set box min to map min boundary");
      global_map_param_->box_mind_[i] = global_map_param_->map_min_boundary_[i];
    }
    if (global_map_param_->box_maxd_[i] > global_map_param_->map_max_boundary_[i]) {
      ROS_WARN_STREAM("[MAP ROS: global map] Box max larger than max boundary, set box max to map max boundary");
      global_map_param_->box_maxd_[i] = global_map_param_->map_max_boundary_[i];
    }
  }


  // Params of raycasting-based fusion
  // Prob occ > min_occupancy_log_
  // Prob unknown < clamp_min_log_ - mp_->unknown_flag_ ( in map ros mp_->clamp_min_log_ - 1e-3)
  // Prob free should > clamp_min_log_  < min_occupancy_log
  nh_.param("map_ros/p_hit", global_map_param_->p_hit_, 0.70);
  nh_.param("map_ros/p_miss", global_map_param_->p_miss_, 0.35);
  nh_.param("map_ros/p_min", global_map_param_->p_min_, 0.12);
  nh_.param("map_ros/p_max", global_map_param_->p_max_, 0.97);
  nh_.param("map_ros/p_occ", global_map_param_->p_occ_, 0.80);
  nh_.param("map_ros/virtual_ceil_height", global_map_param_->virtual_ceil_height_, -0.1);
  
  // #### Params of storage and local map ####
  // storage map should have same resolution as local map
  // storage map should have same x y z center and x_dim y_dim z_dim as global map
  double local_x_size, local_y_size, local_z_size;
  nh_.param("map_ros/local/resolution", local_map_param_->resolution_, -1.0);
  nh_.param("map_ros/local/resolution", storage_map_param_->resolution_, -1.0);
  nh_.param("map_ros/local/map_size_x", local_x_size, -1.0);
  nh_.param("map_ros/local/map_size_y", local_y_size, -1.0);
  nh_.param("map_ros/local/map_size_z", local_z_size, -1.0);
  nh_.param("map_ros/local/inflation_xy", storage_map_param_->inflation_xy_, -1.0);
  nh_.param("map_ros/local/inflation_z", storage_map_param_->inflation_z_, -1.0);
  nh_.param("map_ros/local/max_ray_length", storage_map_param_->max_ray_length_, -0.1);
  nh_.param("map_ros/local/decay_times_to_empty", storage_map_param_->decay_times_to_empty_, -0.1);
  nh_.param("map_ros/local/decay_amount_lower_than_occ", storage_map_param_->decay_amount_lower_than_occ_, -0.1);
  nh_.param("map_ros/map_update_bound_inflate", storage_map_param_->map_update_bound_inflate_);


  storage_map_param_->map_min_boundary_ = Eigen::Vector3d(global_x_min, global_y_min, global_z_min);
  storage_map_param_->map_max_boundary_ = Eigen::Vector3d(global_x_max, global_y_max, global_z_max);
  storage_map_param_->map_origin_ = Eigen::Vector3d(global_x_min, global_y_min, global_z_min);
  storage_map_param_->map_size_ = Eigen::Vector3d(global_x_max - global_x_min, global_y_max - global_y_min, global_z_max - global_z_min);
  // local_map_param_->map_origin_ = Eigen::Vector3d(global_x_origin, global_y_origin, global_z_origin);
  local_map_param_->map_size_ = Eigen::Vector3d(local_x_size, local_y_size, local_z_size);
  ROS_WARN_STREAM("Storage Map Size: " << storage_map_param_->map_size_);

  // Try retriving bounding box of map, set box to map size if not specified
  for (int i = 0; i < 3; ++i) {
    nh_.param("map_ros/global/box_min_" + axis[i], storage_map_param_->box_mind_[i], storage_map_param_->map_min_boundary_[i]);
    nh_.param("map_ros/global/box_max_" + axis[i], storage_map_param_->box_maxd_[i], storage_map_param_->map_max_boundary_[i]);
  }
  // Satefy check of box size and map size, map size should be larger than box_maxd_ - box_mind_, if not, set box_maxd_ according to mind_ + size
  for (int i = 0; i < 3; ++i) {    
    if (storage_map_param_->box_mind_[i] < storage_map_param_->map_min_boundary_[i]) {
      ROS_WARN_STREAM("[MAP ROS: global map] Box min is smaller than min boundary, set box min to map min boundary");
      storage_map_param_->box_mind_[i] = storage_map_param_->map_min_boundary_[i];
    }
    if (storage_map_param_->box_maxd_[i] > storage_map_param_->map_max_boundary_[i]) {
      ROS_WARN_STREAM("[MAP ROS: global map] Box max larger than max boundary, set box max to map max boundary");
      storage_map_param_->box_maxd_[i] = storage_map_param_->map_max_boundary_[i];
    }
  }
  // Params of raycasting-based fusion
  // Prob occ > min_occupancy_log_
  // Prob unknown < clamp_min_log_ - mp_->unknown_flag_ ( in map ros mp_->clamp_min_log_ - 1e-3)
  // Prob free should > clamp_min_log_  < min_occupancy_log
  nh_.param("map_ros/p_hit", storage_map_param_->p_hit_, 0.70);
  nh_.param("map_ros/p_miss", storage_map_param_->p_miss_, 0.35);
  nh_.param("map_ros/p_min", storage_map_param_->p_min_, 0.12);
  nh_.param("map_ros/p_max", storage_map_param_->p_max_, 0.97);
  nh_.param("map_ros/p_occ", storage_map_param_->p_occ_, 0.80);
  nh_.param("map_ros/virtual_ceil_height", storage_map_param_->virtual_ceil_height_, -0.1);


  // Set no flight zones
  double no_flight_zone_min_x, no_flight_zone_max_x;
  double no_flight_zone_min_y, no_flight_zone_max_y; 
  double no_flight_zone_min_z, no_flight_zone_max_z;
  bool set_no_flight_zone;
  int nfz_num;
  nh_.param("map_ros/set_no_flight_zone", set_no_flight_zone, false);
  nh_.param("map_ros/no_flight_zone_num", nfz_num, 0);
  global_map_param_->set_no_flight_zone_ = set_no_flight_zone;
  global_map_param_->nfz_num_ = nfz_num;
  storage_map_param_->set_no_flight_zone_ = set_no_flight_zone;
  storage_map_param_->nfz_num_ = nfz_num;
  if (set_no_flight_zone) {
    ROS_WARN("[MAP ROS] Set no flight zone");
    for (int i = 0; i < nfz_num; ++i) {
      nh_.param("map_ros/no_flight_zone_" + to_string(i) + "_x_min", no_flight_zone_min_x, -1.0);
      nh_.param("map_ros/no_flight_zone_" + to_string(i) + "_x_max", no_flight_zone_max_x, -1.0);
      nh_.param("map_ros/no_flight_zone_" + to_string(i) + "_y_min", no_flight_zone_min_y, -1.0);
      nh_.param("map_ros/no_flight_zone_" + to_string(i) + "_y_max", no_flight_zone_max_y, -1.0);
      nh_.param("map_ros/no_flight_zone_" + to_string(i) + "_z_min", no_flight_zone_min_z, -1.0);
      nh_.param("map_ros/no_flight_zone_" + to_string(i) + "_z_max", no_flight_zone_max_z, -1.0);
      global_map_param_->nfz_min_.push_back(Eigen::Vector3d(no_flight_zone_min_x, no_flight_zone_min_y, no_flight_zone_min_z));
      global_map_param_->nfz_max_.push_back(Eigen::Vector3d(no_flight_zone_max_x, no_flight_zone_max_y, no_flight_zone_max_z));
      storage_map_param_->nfz_min_.push_back(Eigen::Vector3d(no_flight_zone_min_x, no_flight_zone_min_y, no_flight_zone_min_z));
      storage_map_param_->nfz_max_.push_back(Eigen::Vector3d(no_flight_zone_max_x, no_flight_zone_max_y, no_flight_zone_max_z));
    }
  }



  // origin is the left lower corner of the voxel map, therefore, adding
  // this offset make the map centered around the given position
  local_ori_offset_ = -local_map_param_->map_size_/ 2;

  global_map_.reset(new SDFMap);
  storage_map_.reset(new SDFMap);
  initGlobalMap();
  initStorageMap();
  initLocalMap();

  nh_.param("map_ros/fx", fx_, -1.0);
  nh_.param("map_ros/fy", fy_, -1.0);
  nh_.param("map_ros/cx", cx_, -1.0);
  nh_.param("map_ros/cy", cy_, -1.0);
  nh_.param("map_ros/depth_filter_maxdist", depth_filter_maxdist_, -1.0);
  nh_.param("map_ros/depth_filter_mindist", depth_filter_mindist_, -1.0);
  nh_.param("map_ros/depth_filter_margin", depth_filter_margin_, -1);
  nh_.param("map_ros/k_depth_scaling_factor", k_depth_scaling_factor_, -1.0);
  nh_.param("map_ros/skip_pixel", skip_pixel_, -1);

  // nh_.param("map_ros/esdf_slice_height", esdf_slice_height_, -0.1);
  nh_.param("map_ros/visualization_truncate_high", visualization_truncate_high_, -0.1);
  nh_.param("map_ros/visualization_truncate_low", visualization_truncate_low_, -0.1);
  nh_.param("map_ros/show_occ_time", show_occ_time_, false);
  // nh_.param("map_ros/show_esdf_time", show_esdf_time_, false);
  nh_.param("map_ros/show_all_map", show_all_map_, false);
  nh_.param("map_ros/frame_id", frame_id_, string("world"));
  nh_.param("map_ros/expl_stat_fname", out_fname_, string("~/Code/pm_ws/benchmark_gazebo/bg1_vol.txt"));
  init_time_ = false;
  int pose_type; // 1 is pose, 2 is odom
  nh_.param("map_ros/pose_type", pose_type, 1);

  cam2body_ << 0.0, 0.0, 1.0, 0.0,
               -1.0, 0.0, 0.0, 0.0,
               0.0, -1.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 1.0;

  proj_points_.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  point_cloud_.points.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  // proj_points_.reserve(640 * 480 / global_map_->mp_->skip_pixel_ / global_map_->mp_->skip_pixel_);
  proj_points_cnt = 0;

  local_updated_ = false;
  esdf_need_update_ = false;
  fuse_time_ = 0.0;
  // esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
  // max_esdf_time_ = 0.0;
  fuse_num_ = 0;
  // esdf_num_ = 0;
  depth_image_.reset(new cv::Mat);

  rand_noise_ = normal_distribution<double>(0, 0.1);
  random_device rd;
  eng_ = default_random_engine(rd());

  //esdf_timer_ = nh_.createTimer(ros::Duration(0.05), &MapROS::updateESDFCallback, this);
  vis_timer_ = nh_.createTimer(ros::Duration(0.05), &MapROS::visCallback, this);

  map_global_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_global", 10);
  map_storage_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_storage", 10);

  map_inflate_global_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_inflate_global", 10);
  map_inflate_storage_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_inflate_storage", 10);

  // map_local_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 10);
  map_inflate_local_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_inflate_local", 10);

  unknown_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
  esdf_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ = nh_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);
  depth_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/map_ros/depth", 50));
  cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/map_ros/cloud", 50));
  pose_sub_.reset(
      new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/map_ros/pose", 25));


  // Here, the assumption is: the world frame and map frame is the same frame.
  // Imagine we are using vicon, the world/map frame origin will be the vicon origin.
  // Then, the odom is also under vicon frame so that it's initial position can be arbitrary (say x=2,y=3) in the vicon frame
  // Using VIO will make things different. If we keep the code unchanged. It means the world frame origin is the first pose of VIO. (0,0,0)
  // For active slam. If we have sloam pose and VIO pose.
  // We need to subscribe to the sloam pose here to build global map
  // Then local map extraction will also use sloam pose to fetch historical data
  // But the planner should use the local map and the VIO pose to plan
  // Which means we should publish the local map to egocentric VIO frame
  if (pose_type == 1)
  {
    sync_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
        MapROS::SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&MapROS::depthPoseCallback, this, _1, _2));
    sync_cloud_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(
        MapROS::SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
    sync_cloud_pose_->registerCallback(boost::bind(&MapROS::cloudPoseCallback, this, _1, _2));

  }
  else if (pose_type == 2)
  {
    // TODO: should subscribe to sloam synced pose when integrate things.
    slam_odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/odom_slam", 100, ros::TransportHints().tcpNoDelay()));
    vio_odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/odom_vio", 100, ros::TransportHints().tcpNoDelay()));

    sync_image_slam_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *slam_odom_sub_));
    sync_image_slam_odom_->registerCallback(boost::bind(&MapROS::depthSlamOdomCallback, this, _1, _2));
    sync_image_vio_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *vio_odom_sub_));
    sync_image_vio_odom_->registerCallback(boost::bind(&MapROS::depthVioOdomCallback, this, _1, _2));
  }

  map_start_time_ = ros::Time::now();

  ROS_INFO("[MAPROS] MapROS initialized successfully.]");
}



void MapROS::initGlobalMap() {
  global_map_->initMap(nh_, global_map_param_, global_map_data_);
}

void MapROS::initStorageMap() {
  storage_map_->initMap(nh_, storage_map_param_, storage_map_data_);
}

void MapROS::initLocalMap() {
  local_map_param_->resolution_inv_ = 1 / local_map_param_->resolution_;
  for (int i = 0; i < 3; ++i) {
    local_map_param_->map_voxel_num_(i) = ceil(local_map_param_->map_size_(i) / local_map_param_->resolution_);
  }
  int buffer_size = local_map_param_->map_voxel_num_(0) * local_map_param_->map_voxel_num_(1) * local_map_param_->map_voxel_num_(2);
  local_map_data_->occupancy_buffer_ = vector<double>(buffer_size, local_map_param_->clamp_min_log_ - local_map_param_->unknown_flag_);
  local_map_data_->occupancy_buffer_inflate_ = vector<double>(buffer_size, local_map_param_->clamp_min_log_ - local_map_param_->unknown_flag_);
}

void MapROS::visCallback(const ros::TimerEvent& e) {
  // publishMapLocal();
  if (show_all_map_) {
    // Limit the frequency of all map
    static double tpass = 0.0;
    tpass += (e.current_real - e.last_real).toSec();
    if (tpass > 0.2) {
      publishMapGlobal();
      publishMapStorage();
      // publishMapInflateGlobal();      
      // publishMapInflateStorage();
      publishMapInflateLocal();
      tpass = 0.0;
    }
  }
  // publishUnknown();
  // publishUpdateRange();
  // publishDepth();
}



void MapROS::depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
  if (!global_map_->isInMap(camera_pos_))  // exceed mapped region
    return;

  camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                 pose->pose.orientation.y, pose->pose.orientation.z);
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  cv_ptr->image.copyTo(*depth_image_);

  auto t1 = ros::Time::now();

  // generate point cloud, update map
  proessDepthImage();
  // Cam_pos is camera pose in map frame. Not camear odom. 
  storage_map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
  // Get inflated local map from storage map
  ROS_WARN("Call Crop local map in depthPoseCallback");
  cropLocalMap(optimized_pos_, vio_pos_);
  global_map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
  // Here, inputPointCloud will flip local_updated_ to true

  auto t2 = ros::Time::now();
  fuse_time_ += (t2 - t1).toSec();
  max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());
  fuse_num_ += 1;
  if (show_occ_time_)
    ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), fuse_time_ / fuse_num_,
             max_fuse_time_);
}


void MapROS::cropLocalMap(
    const Eigen::Vector3d& center_position_map,
    const Eigen::Vector3d& center_position_vio) {

  // Compute origin of local map from center_position_map and local map size
  local_map_param_->map_origin_ = center_position_map + local_ori_offset_;
  local_map_param_->map_origin_(2) = global_map_param_->map_origin_(2);
  // core function: crop local map from the storage map
  // local map data -> occupancy buffer inflate will be filled
  storage_map_->getInflatedLocalMap(local_map_param_, local_map_data_);
}

// FIXME:Modify this function if use
void MapROS::cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
  optimized_pos_ = camera_pos_;
  // TODO: vio_pos should be raw vio pos
  vio_pos_ = camera_pos_;
  camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                 pose->pose.orientation.y, pose->pose.orientation.z);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  int num = cloud.points.size();
  // Cam_pos is camera pose in map frame. Not camear odom. 
  storage_map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
  // Get inflated local map from storage map
  ROS_WARN("Call Crop local map in cloudPoseCallback");
  cropLocalMap(optimized_pos_, vio_pos_);
  global_map_->inputPointCloud(cloud, num, camera_pos_);
}

void MapROS::depthVioOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{
  //ROS_INFO("Enter depth odom callback");
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * cam2body_;
  camera_pos_(0) = cam_T(0, 3);
  camera_pos_(1) = cam_T(1, 3);
  camera_pos_(2) = cam_T(2, 3);
  optimized_pos_ = camera_pos_;
  // TODO: vio_pos should be raw vio pos
  vio_pos_ = camera_pos_;
  camera_q_ = cam_T.block<3, 3>(0, 0);


  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(*depth_image_);



  auto t1 = ros::Time::now();

  // generate point cloud, update map
  proessDepthImage();
  // Cam_pos is camera pose in map frame. Not camear odom. 
  storage_map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
  // Get inflated local map from storage map
  cropLocalMap(optimized_pos_, vio_pos_);

  auto t2 = ros::Time::now();
  fuse_time_ += (t2 - t1).toSec();
  max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());
  fuse_num_ += 1;
  if (show_occ_time_)
    ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), fuse_time_ / fuse_num_,
             max_fuse_time_);
}





void MapROS::depthSlamOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{
  //ROS_INFO("Enter depth odom callback");
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * cam2body_;
  camera_pos_(0) = cam_T(0, 3);
  camera_pos_(1) = cam_T(1, 3);
  camera_pos_(2) = cam_T(2, 3);
  optimized_pos_ = camera_pos_;

  vio_pos_ = camera_pos_;
  camera_q_ = cam_T.block<3, 3>(0, 0);


  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(*depth_image_);

  auto t1 = ros::Time::now();

  // generate point cloud, update map
  proessDepthImage();
  // Cam_pos is camera pose in map frame. Not camear odom. 
  global_map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
  // Here, inputPointCloud will flip local_updated_ to true
  auto t2 = ros::Time::now();
  fuse_time_ += (t2 - t1).toSec();
  max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());
  fuse_num_ += 1;
  if (show_occ_time_)
    ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), fuse_time_ / fuse_num_,
             max_fuse_time_);
}


void MapROS::proessDepthImage() {
  proj_points_cnt = 0;

  uint16_t* row_ptr;
  int cols = depth_image_->cols;
  int rows = depth_image_->rows;
  double depth;
  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / k_depth_scaling_factor_;

  for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_) {
    row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
    for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_) {
      depth = (*row_ptr) * inv_factor;
      row_ptr = row_ptr + skip_pixel_;
      if (*row_ptr == 0 || depth > depth_filter_maxdist_)
        depth = depth_filter_maxdist_;
      else if (depth < depth_filter_mindist_)
        continue;

      pt_cur(0) = (u - cx_) * depth / fx_;
      pt_cur(1) = (v - cy_) * depth / fy_;
      pt_cur(2) = depth;
      pt_world = camera_r * pt_cur + camera_pos_;
      auto& pt = point_cloud_.points[proj_points_cnt++];
      pt.x = pt_world[0];
      pt.y = pt_world[1];
      pt.z = pt_world[2];
    }
  }

}


void MapROS::publishMapGlobal() {
  if (!init_time_) {
    map_start_time_ = ros::Time::now();
    init_time_ = true;
  }
  // pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1;

  // Get point cloud from global_map_
  global_map_->genPointCloud(cloud1, visualization_truncate_low_, visualization_truncate_high_, frame_id_);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_global_pub_.publish(cloud_msg);

  // Output time and known volumn
  double time_now = (ros::Time::now() - map_start_time_).toSec();
  double known_volumn = 0;
}

void MapROS::publishMapStorage() {
  if (!init_time_) {
    map_start_time_ = ros::Time::now();
    init_time_ = true;
  }
  // pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1;

  // Get point cloud from global_map_
  storage_map_->genPointCloud(cloud1, visualization_truncate_low_, visualization_truncate_high_, frame_id_);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_storage_pub_.publish(cloud_msg);
}

void MapROS::publishMapInflateGlobal() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1;

  // Get point cloud from global_map_
  global_map_->genInflatePointCloud(cloud1, visualization_truncate_low_, visualization_truncate_high_, frame_id_);
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_inflate_global_pub_.publish(cloud_msg);
}

void MapROS::publishMapInflateStorage() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1;

  // Get point cloud from global_map_
  storage_map_->genInflatePointCloud(cloud1, visualization_truncate_low_, visualization_truncate_high_, frame_id_);
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_inflate_storage_pub_.publish(cloud_msg);
}

void MapROS::publishMapInflateLocal() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1;

  // Compute the offset between optimized_pos_ and vio_pos_
  Eigen::Vector3d offset = vio_pos_ + local_ori_offset_ - storage_map_param_->map_origin_;
  offset(2) = 0;
  // Get point cloud from global_map_
  storage_map_->genInflateLocalPointCloud(local_map_param_, local_map_data_, offset, cloud1, visualization_truncate_low_, visualization_truncate_high_, frame_id_);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_inflate_local_pub_.publish(cloud_msg);
}


void MapROS::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_cut = global_map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = global_map_->md_->local_bound_max_;
  global_map_->boundIndex(max_cut);
  global_map_->boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (global_map_->md_->occupancy_buffer_[global_map_->toAddress(x, y, z)] < global_map_->mp_->clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          global_map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_high_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

void MapROS::publishDepth() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < proj_points_cnt; ++i) {
    cloud.push_back(point_cloud_.points[i]);
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_.publish(cloud_msg);
}

void MapROS::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  global_map_->indexToPos(global_map_->md_->local_bound_min_, esdf_min_pos);
  global_map_->indexToPos(global_map_->md_->local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;
  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);
  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

}