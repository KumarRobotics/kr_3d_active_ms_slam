#include "loop_closure/loopClosureServer.h"


LoopClosureServer::LoopClosureServer(ros::NodeHandle &nh) : nh_(nh) {

  // initialize landmark_positions_ to be empty and NumLandmarks_ to be 0
  landmark_positions_ = {};
  NumLandmarks_ = 0;

  std::string ugv0_bearing_topic = "/factor_graph_atl/quadrotor/measurements_with_sync_odom";
  SemanticSub_ = nh_.subscribe(ugv0_bearing_topic, 10,
                               &LoopClosureServer::SemanticMeasCb_, this);

  LoopClosurePosePub_ =
      nh_.advertise<sloam_msgs::SemanticLoopClosure>("/loop_closure/odom", 10);


  // ################# INIT Loop Closure Server #################
  // Action server
  loop_closure_server_ptr_.reset(new DetectLoopClosureServerType(nh_, "/loop_closure/detect_loop_closure_server", false));
  loop_closure_server_ptr_->registerGoalCallback(boost::bind(&LoopClosureServer::goalCb_, this));
  loop_closure_server_ptr_->registerPreemptCallback(boost::bind(&LoopClosureServer::preemptCb_, this));
  loop_closure_server_ptr_->start();
  ROS_INFO("initialized");
}

void LoopClosureServer::goalCb_() {
  // If there is another goal active, cancel it
  if (loop_closure_server_ptr_->isActive()) {
    ROS_WARN("[Loop Closure Server] Received a new goal, canceling the previous one");
    loop_closure_server_ptr_->setAborted();
  }
  // Pointer to msg
  const auto msg = loop_closure_server_ptr_->acceptNewGoal();


  if (loop_closure_server_ptr_->isPreemptRequested())
  {
    ROS_INFO("[Loop Closure Server] Loop Closure goal preempted.");
    loop_closure_server_ptr_->setPreempted();
    return;
  }

  ROS_INFO("[Loop Closure Server] new loop closure triggered");
  // read submap from the msg
  latest_submap_ = msg->submap;
  landmark_positions_.clear();
  // put points in latest_submap_ into landmark_positions_
  for (auto &point : latest_submap_) {
    landmark_positions_.push_back({point.x, point.y, point.z});
  }
  NumLandmarks_ = landmark_positions_.size();
  // print submap updated with landmarks_positions_ and NumLandmarks_
  ROS_INFO_STREAM("[Loop Closure Server] new submap received with " << NumLandmarks_ << " landmarks");
  for (auto &point : landmark_positions_) {
    ROS_INFO_STREAM("landmark position: " << point[0] << ", " << point[1] << ", " << point[2]);
  }
}

void LoopClosureServer::preemptCb_() {
  if (loop_closure_server_ptr_->isActive())
  {
    ROS_INFO("Active Loop Closure goal aborted.");
    loop_closure_server_ptr_->setAborted();
  }
  else
  {
    ROS_INFO("Active Loop Closure goal preempted.");
    loop_closure_server_ptr_->setPreempted();
  }
}


void LoopClosureServer::transitLoopClosureState(LOOP_CLOSURE_STATE new_state, std::string pos_call) {
  int pre_s = int(loop_closure_state_);
  loop_closure_state_ = new_state;
  ROS_INFO_STREAM("[" + pos_call + "]: from " << loop_closure_state_str_[pre_s] << " to " + loop_closure_state_str_[int(new_state)]);
}

LOOP_CLOSURE_STATE LoopClosureServer::getState() {
  return loop_closure_state_;
}


void LoopClosureServer::SemanticMeasCb_(
    const sloam_msgs::ROSRangeBearingSyncOdomConstPtr &range_bearing_msg) {
  semantic_message_received_ = true;
  LatestSemanticMeas_ = range_bearing_msg;

  // Handle synced pose
  OdomQueue_.push_back(range_bearing_msg->corrected_odom);
  vioOdomQueue_.push_back(range_bearing_msg->vio_odom);
  if (OdomQueue_.size() > maxQueueSize_) OdomQueue_.pop_front();
  if (vioOdomQueue_.size() > maxQueueSize_) vioOdomQueue_.pop_front();
  optimized_hist_key_pose_ = range_bearing_msg->optimized_key_pose;
  key_pose_idx_ = range_bearing_msg->key_pose_idx.data;
  ROS_INFO_STREAM("SemanticMeasCb_ optimized_hist_key_pose_: " << optimized_hist_key_pose_.pose.pose.position.x << ", " << optimized_hist_key_pose_.pose.pose.position.y << ", " << optimized_hist_key_pose_.pose.pose.position.z);
  if (semantic_message_received_ && loop_closure_server_ptr_->isActive()) {
    findLoopClosure();
  } else{
    ROS_INFO_STREAM("semantic_message_received_: " << semantic_message_received_);
    ROS_INFO_STREAM("loop_closure_server_ptr_->isActive(): " << loop_closure_server_ptr_->isActive());
  }
}

// Find loop closure, and set goal result according to the result 
void LoopClosureServer::findLoopClosure() {
  ROS_INFO("[findLoopClosure] findLoopClosure triggered");
  if (! loop_closure_server_ptr_->isActive()) {
    ROS_ERROR("[findLoopClosure] No loop closure goal received.");
    return;
  }
  // check if landmarks are received
  if (NumLandmarks_ == 0) {
    ROS_WARN("[findLoopClosure] No landmarks received.");
    loop_closure_server_ptr_->setAborted();
    return;
  }
  
  auto cur_stamp = LatestSemanticMeas_->header.stamp;

  double odom_x, vio_odom_x;
  double odom_y, vio_odom_y;
  double odom_z, vio_odom_z;
  geometry_msgs::Quaternion odom_q, vio_odom_q;
  nav_msgs::Odometry odom_msg, vio_odom_msg;
  double noise_to_each_axis = gaussian_noise_std_ / sqrt(2);
  std::normal_distribution<double> distribution(0.0, noise_to_each_axis);
  double noise;

  int total_ugv_markers = LatestSemanticMeas_->bearing_factors.size();
  int ugv_marker_id;
  ros::Time ugv_detection_stamp;

  if (total_ugv_markers < 4) {
    ROS_WARN_STREAM("number of detected ugvs is less than 4, it is: " << total_ugv_markers);
    sloam_msgs::DetectLoopClosureResult result;
    result.success = false;
    result.status = 1;
    loop_closure_server_ptr_->setSucceeded(result);
    return;
  } else {
    ROS_INFO_STREAM("number of detected ugvs is: " << total_ugv_markers);
  }
  // get the stamp of current ugv_detection
  ugv_detection_stamp = cur_stamp;
  for (auto i = OdomQueue_.size() - 1; i >= 0; i--) {
    odom_msg = OdomQueue_[i];
    vio_odom_msg = vioOdomQueue_[i];
    if (abs(odom_msg.header.stamp.toSec() - ugv_detection_stamp.toSec()) <
        time_offset_tolerance_) {
      ROS_INFO_STREAM("sync odom and ugv detection is successful after "
                      << OdomQueue_.size() - i << " iterations");
      odom_x = odom_msg.pose.pose.position.x;
      odom_y = odom_msg.pose.pose.position.y;
      odom_z = odom_msg.pose.pose.position.z;
      odom_q = odom_msg.pose.pose.orientation;
      vio_odom_x = vio_odom_msg.pose.pose.position.x;
      vio_odom_y = vio_odom_msg.pose.pose.position.y;
      vio_odom_z = vio_odom_msg.pose.pose.position.z;
      vio_odom_q = vio_odom_msg.pose.pose.orientation;
      break;
    }
    if (i == 0) {
      ROS_WARN_STREAM("sync odom and ugv detection failed!!!");
      sloam_msgs::DetectLoopClosureResult result;
      result.success = false;
      result.status = 2;
      loop_closure_server_ptr_->setSucceeded(result);
      return;
    }
  }

  std::vector<Eigen::Vector3d> relative_msts;

  for (size_t i = 0; i < LatestSemanticMeas_->landmark_body_positions.size();
       i++) {
    Eigen::Vector3d current_position_meas{
        LatestSemanticMeas_->landmark_body_positions[i].x,
        LatestSemanticMeas_->landmark_body_positions[i].y,
        LatestSemanticMeas_->landmark_body_positions[i].z};
    relative_msts.push_back(current_position_meas);
  }

  ROS_INFO_STREAM("relative_msts size: " << relative_msts.size());
  Eigen::Vector3d position_estimate;
  double yaw_estimate;
  bool in_fa = false;

  double max_tol_per_landmark_pos_error = 0.08;

  // get eigen quaternion from vio odom
  vio_odom_q = vio_odom_msg.pose.pose.orientation;
  Eigen::Quaterniond vio_odom_quaternion;
  tf2::fromMsg(vio_odom_q, vio_odom_quaternion);

  std::vector<std::array<int, 2>> matched_indices;
  double roll_vio, pitch_vio, yaw_vio;
  Eigen::Matrix3d vio_odom_rotation_matrix;
  vio_odom_rotation_matrix = vio_odom_quaternion.toRotationMatrix();
  Eigen::Vector3d vio_odom_euler_angles =
      vio_odom_rotation_matrix.eulerAngles(0, 1, 2);
  roll_vio = vio_odom_euler_angles[0];
  pitch_vio = vio_odom_euler_angles[1];
  yaw_vio = vio_odom_euler_angles[2];
  int exit_sign = 0;

  // Multiple rotation matrix with relative_msts
  for (size_t i = 0; i < relative_msts.size(); i++) {
    relative_msts[i] = vio_odom_rotation_matrix * relative_msts[i];
  }

  double max_yaw_dev = 0.17;
  in_fa = estimate_bearingpos(relative_msts, landmark_positions_, false, 0,
                              max_yaw_dev, matched_indices, position_estimate,
                              yaw_estimate, exit_sign,
                              max_tol_per_landmark_pos_error);
  // YAW estimate should be relative. Should be very small
  ROS_INFO_STREAM("Position Esimtate: x: \n"
                  << position_estimate[0] << " y:" << position_estimate[1]
                  << " z:" << position_estimate[2]);
  ROS_INFO_STREAM("---\n YAW ESTIMATE: " << yaw_estimate);
  if (in_fa == false) {
    ROS_ERROR_STREAM("\n residual on landmark positions is too large");
    sloam_msgs::DetectLoopClosureResult result;
    result.success = false;
    result.status = 3;
    loop_closure_server_ptr_->setSucceeded(result);
    return;
  } else {
    ROS_INFO_STREAM("\n residual on landmark positions is small, GOOD!");
    ROS_INFO("PUBLISHING LOOP CLOSURE POSE");
    ROS_INFO_STREAM("Position Esimtate: x: \n"
                    << position_estimate[0] << " y:" << position_estimate[1]
                    << " z:" << position_estimate[2]);
    ROS_INFO_STREAM("---\n YAW ESTIMATE: " << yaw_estimate);

    for (auto mst : relative_msts) {
      ROS_INFO_STREAM("relative_mst: x: \n"
                       << mst[0] << " y:" << mst[1] << " z:" << mst[2]);
    }

    // Compute the optimized orientation
    // extract roll and pitch from vio odom, update yaw with optimized yaw, then
    // put together quaternion based on this. get tf2 quaternion from vio odom
    Eigen::Matrix3d rotation_matrix_roll;
    Eigen::Matrix3d rotation_matrix_pitch;
    Eigen::Matrix3d rotation_matrix_yaw;
    Eigen::Matrix3d optimized_rotation_matrix;
    // // create a rotation matrix based on the yaw angle
    rotation_matrix_yaw << cos(yaw_estimate), -sin(yaw_estimate), 0.0,
        sin(yaw_estimate), cos(yaw_estimate), 0.0, 0.0, 0.0, 1.0;
    // compose the rotation matrix
    optimized_rotation_matrix = rotation_matrix_yaw * vio_odom_rotation_matrix;
    // convert optimized_rotation_matrix to quaternion
    Eigen::Quaterniond goal_quaternion(optimized_rotation_matrix);
    // publish the loop closure pose as Odometry msg
    sloam_msgs::SemanticLoopClosure loop_closure_pose;
    loop_closure_pose.header.stamp = cur_stamp;
    loop_closure_pose.header.frame_id = "/map";
    loop_closure_pose.vio_odom.pose.pose.position.x = vio_odom_x;
    loop_closure_pose.vio_odom.pose.pose.position.y = vio_odom_y;
    loop_closure_pose.vio_odom.pose.pose.position.z = vio_odom_z;
    loop_closure_pose.vio_odom.pose.pose.orientation = vio_odom_q;

    // start a odometry pose
    nav_msgs::Odometry loop_closure_absolute_pose;

    loop_closure_absolute_pose.pose.pose.position.x = position_estimate[0];
    loop_closure_absolute_pose.pose.pose.position.y = position_estimate[1];
    loop_closure_absolute_pose.pose.pose.position.z = position_estimate[2];
    loop_closure_absolute_pose.pose.pose.orientation =
        tf2::toMsg(goal_quaternion);

    // history pose
    geometry_msgs::Pose hist_pose = optimized_hist_key_pose_.pose.pose;
    
    ROS_INFO_STREAM("hist_pose: x: \n"
                     << hist_pose.position.x << " y:" << hist_pose.position.y
                     << " z:" << hist_pose.position.z);
    // current pose
    geometry_msgs::Pose cur_pose = loop_closure_absolute_pose.pose.pose;
    ROS_INFO_STREAM("cur_pose: x: \n"
                  << hist_pose.position.x << " y:" << hist_pose.position.y
                  << " z:" << hist_pose.position.z);

    // initialize GTSAM pose from hist_pose
    gtsam::Pose3 hist_pose_gtsam;
    hist_pose_gtsam = gtsam::Pose3(
        gtsam::Rot3(hist_pose.orientation.w, hist_pose.orientation.x,
                    hist_pose.orientation.y, hist_pose.orientation.z),
        gtsam::Point3(hist_pose.position.x, hist_pose.position.y,
                      hist_pose.position.z));
    // initialize GTSAM pose from cur_pose
    gtsam::Pose3 cur_pose_gtsam;
    cur_pose_gtsam = gtsam::Pose3(
        gtsam::Rot3(cur_pose.orientation.w, cur_pose.orientation.x,
                    cur_pose.orientation.y, cur_pose.orientation.z),
        gtsam::Point3(cur_pose.position.x, cur_pose.position.y,
                      cur_pose.position.z));
    // calculate relative pose
    gtsam::Pose3 relative_pose_gtsam =
        hist_pose_gtsam.inverse() * cur_pose_gtsam;
    ROS_INFO_STREAM("relative_pose_gtsam: x: \n"
                     << relative_pose_gtsam.x() << " y:"
                     << relative_pose_gtsam.y() << " z:"
                     << relative_pose_gtsam.z());

    // convert relative pose to nav_msgs::Odometry

    nav_msgs::Odometry relative_pose_gtsam_odom;

    geometry_msgs::Pose rosPose;
    // get translation from relative_pose_gtsam
    rosPose.position.x = relative_pose_gtsam.x();
    rosPose.position.y = relative_pose_gtsam.y();
    rosPose.position.z = relative_pose_gtsam.z();
    // print rosPose position
    ROS_INFO_STREAM("loop_closure_pose.optimized_relative_loop_closure_pose: x: \n"
                     << rosPose.position.x << " y:" << rosPose.position.y
                     << " z:" << rosPose.position.z);
    // get orientation from relative_pose_gtsam and convert to quaternion
    gtsam::Quaternion quat = relative_pose_gtsam.rotation().toQuaternion();
    rosPose.orientation.w = quat.w();
    rosPose.orientation.x = quat.x();
    rosPose.orientation.y = quat.y();
    rosPose.orientation.z = quat.z();
    relative_pose_gtsam_odom.pose.pose = rosPose;
    loop_closure_pose.optimized_relative_loop_closure_pose =
        relative_pose_gtsam_odom;

    loop_closure_pose.optimized_key_pose = optimized_hist_key_pose_;
    loop_closure_pose.optimized_key_pose = optimized_hist_key_pose_;
    loop_closure_pose.key_pose_idx.data = key_pose_idx_;
    // LoopClosurePosePub_.publish(loop_closure_pose);
    // published_loop_closure_pose_ = true;

    sloam_msgs::DetectLoopClosureResult result;
    result.success = true;
    result.status = 0;
    result.loop_closure_msg = loop_closure_pose;
    // FIXME: here, the optimized_hist_key_pose_ is not used. Since the activeInputNode
    // Will take the index and fetch the keypose from factorgraph
    loop_closure_server_ptr_->setSucceeded(result);
    ROS_INFO("Loop Closure Server send back result!!!");
    return;
  }

}


