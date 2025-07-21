#include "activeSlamInputNode.h"

#include "backward.hpp"

namespace backward {
backward::SignalHandling sh;
}

ActiveSlamInputNode::ActiveSlamInputNode(ros::NodeHandle &nh)
    : nh_(nh), tf_listener_{tf_buffer_} {
  nh_.param<float>("run_factor_graph_rate", runFactorGraphRate_, 5.0);
  timer_ = nh.createTimer(ros::Duration(1.0 / runFactorGraphRate_),
                          &ActiveSlamInputNode::runFactorGraphNode, this);
  log_timer_ = nh.createTimer(ros::Duration(0.5), &ActiveSlamInputNode::logEntropyCB, this);

  nh_.param<float>("min_odom_distance", minOdomDistance_, 0.1);
  // x y position noise std
  nh_.param<double>("odom_factor_position_noise_std_xy", odomFactorPosStdXY_,
                    3.0);
  // z position noise std
  nh_.param<double>("odom_factor_position_noise_std_z", odomFactorPosStdZ_,
                    0.3);

  // orientation noise std in rad
  nh_.param<double>("odom_factor_orientation_noise_std", odomFactorRotStd_,
                    0.0001);
  nh_.param<double>("odom_factor_noise_std_inflation_per_min",
                    odomFactorStdInflation_, 0.0);

  nh_.param<double>("data_association_distance_normed",
                    data_association_distance_, 1.0);

  // bearing noise td in rad
  nh_.param<double>("bearing_factor_noise_std", bearingFactorStd_, 1.0);
  nh_.param<double>("range_factor_noise_std", rangeFactorStd_, 10.0);

  maxQueueSize_ = nh_.param("max_queue_size", 100);
  maxOdomQueueSize_ = nh_.param("max_odom_queue_size", 300);

  publishTf_ = nh_.param("publish_tf", true);
  nh_.param<std::string>("odom_topic", robot1_odom_topic_,
                         "/quadrotor/vio/odom");

  nh_.param<double>("closure_noise_std_wrt_odom_factor",
                    closure_noise_std_wrt_odom_factor_, 0.001);

  nh_.param<std::string>("loop_closure_odom_topic",
                         robot1_loop_closure_odom_topic_, "/loop_closure/odom");
  nh_.param<std::string>("robot_frame_id", robot_frame_id_,
                         "quadrotor/base_link");
  nh_.param<std::string>("map_frame_id", map_frame_id_, "quadrotor/map");

  pubRobot1HighFreqSLOAMPose_ =
      nh_.advertise<geometry_msgs::PoseStamped>("quadrotor/high_freq_pose", 10);

  pubLatestPoseIdx_ =
      nh_.advertise<std_msgs::UInt64>("/factor_graph_atl/latest_factor_graph_key_pose_idx", 10);

  pubRobot1Trajectory_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "optimized_trajectory", 1, true);
  pubRobot1TrajPoseInds_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "optimized_trajectory_with_pose_inds", 1, true);
  pubAllPointLandmarks_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "optimized_point_landmarks", 1, true);

  pubRobot1HighFreqSLOAMOdom_ =
      nh_.advertise<nav_msgs::Odometry>("quadrotor/high_freq_odom", 20);

  

  pubSloamToVioOdom_ =
    nh_.advertise<nav_msgs::Odometry>("quadrotor/sloam_to_vio_odom", 20);

  pubRobotMeaSyncOdom_ = nh_.advertise<sloam_msgs::ROSRangeBearingSyncOdom>(
      "quadrotor/measurements_with_sync_odom", 20);

  pubRobotHighFreqSyncOdom_ = nh_.advertise<sloam_msgs::ROSSyncOdom>(
      "quadrotor/high_freq_sync_odom", 20);

  Robot1OdomSub_ = nh_.subscribe(robot1_odom_topic_, 10,
                                 &ActiveSlamInputNode::Robot1OdomCb_, this);

  Robot1LoopClosureSub_ =
      nh_.subscribe(robot1_loop_closure_odom_topic_, 10,
                    &ActiveSlamInputNode::Robot1LoopClosureCb_, this);

  // LoopClosureRequestIdxSub_ =
  //     nh_.subscribe("/loop_closure/loop_closure_request_idx", 1,
  //                   &ActiveSlamInputNode::LoopClosureRequestIdxCb_, this);

  std::string ugv0_bearing_topic = "/semantic_range_bearing_measurements";
  Ugv0BearingSub_ = nh_.subscribe(ugv0_bearing_topic, 10,
                                  &ActiveSlamInputNode::Ugv0BearingCb_, this);
  // auto sloam_ptr = boost::make_shared<sloam::SLOAMNode>(nh_);
  // sloam_ = std::move(sloam_ptr);

  robot1OdomCounter_ = 0;

  Vector6 noise_vec_pose_temp;
  noise_vec_pose_temp << odomFactorRotStd_, odomFactorRotStd_,
      odomFactorRotStd_, odomFactorPosStdXY_, odomFactorPosStdXY_,
      odomFactorPosStdZ_;
  // VERY IMPORTANT: the first three corresponds to Rotation and the last three
  // corresponds to Translation
  factorGraph_.noise_model_pose =
      noiseModel::Diagonal::Sigmas(noise_vec_pose_temp);
  factorGraph_.noise_model_pose_vec_per_m = noise_vec_pose_temp;

  // TODO: scale this with the motion distance
  factorGraph_.noise_model_closure =
      noiseModel::Diagonal::Sigmas(noise_vec_pose_temp * closure_noise_std_wrt_odom_factor_);

  // ##################### IMPORTANT PARAMS for ATL ###################
  factorGraph_.noise_model_bearing = noiseModel::Diagonal::Sigmas(
      (Vector(3) << bearingFactorStd_, bearingFactorStd_, rangeFactorStd_)
          .finished());
  factorGraph_.noise_model_pose_inflation = odomFactorStdInflation_;

  ROS_INFO("factor graph initialized");

  // ###################### INIT Active Loop Closure Server
  // ####################### Action server
  active_loop_closure_server_ptr_.reset(new ActiveLoopClosureServerType(
      nh_, "/loop_closure/active_loop_closure_server", false));
  active_loop_closure_server_ptr_->registerGoalCallback(
      boost::bind(&ActiveSlamInputNode::ActiveLoopClosureGoalCb_, this));
  active_loop_closure_server_ptr_->registerPreemptCallback(
      boost::bind(&ActiveSlamInputNode::ActiveLoopClosurePreemptCb_, this));
  active_loop_closure_server_ptr_->start();
  newLoopClosureTriggered_ = false;
  loop_closure_max_attempts_ = 50;
  // ######################  INIT Detect Loop Closure Client
  // #######################
  server_wait_timeout_ = 5.0;
  detectLoopClosureServerTriggered_ = false;
  loop_closure_num_attempts_ = 0;
  detect_loop_closure_client_ptr_.reset(new DetectLoopClosureClientType(
      nh_, "/loop_closure/detect_loop_closure_server", true));
  if (!detect_loop_closure_client_ptr_->waitForServer(
          ros::Duration(server_wait_timeout_))) {
    ROS_ERROR("Detect Loop Closure server not found.");
  }

  // #######  INIT estimate info gain service server #######
  estimate_info_gain_server_ =
      nh_.advertiseService("/estimate_info_gain_server",
                           &ActiveSlamInputNode::EstimateInfoGainCb_, this);
}

// this should be a service request callback
// We receive a request to do loop closure, this request should
// include the key pose index that we want to find a closure for
// Currently, this is implemented as a ros message
// void ActiveSlamInputNode::LoopClosureRequestIdxCb_(
//     const std_msgs::UInt64 &msg) {
//   closure_request_key_pose_idx_ = static_cast<size_t>(msg.data);
//   newLoopClosureTriggered_ = true;
//   ROS_INFO("new loop closure triggered");
// }

bool ActiveSlamInputNode::EstimateInfoGainCb_(
    sloam_msgs::EvaluateLoopClosure::Request &req,
    sloam_msgs::EvaluateLoopClosure::Response &res) {
  // Header header
  // bool add_factor # if is best action
  // double travel_distance # distance to travel
  // std_msgs/UInt64 current_pose_id # current key pose id in factor graph
  // std_msgs/UInt64 closure_key_pose_id # history key pose id for candidate
  // loop closure in factor graph
  // ---
  // double information_gain # information gain
  // bool success # if success

  // if (req.reset) {
  //   bool reset_success = factorGraph_.resetFakeClosureGraph();
  //   res.success = reset_success;
  //   return true;
  // }

  // cast current_pose_id and closure_key_pose_id into size_t

  std::vector<size_t> c_traj_pose_indices = req.candidate_traj_pose_indices;
  std::vector<double> t_distances = req.travel_distances;

  // if (req.add_factor) {
  //   int success = factorGraph_.addFakeClosureFactor(
  //       current_pose_id, closure_key_pose_id, req.travel_distance);
  //   if (success == -1) {
  //     res.success = false;
  //   } else {
  //     res.success = true;
  //     res.information_gain = 0.0;
  //   }
  // } else {
  double result =
      factorGraph_.estimateClosureInfoGain(c_traj_pose_indices, t_distances);
  if (result == -1) {
    res.success = false;
  } else {
    res.success = true;
    res.information_gain = result;
  }
  // }
  return true;
}

void ActiveSlamInputNode::logEntropyCB(const ros::TimerEvent &e) {
  ROS_WARN_THROTTLE(1, "Logging entropy");
  factorGraph_.logEntropy();
}

void ActiveSlamInputNode::ActiveLoopClosureGoalCb_() {
  // If there is another goal active, cancel it
  if (active_loop_closure_server_ptr_->isActive()) {
    ROS_WARN("Received a new loop closure goal, canceling the previous one");
    active_loop_closure_server_ptr_->setAborted();
    newLoopClosureTriggered_ = false;
    // Cancel the current detect loop closure goal and reset param
    detect_loop_closure_client_ptr_->cancelGoal();
    detectLoopClosureServerTriggered_ = false;
    loop_closure_num_attempts_ = 0;
  }

  ROS_ERROR("Received a new loop closure goal");
  // Pointer to msg
  const auto msg = active_loop_closure_server_ptr_->acceptNewGoal();

  if (active_loop_closure_server_ptr_->isPreemptRequested()) {
    ROS_INFO("Active Loop Closure goal preempted.");
    active_loop_closure_server_ptr_->setPreempted();
    newLoopClosureTriggered_ = false;
    detect_loop_closure_client_ptr_->cancelGoal();
    detectLoopClosureServerTriggered_ = false;
    loop_closure_num_attempts_ = 0;
    return;
  }

  closure_request_key_pose_idx_ = static_cast<size_t>(msg->key_pose_idx.data);
  latest_submap_ = msg->submap;
  // closure_request_key_pose_idx_ =
  // static_cast<size_t>(goal->key_pose_idx.data);
  newLoopClosureTriggered_ = true;
  ROS_ERROR("new loop closure triggered");
  ROS_ERROR("new loop closure triggered");
  ROS_ERROR("new loop closure triggered");
  ROS_ERROR("new loop closure triggered");
  ROS_ERROR("new loop closure triggered");
  ROS_ERROR("new loop closure triggered");
  ROS_ERROR("new loop closure triggered");
}

void ActiveSlamInputNode::ActiveLoopClosurePreemptCb_() {
  if (active_loop_closure_server_ptr_->isActive()) {
    ROS_INFO("Active Loop Closure goal aborted.");
    active_loop_closure_server_ptr_->setAborted();
    detect_loop_closure_client_ptr_->cancelGoal();
    detectLoopClosureServerTriggered_ = false;
    loop_closure_num_attempts_ = 0;
  } else {
    ROS_INFO("Active Loop Closure goal preempted.");
    active_loop_closure_server_ptr_->setPreempted();
    detect_loop_closure_client_ptr_->cancelGoal();
    detectLoopClosureServerTriggered_ = false;
    loop_closure_num_attempts_ = 0;
  }
  newLoopClosureTriggered_ = false;
}

void ActiveSlamInputNode::Robot1OdomCb_(
    const nav_msgs::OdometryConstPtr &odom_msg) {
  // ROS_INFO("entering odom callback");
  auto pose = odom_msg->pose.pose;
  ros::Time odomStamp = odom_msg->header.stamp;
  Quat rot(pose.orientation.w, pose.orientation.x, pose.orientation.y,
           pose.orientation.z);
  Vector3 pos(pose.position.x, pose.position.y, pose.position.z);

  boost::array<double, 36> odom_cov(odom_msg->pose.covariance);

  SE3 odom = SE3();
  odom.setQuaternion(rot);
  odom.translation() = pos;

  odomUpdated_ = true;
  robot1OdomQueueWithCov_.emplace_back(odom, odom_cov, 0, odomStamp);
  if (robot1OdomQueueWithCov_.size() > maxOdomQueueSize_)
    robot1OdomQueueWithCov_.pop_front();
}

// This function is the callback function to handle the loop closure message
// after loop closure has been detected from "mapMerging" Node
// This function process the message and save the loop closure information
void ActiveSlamInputNode::Robot1LoopClosureCb_(
    const sloam_msgs::SemanticLoopClosureConstPtr &odom_msg) {
  ROS_ERROR("Loop closure message received!!!");
  ROS_ERROR("Loop closure message received!!!");
  ROS_ERROR("Loop closure message received!!!");
  ROS_ERROR("Loop closure message received!!!");
  // ROS_INFO("entering odom callback");
  auto pose = odom_msg->optimized_relative_loop_closure_pose.pose.pose;
  ros::Time odomStamp = odom_msg->header.stamp;
  ROS_INFO_STREAM("Quat" << pose.orientation.w << " " << pose.orientation.x
                         << " " << pose.orientation.y << " "
                         << pose.orientation.z);
  Quat rot(pose.orientation.w, pose.orientation.x, pose.orientation.y,
           pose.orientation.z);

  Vector3 pos(pose.position.x, pose.position.y, pose.position.z);

  boost::array<double, 36> loop_closure_relative_pose_cov(
      odom_msg->optimized_relative_loop_closure_pose.pose.covariance);

  // here the odom should be relative pose from key pose to current pose upon
  // loop closure
  SE3 loop_closure_relative_pose = SE3();
  loop_closure_relative_pose.setQuaternion(rot);
  loop_closure_relative_pose.translation() = pos;
  size_t closure_matched_pose_idx =
      static_cast<size_t>(odom_msg->key_pose_idx.data);

  // sloam odom queue
  // robot1LoopClosureQueueWithCov_.emplace_back(loop_closure_relative_pose,
  // loop_closure_relative_pose_cov, odomStamp);

  latestClosureRelativePose_ = StampedSE3IdxCov(
      loop_closure_relative_pose, loop_closure_relative_pose_cov,
      closure_matched_pose_idx, odomStamp);

  auto vio_pose = odom_msg->vio_odom.pose.pose;
  ROS_INFO_STREAM(
      "Quat" << vio_pose.orientation.w << " " << vio_pose.orientation.x << " "
             << vio_pose.orientation.y << " " << vio_pose.orientation.z);
  Quat rot2(vio_pose.orientation.w, vio_pose.orientation.x,
            vio_pose.orientation.y, vio_pose.orientation.z);
  Vector3 pos2(vio_pose.position.x, vio_pose.position.y, vio_pose.position.z);

  boost::array<double, 36> odom_vio_cov(odom_msg->vio_odom.pose.covariance);

  SE3 odom_vio = SE3();
  odom_vio.setQuaternion(rot2);
  odom_vio.translation() = pos2;

  loopClosureFound_ = true;
  waitingForClosure_ = false;

  // vio odom queue
  // robot1LoopClosureVIOQueueWithCov_.emplace_back(odom_vio, odom_vio_cov,
  //                                                odomStamp);
  latestClosureRawVIO_ = StampedSE3IdxCov(odom_vio, odom_vio_cov,
                                          closure_matched_pose_idx, odomStamp);

  // ##################### Handle Action Server Related #####################
  // TODO: need to figure out a way to set the result to be failed if we
  // couldn't find closure make sure that the action hasn't been canceled
  if (!active_loop_closure_server_ptr_->isActive()) {
    ROS_ERROR("[Loop Find Callback] Active Loop Closure Server is not active");
    ROS_ERROR("[Loop Find Callback] Active Loop Closure Server is not active");
    ROS_ERROR("[Loop Find Callback] Active Loop Closure Server is not active");
    return;
  } else {
    ROS_ERROR("[Loop Find Callback] Active Loop Closure Server is active");
    ROS_ERROR("[Loop Find Callback] Active Loop Closure Server is active");
    ROS_ERROR("[Loop Find Callback] Active Loop Closure Server is active");
  }
  // Compose result message
  sloam_msgs::ActiveLoopClosureResult result;
  result.success = true;
  result.vio_pose = odom_msg->vio_odom;
  result.relativeMotion = odom_msg->optimized_relative_loop_closure_pose;
  result.optimized_key_pose = odom_msg->optimized_key_pose;
  ROS_ERROR("===============================================");
  ROS_ERROR("===============================================");
  ROS_ERROR("===============================================");
  ROS_ERROR_STREAM("ActiveSlamInputNode: optimized key pose is: " << result.optimized_key_pose.pose.pose.position.x << " " << result.optimized_key_pose.pose.pose.position.y << " " << result.optimized_key_pose.pose.pose.position.z);
  ROS_ERROR("===============================================");
  ROS_ERROR("===============================================");
  ROS_ERROR("===============================================");

  // Set action server to succeeded
  active_loop_closure_server_ptr_->setSucceeded(result);
  ROS_ERROR(
      "[Loop Find Callback] Active Loop Closure Server is set to succeeded");
  ROS_ERROR(
      "[Loop Find Callback] Active Loop Closure Server is set to succeeded");
  ROS_ERROR(
      "[Loop Find Callback] Active Loop Closure Server is set to succeeded");

  // Flip the trigger flag, incicating we are done with this closure
  // TODO: make this a service/action server
  newLoopClosureTriggered_ = false;
  // loop_closure_->transitLoopClosureState(LOOP_CLOSURE_STATE::WAIT_FOR_TRIGGER,
  // "ActiveSlamInputNode");
}

void ActiveSlamInputNode::detectLoopClosureDoneCallback(
    const actionlib::SimpleClientGoalState &state,
    const sloam_msgs::DetectLoopClosureResultConstPtr &result) {
  ROS_ERROR("[ActiveSLAMNode] Loop closure message received!!!");
  ROS_ERROR("[ActiveSLAMNode] Loop closure message received!!!");
  ROS_ERROR("[ActiveSLAMNode] Loop closure message received!!!");
  ROS_ERROR("[ActiveSLAMNode] Loop closure message received!!!");
  if (!result->success) {
    ROS_ERROR("Loop closure not found!!!");
    ROS_ERROR("Loop closure not found!!!");
    ROS_ERROR("Loop closure not found!!!");
    if (result->status == 3) loop_closure_num_attempts_++;
    detectLoopClosureServerTriggered_ = false;
    if (loop_closure_num_attempts_ >= loop_closure_max_attempts_) {
      ROS_ERROR_STREAM("Loop closure failed after "
                       << loop_closure_num_attempts_ << " attempts");
      ROS_ERROR_STREAM("Loop closure failed after "
                       << loop_closure_num_attempts_ << " attempts");
      ROS_ERROR_STREAM("Loop closure failed after "
                       << loop_closure_num_attempts_ << " attempts");
      newLoopClosureTriggered_ = false;
      waitingForClosure_ = false;
      loop_closure_num_attempts_ = 0;
      if (!active_loop_closure_server_ptr_->isActive()) {
        ROS_ERROR(
            "[detectLoopClosureDoneCallback] Active Loop Closure Server is not "
            "active");
        return;
      } else {
        ROS_INFO(
            "[detectLoopClosureDoneCallback] Active Loop Closure Server is "
            "active");
        // Set result to be failed and send it back
        sloam_msgs::ActiveLoopClosureResult active_loop_closure_result;
        active_loop_closure_result.success = false;
        active_loop_closure_server_ptr_->setSucceeded(
            active_loop_closure_result);
      }
    }
    return;
  }
  // ROS_INFO("entering odom callback");

  auto pose =
      result->loop_closure_msg.optimized_relative_loop_closure_pose.pose.pose;
  ros::Time odomStamp = result->loop_closure_msg.header.stamp;
  ROS_INFO_STREAM("Quat" << pose.orientation.w << " " << pose.orientation.x
                         << " " << pose.orientation.y << " "
                         << pose.orientation.z);
  Quat rot(pose.orientation.w, pose.orientation.x, pose.orientation.y,
           pose.orientation.z);

  Vector3 pos(pose.position.x, pose.position.y, pose.position.z);

  boost::array<double, 36> loop_closure_relative_pose_cov(
      result->loop_closure_msg.optimized_relative_loop_closure_pose.pose
          .covariance);

  // here the odom should be relative pose from key pose to current pose upon
  // loop closure
  SE3 loop_closure_relative_pose = SE3();
  loop_closure_relative_pose.setQuaternion(rot);
  loop_closure_relative_pose.translation() = pos;
  size_t closure_matched_pose_idx =
      static_cast<size_t>(result->loop_closure_msg.key_pose_idx.data);
  ROS_ERROR_STREAM("closure_matched_pose_idx 1 is: " << closure_matched_pose_idx);
  ROS_ERROR_STREAM("closure_matched_pose_idx 1 is: " << closure_matched_pose_idx);
  ROS_ERROR_STREAM("closure_matched_pose_idx 1 is: " << closure_matched_pose_idx);
  ROS_ERROR_STREAM("closure_matched_pose_idx 1 is: " << closure_matched_pose_idx);
  ROS_ERROR_STREAM("closure_matched_pose_idx 1 is: " << closure_matched_pose_idx);
  ROS_ERROR_STREAM("closure_matched_pose_idx 1 is: " << closure_matched_pose_idx);

  // sloam odom queue
  // robot1LoopClosureQueueWithCov_.emplace_back(loop_closure_relative_pose,
  // loop_closure_relative_pose_cov, odomStamp);

  latestClosureRelativePose_ = StampedSE3IdxCov(
      loop_closure_relative_pose, loop_closure_relative_pose_cov,
      closure_matched_pose_idx, odomStamp);

  auto vio_pose = result->loop_closure_msg.vio_odom.pose.pose;
  ROS_INFO_STREAM(
      "Quat" << vio_pose.orientation.w << " " << vio_pose.orientation.x << " "
             << vio_pose.orientation.y << " " << vio_pose.orientation.z);
  Quat rot2(vio_pose.orientation.w, vio_pose.orientation.x,
            vio_pose.orientation.y, vio_pose.orientation.z);
  Vector3 pos2(vio_pose.position.x, vio_pose.position.y, vio_pose.position.z);

  boost::array<double, 36> odom_vio_cov(
      result->loop_closure_msg.vio_odom.pose.covariance);

  SE3 odom_vio = SE3();
  odom_vio.setQuaternion(rot2);
  odom_vio.translation() = pos2;

  loopClosureFound_ = true;
  waitingForClosure_ = false;

  // vio odom queue
  // robot1LoopClosureVIOQueueWithCov_.emplace_back(odom_vio, odom_vio_cov,
  //                                                odomStamp);
  latestClosureRawVIO_ = StampedSE3IdxCov(odom_vio, odom_vio_cov,
                                          closure_matched_pose_idx, odomStamp);

  // ##################### Handle Action Server Related #####################
  // TODO: need to figure out a way to set the result to be failed if we
  // couldn't find closure make sure that the action hasn't been canceled
  if (!active_loop_closure_server_ptr_->isActive()) {
    ROS_ERROR("[Loop Find Callback] Active Loop Closure Server is not active");
    ROS_ERROR("[Loop Find Callback] Active Loop Closure Server is not active");
    ROS_ERROR("[Loop Find Callback] Active Loop Closure Server is not active");
    return;
  } else {
    ROS_ERROR("[Loop Find Callback] Active Loop Closure Server is active");
    ROS_ERROR("[Loop Find Callback] Active Loop Closure Server is active");
    ROS_ERROR("[Loop Find Callback] Active Loop Closure Server is active");
  }
  // Compose result message
  sloam_msgs::ActiveLoopClosureResult active_loop_closure_result;
  active_loop_closure_result.success = true;
  active_loop_closure_result.vio_pose = result->loop_closure_msg.vio_odom;
  active_loop_closure_result.relativeMotion =
      result->loop_closure_msg.optimized_relative_loop_closure_pose;
  active_loop_closure_result.optimized_key_pose =
      result->loop_closure_msg.optimized_key_pose;
  // Set action server to succeeded
  active_loop_closure_server_ptr_->setSucceeded(active_loop_closure_result);
  ROS_ERROR(
      "[Loop Find Callback] Active Loop Closure Server is set to succeeded");
  ROS_ERROR(
      "[Loop Find Callback] Active Loop Closure Server is set to succeeded");
  ROS_ERROR(
      "[Loop Find Callback] Active Loop Closure Server is set to succeeded");

  // Flip the trigger flag, incicating we are done with this closure
  newLoopClosureTriggered_ = false;
  detect_loop_closure_client_ptr_->cancelGoal();
  detectLoopClosureServerTriggered_ = false;
  loop_closure_num_attempts_ = 0;
}

void ActiveSlamInputNode::Ugv0BearingCb_(
    const sloam_msgs::ROSRangeBearingConstPtr &range_bearing_msg) {
  lastBearingMsg_ = range_bearing_msg;
  auto cur_stamp = range_bearing_msg->odom.header.stamp;
  // First handle the odom and save as the data structure consistent
  // with odom in odom queue
  auto pose = range_bearing_msg->odom.pose.pose;
  ros::Time odomStamp = range_bearing_msg->odom.header.stamp;
  Quat rot(pose.orientation.w, pose.orientation.x, pose.orientation.y,
           pose.orientation.z);
  Vector3 pos(pose.position.x, pose.position.y, pose.position.z);

  boost::array<double, 36> odom_cov(range_bearing_msg->odom.pose.covariance);

  SE3 odom = SE3();
  odom.setQuaternion(rot);
  odom.translation() = pos;
  StampedSE3IdxCov odom_from_msg = StampedSE3IdxCov(odom, odom_cov, 0, odomStamp);

  // Now handle range bearing measurements
  std::vector<gtsam::Point3> bearing_factors;
  for (size_t i = 0; i < range_bearing_msg->bearing_factors.size(); i++) {
    gtsam::Point3 current_bearing_meas{range_bearing_msg->bearing_factors[i].x,
                                       range_bearing_msg->bearing_factors[i].y,
                                       range_bearing_msg->bearing_factors[i].z};
    bearing_factors.push_back(current_bearing_meas);
  }
  std::vector<double> range_factors;
  for (size_t i = 0; i < range_bearing_msg->range_factors.size(); i++) {
    range_factors.push_back(range_bearing_msg->range_factors[i]);
  }
  std::vector<gtsam::Point3> landmark_body_positions;
  for (size_t i = 0; i < range_bearing_msg->landmark_body_positions.size();
       i++) {
    gtsam::Point3 current_position_meas{
        range_bearing_msg->landmark_body_positions[i].x,
        range_bearing_msg->landmark_body_positions[i].y,
        range_bearing_msg->landmark_body_positions[i].z};
    landmark_body_positions.push_back(current_position_meas);
  }
  std::vector<size_t> meas_ids;
  for (size_t i = 0; i < range_bearing_msg->meas_ids.size(); i++) {
    // format range_bearing_msg->meas_ids[i] into size_t
    meas_ids.push_back(
        static_cast<size_t>(range_bearing_msg->meas_ids[i].data));
  }
  ugv0BearingUpdated_ = true;
  ugv0BearingQueue_.emplace_back(bearing_factors, range_factors, meas_ids,
                                 landmark_body_positions, odom_from_msg, cur_stamp);

  if (ugv0BearingQueue_.size() > 1) ugv0BearingQueue_.pop_front();

  // iterate through range_bearing_msg->bearing_factors
  // Note: This is the old implementation. We don't need the queue now as we are not
  // going to sync the measurement with Odom from odom queue, we will take the odom
  // from the message directly.
  // std::vector<gtsam::Point3> bearing_factors;
  // for (size_t i = 0; i < range_bearing_msg->bearing_factors.size(); i++) {
  //   gtsam::Point3 current_bearing_meas{range_bearing_msg->bearing_factors[i].x,
  //                                      range_bearing_msg->bearing_factors[i].y,
  //                                      range_bearing_msg->bearing_factors[i].z};
  //   bearing_factors.push_back(current_bearing_meas);
  // }
  // std::vector<double> range_factors;
  // for (size_t i = 0; i < range_bearing_msg->range_factors.size(); i++) {
  //   range_factors.push_back(range_bearing_msg->range_factors[i]);
  // }
  // std::vector<gtsam::Point3> landmark_body_positions;
  // for (size_t i = 0; i < range_bearing_msg->landmark_body_positions.size();
  //      i++) {
  //   gtsam::Point3 current_position_meas{
  //       range_bearing_msg->landmark_body_positions[i].x,
  //       range_bearing_msg->landmark_body_positions[i].y,
  //       range_bearing_msg->landmark_body_positions[i].z};
  //   landmark_body_positions.push_back(current_position_meas);
  // }
  // std::vector<size_t> meas_ids;
  // for (size_t i = 0; i < range_bearing_msg->meas_ids.size(); i++) {
  //   // format range_bearing_msg->meas_ids[i] into size_t
  //   meas_ids.push_back(
  //       static_cast<size_t>(range_bearing_msg->meas_ids[i].data));
  // }
  // ugv0BearingUpdated_ = true;
  // ugv0BearingQueue_.emplace_back(bearing_factors, range_factors, meas_ids,
  //                                landmark_body_positions, cur_stamp);

  // if (ugv0BearingQueue_.size() > maxQueueSize_) ugv0BearingQueue_.pop_front();
}

void ActiveSlamInputNode::resetAllFlags() {
  // set all flag as false to avoid adding same factors repetitively
  ROS_WARN("Reset all flags!!!!!!!!!!!!!");
  // ROS_WARN("Reset all flags!!!!!!!!!!!!!");
  // ROS_WARN("Reset all flags!!!!!!!!!!!!!");
  // loopClosureFound_ = false;
  ugv0BearingUpdated_ = false;
  odomUpdated_ = false;
}

void ActiveSlamInputNode::runFactorGraphNode(const ros::TimerEvent &e) {
  auto odomQueue = robot1OdomQueueWithCov_;
  int robotID = 0;
  // ros::Rate loop_rate(5);

  // ROS_INFO("entering run factor graph callback");
  if (odomQueue.size() == 0) {
    ROS_INFO_STREAM("Odom queue is empty...");
  } else {
    // first, publish the odometry based on the last sloam call's optimized pose
    // and current relative transform estimated by raw odometry
    SE3 highFreqSLOAMPose;
    ros::Time odom_stamp;

    auto cur_vio_odom = odomQueue.back();
    if (robot1OdomReceived_) {
      // if odom has been updated in sloam, use the relative odom pose and add
      // it to the previous sloam pose
      SE3 latestRelativeMotionFactorGraph =
          robot1LatestOdom.pose.inverse() * cur_vio_odom.pose;
      highFreqSLOAMPose =
          robot1LastSLOAMKeyPose_ * latestRelativeMotionFactorGraph;
      odom_stamp = cur_vio_odom.stamp;
      // ROS_ERROR_STREAM("SLOAM KEY POSE " <<
      // robot1LastSLOAMKeyPose_.matrix());
      // ROS_ERROR_STREAM("latestRelativeMotionFactorGraph "
      //  << latestRelativeMotionFactorGraph.matrix());
      // ROS_ERROR_STREAM("highFreqSLOAMPose " << highFreqSLOAMPose.matrix());
    } else {
      // directly take current odometry and publish
      highFreqSLOAMPose = cur_vio_odom.pose;
      odom_stamp = cur_vio_odom.stamp;
    }
    sloam_msgs::ROSSyncOdom syncOdom;
    syncOdom.header.stamp = odom_stamp;
    auto odom_msg = sloam::toRosOdom_(highFreqSLOAMPose, map_frame_id_, odom_stamp);
    syncOdom.vio_odom = sloam::toRosOdom_(cur_vio_odom.pose, map_frame_id_, odom_stamp);
    syncOdom.sloam_odom = odom_msg;
    // calculate this for drift compensation
    SE3 sloam_to_vio_tf = computeSloamToVioOdomTransform(highFreqSLOAMPose, cur_vio_odom.pose);
    // publish sloam_to_vio_tf as a odometry message
    auto sloam_to_vio_msg = sloam::toRosOdom_(sloam_to_vio_tf, map_frame_id_, odom_stamp);
    pubSloamToVioOdom_.publish(sloam_to_vio_msg);


    pubRobot1HighFreqSLOAMPose_.publish(
        sloam::makeROSPose(highFreqSLOAMPose, map_frame_id_, odom_stamp));
    pubRobot1HighFreqSLOAMOdom_.publish(odom_msg);
    pubRobotHighFreqSyncOdom_.publish(syncOdom);

    if (newLoopClosureTriggered_) {
      // NOTE: current logic:
      // When input node receive loop closure trigger,
      // It will check distance between sloam pose and the optimized key pose
      // if the distance is small enough, it will stop adding factors
      // if the distance is small than a threshold (5 now), it will publish
      // synced odom together will keypose indec

      sloam_msgs::ROSRangeBearingSyncOdom syncMeaOdom;
      // Note:
      // If we assume measurement and the synced vio odom from the message has a long
      // latency. The highFreqSLOAMPose computed above might not be correct
      // How should we handle this case? 
      // I recompute the sloam pose according to vio odom from msg and publish
      // the measurement + vio odom + sloam odom for detecting loop closure
      
      // sloam_msgs::SemanticLoopClosure syncOdom;
      // Fetch loop closure history key pose
      SE3 optimized_key_pose;
      if (factorGraph_.getPoseByID(optimized_key_pose,
                                   closure_request_key_pose_idx_)) {
        syncMeaOdom.optimized_key_pose =
            sloam::toRosOdom_(optimized_key_pose, map_frame_id_, odom_stamp);

        // check if odom is close enough to the candidate loop closure pose
        double dist =
            (highFreqSLOAMPose.translation() - optimized_key_pose.translation())
                .norm();
        if (dist < 15) {
          ROS_INFO("stopping adding factors, waiting for closure...");
          waitingForClosure_ = true;
        }
        if (dist > 8) {
          ROS_WARN_STREAM(
              "Current pose is too far from the loop closure "
              "candidate pose, skipping loop closure, their distance is "
              << dist);
        } else {
          // If not trigger detect loop closure yet, trigger it
          if (!detectLoopClosureServerTriggered_ &&
              (loop_closure_num_attempts_ < loop_closure_max_attempts_)) {
            ROS_ERROR_STREAM("triggering loop closure server, num attempts: "
                             << loop_closure_num_attempts_);
            ROS_ERROR_STREAM("triggering loop closure server, num attempts: "
                             << loop_closure_num_attempts_);
            ROS_ERROR_STREAM("triggering loop closure server, num attempts: "
                             << loop_closure_num_attempts_);

            detectLoopClosureServerTriggered_ = true;
            // Send detectLoopClosureGoal
            // TODO: add submap to goal
            sloam_msgs::DetectLoopClosureGoal goal;
            // parse submap, publish to loopClosureServer
            goal.submap = latest_submap_;
            detect_loop_closure_client_ptr_->sendGoal(
                goal,
                boost::bind(&ActiveSlamInputNode::detectLoopClosureDoneCallback,
                            this, _1, _2),
                DetectLoopClosureClientType::SimpleActiveCallback(),
                DetectLoopClosureClientType::SimpleFeedbackCallback());
          }

          ROS_INFO_STREAM(
              "odom is close enough to the candidate loop closure "
              "pose, publishing msg to map merging node");
          // Compute sloam pose
          SE3 syncedSloamPose;
          auto syncedOdomMeasurement = ugv0BearingQueue_.back();
          if (robot1OdomReceived_) {
            SE3 relativeMotion = robot1LatestOdom.pose.inverse() * syncedOdomMeasurement.odom_position.pose;
            syncedSloamPose = robot1LastSLOAMKeyPose_ * relativeMotion;
            odom_stamp = syncedOdomMeasurement.stamp;
          } else {
            // directly take current odometry and publish
            syncedSloamPose = syncedOdomMeasurement.odom_position.pose;
            odom_stamp = syncedOdomMeasurement.stamp;
          }
          syncMeaOdom.header.stamp = odom_stamp;
          syncMeaOdom.bearing_factors = lastBearingMsg_->bearing_factors;
          syncMeaOdom.range_factors = lastBearingMsg_->range_factors;
          syncMeaOdom.landmark_body_positions =
              lastBearingMsg_->landmark_body_positions;
          syncMeaOdom.meas_ids = lastBearingMsg_->meas_ids;
          syncMeaOdom.vio_odom = lastBearingMsg_->odom;
          syncMeaOdom.corrected_odom =
              sloam::toRosOdom_(syncedSloamPose, map_frame_id_, odom_stamp);
          syncMeaOdom.key_pose_idx.data = closure_request_key_pose_idx_;
          pubRobotMeaSyncOdom_.publish(syncMeaOdom);

          // syncOdom.header.stamp = odom_stamp;
          // // SLOAM Pose
          // syncOdom.corrected_odom =
          //     sloam::toRosOdom_(highFreqSLOAMPose, map_frame_id_, odom_stamp);
          // // Map Pose
          // syncOdom.vio_odom.pose.pose = sloam::toRosPose_(cur_vio_odom.pose);
          // // Key pose index
          // syncOdom.key_pose_idx.data = closure_request_key_pose_idx_;
          // pubRobotMeaSyncOdom_.publish(syncOdom);
          // loop_closure_->updateSyncOdomQueue(syncOdom);
          // if (loop_closure_->getState() ==
          // LOOP_CLOSURE_STATE::WAIT_FOR_TRIGGER || loop_closure_->getState()
          // == LOOP_CLOSURE_STATE::FAIL) { loop_closure_->findLoopClosure();
          // }
        }
      } else {
        ROS_ERROR_STREAM("Failed to fetch optimized key pose with ID "
                         << closure_request_key_pose_idx_);
      }
    }
  }

  double latest_ugv_bearing_stamp;
  double ugv0_bearing_latest_stamp;
  gtsam::Point3 bearing_meas_ugv0 = gtsam::Point3(0, 0, 0);
  double range_measurement_ugv0 = 0.0;

  int cur_case = -1;

  std::vector<Point3> bearing_factors;
  std::vector<double> range_factors;
  std::vector<size_t> meas_ids;
  std::vector<Point3> landmark_body_positions;
  StampedSE3IdxCov measurement_synced_odom;

  if (ugv0BearingUpdated_) {
    bearing_factors = ugv0BearingQueue_.back().bearing_factors;
    range_factors = ugv0BearingQueue_.back().range_factors;
    meas_ids = ugv0BearingQueue_.back().meas_ids;
    landmark_body_positions = ugv0BearingQueue_.back().landmark_body_positions;
    ugv0_bearing_latest_stamp = ugv0BearingQueue_.back().stamp.toSec();
    latest_ugv_bearing_stamp = ugv0_bearing_latest_stamp;
    measurement_synced_odom = ugv0BearingQueue_.back().odom_position;
  }

  // use .back to take newest bearing measurement
  bool add_factor = false;
  StampedSE3IdxCov odom;
  StampedSE3 odom_no_cov;
  // iterate through odom queue check timestamp until timestamp matches
  // Note: i = 0 is the oldest element in the odom queue!

  if (loopClosureFound_) {
    // add loop closure factor only
    auto odom_loop_closure_stamp = latestClosureRelativePose_.stamp.toSec();

    // Skip match time stamp, as we know the odom of loop closure
    add_factor = true;
    ROS_WARN_STREAM_THROTTLE(0.1,
                             "odom stamp matched with loop closure "
                             "measurement stamp, the stamp is: "
                                 << (int)odom_loop_closure_stamp);
    ROS_WARN("WILL ADD loop closure factor");
    ROS_WARN("WILL ADD loop closure factor");
    ROS_WARN("WILL ADD loop closure factor");


    // record the VIO corresponding to loop closure pose for calculating
    // relativeMotion purpose
    odom = latestClosureRawVIO_;

    // // convert x.pose.matrix() to roll pitch yaw and translation
    // ROS_ERROR_STREAM(
    //     "\nloop closure synced with odom, odom pose YPR is: \n"
    //     << (odom.pose.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0))
    //     << "\nloop closure pose YPR is: \n"
    //     << (latestClosureRelativePose_.pose.matrix()
    //             .block<3, 3>(0, 0)
    //             .eulerAngles(2, 1, 0)));
    // ROS_ERROR_STREAM(
    //     "\nloop closure synced with odom, odom pose YPR is: \n"
    //     << (odom.pose.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0))
    //     << "\nloop closure pose YPR is: \n"
    //     << (latestClosureRelativePose_.pose.matrix()
    //             .block<3, 3>(0, 0)
    //             .eulerAngles(2, 1, 0)));
    // ROS_ERROR_STREAM(
    //     "\nloop closure synced with odom, odom pose YPR is: \n"
    //     << (odom.pose.matrix().block<3, 3>(0, 0).eulerAngles(2, 1, 0))
    //     << "\nloop closure pose YPR is: \n"
    //     << (latestClosureRelativePose_.pose.matrix()
    //             .block<3, 3>(0, 0)
    //             .eulerAngles(2, 1, 0)));
  } else if (ugv0BearingUpdated_ && !waitingForClosure_) {
    ROS_ERROR_STREAM("bearing measurements updated!!");
    odom = measurement_synced_odom;
    add_factor = true;

    // // Sync measuerement with odom
    // // Should be replaced by synced odom from measurement
    // for (auto i = odomQueue.size() - 1; i > 0; i--) {
    //   odom = odomQueue[i];
    //   if (odom.stamp.toSec() <
    //       latest_ugv_bearing_stamp - max_timestamp_offset_) {
    //     // ROS_INFO_STREAM_THROTTLE(1, "odom MSG TOO OLD for bearing
    //     // measurement");
    //     if (i == odomQueue.size() - 1) {
    //       ROS_ERROR_STREAM("latest odom MSG TOO OLD for bearing measurement");
    //       break;
    //     }
    //     continue;
    //   } else if (odom.stamp.toSec() >
    //              latest_ugv_bearing_stamp + max_timestamp_offset_) {
    //     // ROS_INFO_STREAM_THROTTLE(1,
    //     //                          "odom MSG TOO NEW  for bearing
    //     //                          measurement");
    //     if (i == 1) {
    //       ROS_ERROR_STREAM(
    //           "bearing measurement too old for syncing with odom!!");
    //       // break;
    //     }
    //     continue;
    //   } else {
    //     add_factor = true;
    //     ROS_INFO_STREAM(
    //         "odom stamp matched with bearing measurement stamp, the stamp is: "
    //         << (int)latest_ugv_bearing_stamp);
    //     break;
    //   }
    // }
    if (!add_factor) {
      // TODO: Now we should never enter here, remove this part later
      ROS_ERROR("odom stamp not matched with bearing measurement stamp!");
      // ROS_ERROR_STREAM("newest odom stamp - latest bearing stamp is: " <<
      // odomQueue.back().stamp -  latest_ugv_bearing_stamp);
      // ROS_ERROR_STREAM("latest bearing stamp is: " <<
      // latest_ugv_bearing_stamp); ROS_ERROR_STREAM("oldest odom stamp is: " <<
      // odomQueue.front().stamp);
      ROS_ERROR_STREAM("newest odom stamp - latest bearing stamp is: "
                       << odomQueue.back().stamp.toSec() -
                              latest_ugv_bearing_stamp);
      ROS_ERROR_STREAM("oldest odom stamp - latest bearing stamp is: "
                       << odomQueue.front().stamp.toSec() -
                              latest_ugv_bearing_stamp);
      ROS_ERROR("odom stamp not matched with bearing measurement stamp!");
      ROS_ERROR("odom stamp not matched with bearing measurement stamp!");
      ROS_ERROR("odom stamp not matched with bearing measurement stamp!");
      ROS_ERROR("odom stamp not matched with bearing measurement stamp!");
    }
  } else if (odomUpdated_) {
    // add odom between factor only, directly use latest odom to calculate
    odom = odomQueue.back();
    add_factor = true;
    ROS_INFO("will add odom factor");
  } else {
    ROS_INFO("Nothing is updated, not adding any factor for current stamp");
    add_factor = false;
  }

  if (!add_factor) {
    ROS_INFO("not adding any factor for current stamp");

    // ros::spinOnce();
    // loop_rate.sleep();

    resetAllFlags();
    return;
  }

  SE3 currRelativeMotion;
  // Use odom to estimate motion since last key frame
  currRelativeMotion = robot1LatestOdom.pose.inverse() * odom.pose;

  odom_no_cov.pose = odom.pose;
  odom_no_cov.stamp = odom.stamp;

  if (robot1OdomReceived_ == false) {
    // since we initialize the latestOdom to be identity, upon the first
    // callback, the relativeMotion will be the first pose (i.e.
    // robot_to_world transform)
    ROS_INFO_THROTTLE(1.0, "first call");

    updateFactorGraph(currRelativeMotion, bearing_factors, range_factors,
                      meas_ids, landmark_body_positions, odom.cov);
    updateLastPose(odom_no_cov, robotID);

    resetAllFlags();
    return;
  } else {
    // if (loopClosureFound_) {
    //   // if loop closure is found, use the loop closure pose as the latest
    //   pose
    //   // and reset the odom queue
    //   odom_no_cov.pose = robot1LoopClosureQueueWithCov_.back().pose;
    //   odom_no_cov.stamp = robot1LoopClosureQueueWithCov_.back().stamp;
    // }

    double accumMovement = currRelativeMotion.translation().norm();
    ROS_INFO_STREAM("current odom distance is " << accumMovement);
    bool moved_enough = accumMovement > minOdomDistance_;
    bool run_factor_graph =
        (moved_enough || loopClosureFound_ || ugv0BearingUpdated_);

    if (run_factor_graph) {
      // temporarily disable other stuff upon loop closure
      if (loopClosureFound_) {
        ROS_ERROR("loop closure found, temporarily disable other factors");
        ROS_ERROR("loop closure found, temporarily disable other factors");
        ROS_ERROR("loop closure found, temporarily disable other factors");

        ROS_ERROR(
            "check  (1) yaw range for loop closure matches what GTSAM expects");

        odom_no_cov.pose = odom.pose;
        odom_no_cov.stamp = odom.stamp;
        currRelativeMotion = robot1LatestOdom.pose.inverse() * odom.pose;

        // auto odom_loop_closure = robot1LoopClosureQueueWithCov_.back();

        size_t closure_matched_pose_idx =
            latestClosureRelativePose_.key_pose_idx;

        // relative pose from key pose to current pose upon loop closure
        SE3 loop_closure_relative_pose = latestClosureRelativePose_.pose;

        ROS_ERROR_STREAM(
            "vio odom used for loop closure is: " << odom.pose.matrix());
        ROS_ERROR_STREAM("currRelativeMotion is "
                         << currRelativeMotion.matrix());
        ROS_ERROR_STREAM("odom_loop_closure RELATIVE pose is "
                         << latestClosureRelativePose_.pose.matrix());
        ROS_ERROR_STREAM(
            "odom loop closure pose to vio pose transform is: "
            << (latestClosureRelativePose_.pose.inverse() * odom.pose)
                   .matrix());

        // reset all the bearing and range factors
        bearing_factors.clear();
        range_factors.clear();
        meas_ids.clear();
        landmark_body_positions.clear();
        updateFactorGraph(currRelativeMotion, bearing_factors, range_factors,
                          meas_ids, landmark_body_positions, odom.cov,
                          loopClosureFound_, loop_closure_relative_pose,
                          closure_matched_pose_idx);
        loopClosureFound_ = false;
      } else {
        if (waitingForClosure_) {
          ROS_WARN(
              "waiting for loop closure, not adding range bearing factors");
          // reset all the bearing and range factors
          bearing_factors.clear();
          range_factors.clear();
          meas_ids.clear();
          landmark_body_positions.clear();
        }
        ROS_WARN(
            "Adding factors to the factor graph, which means either "
            "observing different ugvs or moved enough distance");
        ROS_INFO_STREAM("current odom distance is " << accumMovement);
        updateFactorGraph(currRelativeMotion, bearing_factors, range_factors,
                          meas_ids, landmark_body_positions, odom.cov);
      }

      updateLastPose(odom_no_cov, robotID);

      resetAllFlags();
      return;
    }
  }

  resetAllFlags();
  return;
}

void ActiveSlamInputNode::updateLastPose(const StampedSE3 &odom,
                                         const int &robotID) {
  if (robot1OdomReceived_ == false) {
    robot1FirstATLOdom.pose = odom.pose;
    robot1FirstATLOdom.stamp = odom.stamp;
    robot1OdomReceived_ = true;
  }
  robot1LatestOdom.pose = odom.pose;
  // ROS_ERROR_STREAM("odom pose updated");
  // ROS_ERROR_STREAM("pose is: " << robot1LatestOdom.pose.matrix());
  robot1LatestOdom.stamp = odom.stamp;
  if (robot1KeyPoses_.size() == 0) {
    robot1LastSLOAMKeyPose_ = robot1LatestOdom.pose;
  } else {
    robot1LastSLOAMKeyPose_ = robot1KeyPoses_.back();
  }
}

void ActiveSlamInputNode::updateFactorGraph(
    SE3 relativeMotion, const std::vector<gtsam::Point3> &bearing_factors,
    const std::vector<double> &range_factors,
    const std::vector<size_t> &meas_ids,
    const std::vector<gtsam::Point3> &landmark_body_positions,
    const boost::array<double, 36> &cov, const bool &loopClosureFound,
    const SE3 &loop_closure_relative_pose,
    const size_t &closure_matched_pose_idx) {
  SE3 keyPose = SE3();
  const int robotID = 0;

  // If first odom, since we initialize the latestOdom to be identity, the
  // relativeMotion will be the first pose (i.e. robot_to_world transform),
  // and we can just assume the prevKeyPose pose to be identity
  SE3 prevKeyPose;
  bool use_relative_odom = true;
  if (use_relative_odom) {
    prevKeyPose = robot1OdomReceived_
                      ? robot1KeyPoses_[robot1KeyPoses_.size() - 1]
                      : SE3();
  } else {
    prevKeyPose = robot1OdomReceived_ ? robot1FirstATLOdom.pose : SE3();
  }

  SE3 poseEstimate = prevKeyPose * relativeMotion;
  // TODO: add bearing stuff
  // ROS_ERROR_STREAM("\n POSE ESTIMATE: \n " << poseEstimate.matrix());

  bool optimized;

  if (loopClosureFound && (factorGraph_.pose_counter_robot1_ == 0)) {
    ROS_ERROR_STREAM(
        "loop closure found on first pose, which is not allowed, "
        "aborting");
    return;
  }

  if (loopClosureFound) {
    optimized = factorGraph_.addLoopClosureObservation(
        relativeMotion, poseEstimate, cov, loop_closure_relative_pose,
        closure_matched_pose_idx);
  } else {
    optimized = factorGraph_.addOdomBearingObservation(
        relativeMotion, poseEstimate, cov, bearing_factors, range_factors,
        meas_ids, landmark_body_positions, data_association_distance_);
  }

  // ROS_INFO("odom and bearing observation added");

  if (optimized) {
    factorGraph_.getCurrPose(poseEstimate, robotID);
  }
  ROS_INFO_STREAM("\n OPTMIZATION POSE OUTPUT: \n " << poseEstimate.matrix());
  ROS_DEBUG_STREAM("\n OPTMIZATION POSE OUTPUT: \n " << poseEstimate.matrix());
  keyPose = poseEstimate;

  robot1KeyPoses_.push_back(keyPose);
  if (factorGraph_.pose_counter_robot1_ > 1) {
    // make sure we have at least 2 poses in the factor graph to publish
    publishResultsKeyPosesCentroids();
  }
}

void ActiveSlamInputNode::publishResultsKeyPosesCentroids() {
  // all poses (trajectory) in the factor graph up till now
  std::vector<SE3> trajectory;
  // record the pose index for each pose along the trajectory
  std::vector<size_t> pose_inds;

  // this is key poses but ONLY THE MOST RECENT one is the updated factor
  // graph's estimates

  // publish latest pose index for exploration planner
  // create a unit64 msg
  std_msgs::UInt64 key_pose_msg;
  key_pose_msg.data = factorGraph_.pose_counter_robot1_ - 1;
  pubLatestPoseIdx_.publish(key_pose_msg);

  // this is the optimized output of ALL HISTORY POSES
  factorGraph_.getAllPoses(trajectory, pose_inds);

  // publish the pose indices, along with the poses (trajectory)
  visualization_msgs::MarkerArray traj_and_pose_inds_markers =
      sloam::vizTrajectoryAndPoseInds(trajectory, pose_inds, map_frame_id_);
  pubRobot1TrajPoseInds_.publish(traj_and_pose_inds_markers);

  visualization_msgs::MarkerArray trajMarkers =
      sloam::vizTrajectory(trajectory, map_frame_id_);
  pubRobot1Trajectory_.publish(trajMarkers);

  std::vector<SE3> allLandmarks;
  factorGraph_.getAllLandmarks(allLandmarks);
  // print allLandmarks size
  ROS_ERROR_STREAM("allLandmarksMarkers size is: " << allLandmarks.size());
  visualization_msgs::MarkerArray allLandmarksMarkers =
      sloam::vizAllLandmarks(allLandmarks, map_frame_id_);
  // print size of allLandmarksMarkers 
  ROS_ERROR_STREAM("allLandmarksMarkers size is: " << allLandmarksMarkers.markers.size());  
  pubAllPointLandmarks_.publish(allLandmarksMarkers);
}

SE3 ActiveSlamInputNode::computeSloamToVioOdomTransform(const SE3 &sloam_odom,
                                                         const SE3 &vio_odom) {
  // calculate the transform from sloam odom to vio odom
  SE3 sloam_to_vio_transform = vio_odom * sloam_odom.inverse();
  return sloam_to_vio_transform;
  }

void ActiveSlamInputNode::PublishOdomAsTf(const nav_msgs::Odometry &odom_msg,
                                          const std::string &parent_frame_id,
                                          const std::string &child_frame_id) {
  geometry_msgs::TransformStamped tf;
  tf.header = odom_msg.header;
  tf.header.frame_id = parent_frame_id;
  tf.child_frame_id = child_frame_id;
  tf.transform.translation.x = odom_msg.pose.pose.position.x;
  tf.transform.translation.y = odom_msg.pose.pose.position.y;
  tf.transform.translation.z = odom_msg.pose.pose.position.z;
  tf.transform.rotation = odom_msg.pose.pose.orientation;
  broadcaster_.sendTransform(tf);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "factor_graph_atl");
  ros::NodeHandle n("factor_graph_atl");
  ActiveSlamInputNode in(n);

  // This rate should be no lower than the expected rate of adding factors to
  // the factor graph. Note that callback functions will run at original ros
  // message rate regardless of this rate
  while (ros::ok()) {
    ros::spin();
  }

  return 0;
}
