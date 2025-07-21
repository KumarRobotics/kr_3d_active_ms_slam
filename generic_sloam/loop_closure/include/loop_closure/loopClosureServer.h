#ifndef _LOOPCLOSURE_H_
#define _LOOPCLOSURE_H_

#include <geometry_msgs/PoseStamped.h>
#include <gtsam/geometry/Point3.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sloam_msgs/ROSRangeBearing.h>
#include <std_msgs/Header.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// include GTSAM pose
#include <gtsam/geometry/Pose3.h>
// include header for eigen quaterniond
#include <Eigen/Geometry>
// include header for tf2::convert
#include <tf2/convert.h>
// include GTSAM rot3
#include <gtsam/geometry/Rot3.h>

#include <deque>
#include <fstream>
#include <queue>
#include <random>
#include <vector>
#include <string>
#include <actionlib/server/simple_action_server.h>
#include "measurement_processing.hpp"
#include "sloam_msgs/SemanticLoopClosure.h"
#include "sloam_msgs/ROSRangeBearingSyncOdom.h"
#include "sloam_msgs/DetectLoopClosureAction.h"


enum LOOP_CLOSURE_STATE { WAIT_FOR_TRIGGER, ESTIMATING, SUCCESS, FAIL};

class LoopClosureServer {
 public:
  LoopClosureServer(ros::NodeHandle &nh);

  // void updateSyncOdomQueue(const sloam_msgs::SemanticLoopClosure &SloamMsg);
  // Loop closure state transition
  void transitLoopClosureState(LOOP_CLOSURE_STATE new_state, std::string pos_call);
  LOOP_CLOSURE_STATE getState();
  void findLoopClosure();

 private:
  typedef actionlib::SimpleActionServer<sloam_msgs::DetectLoopClosureAction> DetectLoopClosureServerType;
  // action Server and param:
  std::unique_ptr<DetectLoopClosureServerType> loop_closure_server_ptr_;

  void goalCb_();
  void preemptCb_();

  sloam_msgs::ROSRangeBearingSyncOdomConstPtr LatestSemanticMeas_;
  bool semantic_message_received_ = false;
  ros::Subscriber SemanticSub_;
  ros::Subscriber SloamPoseSub_;
  ros::Publisher LoopClosurePosePub_;
  double gaussian_noise_std_;
  std::string dataset_description_;
  std::string stats_save_directory_;
  // not working yet, keep it false
  bool use_relative_error_ = false;

  // intialize generator outside
  std::default_random_engine generator_;

  // only record stats for samples predicted to be within FA, will result in
  // better stats but may lead to no stats at all if noise is too much
  bool only_record_in_fa_ = false;

  int NumLandmarks_;

  std::vector<double> error_x_;
  std::vector<double> error_y_;

  std::vector<double> error_xy_;
  std::vector<double> error_yaw_;

  // store latest submap as geometric_msgs/Point 
  std::vector<geometry_msgs::Point> latest_submap_;

  bool published_loop_closure_pose_ = false;

  std::vector<Eigen::Vector3d> landmark_positions_;


  ros::NodeHandle nh_;
  // void UGVCb_(const visualization_msgs::MarkerArray &ugv_marker_msg);
  void SemanticMeasCb_(const sloam_msgs::ROSRangeBearingSyncOdomConstPtr &odom_msg);

  void SloamPoseCb_(const sloam_msgs::SemanticLoopClosureConstPtr &SloamMsg);

  void write_csv(std::string filename, std::string colname,
                 std::vector<double> vals);

  void RunSim_();
  std::deque<nav_msgs::Odometry> OdomQueue_;
  std::deque<nav_msgs::Odometry> vioOdomQueue_;
  nav_msgs::Odometry optimized_hist_key_pose_;
  size_t key_pose_idx_;
  int maxQueueSize_ = 200;
  double time_offset_tolerance_ = 0.01;

  std::vector<std::string> loop_closure_state_str_ = {"WAIT_FOR_TRIGGER", "ESTIMATING", "SUCCESS", "FAIL"};
  LOOP_CLOSURE_STATE loop_closure_state_; 
};

#endif