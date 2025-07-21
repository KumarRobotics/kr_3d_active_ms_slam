#ifndef _ACTIVESLAMINPUTNODE_H_
#define _ACTIVESLAMINPUTNODE_H_

// #include <cube.h>
#include <definitions.h>
#include <graphWrapper.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <ros/ros.h>
// #include <sloamNode.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtsam/geometry/Point3.h>
#include <sloam_msgs/EvaluateLoopClosure.h>
#include <sloam_msgs/ROSRangeBearing.h>
#include <sloam_msgs/SemanticLoopClosure.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt64.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vizTools.h>

#include <boost/array.hpp>
#include <deque>
#include <memory>
#include <queue>
#include <string>

#include "loop_closure/loopClosureServer.h"
#include "sloam_msgs/ActiveLoopClosureAction.h"
#include "sloam_msgs/ROSRangeBearingSyncOdom.h"
#include "sloam_msgs/ROSSyncOdom.h"

struct StampedSE3 {
  StampedSE3(SE3 p, ros::Time s) : pose(p), stamp(s){};
  StampedSE3() : pose(SE3()), stamp(ros::Time::now()){};
  SE3 pose;
  ros::Time stamp;
};

struct StampedSE3IdxCov {
  StampedSE3IdxCov(SE3 p, boost::array<double, 36> v, size_t idx, ros::Time s)
      : pose(p), cov(v), key_pose_idx(idx), stamp(s){};
  StampedSE3IdxCov()
      : pose(SE3()), cov({}), key_pose_idx(0), stamp(ros::Time::now()){};
  SE3 pose;
  ros::Time stamp;
  boost::array<double, 36> cov;
  size_t key_pose_idx;
};

struct StampedPoint3Double {
  StampedPoint3Double(Point3 p, double d, ros::Time s)
      : point(p), range(d), stamp(s){};
  Point3 point;
  double range;
  ros::Time stamp;
};

struct StampedRangeBearingVector {
  StampedRangeBearingVector(std::vector<Point3> b_factors,
                            std::vector<double> r_factors,
                            std::vector<size_t> m_ids,
                            std::vector<Point3> b_positions,
                            StampedSE3IdxCov o_positions, ros::Time s)
      : bearing_factors(b_factors),
        range_factors(r_factors),
        meas_ids(m_ids),
        landmark_body_positions(b_positions),
        odom_position(o_positions),
        stamp(s){};
  std::vector<Point3> bearing_factors;
  std::vector<double> range_factors;
  std::vector<size_t> meas_ids;
  std::vector<Point3> landmark_body_positions;
  StampedSE3IdxCov odom_position;
  ros::Time stamp;
};

enum { CLOUD_TOO_OLD, CLOUD_TOO_NEW, CLOUD_FOUND };

class LoopClosure;

class ActiveSlamInputNode {
 public:
  ActiveSlamInputNode(ros::NodeHandle &nh);
  void runFactorGraphNode(const ros::TimerEvent &e);
  ros::Timer timer_, log_timer_;

  std::deque<StampedSE3IdxCov> robot1OdomQueueWithCov_;

  StampedSE3IdxCov latestClosureRawVIO_;
  StampedSE3IdxCov latestClosureRelativePose_;

 private:
  typedef actionlib::SimpleActionServer<sloam_msgs::ActiveLoopClosureAction>
      ActiveLoopClosureServerType;
  // action Server and param:
  std::unique_ptr<ActiveLoopClosureServerType> active_loop_closure_server_ptr_;

  typedef actionlib::SimpleActionClient<sloam_msgs::DetectLoopClosureAction>
      DetectLoopClosureClientType;
  std::unique_ptr<DetectLoopClosureClientType> detect_loop_closure_client_ptr_;
  double server_wait_timeout_;

  void ActiveLoopClosureGoalCb_();
  void ActiveLoopClosurePreemptCb_();
  void detectLoopClosureDoneCallback(
      const actionlib::SimpleClientGoalState &state,
      const sloam_msgs::DetectLoopClosureResultConstPtr &result);

  bool newLoopClosureTriggered_;
  bool detectLoopClosureServerTriggered_;
  int loop_closure_num_attempts_;
  int loop_closure_max_attempts_;

  size_t closure_request_key_pose_idx_ = 0;

  bool EstimateInfoGainCb_(sloam_msgs::EvaluateLoopClosure::Request &req,
                           sloam_msgs::EvaluateLoopClosure::Response &res);

  void updateLastPose(const StampedSE3 &odom, const int &robotID);
  void resetAllFlags();

  SemanticFactorGraphWrapper factorGraph_;
  void Robot1OdomCb_(const nav_msgs::OdometryConstPtr &odom_msg);
  // void Robot1LoopClosureCb_(const nav_msgs::OdometryConstPtr &odom_msg);
  void Robot1LoopClosureCb_(
      const sloam_msgs::SemanticLoopClosureConstPtr &odom_msg);

  void LoopClosureRequestIdxCb_(const std_msgs::UInt64 &idx);

  void Ugv0BearingCb_(const sloam_msgs::ROSRangeBearingConstPtr &odom_msg);
  std::deque<StampedRangeBearingVector> ugv0BearingQueue_;
  // StampedRangeBearingVector syncedOdomMeasurement_;
  sloam_msgs::ROSRangeBearingConstPtr lastBearingMsg_;

  double max_timestamp_offset_ = 0.01;
  void updateFactorGraph(
      SE3 relativeMotion, const std::vector<gtsam::Point3> &bearing_factors,
      const std::vector<double> &range_factors,
      const std::vector<size_t> &meas_ids,
      const std::vector<gtsam::Point3> &landmark_body_positions,
      const boost::array<double, 36> &cov, const bool &loopClosureFound = false,
      const SE3 &loop_closure_relative_pose = SE3(),
      const size_t &closure_matched_pose_idx = 0);


  SE3 computeSloamToVioOdomTransform(const SE3 &sloam_odom,
                                                      const SE3 &vio_odom);

  void PublishOdomAsTf(const nav_msgs::Odometry &odom_msg,
                       const std::string &parent_frame_id,
                       const std::string &child_frame_id);

  // publish robot key poses and landmark centroids results for visualization
  void publishResultsKeyPosesCentroids();
  
  void logEntropyCB(const ros::TimerEvent &e);
  
  std::queue<sensor_msgs::PointCloud2ConstPtr> robot1TreePcQueue_;

  ros::NodeHandle nh_;
  ros::Publisher pubRobot1HighFreqSLOAMPose_;
  ros::Publisher pubRobot1HighFreqSLOAMOdom_;
  ros::Publisher pubSloamToVioOdom_;
  ros::Publisher pubRobot1Trajectory_;
  ros::Publisher pubRobot1TrajPoseInds_;
  ros::Publisher pubAllPointLandmarks_;
  ros::Publisher pubRobotMeaSyncOdom_;
  ros::Publisher pubRobotHighFreqSyncOdom_;
  ros::Publisher pubLatestPoseIdx_;
  ros::Subscriber Robot1OdomSub_;
  ros::Subscriber Ugv0BearingSub_;
  ros::Subscriber Ugv1BearingSub_;
  ros::Subscriber Ugv2BearingSub_;
  ros::Subscriber LoopClosureRequestIdxSub_;
  ros::Subscriber Robot1LoopClosureSub_;
  ros::Subscriber Robot1TreePCSub_;
 
  ros::ServiceServer estimate_info_gain_server_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster broadcaster_;

  // params
  std::string map_frame_id_;
  std::string robot_frame_id_;
  std::string robot1_odom_topic_;
  std::string robot1_loop_closure_odom_topic_;

  // store latest submap as geometric_msgs/Point 
  std::vector<geometry_msgs::Point> latest_submap_;
  
  float minOdomDistance_;
  size_t maxQueueSize_;
  size_t maxOdomQueueSize_;

  // data association of point landmarks, normed distance along X Y Z axis
  double data_association_distance_;

  // noise parameters for factor graph
  float runFactorGraphRate_;
  double odomFactorPosStdXY_;
  double odomFactorPosStdZ_;
  double odomFactorRotStd_;
  double bearingFactorStd_;
  double rangeFactorStd_;
  double odomFactorStdInflation_;

  double closure_noise_std_wrt_odom_factor_;

  // flags
  bool robot1OdomReceived_ = false;
  bool odomUpdated_ = false;
  bool loopClosureFound_ = false;
  bool ugv0BearingUpdated_ = false;
  bool waitingForClosure_ = false;
  // parameter that records the measurement case when optimization is triggered
  // last time
  int prev_meas_case_ = -1;

  // vars
  std::vector<SE3> robot1KeyPoses_;

  // since we initialize the latestOdom to be identity, upon the first callback,
  // the relativeMotion will be the first pose (i.e. robot_to_world transform)
  StampedSE3 robot1LatestOdom;
  StampedSE3 robot1FirstATLOdom;
  SE3 robot1LastSLOAMKeyPose_ = SE3();

  bool publishTf_;
  size_t robot1OdomCounter_;
};

#endif