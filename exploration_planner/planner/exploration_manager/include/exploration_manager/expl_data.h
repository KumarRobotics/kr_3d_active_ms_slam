#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <vector>
#include <kr_tracker_msgs/Transition.h>
#include "kr_tracker_msgs/PolyTrackerAction.h"
#include <std_srvs/Trigger.h>



using std::vector;
using Eigen::Vector3d;

namespace ms_planner {
struct FSMData {
  // FSM data
  bool trigger_, have_odom_, static_state_;
  vector<string> state_str_;

  Eigen::Vector3d vio_odom_pos_, vio_odom_vel_;  // VIO odometry state
  Eigen::Vector3d slam_odom_pos_, slam_odom_vel_;
  Eigen::Quaterniond vio_odom_orient_; // VIO odom orientation
  Eigen::Quaterniond slam_odom_orient_;
  double vio_odom_yaw_;
  double slam_odom_yaw_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d start_pt_slam_, start_yaw_slam_;
  // vector<Eigen::Vector3d> start_poss;
  kr_tracker_msgs::PolyTrackerActionGoal newest_traj_;
};

struct FSMParam {
  double closure_replan_thresh_;
  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double replan_time_;  // second
  double estop_time_;
};

struct ExplorationData {
  vector<vector<Vector3d>> frontiers_; // 3D position of frontiers (all points)
  vector<vector<Vector3d>> dead_frontiers_;
  vector<pair<Vector3d, Vector3d>> frontier_boxes_;
  vector<Vector3d> points_;
  vector<Vector3d> averages_;
  vector<Vector3d> views_;
  vector<double> yaws_;
  vector<int> info_gains_;
  vector<int> topo_ids_;
  vector<double> costs_;
  vector<double> utilities_;
  vector<Vector3d> door_pos_;
  vector<double> door_yaws_;
  vector<Vector3d> global_tour_;

  vector<int> refined_ids_;
  vector<vector<Vector3d>> n_points_;
  vector<Vector3d> unrefined_points_;
  vector<Vector3d> refined_points_;
  vector<Vector3d> refined_views_;  // points + dir(yaw)
  vector<Vector3d> refined_views1_, refined_views2_;
  vector<Vector3d> refined_tour_;

  Vector3d next_goal_;
  vector<Vector3d> path_next_goal_;

  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;

  // Active Loop Closure Stuff
  int closure_idx_;
  bool closure_set_;
  bool closure_planned_;
};

struct ExplorationParam {
  // params
  bool refine_local_;
  int refined_num_;
  double refined_radius_;
  int top_view_num_;
  double max_decay_;
  string tsp_dir_;  // resource dir of tsp solver
  double relax_time_;
  bool use_ig_;
  bool classic_;
  bool detect_semantics_;
  bool closure_just_triggered_;
};

}  // namespace ms_planner

#endif