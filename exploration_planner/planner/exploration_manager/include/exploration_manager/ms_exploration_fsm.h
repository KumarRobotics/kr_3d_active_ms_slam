#ifndef _MS_EXPLORATION_FSM_H_
#define _MS_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

#include "gcopter/trajectory.hpp"
#include "sloam_msgs/ActiveLoopClosureAction.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>


using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace ms_planner {
class msPlannerManager;
class msExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { 
  INIT, 
  WAIT_TRIGGER, 
  INIT_ROTATE,
  PLAN_BEHAVIOR, 
  REPLAN_TRAJ, 
  EXEC_TRAJ, 
  FINISH, 
  EMERGENCY_STOP,
  CLOSURE_PANO};

class msExplorationFSM {
private:
  /* planning utils */
  shared_ptr<msPlannerManager> planner_manager_;
  shared_ptr<msExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;
  bool flag_escape_emergency_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_slam_, odom_sub_vio_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_, traj_goal_pub_;
  std::string srv_name_;
  
  ros::Time odom_time_, plan_stime_, behavior_stime_;
  ros::Time closure_trigger_time_;
  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallbackSlam(const nav_msgs::OdometryConstPtr& msg);  
  void odometryCallbackVio(const nav_msgs::OdometryConstPtr& msg);
  void activeClosureDoneCB(const actionlib::SimpleClientGoalState &state,
                        const sloam_msgs::ActiveLoopClosureResultConstPtr &result);
  void triggerClosureService();
  void abortClosure();
  void visualize();
  void publishTraj();
  void updateStartState(double delay = 0.01);
  Vector3d replan_next_pos_;
  double replan_next_yaw_;
  bool has_traj_ = false;
  double priodic_time_ = 0.0;

  inline double range(double angle){
    // range the angle into [-PI, PI)
    double psi = angle;

    while (psi < -M_PI)
      psi += 2 * M_PI;
    while (psi >= M_PI)
      psi -= 2 * M_PI;

    return psi;
  }

  std::vector<double> init_target_yaw_;
  double cur_target_yaw_;
  Eigen::Vector3d cur_target_pos_;
  bool set_init_yaw_ = false;

  int stopping_counter = 1;
  bool random_rotate_ = false;
  int closure_pano_counter = 2;

  // Active Loop Closure Stuff
  typedef actionlib::SimpleActionClient<sloam_msgs::ActiveLoopClosureAction> ActiveClosureClientType;
  std::unique_ptr<ActiveClosureClientType> active_closure_client_ptr_;

 public:
  msExplorationFSM(/* args */) {
  }
  ~msExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace

#endif