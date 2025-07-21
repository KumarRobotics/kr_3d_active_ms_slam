#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <path_searching/astar2.h>
#include "kino_acc_astar.hpp"
#include "kino_yaw.hpp"
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <active_perception/heading_planner.h>
#include <traj_utils/plan_container.hpp>
#include <ros/ros.h>

//optimization related
#include <plan_manage/poly_opt.h>
#include <plan_manage/sfc_generator.hpp>

// GCOPTER
#include <gcopter/planner.hpp>

// Line tracker
#include <kr_tracker_msgs/LineTrackerAction.h>
#include <kr_tracker_msgs/Transition.h>
#include <actionlib/client/simple_action_client.h>


#include <traj_utils/planning_visualization.h>


namespace ms_planner {
// ms Planner Manager
// Key algorithms of mapping and planning are called

class msPlannerManager {
  // SECTION stable
public:
  msPlannerManager();
  ~msPlannerManager();

  /* main planning interface */
  void initPlanModules(ros::NodeHandle& nh);
  void setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints);
  bool checkGoalClearance(Eigen::Vector3d &goal, double distance);
  double checkTrajCollision(double& distance); //return the collision time, if <0, no collision
  double checkTrajCollision(min_jerk::Trajectory& traj);
  double checkTrajCollision(Trajectory<5>& traj);

  bool planYawExplore(const Eigen::Vector3d& start_yaw, double next_yaw);
  bool planYawNoExplore(const Eigen::Vector3d &start_yaw,
                        const double &end_yaw,
                        bool lookfwd,
                        const double& relax_time = 0);
  bool planExploreTraj(const Eigen::Vector3d& goal,
                       Eigen::MatrixXd &start_state,
                       double &h_yaw);
  bool planExploreTrajGcopter(const Eigen::Vector3d &goal,
                                         Eigen::MatrixXd &start_state,
                                         double &h_yaw);
  bool emergencyStop(Eigen::MatrixXd &start_state, 
                     Eigen::Vector3d &yaw_vec);


  void setPlanVis(PlanningVisualization::Ptr &vis);
  void setFrontier(shared_ptr<FrontierFinder> &ff);

  bool stay(Eigen::MatrixXd &start_state, double dur);

  PlanParameters pp_;
  LocalTrajData local_data_;
  EDTEnvironment::Ptr edt_environment_, edt_environment_local_;
  pcl::PointCloud<pcl::PointXYZ> local_cloud_;

  //front-end planning
  std::unique_ptr<Astar> path_finder_, path_finder_local_;
  std::unique_ptr<KinoAccAstar> kinoacc_path_finder_;
  std::unique_ptr<KinoYaw> kino_yaw_finder_;
  double time_res_ = 0.1;

  //opt planning
  PolySolver poly_traj_solver_;
  PolyYawSolver poly_yaw_solver_;
  bool have_opt_path_ = false;
  double local_plan_horizon_ = 7;
  double radius_ = 0.25;
  sfc_gen::SfcGenerator::Ptr sfc_gen_;

  bool goTo(double x, double y, double z, double yaw, double v_des, double a_des, bool relative, double curr_yaw);
  bool trackerTransition(const std::string& tracker_str);
  int line_tracker_status_; // 0: not running, 1: running; 2: done
  std::string stopping_policy_str_, line_tracker_min_jerk_;
  std::string line_tracker_topic_, service_transition_topic_;

private:
  /* main planning algorithms & modules */
  shared_ptr<SDFMap> global_map_;
  shared_ptr<SDFMap> storage_map_;
  shared_ptr<MapROS> map_ros_;
  unique_ptr<HeadingPlanner> heading_planner_;
  //unique_ptr<VisibilityUtil> visib_util_;
  bool use_astar_path_, use_kino_acc_path_, use_kino_jerk_path_;

  PlanningVisualization::Ptr visualization_;


  void calcNextYaw(Eigen::Vector3d &next_yaw, 
                  Eigen::Vector3d &last_yaw, 
                  Eigen::Vector3d &dir, 
                  double dt);
  void refineEndYaw(double &start_yaw, double &end_yaw);


  template <typename T>
  bool kinoPlan(Eigen::MatrixXd &start_state,
                Eigen::MatrixXd &end_state,
                ros::Time plan_time,
                std::vector<Eigen::Vector3d> &kino_path,
                T &finder);

  // Try Gcopter
  gcopter::GcopterPlanner::Ptr gcopter_planner;
  Trajectory<5> opt_traj_;

  // action client done callback
  typedef actionlib::SimpleActionClient<kr_tracker_msgs::LineTrackerAction> LineClientType;
  void lineTrackerDoneCB(const actionlib::SimpleClientGoalState &state, const kr_tracker_msgs::LineTrackerResultConstPtr &result);
  std::unique_ptr<LineClientType> line_tracker_min_jerk_client_ptr_;

  
  // Tracker transition service
  ros::ServiceClient srv_transition_;

  

public:

  // !SECTION
  typedef shared_ptr<msPlannerManager> Ptr;
};
}  // namespace ms_planner

#endif