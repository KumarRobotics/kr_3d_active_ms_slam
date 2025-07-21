// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>
#include <plan_env/raycast.h>
namespace ms_planner
{
  // SECTION interfaces for setup and query

  msPlannerManager::msPlannerManager()
  {
  }

  msPlannerManager::~msPlannerManager()
  {
    std::cout << "des manager" << std::endl;
  }

  void msPlannerManager::initPlanModules(ros::NodeHandle &nh)
  {
    /* read algorithm parameters */
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);

    nh.param("manager/use_astar_path", use_astar_path_, false);
    nh.param("manager/use_kino_acc_path", use_kino_acc_path_, false);
    nh.param("manager/use_kino_jerk_path", use_kino_jerk_path_, false);
    nh.param("manager/time_res", time_res_, 0.1);

    nh.param("search/horizon", local_plan_horizon_, 7.0);

    /* optimization parameters */
    nh.param("optimization/ego_radius", radius_, 0.2);

    Eigen::VectorXd w_total(4), b_total(2), w_yaw_total(3), b_yaw_total(2);
    nh.param("optimization/w_time", w_total(0), 32.0);
    nh.param("optimization/w_vel", w_total(1), 128.0);
    nh.param("optimization/w_acc", w_total(2), 128.0);
    nh.param("optimization/w_sta_obs", w_total(3), 128.0);

    nh.param("optimization/w_refyaw", w_yaw_total(0), 2.0);
    nh.param("optimization/w_dyaw", w_yaw_total(1), 128.0);
    nh.param("optimization/w_ddyaw", w_yaw_total(2), 128.0);

    nh.param("manager/max_dyaw", pp_.max_dyaw_, -1.0);
    nh.param("manager/max_ddyaw", pp_.max_ddyaw_, -1.0);

    nh.param("manager/line_tracker_topic", line_tracker_topic_, std::string("/ddk/trackers_manager/line_tracker_min_jerk/LineTracker"));
    nh.param("manager/service_transition_topic", service_transition_topic_, std::string("/ddk/trackers_manager/transition"));

    b_total << pp_.max_vel_, pp_.max_acc_;
    b_yaw_total << pp_.max_dyaw_, pp_.max_ddyaw_;
    
    pp_.max_dyaw_ -= 0.2;
    local_data_.traj_id_ = 0;

    // Init map module
    map_ros_.reset(new MapROS);
    map_ros_->init(nh);
    global_map_ = map_ros_->global_map_;
    storage_map_ = map_ros_->storage_map_;
    edt_environment_.reset(new EDTEnvironment);
    edt_environment_local_.reset(new EDTEnvironment);

    edt_environment_->setMap(global_map_);
    edt_environment_local_->setMap(storage_map_);


    /*  local planner intial  */
    if (use_astar_path_)
    {
      path_finder_.reset(new Astar);
      path_finder_local_.reset(new Astar);
      path_finder_->init(nh, edt_environment_);
      path_finder_local_->init(nh, edt_environment_local_);
      std::cout << "[PlannerManager]: use astar mode" << std::endl;
    }

    if (use_kino_acc_path_)
    {
      kinoacc_path_finder_.reset(new ms_planner::KinoAccAstar);
      kinoacc_path_finder_->init(nh, edt_environment_local_);
      std::cout << "[PlannerManager]: use acc mode" << std::endl;
    }

    kino_yaw_finder_.reset(new KinoYaw);
    kino_yaw_finder_->init(nh);

    poly_traj_solver_.init(w_total, b_total);
    poly_yaw_solver_.init(w_yaw_total, b_yaw_total);
    
    //safe corridor setup
    sfc_gen_.reset(new sfc_gen::SfcGenerator);
    sfc_gen_->initROS(nh);

    // tracker transition
    // Init tracker transition services
    srv_transition_ = nh.serviceClient<kr_tracker_msgs::Transition>(service_transition_topic_);
    stopping_policy_str_ = "kr_trackers/StoppingPolicy";
    line_tracker_min_jerk_ = "kr_trackers/LineTrackerMinJerk";

    // Gcopter
    ROS_INFO("Init GcopterPlanner");
    gcopter_planner.reset(new gcopter::GcopterPlanner(nh, "world"));

    // Line tracker
    // Action client
    line_tracker_min_jerk_client_ptr_.reset(new LineClientType(nh, line_tracker_topic_, true));
    if (!line_tracker_min_jerk_client_ptr_->waitForServer(ros::Duration(10.0))) {
      ROS_ERROR("LineTrackerMinJerk server not found.");
    }
    line_tracker_status_ = 0;
  }

  void msPlannerManager::setPlanVis(PlanningVisualization::Ptr &vis)
  {
    visualization_ = vis;
  }


  void msPlannerManager::setFrontier(shared_ptr<FrontierFinder> &ff)
  {
    kino_yaw_finder_->frontier_finder_ = ff;
  }


  bool msPlannerManager::checkGoalClearance(Eigen::Vector3d &goal, double distance)
  {

    const double dr = 0.1;
    double dtheta = 30;
    double new_x, new_y;
    for (double r = 0.0; r <= distance + 1e-3; r += dr)
    {
      for (double theta = -90; theta <= 270; theta += dtheta)
      {
        new_x = goal(0) + r * cos(theta);
        new_y = goal(1) + r * sin(theta);
        Eigen::Vector3d new_pt(new_x, new_y, goal(2));
        if (global_map_->getInflateOccupancy(new_pt))
        {
          return false;
        }
      }
    }

    return true;
  }

  // Check collision in the future "distance" meters
  double msPlannerManager::checkTrajCollision(double &distance)
  {

    double t_now = (ros::Time::now() - local_data_.start_time_).toSec();
    Eigen::Vector3d cur_pt = local_data_.traj_.getPos(t_now);
    double radius = 0.0;
    Eigen::Vector3d fut_pt;
    double check_time = t_now;
    while (radius < distance && check_time < local_data_.duration_)
    {
      fut_pt = local_data_.traj_.getPos(check_time);
      if (storage_map_->getInflateOccupancy(fut_pt) == 1)
      {
        std::cout << "============= collision at: " << fut_pt.transpose() << std::endl;
        std::cout << "local_data_.start_time_ " << local_data_.start_time_ << std::endl;
        std::cout << " t_now " <<  t_now << "check_time  " << check_time  << std::endl;

        return check_time - t_now; // return the time to collide
      }
      radius = (fut_pt - cur_pt).norm();
      check_time += 0.02;
    }
    return -1.0;
  }


  double msPlannerManager::checkTrajCollision(min_jerk::Trajectory& traj)
  {
    Eigen::Vector3d cur_pt = traj.getPos(0.0);
    double radius = 0.0;
    Eigen::Vector3d fut_pt;
    double check_time = 0.02;
    double dur = traj.getTotalDuration();
    while (check_time < dur)
    {
      fut_pt = traj.getPos(check_time);
      if (storage_map_->getInflateOccupancy(fut_pt) == 1)
      {
        std::cout << "============= collision at: " << fut_pt.transpose() << std::endl;
        return check_time; // return the time to collide
      }
      check_time += 0.02;
    }
    return -1.0;
  }

  double msPlannerManager::checkTrajCollision(Trajectory<5>& traj)
  {
    Eigen::Vector3d cur_pt = traj.getPos(0.0);
    double radius = 0.0;
    Eigen::Vector3d fut_pt;
    double check_time = 0.02;
    double dur = traj.getTotalDuration();
    while (check_time < dur)
    {
      fut_pt = traj.getPos(check_time);
      if (storage_map_->getInflateOccupancy(fut_pt) == 1)
      {
        std::cout << "============= collision at: " << fut_pt.transpose() << std::endl;
        return check_time; // return the time to collide
      }
      check_time += 0.02;
    }
    return -1.0;
  }



  // ============= trajectory planning
  bool msPlannerManager::planExploreTrajGcopter(const Eigen::Vector3d &goal,
                                         Eigen::MatrixXd &start_state,
                                         double &h_yaw)
  { // 3 * 3  [pos, vel, acc]
    // end State could be obtained from previous planned trajs
    Eigen::MatrixXd end_state(3, 3); // include the start and end state
    Eigen::Vector3d local_target = goal;
    Eigen::MatrixXd inner_pts; // (3, N -1)
    Eigen::VectorXd allo_ts;
    std::vector<Eigen::MatrixXd> hPolys;
    std::vector<Eigen::Vector3d> path_pts;
    std::vector<double> path_psi;
    end_state.setZero();

    // step one: get the local target
    double dist = (goal - start_state.col(0)).norm();

    end_state << local_target, Eigen::MatrixXd::Zero(3, 1), Eigen::MatrixXd::Zero(3, 1);

    visualization_->displayGoalPoint(local_target, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);

    ros::Time time_now = ros::Time::now();
    // step two: kinodynamic path searching considering obstacles avoidance
    // Note: Kino Plan should use local map and vio odom
    if (!kinoPlan(start_state, end_state, time_now, path_pts, kinoacc_path_finder_))
    {
      std::cout << "[planExploreTraj]: kinodynamic search fails!" << std::endl;
      // Note: If kino fail, use A star. (Use local A star local instance)
      path_finder_local_->reset();
      int status = path_finder_local_->search(start_state.col(0), local_target);
      ROS_ERROR_STREAM("A star status: " << status);
      if (status == Astar::REACH_END) {
        std::cout << "[planExploreTraj]: A star search success" << std::endl;
        path_pts = path_finder_local_->getPath();
      } else {
        std::cout << "[planExploreTraj]: A star search fail!!!" << std::endl;
        return false;
      }
    }
    visualization_->displayKinoAStarList(path_pts, Eigen::Vector4d(0.8, 1, 0, 1), 0);
    end_state.col(0) = path_pts.back(); // to ensure the goal position is collision-free

    gcopter_planner->setMap(storage_map_);
    bool valid = gcopter_planner->plan(start_state, end_state, path_pts);

    if (!valid) {
      ROS_ERROR("gcopter planner fails");
      return false;
    }
    // Found valid plans
    // step up local data
    have_opt_path_ = true;

    opt_traj_ = gcopter_planner->getTraj();
    local_data_.traj_ = opt_traj_;
    local_data_.duration_ = local_data_.traj_.getTotalDuration();
    local_data_.traj_id_ += 1;
    local_data_.start_time_ = time_now;

    std::cout << "local_data_.duration_  " << local_data_.duration_  << std::endl;
    std::cout << "\n[planExploreTraj]: local planning success!" << std::endl;
    return true;

  }


  // ============= yaw planning
  bool msPlannerManager::planYawExplore(const Eigen::Vector3d &start_yaw, double next_yaw)
  {
    if(local_data_.traj_.is_empty())
    {
      std::cout << "[localPlanner]: No trajectory now, canot plan yaw ! "  << std::endl;
      return false;
    }

    // min_jerk::Trajectory temp_traj;
    Trajectory<5> temp_traj;
    temp_traj = local_data_.traj_;

    Eigen::MatrixXd yawState(2, 3);
    yawState.setZero();
    yawState.row(0) = start_yaw;
    yawState(1, 0) = next_yaw;
    kino_yaw_finder_->reset();

    if (!kino_yaw_finder_->yawSearch(yawState, temp_traj))
    {
      std::cout << "[planExploreTraj]: kinodynamic search fails!" << std::endl;
      return false;
    }

    Eigen::MatrixXd inner_yaws;
    Eigen::VectorXd durs;
    kino_yaw_finder_->getYawInners(inner_yaws, durs);

    if (!poly_yaw_solver_.minJerkYawOpt(inner_yaws,
                                        durs,
                                        yawState,
                                        true))
    {
      printf("\033[33m[localPlanner]: yaw optimization fails! \n\033[0m");
      return false;
    }

    poly_yaw_solver_.getTraj(local_data_.yaw_traj_);
    local_data_.yaw_duration_ = local_data_.yaw_traj_.getTotalDuration();
    return true;
  }

  // ============= yaw fit
  bool msPlannerManager::planYawNoExplore(const Eigen::Vector3d &start_yaw,
                                          const double &end_yaw,
                                          bool lookfwd,
                                          const double &relax_time)
  {

    if(local_data_.traj_.is_empty())
    {
      std::cout << "[localPlanner]: No trajectory now, canot plan yaw ! "  << std::endl;
      return false;
    }

    const int seg_num = std::max(local_data_.traj_.getPieceNum(), 8);
    double total_time = local_data_.duration_  + relax_time;
    double dt_yaw = total_time/ seg_num; // time of B-spline segment

    Eigen::MatrixXd inner_yaws(1, seg_num - 1);
    Eigen::VectorXd durs = dt_yaw * Eigen::VectorXd::Ones(seg_num);
    inner_yaws.setZero();
    Eigen::MatrixXd yawState(2, 3);
    yawState.setZero();
    yawState.row(0) = start_yaw;

    Eigen::Vector3d pc, pf, pd;

    if (lookfwd)
    {
      Eigen::Vector3d last_yaw3d = start_yaw;
      Eigen::Vector3d waypt;
      double forward_t = 1.0;

      for (int i = 0; i < seg_num; ++i)
      {
        double tc = i * dt_yaw;

        pc = local_data_.traj_.getPos(tc);
        double tf = min(local_data_.duration_, tc + forward_t);
        pf = local_data_.traj_.getPos(tf);
        pd = pf - pc;

        calcNextYaw(waypt, last_yaw3d, pd, dt_yaw);
        if (i < seg_num -1)
        {
          last_yaw3d = waypt;
          inner_yaws(0, i) =  waypt(0);
        }else{
          yawState.row(1) = waypt;
        }

      }

    }else{


      double new_end_yaw = end_yaw;
      double init_start_yaw = start_yaw(0);

      refineEndYaw(init_start_yaw, new_end_yaw);

      double init_dyaw = (new_end_yaw - start_yaw(0)) / total_time;

      while (init_dyaw < -M_PI)
        init_dyaw += 2 * M_PI;
      while (init_dyaw >= M_PI)
        init_dyaw -= 2 * M_PI;

      // clip the yaw dot
      if (init_dyaw > pp_.max_dyaw_)
      {
        init_dyaw = pp_.max_dyaw_;
      }
      else if (init_dyaw < -pp_.max_dyaw_)
      {
        init_dyaw = -pp_.max_dyaw_;
      }
     
      Eigen::Vector2d start_state, end_state;
      start_state << start_yaw(0), start_yaw(1);
      end_state << start_yaw(0) + init_dyaw * seg_num * dt_yaw, 0.0;

      kino_yaw_finder_->computeShotTraj(start_state, end_state, total_time);
      kino_yaw_finder_->getShotTraj(dt_yaw, inner_yaws);
      yawState(1, 0) = end_state(0);

    }

    if (!poly_yaw_solver_.minJerkYawOpt(inner_yaws,
                                        durs,
                                        yawState,
                                        false))
    {
      printf("\033[33m[localPlanner]: yaw optimization fails! \n\033[0m");

      if(!local_data_.yaw_traj_.is_empty())
      {
        local_data_.yaw_traj_.clear();
      }
      return false;
    }

    poly_yaw_solver_.getTraj(local_data_.yaw_traj_);
    local_data_.yaw_duration_ = local_data_.yaw_traj_.getTotalDuration();
    return true;
  }


  void msPlannerManager::refineEndYaw(double &start_yaw, double &end_yaw)

  {
    //range the angle into [-PI, PI)
    if (end_yaw <= 0)
    {
      while ( end_yaw - start_yaw< -M_PI)
        end_yaw += 2 * M_PI;

    }else if (end_yaw > 0)
    {
      while (end_yaw - start_yaw >= M_PI)
      end_yaw -= 2 * M_PI;
    }

    std::cout << "[localPlanner]:end_yaw" << end_yaw << std::endl;
    double diff = fabs(end_yaw - start_yaw);
    if (diff >= 2 * M_PI - diff)
    {
      if (end_yaw <= 0)
      { 
        if(start_yaw >= 0)
        {
          end_yaw += 2 * M_PI;
        }
      }
      if (end_yaw >= 0)
      {
        if (start_yaw <= 0)
        {
          end_yaw -= 2 * M_PI;
        }
        
      }
    }
  }



  void msPlannerManager::calcNextYaw(Eigen::Vector3d &next_yaw, 
                                     Eigen::Vector3d &last_yaw, 
                                     Eigen::Vector3d &dir, 
                                     double dt)
  {



    double yaw_temp = dir.norm() > 0.1
                    ? atan2(dir(1), dir(0))
                    : last_yaw(0);
    //std::cout << " yaw_temp is  " << yaw_temp << std::endl;


    double diff = fabs(yaw_temp - last_yaw(0));
    if (diff >= 2 * M_PI - diff)
    {
      if (yaw_temp <= 0 && last_yaw(0) >= 0)
      {
        yaw_temp += 2 * M_PI;
      }
      if (yaw_temp >= 0 && last_yaw(0) <= 0)
      {
        yaw_temp -= 2 * M_PI;
      }
    }

    double d_yaw = (yaw_temp - last_yaw(0))/dt;


    if (d_yaw >= M_PI)
    {
      d_yaw -= 2 * M_PI;
    }
    if (d_yaw < -M_PI)
    {
      d_yaw += 2 * M_PI;
    }

    // clip the yaw dot
    if (d_yaw > pp_.max_dyaw_)
    {
      d_yaw = pp_.max_dyaw_;
    }
    else if (d_yaw < -pp_.max_dyaw_)
    {
      d_yaw= -pp_.max_dyaw_;
    }

    next_yaw <<  last_yaw(0) + d_yaw * dt, d_yaw,  0.0;

    return;
  }

  // use kinodynamic a* to generate a path and get hpoly
  template <typename T>
  bool msPlannerManager::kinoPlan(Eigen::MatrixXd &start_state,
                                  Eigen::MatrixXd &end_state,
                                  ros::Time plan_time,
                                  std::vector<Eigen::Vector3d> &kino_path,
                                  T &finder)
  {

    kino_path.clear();
    finder->reset();

    int status = finder->search(start_state, end_state, plan_time, false);

    if (status == KINO_SEARCH_RESULT::NO_PATH)
    {
      std::cout << "[kino replan]: kinodynamic search fail!" << std::endl;
      // retry searching with discontinuous initial state
      finder->reset();
      status = finder->search(start_state, end_state, plan_time, false);
      if (status == KINO_SEARCH_RESULT::NO_PATH)
      {
        std::cout << "[kino replan]: Can't find path." << std::endl;
        return false;
      }
      else
      {
        std::cout << "[kino replan]: retry search success." << std::endl;
      }
    }
    else
    {
      std::cout << "[kino replan]: kinodynamic search success." << std::endl;
    }
    finder->getKinoTraj(time_res_, kino_path);

    if ( (kino_path.back() - end_state.col(0)).norm() > 1.0)
    {
      end_state.col(0) = kino_path.back();
    }

    return true;
  }


  bool msPlannerManager::goTo(double x, double y, double z, double yaw, double v_des, double a_des, bool relative, double curr_yaw) {
    kr_tracker_msgs::LineTrackerGoal goal;
    goal.x = x;
    goal.y = y;
    goal.relative = relative;
    // Convert relative translation in body frame to global frame
    if (relative) {
      goal.x = x * std::cos(curr_yaw) - y * std::sin(curr_yaw);
      goal.y = x * std::sin(curr_yaw) + y * std::cos(curr_yaw);
    }
    goal.z = z;
    goal.yaw = yaw;
    goal.v_des = v_des;
    goal.a_des = a_des;

    line_tracker_min_jerk_client_ptr_->sendGoal(
        goal, boost::bind(&msPlannerManager::lineTrackerDoneCB, this, _1, _2),
        LineClientType::SimpleActiveCallback(), LineClientType::SimpleFeedbackCallback());
    line_tracker_status_ = 1;
    return trackerTransition(line_tracker_min_jerk_);
  }


  void msPlannerManager::lineTrackerDoneCB(const actionlib::SimpleClientGoalState &state, const kr_tracker_msgs::LineTrackerResultConstPtr &result) {
    ROS_INFO("Line tracker goal reached.");
    line_tracker_status_ = 2;
  }

  bool msPlannerManager::trackerTransition(const std::string &tracker_str) {
    kr_tracker_msgs::Transition transition_cmd;
    transition_cmd.request.tracker = tracker_str;

    if (srv_transition_.call(transition_cmd) && transition_cmd.response.success) {
      ROS_ERROR("Current tracker: %s", tracker_str.c_str());
      return true;
    }
    ROS_ERROR_STREAM("Tracker transition failed" << transition_cmd.response.message << "!");
    return false;
  }

} // namespace ms_planner
