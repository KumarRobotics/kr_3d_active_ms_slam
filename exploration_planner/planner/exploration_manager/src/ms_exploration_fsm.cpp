#include <plan_manage/planner_manager.h>
#include <exploration_manager/ms_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/ms_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector4d;

namespace ms_planner
{
  void msExplorationFSM::init(ros::NodeHandle &nh)
  {
    fp_.reset(new FSMParam);
    fd_.reset(new FSMData);

    /*  Fsm param  */
    nh.param("fsm/closure_replan_thresh", fp_->closure_replan_thresh_, -1.0);
    nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
    nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
    nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
    nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
    nh.param("fsm/estop_time",  fp_->estop_time_, -1.0);

    nh.param("srv_name", srv_name_, std::string(" "));

    /* Initialize main modules */
    expl_manager_.reset(new msExplorationManager);
    expl_manager_->initialize(nh);
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_ = expl_manager_->planner_manager_;
    expl_manager_->setVis(visualization_);

    state_ = EXPL_STATE::INIT;
    fd_->have_odom_ = false;
    fd_->state_str_ = {"INIT", "WAIT_TRIGGER", "INIT_ROTATE", "PLAN_BEHAVIOR", "REPLAN_TRAJ", "EXEC_TRAJ", "FINISH", "EMERGENCY_STOP", "CLOSURE_PANO"};
    fd_->static_state_ = true;
    fd_->trigger_ = false;
    fd_->start_acc_.setZero();
    expl_manager_->fd_ = fd_;
    /* Ros sub, pub and timer */
    
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &msExplorationFSM::FSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &msExplorationFSM::safetyCallback, this);
    frontier_timer_ = nh.createTimer(ros::Duration(0.5), &msExplorationFSM::frontierCallback, this);
    trigger_sub_ = nh.subscribe("waypoints", 1, &msExplorationFSM::triggerCallback, this);
    odom_sub_slam_ = nh.subscribe("/odom_slam", 1, &msExplorationFSM::odometryCallbackSlam, this);
    odom_sub_vio_ = nh.subscribe("/odom_vio", 1,
                              &msExplorationFSM::odometryCallbackVio, this);
    traj_goal_pub_ = nh.advertise<kr_tracker_msgs::PolyTrackerActionGoal>("tracker_cmd", 10);

    // Loop Closure Stuff
    double  server_wait_timeout_ = 5.0;
    active_closure_client_ptr_.reset(new ActiveClosureClientType(nh, "/loop_closure/active_loop_closure_server", true));
    if (!active_closure_client_ptr_->waitForServer(ros::Duration(server_wait_timeout_)))
    {
      ROS_ERROR("active loop closure action server not found.");
    }

    closure_trigger_time_ = ros::Time::now();
    std::cout << "[msExplorationFSM::init] finished" << std::endl;

  }

  void msExplorationFSM::FSMCallback(const ros::TimerEvent &e)
  {
    if (! expl_manager_->ep_->closure_just_triggered_) {
      closure_trigger_time_ = ros::Time::now();
    } else {
      double time_passed = (ros::Time::now() - closure_trigger_time_).toSec();
      if (time_passed > 60.0) {
        expl_manager_->ep_->closure_just_triggered_ = false;
        closure_trigger_time_ = ros::Time::now();
      }
    }
    ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);
    switch (state_)
    {
    case INIT:
    {
      // Wait for odometry ready
      if (!fd_->have_odom_)
      {
        ROS_WARN_THROTTLE(1.0, "no odom.");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER:
    {
      // Do nothing but wait for trigger
      // ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case INIT_ROTATE:
    {
      if (!random_rotate_) {
        // targeted rotate, meaning we are close to goal, just need to match yaw.
        if (planner_manager_->line_tracker_status_ == 0) {
          // fd_->static_state_ = true;
          updateStartState(0.2);
          if (planner_manager_->goTo(fd_->start_pt_(0), fd_->start_pt_(1), fd_->start_pt_(2), replan_next_yaw_, 0.0f, 0.0f, false, fd_->vio_odom_yaw_)) {
            ROS_INFO("[INIT ROTATE] Rotate to target yaw directly.");
          }
        } else if (planner_manager_->line_tracker_status_ == 1) {
          ROS_INFO_THROTTLE(1.0, "[INIT ROTATE] Rotating.");
        }
        else {
          // status = 2, done
          expl_manager_->plan_fail_cnt_ = 0;
          planner_manager_->line_tracker_status_ = 0;
          fd_->static_state_ = true;
          transitState(PLAN_BEHAVIOR, "FSM");
        }
      } else {
        // random rotate
        // rotate a fixed angle and transit to plan behavior
        if (planner_manager_->line_tracker_status_ == 0) {
          updateStartState(0.2);
          if (planner_manager_->goTo(fd_->start_pt_(0), fd_->start_pt_(1), fd_->start_pt_(2), fd_->start_yaw_(0)+0.96, 0.0f, 0.0f, false, fd_->vio_odom_yaw_)) {
            ROS_INFO("[INIT ROTATE] Rotate a fixed angle.");
          }
        } else if (planner_manager_->line_tracker_status_ == 1) {
          ROS_INFO_THROTTLE(1.0, "[INIT ROTATE] Rotating.");
        }
        else {
          // status = 2, done. But we should not reset plan fail count 
          // as we may need to skip this goal
          // expl_manager_->plan_fail_cnt_ = 0;
          planner_manager_->line_tracker_status_ = 0;
          fd_->static_state_ = true;
          transitState(PLAN_BEHAVIOR, "FSM");
        }
      }
      break;

      // break;
    }
    case FINISH:
    {
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }
    case PLAN_BEHAVIOR:
    {
      updateStartState(0.08);
      behavior_stime_ = plan_stime_;
      int res = callExplorationPlanner();
      ROS_ERROR("PLAN BEHAVIOR Result: %d", res);
      if (res == EXPL_RESULT::SUCCEED)
      {
        // 1. If we are going to a closure position, trigger closure service
        // This function call will check the flag and trigger if needed
        triggerClosureService();
        // 2. pub traj
        publishTraj();
        transitState(EXEC_TRAJ, "FSM");
      }
      else if (res == EXPL_RESULT::SUCCEED_AND_STAY) 
      {
        triggerClosureService();
        fd_->static_state_ = true;
        ROS_INFO("Return succeed and stay");
        ROS_INFO_STREAM("closure set" << expl_manager_->planned_ed_->closure_set_ << " closure planned" << expl_manager_->planned_ed_->closure_planned_);

        if (expl_manager_->planned_ed_->closure_set_ && expl_manager_->planned_ed_->closure_planned_) {
          ROS_INFO("We are going to closure position.");
          closure_pano_counter = 1;
          transitState(CLOSURE_PANO, "FSM");
          ROS_INFO("We are going to closure position haha.");
        } else {
          ROS_INFO("We are going to normal position.");
          // It means we are close to goal,
          // and no rotation needed.
          transitState(PLAN_BEHAVIOR, "FSM");
        }
        ROS_INFO("Return succeed and stay");
      }
      else if (res == EXPL_RESULT::NO_FRONTIER)
      {
        transitState(FINISH, "FSM");
        fd_->static_state_ = true;
      }
      else if (res == EXPL_RESULT::WAIT_PLAN) 
      {
        // DO nothing, we stay in plan behavior state
        ROS_INFO_THROTTLE(1, "Exploration INIT PLAN, wait for cop path.");
      }
      else if (res == EXPL_RESULT::RANDOM_ROTATE) 
      {
        ROS_WARN("try to stop then rotate first");
        // If we want to rotate and have init velocity.
        if (fd_->start_vel_.norm() > 0.5) {
          fd_->static_state_ = false;
        } else {
          fd_->static_state_ = true;
        }
        random_rotate_ = true;
        transitState(INIT_ROTATE, "FSM");
      }
      else if (res == EXPL_RESULT::TARGET_ROTATE) 
      {
        ROS_WARN("try to rotate to target");
        triggerClosureService();
        if (fd_->start_vel_.norm() > 0.5) {
          fd_->static_state_ = false;
        } else {
          fd_->static_state_ = true;
        }
        if (expl_manager_->planned_ed_->closure_set_ && expl_manager_->planned_ed_->closure_planned_) {
          closure_pano_counter = 1;
          transitState(CLOSURE_PANO, "FSM");
        } else {
          random_rotate_ = false;
          transitState(INIT_ROTATE, "FSM");
        }
      }
      else if (res == EXPL_RESULT::CLOSURE_FAIL) 
      {
        abortClosure();
      }
      else //  || res == FAIL
      {
        transitState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case REPLAN_TRAJ:
    {

      updateStartState(0.10);
      if ((replan_next_pos_ - fd_->vio_odom_pos_).norm() < 0.55 && abs(replan_next_yaw_-fd_->vio_odom_yaw_) < 0.2 ){
        transitState(PLAN_BEHAVIOR, "FSM");
        break;
      }

      int res = expl_manager_->planBC(replan_next_pos_, 
                                      replan_next_yaw_,
                                      fd_->start_pt_,
                                      fd_->start_pt_slam_,
                                      fd_->start_vel_,
                                      fd_->start_acc_,
                                      fd_->start_yaw_,
                                      fd_->start_yaw_slam_);
      ROS_ERROR("replan result: %d", res);
      if (res == EXPL_RESULT::SUCCEED)
      {
        triggerClosureService();
        // 1. publish the trajectory
        publishTraj();
        // 2. set the emer as true
        // 3. transite to exec
        transitState(EXEC_TRAJ, "FSM");
      }
      else if (res == EXPL_RESULT::SUCCEED_AND_STAY) 
      {
        triggerClosureService();
        fd_->static_state_ = true;
        if (expl_manager_->planned_ed_->closure_set_ && expl_manager_->planned_ed_->closure_planned_) {
          closure_pano_counter = 1;
          transitState(CLOSURE_PANO, "FSM");
        } else {
          // It means we are close to goal,
          // and no rotation needed.
          transitState(PLAN_BEHAVIOR, "FSM");
        }
      }
      else if (res == EXPL_RESULT::FAIL)
      {
        // Still in PLAN_TRAJ state, keep replanning
        ROS_WARN("plan fail, plan behavior");
        // fd_->static_state_ = true;
        transitState(PLAN_BEHAVIOR, "FSM");
      }else if (res == EXPL_RESULT::RANDOM_ROTATE)
      {
        ROS_WARN("try to rotate first");
        if (fd_->start_vel_.norm() > 0.5) {
          fd_->static_state_ = false;
        } else {
          fd_->static_state_ = true;
        }
        transitState(INIT_ROTATE, "FSM");
        random_rotate_ = true;
        transitState(INIT_ROTATE, "FSM");
      }
      else if (res == EXPL_RESULT::TARGET_ROTATE)
      {
        ROS_WARN("try to rotate to target");
        triggerClosureService();
        if (fd_->start_vel_.norm() > 0.5) {
          fd_->static_state_ = false;
        } else {
          fd_->static_state_ = true;
        }
        if (expl_manager_->planned_ed_->closure_set_ && expl_manager_->planned_ed_->closure_planned_) {
          closure_pano_counter = 1;
          transitState(CLOSURE_PANO, "FSM");
        } else {
          random_rotate_ = false;
          transitState(INIT_ROTATE, "FSM");
        }
      }
      else if (res == EXPL_RESULT::CLOSURE_FAIL) 
      {
        abortClosure();
      }
      break;
    }

    case EXEC_TRAJ:
    {
      ROS_WARN_THROTTLE(2, "EXEC TRAJ!!!");

      // TODO: we should check which odom to use here to check for replan
      LocalTrajData *info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->traj_.getPos(t_cur);
      double yaw = info->yaw_traj_.getPos(t_cur);

      // Replan if traj is almost fully executed
      double time_to_end = info->duration_ - t_cur;

      expl_manager_->mutex_.lock();
      bool fc = expl_manager_->frontier_finder_->isFrontierCovered();
      expl_manager_->mutex_.unlock();

      // If we are navigate towards a closure point
      if (expl_manager_->planned_ed_->closure_set_ && expl_manager_->planned_ed_->closure_planned_) {
        if (time_to_end < fp_->closure_replan_thresh_) {
          // 1. If the goal is a local goal cut off from Astar, we should still navigate to the goal
          // so we should transit to replan traj
          if ((pos - replan_next_pos_).norm() > 2.0) {
            // we should replan
            transitState(REPLAN_TRAJ, "FSM");
            return;
          }
          closure_pano_counter = 1;
          transitState(CLOSURE_PANO, "FSM");
          return;
        }
      }


      if (time_to_end < fp_->replan_thresh1_)
      {
        ROS_WARN("Replan: traj fully executed, plan behaviors =================================");
        has_traj_ = false;
        transitState(PLAN_BEHAVIOR, "FSM");
        return;
      }

      break;
    }

    case EMERGENCY_STOP:
    {
        updateStartState(0.00);
        ROS_WARN("Emergency Stop =======================================");
        ROS_ERROR("Emergency Stop: call stopping policy tracker");
        if (!planner_manager_->trackerTransition(planner_manager_->stopping_policy_str_)) {
          // Note, if transition failed. the tracker will still be in the previous tracker.
          // At least, we should not call sleep here and let the current tracker execute
          ROS_ERROR("Stopping policy transition failed!!!!!!!!");
        } else {
          // wait for stopping policy to finish
          ros::Duration(2.0).sleep();
          fd_->static_state_ = true;
        }
        // call replan
        transitState(PLAN_BEHAVIOR, "FSM");
      break;
    }

    case CLOSURE_PANO:
    {        
      // Rotate 360 by two goTo commands
      // rotate a fixed angle and transit to plan behavior
      if (planner_manager_->line_tracker_status_ == 0) {
        updateStartState(0.2);
        ROS_INFO_STREAM("closure pano counter: " << closure_pano_counter);
        double goal_yaw = fd_->start_yaw_(0)+2.8;
        while (goal_yaw < -M_PI)
          goal_yaw += 2 * M_PI;
        while (goal_yaw >= M_PI)
          goal_yaw -= 2 * M_PI;
        if (planner_manager_->goTo(fd_->start_pt_(0), fd_->start_pt_(1), fd_->start_pt_(2), goal_yaw, 0.0f, 0.0f, false, fd_->vio_odom_yaw_)) {
          ROS_INFO("[CLOSURE_PANO] Rotate 360.");        
        }
      } else if (planner_manager_->line_tracker_status_ == 1) {
        ROS_INFO_THROTTLE(1.0, "[INIT ROTATE] Rotating.");
      }
      else {
        // status = 2, done. But we should not reset plan fail count 
        // as we may need to skip this goal
        if (closure_pano_counter > 0) {
          closure_pano_counter--;
          planner_manager_->line_tracker_status_ = 0;
          fd_->static_state_ = true;
          transitState(CLOSURE_PANO, "FSM");
        } else {
          planner_manager_->line_tracker_status_ = 0;
          fd_->static_state_ = true;
          // Set closure set and planned to false
          expl_manager_->planned_ed_->closure_set_ = false;
          expl_manager_->planned_ed_->closure_planned_ = false;
          transitState(PLAN_BEHAVIOR, "FSM");
        }
      }
      break;
    }

    }
  }

  void msExplorationFSM::updateStartState(double delay)
  {
    plan_stime_ = odom_time_ + ros::Duration(delay);

    if (fd_->static_state_)
    {
      // Start pt, vel, yaw from vio odom
      // Plan from static state (hover)
      fd_->start_pt_ = fd_->vio_odom_pos_;
      fd_->start_vel_ = fd_->vio_odom_vel_;
      fd_->start_acc_.setZero();

      fd_->start_yaw_(0) = fd_->vio_odom_yaw_;
      fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;

      // start pt slam and yaw slam from slam odom
      fd_->start_pt_slam_ = fd_->slam_odom_pos_;
      fd_->start_yaw_slam_(0) = fd_->slam_odom_yaw_;
      fd_->start_yaw_slam_(1) = fd_->start_yaw_slam_(2) = 0.0;
      LocalTrajData *info = &planner_manager_->local_data_;
      if (info != nullptr && !info->traj_.is_empty()) {
      ROS_INFO_STREAM("[UpdateStartState] Static state. Plan Start Pos: " << fd_->start_pt_.transpose() << 
      "odom pos: " << fd_->vio_odom_pos_.transpose() <<
      "Traj end pos: " << info->traj_.getPos(info->duration_).transpose()); 
      }

    }
    else
    {
      // Replan from non-static state, starting from 'replan_time' seconds later
      LocalTrajData *info = &planner_manager_->local_data_;
      double t_r = std::min((plan_stime_ - info->start_time_).toSec(), info->duration_);
      fd_->start_pt_ = info->traj_.getPos(t_r);
      fd_->start_vel_ = info->traj_.getVel(t_r);
      fd_->start_acc_ = info->traj_.getAcc(t_r);
      ROS_INFO_STREAM("[UpdateStartState] Non static state. Plan Start Time: " << plan_stime_.toSec() << 
      "traj start time:" << info->start_time_.toSec() << 
      "traj duration:" << info->duration_);
      ROS_INFO_STREAM("[UpdateStartState] Non static state. Plan Start Pos: " << fd_->start_pt_.transpose() << 
      "odom pos: " << fd_->vio_odom_pos_.transpose() <<
      "Traj end pos: " << info->traj_.getPos(info->duration_).transpose()); 
      // start pt slam and yaw slam from slam odom
      fd_->start_pt_slam_ = fd_->slam_odom_pos_;
      fd_->start_yaw_slam_(0) = fd_->slam_odom_yaw_;
      fd_->start_yaw_slam_(1) = fd_->start_yaw_slam_(2) = 0.0;

      if(!info->yaw_traj_.is_empty())
      {
        double yt_r = std::min((plan_stime_ - info->start_time_).toSec(), info->yaw_duration_);

        fd_->start_yaw_(0) = info->yaw_traj_.getPos(yt_r);
        fd_->start_yaw_(1) = info->yaw_traj_.getVel(yt_r);
        fd_->start_yaw_(2) = range(info->yaw_traj_.getAcc(yt_r));
      }else{
        fd_->start_yaw_(0) = fd_->slam_odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      }

    }
  }

  // update both behavior planner and trajectory planner
  int msExplorationFSM::callExplorationPlanner()
  {
    int res;
    res = expl_manager_->planExploreMotionCOP(fd_->start_pt_, 
                                              fd_->start_pt_slam_,
                                              fd_->start_vel_, 
                                              fd_->start_acc_,
                                             fd_->start_yaw_,
                                             fd_->start_yaw_slam_);
    expl_manager_->getReplanGoal(replan_next_pos_, replan_next_yaw_);
    return res;
  }

  // Depend on if closure is set, trigger the loop closure service
  void msExplorationFSM::triggerClosureService() {
    // Check if we are going to closure
    if (expl_manager_->planned_ed_->closure_set_ && expl_manager_->planned_ed_->closure_planned_ && expl_manager_->ep_->closure_just_triggered_ == false) {
      ROS_INFO("Triggering Loop Closure Service");
      // Compose ActiveLoopClousreGoal message
      sloam_msgs::ActiveLoopClosureGoal loop_closure_msg;
      int closure_idx = expl_manager_->planned_ed_->closure_idx_;
      loop_closure_msg.key_pose_idx.data = closure_idx;
      // print entering closure index
      ROS_INFO_STREAM("Entering Closure Index: " << closure_idx);
      ROS_INFO_STREAM("Closure landmarks size: " << expl_manager_->planeed_closure_id_to_landmarks_[closure_idx].size());

      for (int i=0; i < expl_manager_->planeed_closure_id_to_landmarks_[closure_idx].size(); i++) {
        geometry_msgs::Point tmp_point;
        tmp_point.x = expl_manager_->planeed_closure_id_to_landmarks_[closure_idx][i][0];
        tmp_point.y = expl_manager_->planeed_closure_id_to_landmarks_[closure_idx][i][1];
        tmp_point.z = expl_manager_->planeed_closure_id_to_landmarks_[closure_idx][i][2];
        // print tep_point 
        ROS_INFO_STREAM("Closure Point: " << tmp_point.x << " " << tmp_point.y << " " << tmp_point.z);
        loop_closure_msg.submap.push_back(tmp_point);
      }
      // Send the goal to the action server.
      ROS_INFO_STREAM("Sending Loop Closure Goal");
      active_closure_client_ptr_->sendGoal(loop_closure_msg, boost::bind(&msExplorationFSM::activeClosureDoneCB, this, _1, _2),
                                            ActiveClosureClientType::SimpleActiveCallback(), 
                                            ActiveClosureClientType::SimpleFeedbackCallback());
      expl_manager_->ep_->closure_just_triggered_ = true;
      ROS_ERROR_STREAM("active_closure_client_ptr send goal done!");
    }
  }

  void msExplorationFSM::activeClosureDoneCB(const actionlib::SimpleClientGoalState &state,
                        const sloam_msgs::ActiveLoopClosureResultConstPtr &result) {
    ROS_INFO("LoopClosureDone CB");
    //TODO: call estop and compensate drift
    transitState(EMERGENCY_STOP, "ClosureFound");
    // Then transit state to plan recv to continue execute the plan
  }

  void msExplorationFSM::abortClosure() {
    ROS_INFO("Aborting Loop Closure Service");
    active_closure_client_ptr_->cancelGoal();
  }

  void msExplorationFSM::publishTraj()
  {
    // get the trajectory
    auto info = &planner_manager_->local_data_;
    info->start_time_ = plan_stime_;
    std::cout << "msExplorationFSM::publishTraj plan_stime_ is " << plan_stime_ <<  std::endl;
    // publish the message for multi-agent connections and for trajectory server
    kr_tracker_msgs::PolyTrackerActionGoal traj_act_msg;
    int order = 5;

    traj_act_msg.goal.order = order;
    traj_act_msg.goal.set_yaw = false;
    Eigen::VectorXd durs = info->traj_.getDurations();
    int piece_num = info->traj_.getPieceNum();

    traj_act_msg.goal.t_start = info->start_time_;
    traj_act_msg.goal.seg_x.resize(piece_num);
    traj_act_msg.goal.seg_y.resize(piece_num);
    traj_act_msg.goal.seg_z.resize(piece_num);

    for (int i = 0; i < piece_num; ++i)
    {
      Piece<5>::CoefficientMat coeff = info->traj_[i].normalizePosCoeffMat();

      for (uint c = 0; c <= order; c++) {
        traj_act_msg.goal.seg_x[i].coeffs.push_back(coeff(0,order-c));
        traj_act_msg.goal.seg_y[i].coeffs.push_back(coeff(1,order-c));
        traj_act_msg.goal.seg_z[i].coeffs.push_back(coeff(2,order-c));
      }
      traj_act_msg.goal.seg_x[i].dt = durs[i];
      traj_act_msg.goal.seg_x[i].degree = order;

    }

    std::cout << "[ReplanFSM::publishTraj] durs is " << durs << std::endl;

    if(!info->yaw_traj_.is_empty())
    {
      int piece_num_yaw = info->yaw_traj_.getPieceNum();

      Eigen::VectorXd durs_yaw = info->yaw_traj_.getDurations();

      traj_act_msg.goal.seg_yaw.resize(piece_num_yaw);

      for (int i = 0; i < piece_num_yaw; ++i)
      {
        min_yaw_jerk::CoefficientMat coeff_yaw = info->yaw_traj_[i].getCoeffMat(true);

        for (uint c = 0; c <= order; c++) {
          traj_act_msg.goal.seg_yaw[i].coeffs.push_back(coeff_yaw(0,order-c));
        }
        traj_act_msg.goal.seg_yaw[i].dt = durs_yaw[i];
        traj_act_msg.goal.seg_yaw[i].degree = order;

      }
    }
    traj_act_msg.goal.separate_yaw = true;
    // publish the trajectory
    traj_goal_pub_.publish(traj_act_msg);
    std_srvs::Trigger trg;
    ros::service::call(srv_name_, trg);

    fd_->static_state_ = false;

    thread vis_thread(&msExplorationFSM::visualize, this);
    vis_thread.detach();

    has_traj_ = true;
    priodic_time_ = 0.0;
  }

  void msExplorationFSM::visualize()
  {
    auto info = &planner_manager_->local_data_;
    auto ed_ptr = expl_manager_->ed_;
    auto planned_ed_ptr = expl_manager_->planned_ed_;
    static int last_ftr_num = 0;
    for (unsigned int i = 0; i < ed_ptr->frontiers_.size(); ++i)
    {
      visualization_->drawCubes(ed_ptr->frontiers_[i], 0.05,
                                visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
                                "frontier", i, 4);
    }
    for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i)
    {
      visualization_->drawCubes({}, 0.05, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    }
    last_ftr_num = ed_ptr->frontiers_.size();

    // Draw global top viewpoints info and global tour
    visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
    std::vector<Vector3d> global_tour_seq;
    int tmp_counter = 0;
    for (auto p : expl_manager_->goal_seq_pos) {
      if (tmp_counter > 5) break;
      global_tour_seq.push_back(p);
      tmp_counter++;
    }
    visualization_->drawLines(global_tour_seq, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
    visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
    visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
    "point-average", 0, 6);

  }

  void msExplorationFSM::frontierCallback(const ros::TimerEvent &e)
  {
    if (state_ == WAIT_TRIGGER || state_ == FINISH || 
        state_ == INIT_ROTATE || state_ == EMERGENCY_STOP ||
        state_ == EXEC_TRAJ || state_ == CLOSURE_PANO)
    {
      expl_manager_->mutex_.lock();
      auto ft = expl_manager_->frontier_finder_;
      auto ed = expl_manager_->ed_;
      ft->searchFrontiers();
      ft->computeFrontiersToVisitIGNoPred();
      ft->updateFrontierCostMatrix();
      ft->getFrontiers(ed->frontiers_);
      ft->getFrontierBoxes(ed->frontier_boxes_);
      ft->getDormantFrontiers(ed->dead_frontiers_);


      // Draw frontier and bounding box
      for (unsigned int i = 0; i < ed->frontiers_.size(); ++i)
      {
        visualization_->drawCubes(ed->frontiers_[i], 0.05,
                                  visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
                                  "frontier", i, 4);
      }
      for (int i = ed->frontiers_.size(); i < 50; ++i)
      {
        visualization_->drawCubes({}, 0.05, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      }
      expl_manager_->mutex_.unlock();
    }
  }

  void msExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr &msg)
  {
    ROS_INFO("Triggercallback");
    if (msg->poses[0].pose.position.z < -0.1)
      return;
    if (state_ != WAIT_TRIGGER)
      return;
    fd_->trigger_ = true;
    cout << "Triggered!" << endl;
    // transitState(INIT_ROTATE, "triggerCallback");
    fd_->static_state_ = true;
    transitState(PLAN_BEHAVIOR, "triggerCallback");
  }

  void msExplorationFSM::safetyCallback(const ros::TimerEvent &e)
  {
    if (state_ == EXPL_STATE::EXEC_TRAJ)
    {
      // Check safety and trigger replan if necessary
      /***     Checking the trajectory  ***/
      double dist = 6.0;
      double ctime = planner_manager_->checkTrajCollision(dist);

      if (ctime < 0.0)
      {
        return;
      }
      if (ctime <= fp_->estop_time_) // as emergency time
      {
        ROS_WARN("safetyCallback: emergency stop! collision time is %f", ctime);
        transitState(EMERGENCY_STOP, "SAFETY");
      }
      else
      {
        ROS_WARN("safetyCallback: trajectory collision detected, replan behavior and trajectory ==============================");
        transitState(PLAN_BEHAVIOR, "safetyCallback");
      }
    }

  }


  void msExplorationFSM::odometryCallbackSlam(const nav_msgs::OdometryConstPtr &msg)
  {
    fd_->slam_odom_pos_(0) = msg->pose.pose.position.x;
    fd_->slam_odom_pos_(1) = msg->pose.pose.position.y;
    fd_->slam_odom_pos_(2) = msg->pose.pose.position.z;

    fd_->slam_odom_vel_(0) = msg->twist.twist.linear.x;
    fd_->slam_odom_vel_(1) = msg->twist.twist.linear.y;
    fd_->slam_odom_vel_(2) = msg->twist.twist.linear.z;

    fd_->slam_odom_orient_.w() = msg->pose.pose.orientation.w;
    fd_->slam_odom_orient_.x() = msg->pose.pose.orientation.x;
    fd_->slam_odom_orient_.y() = msg->pose.pose.orientation.y;
    fd_->slam_odom_orient_.z() = msg->pose.pose.orientation.z;

    Eigen::Vector3d rot_x = fd_->slam_odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
    fd_->slam_odom_yaw_ = atan2(rot_x(1), rot_x(0));
    // fd_->have_odom_ = true;
  }

  void msExplorationFSM::odometryCallbackVio(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_time_ = msg->header.stamp;
    fd_->vio_odom_pos_(0) = msg->pose.pose.position.x;
    fd_->vio_odom_pos_(1) = msg->pose.pose.position.y;
    fd_->vio_odom_pos_(2) = msg->pose.pose.position.z;

    fd_->vio_odom_vel_(0) = msg->twist.twist.linear.x;
    fd_->vio_odom_vel_(1) = msg->twist.twist.linear.y;
    fd_->vio_odom_vel_(2) = msg->twist.twist.linear.z;

    fd_->vio_odom_orient_.w() = msg->pose.pose.orientation.w;
    fd_->vio_odom_orient_.x() = msg->pose.pose.orientation.x;
    fd_->vio_odom_orient_.y() = msg->pose.pose.orientation.y;
    fd_->vio_odom_orient_.z() = msg->pose.pose.orientation.z;

    Eigen::Vector3d rot_x = fd_->vio_odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
    fd_->vio_odom_yaw_ = atan2(rot_x(1), rot_x(0));

    fd_->have_odom_ = true;
  }

  void msExplorationFSM::transitState(EXPL_STATE new_state, string pos_call)
  {
    int pre_s = int(state_);
    state_ = new_state;
    cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
         << endl;
  }


} // namespace ms_planner
