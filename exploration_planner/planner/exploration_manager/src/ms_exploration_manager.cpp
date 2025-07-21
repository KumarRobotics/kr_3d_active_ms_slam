// #include <fstream>
#include <exploration_manager/ms_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>
#include "exploration_manager/expl_data.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

using namespace Eigen;

namespace ms_planner
{
  // SECTION interfaces for setup and query

  msExplorationManager::msExplorationManager()
  {
  }

  msExplorationManager::~msExplorationManager()
  {
    ViewNode::astar_.reset();
    ViewNode::caster_.reset();
    ViewNode::map_.reset();
  }

  void msExplorationManager::initialize(ros::NodeHandle &nh)
  {
    planner_manager_.reset(new msPlannerManager);
    planner_manager_->initPlanModules(nh);

    edt_environment_ = planner_manager_->edt_environment_;
    global_map_ = edt_environment_->occ_map_;
    frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
    // view_finder_.reset(new ViewFinder(edt_environment_, nh));

    planner_manager_->setFrontier(frontier_finder_);

    ed_.reset(new ExplorationData);
    planned_ed_.reset(new ExplorationData);
    planning_ed_.reset(new ExplorationData);
    ep_.reset(new ExplorationParam);

    nh.param("exploration/refine_local", ep_->refine_local_, true);
    nh.param("exploration/refined_num", ep_->refined_num_, -1);
    nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
    nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
    nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
    nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
    nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
    nh.param("exploration/detect_semantics", ep_->detect_semantics_, false);

    nh.param("exploration/use_IG", ep_->use_ig_, false);
    nh.param("exploration/classic", ep_->classic_, false);

    nh.param("exploration/vm", ViewNode::vm_, -1.0);
    nh.param("exploration/am", ViewNode::am_, -1.0);
    nh.param("exploration/yd", ViewNode::yd_, -1.0);
    nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
    nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

    nh.param("exploration/radius_far", radius_far_, -1.0);
    nh.param("exploration/radius_close", radius_close_, -1.0);
    nh.param("exploration/radius_noplan", radius_noplan_, -1.0);

    nh.param("frontier/v_fov", v_fov_, -1.0);

    ViewNode::astar_.reset(new Astar);
    ViewNode::astar_->init(nh, edt_environment_);
    ViewNode::map_ = global_map_;

    double resolution_ = global_map_->getResolution();
    Eigen::Vector3d origin, size;
    global_map_->getRegion(origin, size);
    ViewNode::caster_.reset(new RayCaster);
    ViewNode::caster_->setParams(resolution_, origin);

    // Initialize global plan client
    server_wait_timeout_ = 5.0;
    global_plan_client_ptr_.reset(new GlobalPlanClientType(nh, "/exploration/global_plan_server", true));
    if (!global_plan_client_ptr_->waitForServer(ros::Duration(server_wait_timeout_)))
    {
      ROS_ERROR("global plan action server not found.");
    }

    frontier_finder_->checkParam();
    // Global plan state machine param
    global_plan_state_ = GLOBAL_PLAN_STATE::INIT_PLAN;
    global_plan_str_ = {"INIT_PLAN", "PLAN_RECV", "REPLAN"};
    next_goal_pub_ = nh.advertise<visualization_msgs::Marker>("/behavior_goal_vis/next_goal", 100);
    next_goal_compensated_pub_ = nh.advertise<visualization_msgs::Marker>("/behavior_goal_vis/next_goal_compensated", 100);

    // ################## All SLOAM related stuff #################
    // #### Init service client ####
    estimate_closure_ig_client_ = nh.serviceClient<sloam_msgs::EvaluateLoopClosure>("/estimate_info_gain_server");
    latest_key_pose_sub_ = nh.subscribe("latest_factor_graph_key_pose_idx", 1, &msExplorationManager::latestKeyPoseCallback_, this);
    sloam_to_vio_odom_sub_ = nh.subscribe("/factor_graph_atl/quadrotor/sloam_to_vio_odom_fake", 1, &msExplorationManager::sloamToVioCallback_, this);
    loop_closure_submap_sub_ = nh.subscribe("/factor_graph_atl/loop_closure_submap", 1, &msExplorationManager::loopClosureSubmapCallback_, this);

    mk.id = 1;
    plan_fail_cnt_ = 0;
    ep_->closure_just_triggered_ = false;
  }

  void msExplorationManager::setVis(shared_ptr<PlanningVisualization> &vis)
  {

    planner_manager_->setPlanVis(vis);
  }

  void msExplorationManager::copyEdToPlanning() 
  {
    ROS_WARN("Copy exploration data to planning exploration data.");
    planning_ed_->points_.clear();
    planning_ed_->yaws_.clear();
    planning_ed_->averages_.clear();
    planning_ed_->info_gains_.clear();
    planning_ed_->global_tour_.clear();
    std::copy(ed_->points_.begin(), ed_->points_.end(), std::back_inserter(planning_ed_->points_));
    std::copy(ed_->yaws_.begin(), ed_->yaws_.end(), std::back_inserter(planning_ed_->yaws_));
    std::copy(ed_->averages_.begin(), ed_->averages_.end(), std::back_inserter(planning_ed_->averages_));
    std::copy(ed_->info_gains_.begin(), ed_->info_gains_.end(), std::back_inserter(planning_ed_->info_gains_));
    std::copy(ed_->global_tour_.begin(), ed_->global_tour_.end(), std::back_inserter(planning_ed_->global_tour_));
  }

  void msExplorationManager::latestKeyPoseCallback_(const std_msgs::UInt64ConstPtr &msg)
  {
    latest_key_pose_idx_ = msg->data;
  }

  void msExplorationManager::sloamToVioCallback_(const nav_msgs::OdometryConstPtr &msg)
  {
    sloam_to_vio_odom_ = msg;
  }

  void msExplorationManager::loopClosureSubmapCallback_(const visualization_msgs::MarkerArrayConstPtr &msg)
  {
    ROS_INFO("[ExplManager]Loop closure submap callback.");
    bool cleared = false;
    for (int i = 0; i < msg->markers.size(); i++)
    {

      auto marker = msg->markers[i];
      if (marker.ns.find("delete") != std::string::npos) {
        return;
      } else if (!cleared) {
        // Clear stored info
        loop_closure_ids_.clear();
        loop_closure_id_to_pose_.clear();
        loop_closure_id_to_cluster_centroid_.clear();
        loop_closure_id_to_landmarks_.clear();
        cleared = true;
      }
      ROS_INFO_STREAM("key_pose marker.ns: " << marker.ns);
      if (marker.ns.find("key_pose") != std::string::npos)
      {
        int key_pose_id = stoi(marker.ns.substr(9)); // 8+1 since key_pose_i 
        loop_closure_ids_.push_back(key_pose_id);
        loop_closure_id_to_pose_[key_pose_id] = Eigen::Vector3d(marker.points[0].x, marker.points[0].y, marker.points[0].z);
      }
      else if (marker.ns.find("submap_landmarks") != std::string::npos) 
      {
        int key_pose_id = stoi(marker.ns.substr(17));
        loop_closure_id_to_landmarks_[key_pose_id].push_back(Eigen::Vector3d(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z));
      }
      else if (marker.ns.find("submap_centroid") != std::string::npos)
      {
        int key_pose_id = stoi(marker.ns.substr(16));
        loop_closure_id_to_cluster_centroid_[key_pose_id] = Eigen::Vector3d(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
      }
    }
  }

  void msExplorationManager::copyPlanningEdToPlanned() 
  {
    planned_ed_->points_.clear();
    planned_ed_->yaws_.clear();
    planned_ed_->averages_.clear();
    planned_ed_->info_gains_.clear();
    planned_ed_->global_tour_.clear();
    std::copy(planning_ed_->points_.begin(), planning_ed_->points_.end(), std::back_inserter(planned_ed_->points_));
    std::copy(planning_ed_->yaws_.begin(), planning_ed_->yaws_.end(), std::back_inserter(planned_ed_->yaws_));
    std::copy(planning_ed_->averages_.begin(), planning_ed_->averages_.end(), std::back_inserter(planned_ed_->averages_));
    std::copy(planning_ed_->info_gains_.begin(), planning_ed_->info_gains_.end(), std::back_inserter(planned_ed_->info_gains_));
    std::copy(planning_ed_->global_tour_.begin(), planning_ed_->global_tour_.end(), std::back_inserter(planned_ed_->global_tour_));
  }

  int msExplorationManager::planExploreMotionCOP(
      const Vector3d &pos, // in VIO frame
       const Vector3d &pos_slam, // in slam frame
       const Vector3d &vel, // in VIO frame
       const Vector3d &acc, // in VIO frame
       const Vector3d &yaw, // in VIO frame
       const Vector3d &yaw_slam) // in slam frame
  { 
    ROS_INFO("Enter Plan Explore Motion COP function");
    mutex_.lock();

    switch (global_plan_state_)
    {
    case GLOBAL_PLAN_STATE::INIT_PLAN:
    {
      ROS_WARN("[Plan Explore motion] global_plan_state_ == GLOBAL_PLAN_STATE::INIT_PLAN");
      ros::Time t1 = ros::Time::now();
      // First, check global plan state. If we are at INIT PLAN, request a plan from server
      // and then return (Expect rotate in place)
      ROS_WARN_STREAM("[EXP MANAGER] global_plan_state_ == GLOBAL_PLAN_STATE::INIT_PLAN");
      // Request a plan from server
      // Clear the ed_ data
      goal_indices_.clear();
      active_slam_goal_indices_.clear();
      goal_seq_pos.clear();
      ed_->views_.clear();
      ed_->global_tour_.clear();
      planned_ed_->closure_idx_ = -1;
      planned_ed_->closure_set_ = false;
      planned_ed_->closure_planned_ = false;

      frontier_finder_->searchFrontiers();

      double frontier_time = (ros::Time::now() - t1).toSec();
      t1 = ros::Time::now();

      // NOTE (YZ): before compute information, should have prediction for each of new frontiers.
      // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
      frontier_finder_->computeFrontiersToVisitIGNoPred();
      frontier_finder_->updateFrontierCostMatrix();
      // frontier_finder_->computeFrontiersToVisit();
      frontier_finder_->getFrontiers(ed_->frontiers_);
      frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
      frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

      if (ed_->frontiers_.empty())
      {
        ROS_WARN("[Plan Explore Motion COP::INIT_PLAN case 1] No coverable frontier.");
        mutex_.unlock();
        return NO_FRONTIER;
      }
      /*
       * NOTE: getTopViewpointsInfo provide current position
       * retrieve all cluters best viewpoints pos, yaw,
       * and avgs positions of all cells
       */
      frontier_finder_->getTopViewpointsIGNoPred(pos, ed_->points_, ed_->yaws_, ed_->averages_, ed_->info_gains_);
      for (int i = 0; i < ed_->points_.size(); ++i)
      {
        ed_->views_.push_back(
            ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));
      }

      // Do global and local tour planning and retrieve the next viewpoint
      if (ed_->points_.size() > 1)
      {
        ROS_INFO("CALL FIND GLOBAL TOUR SERVER!!!!");
        // Note: return here, as we assume the global planning needs some time to finish.
        // Copy the ed_ to planned_ed_
        copyEdToPlanning();
        frontier_finder_->copyFrontierToPlanning();
        frontier_finder_->global_tour_planned = false;
        findGlobalTour(pos, vel, yaw, ed_->info_gains_);

        // FSM switch to REPLAN_TRAJ
        mutex_.unlock();
        return WAIT_PLAN;
      }
      else if (ed_->points_.size() == 1) {
        // If only 1 frontier, move there directly
        ROS_WARN("[Plan Explore Motion COP::INIT_PLAN case 1] Only 1 frontier, move there directly.");
        init_tour_len_ = 0;
        replan_next_pos_ = ed_->points_[0];
        replan_next_yaw_ = ed_->yaws_[0];
        std::cout << "Next view: " << replan_next_pos_.transpose() << ", yaw: " << replan_next_yaw_ << std::endl;

        int plan_status = planBC(replan_next_pos_, replan_next_yaw_, pos, pos_slam, vel, acc, yaw, yaw_slam);
        mutex_.unlock();
        return plan_status;
      } else  {
        ROS_ERROR_STREAM("[EXP MANAGER] No frontier points. Ftr points size: " << ed_->points_.size());
        ROS_WARN("[Plan Explore Motion COP::INIT_PLAN case 2] No coverable frontier.");
        mutex_.unlock();
        return NO_FRONTIER;
      }
    } break;


    case GLOBAL_PLAN_STATE::PLAN_RECV:
    { 
      ROS_INFO("[EXP MANAGER] global_plan_state_ == GLOBAL_PLAN_STATE::PLAN_RECV");
      ROS_INFO_STREAM(" active_slam_goal_indices_ size: " << active_slam_goal_indices_.size());
      // We now have a global plan
      // Update graph
      std::cout << "start pos: " << pos.transpose() << "\nstart vel: " << vel.transpose()
                << "\nstart acc: " << acc.transpose() << std::endl;

      ed_->views_.clear();
      ed_->global_tour_.clear();
      ros::Time t1 = ros::Time::now();
      frontier_finder_->searchFrontiers();

      double frontier_time = (ros::Time::now() - t1).toSec();
      t1 = ros::Time::now();

      // NOTE: before compute information, should have prediction for each of new frontiers.
      // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
      frontier_finder_->computeFrontiersToVisitIGNoPred();
      frontier_finder_->updateFrontierCostMatrix();
      frontier_finder_->getFrontiers(ed_->frontiers_);
      frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
      frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

      if (ed_->frontiers_.empty())
      {
        ROS_WARN("[Plan Explore Motion COP::PLAN_RECV] No coverable frontier.");
        mutex_.unlock();
        return NO_FRONTIER;
      }
      /*
      * NOTE: getTopViewpointsInfo provide current position
      * retrieve all cluters best viewpoints pos, yaw,
      * and avgs positions of all cells
      */
      frontier_finder_->getTopViewpointsIGNoPred(pos, ed_->points_, ed_->yaws_, ed_->averages_, ed_->info_gains_);
      for (int i = 0; i < ed_->points_.size(); ++i)
      {
        ed_->views_.push_back(
            ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));
      }

      if (active_slam_goal_indices_.size() > 0 && (frontier_finder_->evaFrontierChanged() < 0.15 || (double)active_slam_goal_indices_.size() / (double)init_tour_len_ > 0.9))
      {        
        // No need to replan, so we extract next goal, plan and return
        std::cout << "goal_indices: " << active_slam_goal_indices_[0] << std::endl;

        // When extracting goals, use planned ed.
        if (active_slam_goal_indices_[0].find("ftr") != std::string::npos) {
          // If the goal is a frontier, nav to frontier
          int ftr_idx = std::stoi(active_slam_goal_indices_[0].substr(3));
          replan_next_pos_ = planned_ed_->points_[ftr_idx];
          replan_next_yaw_ = planned_ed_->yaws_[ftr_idx];

        } else {
          // Closure (clo)
          // If the goal is a closure point, set next goal as closure. 
          int closure_idx = std::stoi(active_slam_goal_indices_[0].substr(3));
          replan_next_pos_ = planned_closure_id_to_pose_[closure_idx];
          replan_next_pos_(2) = 0.9;
          replan_next_yaw_ = yaw[0];
          planned_ed_->closure_idx_ = closure_idx;
          planned_ed_->closure_set_ = true;
          planned_ed_->closure_planned_ = false;
        }

        int plan_status = planBC(replan_next_pos_, replan_next_yaw_, pos, pos_slam, vel, acc, yaw, yaw_slam);
        if (plan_status == SUCCEED|| plan_status == TARGET_ROTATE || plan_status == SUCCEED_AND_STAY) {
          active_slam_goal_indices_.erase(active_slam_goal_indices_.begin());
          ROS_INFO_STREAM("PlanRECV: Current plan succeed, erase the goal");
          ROS_INFO_STREAM("TARGET_ROTATE: " << (plan_status == TARGET_ROTATE));
          ROS_INFO_STREAM("SUCCEED_AND_STAY: " << (plan_status == SUCCEED_AND_STAY));
          if (planned_ed_->closure_set_) {
            planned_ed_->closure_planned_ = true;
          }
        } else if (plan_fail_cnt_ > 3) {
          // Fail too many times for the current goal, skip it now
          active_slam_goal_indices_.erase(active_slam_goal_indices_.begin());
          ROS_WARN_STREAM("PlanRECV: Current plan cnt > 3, erase the goal");
          plan_fail_cnt_ = 0;
          if (planned_ed_->closure_set_) {
            planned_ed_->closure_set_ = false;
            planned_ed_->closure_planned_ = false;
            mutex_.unlock();
            return CLOSURE_FAIL;
          }
        }
        ROS_INFO_STREAM("PlanRECV: Current plan fail count:" << plan_fail_cnt_);
        mutex_.unlock();
        return plan_status;
      } else {
        // We need to replan
        // Set next goal for current loop.
        if (active_slam_goal_indices_.size() > 0) {
          // we extract next goal, plan and return
          // When extracting goals, use planned ed.
          if (active_slam_goal_indices_[0].find("ftr") != std::string::npos) {
            // If the goal is a frontier, nav to frontier
            int ftr_idx = std::stoi(active_slam_goal_indices_[0].substr(3));
            replan_next_pos_ = planned_ed_->points_[ftr_idx];
            replan_next_yaw_ = planned_ed_->yaws_[ftr_idx];
          } else {
            // Closure (clo)
            // If the goal is a closure point
            int closure_idx = std::stoi(active_slam_goal_indices_[0].substr(3));
            replan_next_pos_ = planned_closure_id_to_pose_[closure_idx];
            replan_next_pos_(2) = 0.9;
            replan_next_yaw_ = yaw[0];
            planned_ed_->closure_idx_ = closure_idx;
            planned_ed_->closure_set_ = true;
            planned_ed_->closure_planned_ = false;
          }
          int plan_status = planBC(replan_next_pos_, replan_next_yaw_, pos, pos_slam, vel, acc, yaw, yaw_slam);
          if (plan_status == SUCCEED || plan_status == TARGET_ROTATE || plan_status == SUCCEED_AND_STAY) {
            active_slam_goal_indices_.erase(active_slam_goal_indices_.begin());
            ROS_INFO_STREAM("PlanRECV: Current plan succeed, erase the goal");
            ROS_INFO_STREAM("TARGET_ROTATE: " << (plan_status == TARGET_ROTATE));
            ROS_INFO_STREAM("SUCCEED_AND_STAY: " << (plan_status == SUCCEED_AND_STAY));
            if (planned_ed_->closure_set_) {
              planned_ed_->closure_planned_ = true;
            }
            // If yes, replan and follow current plan until receive new plan (If current plan is finished, request a new plan)
            // Copy the ed_ to planning_ed_
            // If plan succeed, we take 
            copyEdToPlanning();
            frontier_finder_->copyFrontierToPlanning();
            frontier_finder_->global_tour_planned = false;
            findGlobalTour(replan_next_pos_, vel, Eigen::Vector3d(replan_next_yaw_, 0, 0), ed_->info_gains_);
            // Tansit to REPLAN state to wait for new plan and execute current plan
            transitGlobalPlanState(GLOBAL_PLAN_STATE::REPLAN, "PLAN_RECV");
          } else if (plan_fail_cnt_ > 3) {
            // PlanBC returned RANDOM_ROTATE or FAIL
            // and we fail too many times for the current goal, skip it now
            active_slam_goal_indices_.erase(active_slam_goal_indices_.begin());
            ROS_WARN_STREAM("PlanRECV: Current plan cnt > 3, erase the goal");
            plan_fail_cnt_ = 0;
            ROS_WARN_STREAM("Reset plan fail count: " << plan_fail_cnt_);

            // If plan succeed, we take 
            copyEdToPlanning();
            frontier_finder_->copyFrontierToPlanning();
            frontier_finder_->global_tour_planned = false;
            findGlobalTour(pos, vel, yaw, ed_->info_gains_);
            // Tansit to REPLAN state to wait for new plan and execute current plan
            transitGlobalPlanState(GLOBAL_PLAN_STATE::REPLAN, "PLAN_RECV");
            // If current waypoint is a closure waypoint, we should return CLOSURE_FAIL
            // to inform FSM to abort the closure request
            if (planned_ed_->closure_set_) {
              planned_ed_->closure_set_ = false;
              planned_ed_->closure_planned_ = false;
              mutex_.unlock();
              return CLOSURE_FAIL;
            }
          }
          ROS_WARN_STREAM("PlanRECV: Current plan fail count:" << plan_fail_cnt_);
          mutex_.unlock();
          return plan_status;
        } else {
          // Current plan finished. Request new plan, Rotate and wait
          // If plan succeed, we take 
          copyEdToPlanning();
          frontier_finder_->copyFrontierToPlanning();
          frontier_finder_->global_tour_planned = false;
          findGlobalTour(pos, vel, yaw, ed_->info_gains_);

          // Tansit to REPLAN state to wait for new plan and execute current plan
          transitGlobalPlanState(GLOBAL_PLAN_STATE::REPLAN, "PLAN_RECV");
          mutex_.unlock();
          return RANDOM_ROTATE;
        }
      }
    } break;


    case GLOBAL_PLAN_STATE::REPLAN:
    {
      std::cout << "start pos: " << pos.transpose() << "\nstart vel: " << vel.transpose()
                << "\nstart acc: " << acc.transpose() << std::endl;

      // Update graph:
      ed_->views_.clear();
      ed_->global_tour_.clear();
      ros::Time t1 = ros::Time::now();
      frontier_finder_->searchFrontiers();
      double frontier_time = (ros::Time::now() - t1).toSec();
      t1 = ros::Time::now();
      // NOTE: before compute information, should have prediction for each of new frontiers.
      // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
      frontier_finder_->computeFrontiersToVisitIGNoPred();
      frontier_finder_->updateFrontierCostMatrix();
      frontier_finder_->getFrontiers(ed_->frontiers_);
      frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
      frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
      if (ed_->frontiers_.empty())
      {
        ROS_WARN("[Plan Explore Motion COP::REPLAN] No coverable frontier.");
        mutex_.unlock();
        return NO_FRONTIER;
      }
      /*
      * NOTE: getTopViewpointsInfo provide current position
      * retrieve all cluters best viewpoints pos, yaw,
      * and avgs positions of all cells
      */
      frontier_finder_->getTopViewpointsIGNoPred(pos, ed_->points_, ed_->yaws_, ed_->averages_, ed_->info_gains_);
      for (int i = 0; i < ed_->points_.size(); ++i)
      {
        ed_->views_.push_back(
            ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));
      }

      if (active_slam_goal_indices_.size() > 0) {
        // we extract next goal, plan and return
        // When extracting goals, use planned ed.
        if (active_slam_goal_indices_[0].find("ftr") != std::string::npos) {
          // If the goal is a frontier, nav to frontier
          int ftr_idx = std::stoi(active_slam_goal_indices_[0].substr(3));
          replan_next_pos_ = planned_ed_->points_[ftr_idx];
          replan_next_yaw_ = planned_ed_->yaws_[ftr_idx];
        } else {
          // Closure (clo)
          // If the goal is a closure point
          int closure_idx = std::stoi(active_slam_goal_indices_[0].substr(3));
          replan_next_pos_ = planned_closure_id_to_pose_[closure_idx];
          replan_next_pos_(2) = 0.9;
          replan_next_yaw_ = yaw[0];
          planned_ed_->closure_idx_ = closure_idx;
          planned_ed_->closure_set_ = true;
          planned_ed_->closure_planned_ = false;
        }
        int plan_status = planBC(replan_next_pos_, replan_next_yaw_, pos, pos_slam, vel, acc, yaw, yaw_slam);
        if (plan_status == SUCCEED || plan_status == TARGET_ROTATE || plan_status == SUCCEED_AND_STAY) {
          active_slam_goal_indices_.erase(active_slam_goal_indices_.begin());
          ROS_WARN_STREAM("PlanRECV: Current plan succeed, erase the goal");
          ROS_WARN_STREAM("TARGET_ROTATE: " << (plan_status == TARGET_ROTATE));
          ROS_WARN_STREAM("SUCCEED_AND_STAY: " << (plan_status == SUCCEED_AND_STAY));
          if (planned_ed_->closure_set_) {
            planned_ed_->closure_planned_ = true;
          }
        } else if (plan_fail_cnt_ > 3) {
          // Fail too many times for the current goal, skip it now
          active_slam_goal_indices_.erase(active_slam_goal_indices_.begin());
          ROS_WARN_STREAM("PlanRECV: Current plan cnt > 3, erase the goal");
          plan_fail_cnt_ = 0;
          ROS_WARN_STREAM("Reset plan fail count: " << plan_fail_cnt_);
          if (planned_ed_->closure_set_) {
            planned_ed_->closure_set_ = false;
            planned_ed_->closure_planned_ = false;
            mutex_.unlock();
            return CLOSURE_FAIL;
          }
        }
        ROS_WARN_STREAM("REPLAN: Current plan fail count:" << plan_fail_cnt_);
        mutex_.unlock();
        return plan_status;
      } else {
        // Current plan finished. Rotate and wait
        mutex_.unlock();
        return RANDOM_ROTATE;
      }
    } break;
    }
  }

  int msExplorationManager::planBC(const Vector3d &next_pos_slam, // In slam frame
                                   const double &next_yaw_slam, // In slam frame
                                   const Vector3d &pos, // In VIO frame
                                   const Vector3d &pos_slam, // in slam frame
                                   const Vector3d &vel, // in VIO frame
                                   const Vector3d &acc, // in VIO frame
                                   const Vector3d &yaw, // in VIO frame, 
                                   const Vector3d &yaw_slam // in slam frame, 
                                   )
  {
    Vector3d next_pos;
    double next_yaw;
    compensateDrift(next_pos_slam, next_yaw_slam, next_pos, next_yaw);
    std::cout << "[=== 1. Init planning status as === ]:\nstart pos: " << pos.transpose()
              << "\nstart vel: " << vel.transpose()
              << "\nstart acc: " << acc.transpose()
              << "\nstart yaw: " << yaw.transpose()
              << "\ngoal  next_pos: " << next_pos.transpose()
              << "\ngoal  next_yaw: " << next_yaw
              << "\nvio odom: " << fd_->vio_odom_pos_.transpose() << std::endl;

    // visualize global goal (should look correct)
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id += 1;
    mk.ns = "next_goal";
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::ARROW;
    mk.color.r = 0;
    mk.color.g = 0;
    mk.color.b = 0.8;
    mk.color.a = 1;
    mk.scale.x = 0.8;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    // Roll pitch and yaw in Radians
    double tmp_roll = 0, tmp_pitch = 0, tmp_yaw = next_yaw_slam;
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(tmp_roll, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(tmp_pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(tmp_yaw, Eigen::Vector3f::UnitZ());
    mk.pose.position.x = next_pos_slam(0);
    mk.pose.position.y = next_pos_slam(1);
    mk.pose.position.z = next_pos_slam(2);
    mk.pose.orientation.w = q.w();
    mk.pose.orientation.x = q.x();
    mk.pose.orientation.y = q.y();
    mk.pose.orientation.z = q.z();
    next_goal_pub_.publish(mk);
    // visualize compensated goal (should look drifted)
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id += 1;
    mk.ns = "next_goal";
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::ARROW;
    mk.color.r = 0;
    mk.color.g = 0.8;
    mk.color.b = 0.0;
    mk.color.a = 1;
    mk.scale.x = 1.0;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;
    // Roll pitch and yaw in Radians
    tmp_yaw = next_yaw;
    q = Eigen::AngleAxisf(tmp_roll, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(tmp_pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(tmp_yaw, Eigen::Vector3f::UnitZ());
    mk.pose.position.x = next_pos(0);
    mk.pose.position.y = next_pos(1);
    mk.pose.position.z = next_pos(2);
    mk.pose.orientation.w = q.w();
    mk.pose.orientation.x = q.x();
    mk.pose.orientation.y = q.y();
    mk.pose.orientation.z = q.z();
    next_goal_compensated_pub_.publish(mk);

    if (plan_fail_cnt_ >= 2)
    {
      plan_fail_cnt_ ++;
      ROS_WARN("[msExplorationManager::planBC] Fail too many times, random rotate first!" );
      return RANDOM_ROTATE;
    }

    // Plan trajectory (position and yaw) to the next viewpoint
    ros::Time t1 = ros::Time::now();

    // Start state contains pos, vel, acc all in VIO frame
    Eigen::MatrixXd start_state(3, 3); // include the start and end state
    start_state << pos, vel, acc;

    // Compute time lower bound of yaw and use in trajectory generation
    double diff = next_yaw - yaw[0];
    while (diff < -M_PI)
      diff += 2 * M_PI;
    while (diff >= M_PI)
      diff -= 2 * M_PI;

    diff = abs(diff);
    diff = min(diff, 2 * M_PI - diff);
    double time_lb = diff / ViewNode::yd_; // NOTE: yd_=60 degree in config

    // use drift compensated goal to initialize
    ed_->next_goal_ = next_pos;
    bool success1 = false, success2 = true;
    // Generate trajectory of x,y,z
    double goal_dist = (next_pos - pos).norm();

    std::cout << "[=== 2. Goal evaluation   === ] \ngoal_dist is : " << goal_dist << ", the yaw diff is : " << diff  << std::endl;

    if (goal_dist <= radius_noplan_ && diff < 0.2)
    {
      ROS_ERROR("[msExplorationManager] Goal is too close and no need for rotation, end ..." );
      plan_fail_cnt_ = 0;
      return SUCCEED_AND_STAY;
    }
    else if (goal_dist <= radius_noplan_)
    {
      /******************************************************************/
      /**********************The goal is very close**********************/
      /******************************************************************/
      // Rotate first and hope we reach the desired yaw
      ROS_WARN("[msExplorationManager] Goal is too close but need for rotation ..." );

      // Let's simply call line tracker to rotate inplace
      plan_fail_cnt_ = 0;
      return TARGET_ROTATE;
    }
    else
    {
      /******************************************************************/
      /*The goal is valid and far enough and should heading to the trajectory first */
      /******************************************************************/
      // call a* first to decide whether we should head the trajectory
      ROS_WARN("[msExplorationManager] The goal is valid and should heading to the trajectory first" );

      if (goal_dist > radius_close_)
      { 
        ROS_WARN("Call Astar planning first to get a near close goal ...");
        auto  plan_t0 = ros::Time::now();

        planner_manager_->path_finder_->reset();
        // use slam start position, and goal position in slam frame 
        if (planner_manager_->path_finder_->search(pos_slam, next_pos_slam) == Astar::REACH_END)
        {
          ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
          shortenPath(ed_->path_next_goal_);
          // Next viewpoint is far away
          double len2 = 0.0;
          vector<Eigen::Vector3d> truncated_path = {ed_->path_next_goal_.front()};
          // radius_far should be slightly smaller than local map size
          for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far_; ++i) // 7.5m
          {
            auto cur_pt = ed_->path_next_goal_[i];
            len2 += (cur_pt - truncated_path.back()).norm();
            truncated_path.push_back(cur_pt);
          }
          Eigen::Vector3d tmp_goal_slam = truncated_path.back();
          Eigen::Vector3d tmp_goal_vio;
          double tmp_yaw_vio;
          compensateDrift(tmp_goal_slam, 0.0, tmp_goal_vio, tmp_yaw_vio);
          // Note: Compensate next goal and next yaw into VIO frame.
          // Note: Make sure we only compensate next goal and next yaw once
          ed_->next_goal_ = tmp_goal_vio;

          ROS_WARN("Astar Planning time: %lf", (ros::Time::now()-plan_t0).toSec());
        } 

      }
    
      //check rotation diff of two points
      //Note: This dir should be compensated as this is used to compute heading when goal is far away
      auto dir = ed_->next_goal_.head(2) - pos.head(2);
      double h_yaw = atan2(dir(1), dir(0));

      auto  plan_t1 = ros::Time::now();
      // Note: Plan to goal in VIO frame (drift compensated)
      ROS_WARN_STREAM("Start state: " << start_state.col(0).transpose() << " fd vio odom: " << fd_->vio_odom_pos_);

      // success1 = planner_manager_->planExploreTraj(ed_->next_goal_, start_state, h_yaw);
      success1 = planner_manager_->planExploreTrajGcopter(ed_->next_goal_, start_state, h_yaw);
      ROS_WARN("Plan Explore Traj Planning time: %lf", (ros::Time::now()-plan_t1).toSec());

      if (success1)
      {
        auto  plan_t2 = ros::Time::now();
        // compute h_diff
        double h_diff = h_yaw - yaw[0];
        while (h_diff < -M_PI)
          h_diff += 2 * M_PI;
        while (h_diff >= M_PI)
          h_diff -= 2 * M_PI;
        h_diff = abs(h_diff);
        h_diff = min(h_diff, 2 * M_PI - h_diff);
        time_lb = h_diff / ViewNode::yd_;

        if (goal_dist <= radius_close_)
        {
          /******************************************************************/
          /***The goal is approach and we should rotate to target yaw ... ***/
          /******************************************************************/
          ROS_WARN("[Yaw Planning] The goal is approach and we should rotate to target yaw ..." );
          // Note: start yaw should be VIO yaw, next yaw should be compensated
          success2 = planner_manager_->planYawNoExplore(yaw, next_yaw, false, 0.0);
        }
        else if (goal_dist > radius_close_ && goal_dist <= radius_far_)
        {
          /***The goal is within range and we can do exploration ... ***/
          ROS_WARN("[Yaw Planning] The goal is within range and we can do exploration ..." );
          // Note: start yaw should be VIO yaw, next yaw should be compensated
          success2 = planner_manager_->planYawExplore(yaw, next_yaw);
        }
        else
        {
          /***The goal is far away or need look ahead first... ***/
          ROS_WARN("[Yaw Planning] The goal is far away or need look ahead first..." );
          if (h_diff > 0.5 * v_fov_)
          {
            // Note: start yaw should be VIO yaw, next yaw should be compensated
            success2 = planner_manager_->planYawNoExplore(yaw, h_yaw, false, 0.0);
          }else{
            // Note: start yaw should be VIO yaw, next yaw should be compensated
            success2 = planner_manager_->planYawNoExplore(yaw, h_yaw, true, 0.0);
          }
        }

        ROS_WARN("Yaw Planning time: %lf", (ros::Time::now()-plan_t2).toSec());

      }else
      {
        ROS_WARN("======================== Planning fails... no need for yaw planning ");
      }

    }

    auto total = (ros::Time::now() - t1).toSec();
    ROS_WARN("Traj + Yaw Total time is : %lf", total);
    ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

    if (success1 && success2)
    {
      plan_fail_cnt_ = 0;
      ROS_WARN("======================== Planning success with yaw trajectory!!!!!!!!!!!!!!!!!!");
      return SUCCEED;
    } 
    ROS_ERROR_STREAM("Traj succ? " << success1 << " Yaw succ? " << success2);
    plan_fail_cnt_++;
    return FAIL;
  }


  Eigen::Matrix4d msExplorationManager::odometryToTransformationMatrix(const nav_msgs::Odometry &odometry_msg) {
      // Convert quaternion to rotation matrix
      Eigen::Matrix3d R = quaternionToRotationMatrix(odometry_msg.pose.pose.orientation);

      // Get translation from Odometry message
      Eigen::Vector3d T(odometry_msg.pose.pose.position.x, 
                        odometry_msg.pose.pose.position.y, 
                        odometry_msg.pose.pose.position.z);

      // Create 4x4 transformation matrix from R and T
      Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
      
      transformation_matrix.block<3,3>(0,0) = R;
      transformation_matrix.block<3,1>(0,3) = T;

      return transformation_matrix;
  }

  Eigen::Matrix3d msExplorationManager::quaternionToRotationMatrix(const geometry_msgs::Quaternion &q) {
      Eigen::Matrix3d R;

      float qx = q.x;
      float qy = q.y;
      float qz = q.z;
      float qw = q.w;

      R(0, 0) = 1 - 2*qy*qy - 2*qz*qz;
      R(0, 1) = 2*qx*qy - 2*qz*qw;
      R(0, 2) = 2*qx*qz + 2*qy*qw;
      R(1, 0) = 2*qx*qy + 2*qz*qw;
      R(1, 1) = 1 - 2*qx*qx - 2*qz*qz;
      R(1, 2) = 2*qy*qz - 2*qx*qw;
      R(2, 0) = 2*qx*qz - 2*qy*qw;
      R(2, 1) = 2*qy*qz + 2*qx*qw;
      R(2, 2) = 1 - 2*qx*qx - 2*qy*qy;
      return R;
  }


  void msExplorationManager::compensateDrift(const Vector3d &position, const double &yaw, Vector3d &position_out, double &yaw_out) {
    if (sloam_to_vio_odom_ == nullptr) {
      position_out = position;
      yaw_out = yaw;
      ROS_WARN("No drift compensation yet...");
      return;
    } 
    Eigen::Matrix3d rotation_matrix_yaw;
    rotation_matrix_yaw << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0,
        0.0, 0.0, 1.0;

    // generate 4x4 eigen matrix based on position and rotation_matrix_yaw
    Eigen::Matrix4d goal_pose_slam = Eigen::Matrix4d::Identity();
    goal_pose_slam.block<3,3>(0,0) = rotation_matrix_yaw;
    // make last column as position;
    goal_pose_slam.block<3,1>(0,3) = position;

    Eigen::Matrix4d sloam_to_vio_mat =
        odometryToTransformationMatrix(*(sloam_to_vio_odom_));

    // apply sloam_to_vio on goal_pose_slam
    Eigen::Matrix4d goal_pose_vio = sloam_to_vio_mat * goal_pose_slam;

    // convert goal_pose_vio to position and yaw
    position_out = goal_pose_vio.block<3,1>(0,3);
    // check if goal_pose_vio(0,0) is zero, if yes, set yaw_out as pi/2, print error msg
    if (goal_pose_vio(0,0) == 0.0) {
      ROS_ERROR("goal_pose_vio(0,0) is zero, set yaw_out as pi/2");
      yaw_out = M_PI/2;
    } else {
      yaw_out = atan2(goal_pose_vio(1,0), goal_pose_vio(0,0));
    }
  }

  // Return index of viewpoint which has highest utility
  int msExplorationManager::computeFrontierUtilities(
      const Vector3d &cur_pos, const Vector3d &vel, const Vector3d &cur_yaw,
      const vector<Eigen::Vector3d> &points, const vector<double> &yaws,
      const vector<int> &info_gains, vector<double> &costs, vector<double> &utilities)
  {
    costs.clear();
    utilities.clear();
    ROS_WARN("vp size: %ld", points.size());
    double max_util = -1.0;
    double max_util_id = -1;
    for (unsigned int i = 0; i < points.size(); i++)
    {
      vector<Vector3d> tmp_path;
      // double tmp_cost = ViewNode::computeCost(cur_pos, points[i], cur_yaw[0], yaws[i], vel, cur_yaw[1], tmp_path);
      double tmp_cost = ViewNode::computePathTimeCost(cur_pos, points[i], cur_yaw[0], yaws[i], vel, cur_yaw[1], tmp_path);
      double tmp_util = info_gains[i] / tmp_cost;
      costs.push_back(tmp_cost);
      utilities.push_back(tmp_util);
      if (tmp_util > max_util)
      {
        max_util = tmp_util;
        max_util_id = i;
      }
    }
    return max_util_id;
  }

  void msExplorationManager::shortenPath(vector<Vector3d> &path)
  {
    if (path.empty())
    {
      ROS_ERROR("Empty path to shorten");
      return;
    }
    // Shorten the tour, only critical intermediate points are reserved.
    const double dist_thresh = 3.0;
    vector<Vector3d> short_tour = {path.front()};
    for (unsigned int i = 1; i < path.size() - 1; ++i)
    {
      if ((path[i] - short_tour.back()).norm() > dist_thresh)
        short_tour.push_back(path[i]);
      else
      {
        // Add waypoints to shorten path only to avoid collision
        ViewNode::caster_->input(short_tour.back(), path[i + 1]);
        Eigen::Vector3i idx;
        while (ViewNode::caster_->nextId(idx) && ros::ok())
        {
          if (edt_environment_->occ_map_->getInflateOccupancy(idx) == 1 ||
              edt_environment_->occ_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
          {
            short_tour.push_back(path[i]);
            break;
          }
        }
      }
    }
    if ((path.back() - short_tour.back()).norm() > 1e-3)
      short_tour.push_back(path.back());

    // Ensure at least three points in the path
    if (short_tour.size() == 2)
      short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
    path = short_tour;
  }

  void msExplorationManager::findGlobalTour(
      const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw, const vector<int> &info_gains)
  {

    // Firstly, backup the ed_ data. 
    // 
    vector<vector<double>> tmp_corr_mat;
    auto t1 = ros::Time::now();

    // Get cost matrix for current state and clusters
    Eigen::MatrixXd cost_mat;
    frontier_finder_->updateFrontierCostMatrix();
    frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
    const int dimension = cost_mat.rows();
    int problem_size = std::min(11, dimension);
    double mat_time = (ros::Time::now() - t1).toSec();
    ROS_INFO("[findGlobalTour] Compute Cost Matrix time: %lf", mat_time);

    // Get correlation matrix
    t1 = ros::Time::now();
    frontier_finder_->getCorrelationMatrix(cur_pos, cur_yaw[0], tmp_corr_mat);

    // Compose PlanCOP action msg
    exploration_msgs::PlanCOPGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    // action_goal.goal.header.frame_id = "world";
    action_goal.mat_size = dimension;

    // IMPORTANT: insert 0 to info gains to represent current state
    action_goal.info_gain.push_back(0);
    for (size_t i = 0; i < info_gains.size(); ++i)
    {
      action_goal.info_gain.push_back(info_gains[i]);
    }
    for (size_t i = 0; i < dimension; ++i)
    {
      for (size_t j = 0; j < dimension; ++j)
      {
        action_goal.cost_mat.push_back(cost_mat(i, j));
      }
    }
    for (size_t i = 0; i < dimension; ++i)
    {
      for (size_t j = 0; j < dimension; ++j)
      {
        action_goal.correlation_mat.push_back(tmp_corr_mat[i][j]);
      }
    }

    // Sanity check:
    if (action_goal.info_gain.size() != action_goal.mat_size)
    {
      ROS_ERROR("[EXP MANAGER] info_gain size is not equal to mat_size");
    }

    // Compute a budget:
    double budget = 0;
    for (int i = 0; i < problem_size; i++)
    {
      budget += 3 * action_goal.cost_mat[i];
    }
    ROS_INFO("[EXP MANAGER] solve COP, the budget is, %f", budget);
    action_goal.budget = budget;
    action_goal.problem_size = problem_size;

    global_plan_client_ptr_->sendGoal(
        action_goal, boost::bind(&msExplorationManager::globalPlanDoneCB, this, _1, _2),
        GlobalPlanClientType::SimpleActiveCallback(), 
        GlobalPlanClientType::SimpleFeedbackCallback());
    ROS_INFO("Send global plan action goal");
  }


  void msExplorationManager::transitGlobalPlanState(GLOBAL_PLAN_STATE new_state, std::string pos_call) {
    int pre_s = int(global_plan_state_);
    global_plan_state_ = new_state;
    cout << "[" + pos_call + "]: from " + global_plan_str_[pre_s] + " to " + global_plan_str_[int(new_state)]
        << endl;
  }

  void msExplorationManager::globalPlanDoneCB(const actionlib::SimpleClientGoalState &state,
                        const exploration_msgs::PlanCOPResultConstPtr &result)
  {
    // We receive the plan. Now, first decompose the msg
    ed_->global_tour_.clear();
    planning_ed_->global_tour_.clear();
    planned_ed_->global_tour_.clear();
    copyPlanningEdToPlanned();
    // IMPORTANT: goal indices is with respect to planned_ed_ points. 
    goal_indices_.clear();
    goal_seq_pos.clear();
    ROS_INFO("[EXP MANAGER] Received global plan");

    vector<int> sequence = result->sequence;
    
    // Safety check:
    if (sequence.size() == 0) {
      ROS_ERROR("[EXP MANAGER] Received empty global plan");
      transitGlobalPlanState(GLOBAL_PLAN_STATE::REPLAN, "globalPlanDoneCB");
      return;
    }

    // Note: goal indices store the index sequence for frontiers to visit. 
    // If sequence say 0 1 2 3 0, which means we start from current position(0), move to first frontier (1), 
    // then (2), (3) and go back to current position (0) 
    // So we need to skip i=0, and store i-1 as frontier sequence
    for (int i = 0; i < sequence.size(); i++)
    {
      if (i == 0 || i == sequence.size() - 1)
      {
        continue;
      }
      goal_indices_.push_back(sequence[i] - 1);
      // Goal seq pos is used for visualization of global plan
      goal_seq_pos.push_back(planned_ed_->points_[sequence[i] - 1]);
    }
    init_tour_len_ = goal_indices_.size();
    auto pos_first = planned_ed_->points_[goal_indices_[0]];
    auto pos_last = planned_ed_->points_[goal_indices_[goal_indices_.size() - 1]];
    if ((pos_first - fd_->start_pt_).norm() > (pos_last - fd_->start_pt_).norm())
    {
      ROS_WARN_STREAM("Reverse the goal indices");
      std::reverse(goal_indices_.begin(), goal_indices_.end());
      std::reverse(goal_seq_pos.begin(), goal_seq_pos.end());
    }
    double budget = result->budget;
    vector<double> tmp_cost_mat = result->cost_mat;
    vector<double> tmp_corr_mat = result->correlation_mat;
    int dim = result->mat_size;
    // TO be thread safe, deep copy the keypose related date into a tmp vector
    std::vector<int> tmp_keypose_ids = loop_closure_ids_;
    std::unordered_map<int, Eigen::Vector3d> tmp_keypose_poses, tmp_keypose_centroids;
    for (int i = 0; i < tmp_keypose_ids.size(); i++)
    {
      tmp_keypose_poses[tmp_keypose_ids[i]] = loop_closure_id_to_pose_[tmp_keypose_ids[i]];
      tmp_keypose_centroids[tmp_keypose_ids[i]] = loop_closure_id_to_cluster_centroid_[tmp_keypose_ids[i]];
    }
    size_t curr_key_pose_idx = latest_key_pose_idx_;
    std::vector<std::string> tmp_refined_seq;
    planActiveLoopClosurePath(fd_->start_pt_, goal_indices_, budget, 
                            tmp_cost_mat, tmp_corr_mat, dim, 
                            curr_key_pose_idx, tmp_keypose_ids, tmp_keypose_poses,
                            tmp_keypose_centroids,
                            tmp_refined_seq);
    // Save the data for asynchroneous execution
    active_slam_goal_indices_ = tmp_refined_seq;
    planned_closure_ids_ = tmp_keypose_ids;
    for (int i = 0; i < tmp_keypose_ids.size(); i++) {
      planned_closure_id_to_pose_[tmp_keypose_ids[i]] = tmp_keypose_poses[tmp_keypose_ids[i]];
      planned_closure_id_to_cluster_centroid_[tmp_keypose_ids[i]] = tmp_keypose_centroids[tmp_keypose_ids[i]];
      for (int j = 0; j < loop_closure_id_to_landmarks_[tmp_keypose_ids[i]].size(); j++) {
        planeed_closure_id_to_landmarks_[tmp_keypose_ids[i]].push_back(loop_closure_id_to_landmarks_[tmp_keypose_ids[i]][j]);
      }
    }
    
    frontier_finder_->global_tour_planned = true;
    frontier_finder_->getPathForRefinedTour(fd_->start_pt_, tmp_refined_seq, tmp_keypose_poses, planned_ed_->global_tour_);
    transitGlobalPlanState(GLOBAL_PLAN_STATE::PLAN_RECV, "globalPlanDoneCB");
  }


  // Assume we already have a sequence of visiting existing frontiers from COP planner
  // This function will take the planned path and consider inserting loop closure candiates into the path
  void msExplorationManager::planActiveLoopClosurePath(const Vector3d& curr_pos, const std::vector<int>& index_sequence, const double& budget, 
                                                      const std::vector<double>& cost_mat, const std::vector<double>& corr_mat, const int& dim, 
                                                      const size_t& curr_pose_idx, const std::vector<int>& key_pose_ids, 
                                                      const std::unordered_map<int, Eigen::Vector3d>& key_pose_pos,
                                                      const std::unordered_map<int, Eigen::Vector3d>& key_pose_centroids,
                                                      std::vector<std::string>& refined_sequence) {
    ROS_INFO("[planActiveLoopClosurePath] Start planning active loop closure path");
    Vector3d current_position = curr_pos;
    std::vector<size_t> refined_pose_indices;
    std::vector<double> refined_travel_distances;
    refined_pose_indices.push_back(curr_pose_idx);
    double curr_budget = budget;
    Eigen::Vector3d tmp_closure_goal_pos;
    if (key_pose_ids.size() == 0) {
      // Special case, directly copy index_sequence into refined_sequence
      for (int i = 0; i < index_sequence.size(); i++) {
        refined_sequence.push_back("ftr"+std::to_string(index_sequence[i]));
      }
      return;
    }

    // If closure just triggered, not adding any closure candidates
    if (ep_->closure_just_triggered_) {
      // Special case, directly copy index_sequence into refined_sequence
      for (int i = 0; i < index_sequence.size(); i++) {
        refined_sequence.push_back("ftr"+std::to_string(index_sequence[i]));
      }
      return;
    }

    for (int i = 0; i < index_sequence.size(); ) {
      // Compute the remaining reward of frontier i
      double ftr_reward = planned_ed_->info_gains_[index_sequence[i]];
      // Don't need to consider the curr_pos to frontier[index_sequence[i]] as corr=0
      for (int j = 0; j < i; j++) {
        // Note, here the index starts from 0 and exclude the current pos, 
        // so cost and corr matrix needs to consider this difference 
        ftr_reward -= planned_ed_->info_gains_[index_sequence[i]] * corr_mat[(i+1)*dim + (j+1)];
      }
      ftr_reward = std::max(0.0, ftr_reward); // If ftr_reward < 0, set it to 0
      
      // Compute utility to frontier[index_sequence[i]]
      // Since current_position can either be start position, at some frontier or at candidate closure position
      // it's better to re-search the cost to frontier[index_sequence[i]] every time
      vector<Vector3d> tmp_path;
      double tmp_cost_ftr = ViewNode::computeCost(current_position, planned_ed_->points_[index_sequence[i]], 0, 0, Vector3d(0,0,0), 0, tmp_path);
      // Assume no info gain on the factor graph size since there should have no new measurement when we do exploration
      double utility_ftr = ftr_reward / tmp_cost_ftr;

      // Compute best closure candidates
      int best_closure = -1;
      double best_closure_cost = -1;
      double best_closure_utility = -1;
      int best_closure_key_pose_idx = -1;
      Eigen::Vector3d best_closure_pose;
      for (int k = 0; k < key_pose_ids.size(); k++) {
        // All pose index so far and travel distance
        // We attempt to add k, compute the distance from our current position to closure k
        vector<Vector3d> tmp_path_closure;
        tmp_closure_goal_pos = key_pose_pos.at(key_pose_ids[k]);
        tmp_closure_goal_pos(2) = key_pose_centroids.at(key_pose_ids[k])(2);
        double tmp_cost_closure = ViewNode::computeCost(current_position, tmp_closure_goal_pos, 0, 0, Vector3d(0,0,0), 0, tmp_path);
        tmp_cost_closure = std::max(1.0,tmp_cost_closure);
        ROS_INFO_STREAM("[Active Loop Closure] temp closure cost: " << tmp_cost_closure);
        if (tmp_cost_closure >= 10000) {
          // means this closure node is not accessible from current position, skip it
          ROS_WARN("[planActiveLoopClosurePath] Closure node %d is not accessible from current position", key_pose_ids[k]);
          continue;
        }

        // Assume we move to key pose id k, ...... This contains a sequence of closure key pose index
        // travel distace is talking about how long we will travel from current pos to first key pose,
        // from first key pose to second, ....
        refined_pose_indices.push_back(key_pose_ids[k]);
        refined_travel_distances.push_back(tmp_cost_closure);
        // Compute utility to closure k
        sloam_msgs::EvaluateLoopClosure srv;
        srv.request.candidate_traj_pose_indices = refined_pose_indices;
        srv.request.travel_distances = refined_travel_distances;
        ROS_INFO_STREAM("[planActiveLoopClosurePath] Calling service evaluate_loop_closure");
        if (!ros::service::waitForService("/estimate_info_gain_server", ros::Duration(0.5))) ROS_ERROR("[planActiveLoopClosurePath] EstimateInfoGain Service not available");
        ROS_INFO_STREAM("[planActiveLoopClosurePath] Called service evaluate_loop_closure");
        double info_closure;
        if (estimate_closure_ig_client_.call(srv)) {
          // Process the result
          if (srv.response.success) {
            ROS_INFO_STREAM("[Plan Active Closure]" << " Info gain: " << srv.response.information_gain);
            info_closure = srv.response.information_gain * 6; // The weighting f_sc to balance the scale of IG.
          } else {
            info_closure = -1;
          }
        } else {
          ROS_ERROR("[planActiveLoopClosurePath] Failed to call service estimate_info_gain_server");
          info_closure = -1;
        }
        // Remove the attempted closure node
        refined_pose_indices.pop_back();
        refined_travel_distances.pop_back();
        // Note: eval_info_gain_factor_graph can be computed incrementally so that it doesn't have to recompute from the first node every time 
        double utility_closure = info_closure;


        if (utility_closure > best_closure_utility) {
            best_closure_utility = utility_closure;
            best_closure_cost = tmp_cost_closure;
            best_closure = key_pose_ids[k];
            best_closure_key_pose_idx = key_pose_ids[k];
            best_closure_pose = tmp_closure_goal_pos;
        }
      }
      ROS_WARN_STREAM("[Plan Active Closure]" << "Closure Utility: " << best_closure_utility); 
      ROS_INFO_STREAM("[Active Loop Closure] FTR cost: " << tmp_cost_ftr);
      ROS_INFO_STREAM("[Active Loop Closure] Closure cost: " << best_closure_cost);
      ROS_INFO_STREAM("[Active Loop Closure] curr_budget: " << curr_budget);
      // Insertion:
      if ((curr_budget >= tmp_cost_ftr) && (curr_budget >= best_closure_cost && best_closure_cost > 0)) {
        // Compare which one gives better utility
        if (utility_ftr > best_closure_utility) {
          ROS_INFO("[planActiveLoopClosurePath] Adding ftr node");
          refined_sequence.push_back("ftr"+std::to_string(index_sequence[i]));
          curr_budget -= tmp_cost_ftr;
          current_position = planned_ed_->points_[index_sequence[i]]; // Assume we are at frontier[i] now
          i++;
        } else {
          ROS_INFO("[planActiveLoopClosurePath] Adding closure node");
          refined_sequence.push_back("clo"+std::to_string(best_closure));
          curr_budget -= best_closure_cost;
          // current_position = key_pose_pos.at(best_closure); // Assume we are at best_closure now
          current_position = best_closure_pose; // Assume we are at best_closure now
          // Add the closure node to the sequence
          refined_pose_indices.push_back(best_closure_key_pose_idx);
          refined_travel_distances.push_back(best_closure_cost);
          i++;
        }
      } else if (curr_budget >= tmp_cost_ftr) {
        // Can only afford visiting frontier[i]
        ROS_INFO("[planActiveLoopClosurePath] Adding ftr node");
        refined_sequence.push_back("ftr"+std::to_string(index_sequence[i]));
        curr_budget -= tmp_cost_ftr;
        current_position = planned_ed_->points_[index_sequence[i]]; // Assume we are at frontier[i] now
        i++;
      } else if (curr_budget >= best_closure_cost && best_closure_cost > 0) {
        // Can only afford visiting k
        ROS_INFO("[planActiveLoopClosurePath] Adding closure node");
        refined_sequence.push_back("clo"+std::to_string(best_closure));
        curr_budget -= best_closure_cost;
        current_position = best_closure_pose; // Assume we are at best_closure now
        // Add the closure node to the sequence
        refined_pose_indices.push_back(best_closure_key_pose_idx);
        refined_travel_distances.push_back(best_closure_cost);
        i++;
      } else {
        return;
      }
    }
    return;
  }

  void msExplorationManager::refineLocalTour(
      const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
      const vector<vector<Vector3d>> &n_points, const vector<vector<double>> &n_yaws,
      vector<Vector3d> &refined_pts, vector<double> &refined_yaws)
  {
    double create_time, search_time, parse_time;
    auto t1 = ros::Time::now();

    // Create graph for viewpoints selection
    GraphSearch<ViewNode> g_search;
    vector<ViewNode::Ptr> last_group, cur_group;

    // Add the current state
    ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
    first->vel_ = cur_vel;
    g_search.addNode(first);
    last_group.push_back(first);
    ViewNode::Ptr final_node;

    // Add viewpoints
    std::cout << "Local tour graph: ";
    for (int i = 0; i < n_points.size(); ++i)
    {
      // Create nodes for viewpoints of one frontier
      for (int j = 0; j < n_points[i].size(); ++j)
      {
        ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
        g_search.addNode(node);
        // Connect a node to nodes in last group
        for (auto nd : last_group)
          g_search.addEdge(nd->id_, node->id_);
        cur_group.push_back(node);

        // Only keep the first viewpoint of the last local frontier
        if (i == n_points.size() - 1)
        {
          final_node = node;
          break;
        }
      }
      // Store nodes for this group for connecting edges
      std::cout << cur_group.size() << ", ";
      last_group = cur_group;
      cur_group.clear();
    }
    std::cout << "" << std::endl;
    create_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Search optimal sequence
    vector<ViewNode::Ptr> path;
    g_search.DijkstraSearch(first->id_, final_node->id_, path);

    search_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Return searched sequence
    for (int i = 1; i < path.size(); ++i)
    {
      refined_pts.push_back(path[i]->pos_);
      refined_yaws.push_back(path[i]->yaw_);
    }

    // Extract optimal local tour (for visualization)
    ed_->refined_tour_.clear();
    ed_->refined_tour_.push_back(cur_pos);
    ViewNode::astar_->lambda_heu_ = 1.0;
    ViewNode::astar_->setResolution(0.2);
    for (auto pt : refined_pts)
    {
      vector<Vector3d> path;
      if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
        ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
      else
        ed_->refined_tour_.push_back(pt);
    }
    ViewNode::astar_->lambda_heu_ = 10000;

    parse_time = (ros::Time::now() - t1).toSec();
  }

} // namespace ms_planner
