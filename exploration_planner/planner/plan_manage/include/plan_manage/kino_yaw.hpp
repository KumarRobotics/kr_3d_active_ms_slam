#ifndef _KINO_YAW_H
#define _KINO_YAW_H

#include <ros/console.h>
#include <ros/ros.h>
#include "kino_utils.hpp"
#include <active_perception/frontier_finder.h>
#include <active_perception/heading_planner.h>

#include "gcopter/trajectory.hpp"

namespace ms_planner {


using namespace std;
using namespace Eigen;


class KinoYaw {
private:

  std::vector<YawNodePtr>  yaw_nodes_;
  double max_dyaw_  = 0.8;
  double max_ddyaw_ = 0.2;
  double yaw_resolution_ = 0.05;

  // parameters
  double w_time_ = 3.0; 
  double w_uyaw_ = 5.0;
  double w_ref_ = 5.0;
  double w_semantics_ = 1.0;
  double w_info_ = 0.001;

  int vis_check_num_ = 4;
  double half_fov_theta_ = 2.0;

  Eigen::VectorXd durations_;
  Eigen::VectorXd coef_shot_;
  double t_shot_ = 0.0;
  bool is_shot_succ_ = false;

  std::vector<Eigen::Vector3d> door_pos;
  std::vector<double> door_yaws;


public:
  KinoYaw(){};
  ~KinoYaw(){};

  shared_ptr<FrontierFinder> frontier_finder_;
             
  typedef std::shared_ptr<KinoYaw> Ptr;

  void init(ros::NodeHandle& nh)
  {
    nh.param("manager/max_dyaw", max_dyaw_, -1.0);
    nh.param("manager/max_ddyaw", max_ddyaw_, -1.0);

    nh.param("search/w_yaw_time", w_time_, -1.0);
    nh.param("search/w_uyaw", w_uyaw_, -1.0);
    nh.param("search/w_ref", w_ref_, -1.0);
    nh.param("search/w_semantics", w_semantics_, -1.0);
    nh.param("search/w_info", w_info_, -1.0);

    nh.param("search/yaw_resolution", yaw_resolution_, -1.0);

    std::cout << " init kino_yaw_finder_" << std::endl;
  }

  void reset()
  {
    yaw_nodes_.clear();
    is_shot_succ_ = false;
  }

  inline double small_abs(double angle){
    // range the angle into (-PI, PI]

    double psi_abs = abs(angle);
    double psi = std::min(psi_abs,   2*M_PI - psi_abs);

    return psi;
  }

  /****************************perception-aware yaw planning********************************/
  bool yawSearch(Eigen::MatrixXd &yaw_state,
                          min_jerk::Trajectory &traj)
  {
    /* ---------- initialize ---------- */

    YawNodePtr cur_node = new YawNode;
    cur_node->parent = NULL;
    cur_node->time = (ros::Time::now()).toSec(); // primitive start time

    /* ---- initialize yaw angles ---- */
    // start yaw range is (-pi, pi]
    cur_node->yaw_state = (yaw_state.row(0)).head(2);
    cur_node->yaw_idx = yawToIndex(cur_node->yaw_state(0));
    cur_node->path_idx = 0;
    cur_node->f_score = 0.0;

    // PathNodePtr neighbor = NULL;
    YawNodePtr terminate_node = new YawNode;
    terminate_node->f_score = INFINITY;
    terminate_node->yaw_state = (yaw_state.row(1)).head(2);
    /* ---------- init state propagation ---------- */
    double res = 1 / 2.0;
    vector<double> yaw_inputs;
    double u_yaw;
    
    /***get yaw samples***/
    for (double u_yaw = -max_ddyaw_; u_yaw <= max_ddyaw_ + 1e-3; u_yaw += max_ddyaw_ * res)
    {
      yaw_inputs.push_back(u_yaw);
    }

    std::cout << "[:yawSearch]: yaw_state(1,0)  "  << yaw_state(1,0) << std::endl;
    std::cout << "[:yawSearch]: yaw_state(0,0)  "  << yaw_state(0,0) << std::endl;

    double total_time = traj.getTotalDuration();
    int seg_num = std::max(traj.getPieceNum(),  int(small_abs(yaw_state(1,0) - yaw_state(0,0))/max_dyaw_) + 2);
    double ts = total_time/double(seg_num) ;
    std::cout << "[:yawSearch]: seg_num   "  << seg_num  << std::endl;

    durations_ = ts * Eigen::VectorXd::Ones(seg_num);

    double dt_yaw = small_abs(yaw_state(1,0) - yaw_state(0,0)) / (total_time);

    double dt, tau, vis_res;
    Eigen::Matrix<double, 6, 1> p_state;
    double p_yaw, h_yaw;
    Eigen::Vector2d pro_yaw(0.0, 0.0), cur_yaw(0.0, 0.0);

    double current_time = cur_node->time;
    YawHashTable<YawNodePtr> tmp_expanded_nodes, expanded_nodes;
    expanded_nodes.insert(cur_node->yaw_idx, cur_node);


    Eigen::Vector3d pre_pos, dir, cur_pos;
    double pre_yaw, heading_yaw;
    
    for (unsigned int i = 0; i < durations_.size(); ++i)
    {
      /* ---------- state propagation loop ---------- */
      pre_pos = traj.getPos(0.0 + i * ts);
      pre_yaw = cur_node->yaw_state(0);
      tau = durations_[i];

      for (auto &node : expanded_nodes.data_)
      {
        cur_node = node.second;
        cur_yaw = cur_node->yaw_state;
        cur_pos = traj.getPos( (i+1) * ts);

        for (unsigned int k = 0; k < yaw_inputs.size(); ++k)
        {

          u_yaw = yaw_inputs[k];
          pro_yaw(0) = cur_yaw(0) + cur_yaw(1) * tau + 0.5 * u_yaw * pow(tau, 2);
          pro_yaw(1) = cur_yaw(1) + u_yaw * tau;

          int pro_yaw_id = yawToIndex(pro_yaw(0));

          /* yaw feasibility */
          if (abs(pro_yaw(1)) > max_dyaw_ + 1e-2)
          {
            continue;
          }

          /* ---------- compute cost ---------- */
          double tmp_f_score, tmp_gs_score = 0, tmp_ga_score = 0, tmp_gp_score = 0;

          /* -------- add perception aware planning ------- */
          Eigen::Vector2d dir;

          dir = cur_pos.head(2) - pre_pos.head(2);

          if (dir.norm() > 0.1){
            h_yaw = atan2(dir(1), dir(0));
          }
          else{
            h_yaw = pre_yaw + dt_yaw * tau;
          }

          tmp_gs_score += getSemanticsPenalty(cur_pos, p_yaw, h_yaw);
          tmp_ga_score += getInfoPenalty(cur_pos, p_yaw);
          tmp_f_score = (w_uyaw_ * u_yaw * u_yaw) * tau
                       + w_ref_ * tmp_gp_score
                       - w_semantics_ * tmp_gs_score
                       - w_info_ * tmp_ga_score 
                       + cur_node->f_score;

          /* check whether to update*/

          YawNodePtr pro_node = tmp_expanded_nodes.find(pro_yaw_id);
          if (pro_node != NULL)
          {
            if (tmp_f_score < pro_node->f_score)
            {

              pro_node->yaw_state = pro_yaw;
              pro_node->yaw_input = u_yaw;
              pro_node->yaw_idx = pro_yaw_id;
            }
          }
          else
          {

            pro_node = new YawNode;
            pro_node->f_score = tmp_f_score;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->time = cur_node->time + tau;

            pro_node->yaw_state = pro_yaw;
            pro_node->yaw_input = u_yaw;
            pro_node->yaw_idx = pro_yaw_id;
            pro_node->path_idx = i;

            tmp_expanded_nodes.insert(pro_yaw_id, pro_node);
          }
        }
      }

      if(tmp_expanded_nodes.size() < 1){
        cout << "no expended node !" << endl;
        return false;
      }
      expanded_nodes = tmp_expanded_nodes;
      tmp_expanded_nodes.clear();
      current_time += tau;
    }

    for (auto &node : expanded_nodes.data_)
    {
      cur_node = node.second;

      if (cur_node->f_score < terminate_node->f_score)
      {
        terminate_node = cur_node;
      }
    }

    yaw_state(1, 0) = terminate_node->yaw_state(0);
    yaw_state(1, 1) = terminate_node->yaw_state(1);

    retrievePath(terminate_node, yaw_nodes_);
    cout << "[yawSearch] Finished!  terminate_node->yaw_state is " << terminate_node->yaw_state << endl;
    cout << "[yawSearch] Finished!  yaw_nodes_.size() is " << yaw_nodes_.size() << endl;

    /* ---------- open set empty, no path ---------- */
    return true;
  }


  // Note: To interface with GCOPTER
  /****************************perception-aware yaw planning********************************/
  bool yawSearch(Eigen::MatrixXd &yaw_state,
                          Trajectory<5> &traj)
  {
    /* ---------- initialize ---------- */
    YawNodePtr cur_node = new YawNode;
    cur_node->parent = NULL;
    cur_node->time = (ros::Time::now()).toSec(); // primitive start time

    /* ---- initialize yaw angles ---- */
    // start yaw range is (-pi, pi]
    cur_node->yaw_state = (yaw_state.row(0)).head(2);
    cur_node->yaw_idx = yawToIndex(cur_node->yaw_state(0));
    cur_node->path_idx = 0;
    cur_node->f_score = 0.0;

    // PathNodePtr neighbor = NULL;
    YawNodePtr terminate_node = new YawNode;
    terminate_node->f_score = INFINITY;
    terminate_node->yaw_state = (yaw_state.row(1)).head(2);
    /* ---------- init state propagation ---------- */
    double res = 1 / 2.0;
    vector<double> yaw_inputs;
    double u_yaw;
    
    /***get yaw samples***/
    for (double u_yaw = -max_ddyaw_; u_yaw <= max_ddyaw_ + 1e-3; u_yaw += max_ddyaw_ * res)
    {
      yaw_inputs.push_back(u_yaw);
    }

    std::cout << "[:yawSearch]: yaw_state(1,0)  "  << yaw_state(1,0) << std::endl;
    std::cout << "[:yawSearch]: yaw_state(0,0)  "  << yaw_state(0,0) << std::endl;

    double total_time = traj.getTotalDuration();
    int seg_num = std::max(traj.getPieceNum(),  int(small_abs(yaw_state(1,0) - yaw_state(0,0))/max_dyaw_) + 2);
    double ts = total_time/double(seg_num) ;
    std::cout << "[:yawSearch]: seg_num   "  << seg_num  << std::endl;

    durations_ = ts * Eigen::VectorXd::Ones(seg_num);

    double dt_yaw = small_abs(yaw_state(1,0) - yaw_state(0,0)) / (total_time);

    double dt, tau, vis_res;
    Eigen::Matrix<double, 6, 1> p_state;
    double p_yaw, h_yaw;
    Eigen::Vector2d pro_yaw(0.0, 0.0), cur_yaw(0.0, 0.0);

    double current_time = cur_node->time;
    YawHashTable<YawNodePtr> tmp_expanded_nodes, expanded_nodes;
    expanded_nodes.insert(cur_node->yaw_idx, cur_node);


    Eigen::Vector3d pre_pos, dir, cur_pos;
    double pre_yaw, heading_yaw;
    
    for (unsigned int i = 0; i < durations_.size(); ++i)
    {
      /* ---------- state propagation loop ---------- */
      pre_pos = traj.getPos(0.0 + i * ts);
      pre_yaw = cur_node->yaw_state(0);
      tau = durations_[i];

      for (auto &node : expanded_nodes.data_)
      {
        cur_node = node.second;
        cur_yaw = cur_node->yaw_state;
        cur_pos = traj.getPos( (i+1) * ts);

        for (unsigned int k = 0; k < yaw_inputs.size(); ++k)
        {

          u_yaw = yaw_inputs[k];
          pro_yaw(0) = cur_yaw(0) + cur_yaw(1) * tau + 0.5 * u_yaw * pow(tau, 2);
          pro_yaw(1) = cur_yaw(1) + u_yaw * tau;

          int pro_yaw_id = yawToIndex(pro_yaw(0));

          /* yaw feasibility */
          if (abs(pro_yaw(1)) > max_dyaw_ + 1e-2)
          {
            continue;
          }

          /* ---------- compute cost ---------- */
          double tmp_f_score, tmp_gs_score = 0, tmp_ga_score = 0, tmp_gp_score = 0;

          /* -------- add perception aware planning ------- */
          Eigen::Vector2d dir;

          dir = cur_pos.head(2) - pre_pos.head(2);

          if (dir.norm() > 0.1){
            h_yaw = atan2(dir(1), dir(0));
          }
          else{
            h_yaw = pre_yaw + dt_yaw * tau;
          }

          tmp_gs_score += getSemanticsPenalty(cur_pos, p_yaw, h_yaw);
          tmp_ga_score += getInfoPenalty(cur_pos, p_yaw);
          tmp_f_score = (w_uyaw_ * u_yaw * u_yaw) * tau
                       + w_ref_ * tmp_gp_score
                       - w_semantics_ * tmp_gs_score
                       - w_info_ * tmp_ga_score 
                       + cur_node->f_score;

          /* check whether to update*/

          YawNodePtr pro_node = tmp_expanded_nodes.find(pro_yaw_id);
          if (pro_node != NULL)
          {
            if (tmp_f_score < pro_node->f_score)
            {

              pro_node->yaw_state = pro_yaw;
              pro_node->yaw_input = u_yaw;
              pro_node->yaw_idx = pro_yaw_id;
            }
          }
          else
          {

            pro_node = new YawNode;
            pro_node->f_score = tmp_f_score;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->time = cur_node->time + tau;

            pro_node->yaw_state = pro_yaw;
            pro_node->yaw_input = u_yaw;
            pro_node->yaw_idx = pro_yaw_id;
            pro_node->path_idx = i;

            tmp_expanded_nodes.insert(pro_yaw_id, pro_node);
          }
        }
      }

      if(tmp_expanded_nodes.size() < 1){
        cout << "no expended node !" << endl;
        return false;
      }
      expanded_nodes = tmp_expanded_nodes;
      tmp_expanded_nodes.clear();
      current_time += tau;
    }

    for (auto &node : expanded_nodes.data_)
    {
      cur_node = node.second;

      if (cur_node->f_score < terminate_node->f_score)
      {
        terminate_node = cur_node;
      }
    }

    yaw_state(1, 0) = terminate_node->yaw_state(0);
    yaw_state(1, 1) = terminate_node->yaw_state(1);

    retrievePath(terminate_node, yaw_nodes_);
    cout << "[yawSearch] Finished!  terminate_node->yaw_state is " << terminate_node->yaw_state << endl;
    cout << "[yawSearch] Finished!  yaw_nodes_.size() is " << yaw_nodes_.size() << endl;

    /* ---------- open set empty, no path ---------- */
    return true;
  }

  int getInfoPenalty(const Eigen::Vector3d &ego_pos, 
                              double &ego_yaw)
  {
    return frontier_finder_->computeYawInfoGain(ego_pos, ego_yaw); 
  }
  
  /* perception value computation*/
  double getSemanticsPenalty(const Eigen::Vector3d &ego_pos, 
                                      double &ego_yaw,
                                      double &heading_yaw)
  {
    double semantic_penalty = 0.0;
    double best_yaw, vioVal;

    if (door_pos.size() < 0)
    {
      return semantic_penalty;
    }
    for (size_t i = 0; i < door_pos.size(); ++i)
    {
      Eigen::Vector3d se_pos = door_pos.at(i);

      best_yaw = std::atan2(se_pos(1) - ego_pos(1), se_pos(0) - ego_pos(0));

      if (small_abs(heading_yaw - best_yaw) > half_fov_theta_)
      {
        continue;
      }

      vioVal = half_fov_theta_ - small_abs(ego_yaw - best_yaw);

      if (vioVal > 0)
      {
        semantic_penalty += vioVal; //* vioVal * vioVal;
      }
    }

    
    return semantic_penalty;
  }

 //Eigen::Vector2d yaw_state
  bool computeShotTraj(Eigen::Vector2d state1, 
                                Eigen::Vector2d state2,
                                double time_to_goal)
  {
    /* ---------- get coefficient ---------- */
    const double p0 = state1(0);
    const double dp = state2(0) - p0;
    const double v0 = state1(1);
    const double v1 = state2(1);
    const double dv = v1 - v0;
    double t_d = time_to_goal;
    Eigen::VectorXd coef(4);

    double a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
    double b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
    double c = v0;
    double d = p0;

    // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
    // a*t^3 + b*t^2 + v0*t + p0
    coef(3) = a, coef(2) = b, coef(1) = c, coef(0) = d;
     // cout << "coef is " << coef << endl;
    double coord = 0, vel = 0, acc = 0;
    VectorXd poly1d, t, polyv, polya;
    int index;

    Eigen::MatrixXd Tm(4, 4);
    Tm << 0, 1, 0, 0, 
          0, 0, 2, 0, 
          0, 0, 0, 3, 
          0, 0, 0, 0;

    /* ---------- forward checking of trajectory ---------- */
    double t_delta = t_d / 10;
    poly1d = coef;
    for (double time = t_delta; time <= t_d; time += t_delta)
    {
      t = VectorXd::Zero(4);
      for (int j = 0; j < 4; j++)
      {  
        t(j) = pow(time, j);
      }
      coord = poly1d.dot(t);
      vel = (Tm * poly1d).dot(t);
      acc = (Tm * Tm * poly1d).dot(t);
      
    }
    coef_shot_ = coef;
    t_shot_ = t_d;
    is_shot_succ_ = true;
    return true;
  }

  void getShotTraj(double &delta_t,
                            Eigen::MatrixXd &inner_yaws)
  {
    Eigen::VectorXd time(4);
    double t = 0;

    for (int i = 0; i < inner_yaws.size(); ++i)
    {
      t = delta_t * (i + 1);
      for (int j = 0; j < 4; j++)
      {
        time(j) = pow(t, j);
      } 

      inner_yaws(0, i) = coef_shot_.dot(time);
    }
  }



  void getYawInners(Eigen::MatrixXd &yaw_list, 
                             Eigen::VectorXd &durs)
  {
    yaw_list.resize(1, yaw_nodes_.size() - 2);
    yaw_list.setZero();
    durs =  durations_;

    /* ---------- get traj of searching ---------- */
    for (unsigned int i = 0; i < yaw_nodes_.size(); i++)
    {
      cout << "yaw_nodes_.at(i)->yaw_state is " << yaw_nodes_.at(i)->yaw_state << endl;
      YawNodePtr node = yaw_nodes_.at(i);
       
      if (i > 0 && i < yaw_nodes_.size()-1){
        yaw_list(0, i - 1) = node->yaw_state(0);
      }
    }

  }

  int yawToIndex(double yaw)
  {
    int idx = floor((yaw + M_PI) / yaw_resolution_);
    return idx;
  }


};

}  // namespace 

#endif
