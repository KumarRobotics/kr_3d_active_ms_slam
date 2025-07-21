#ifndef _COP_PLAN_SERVER_H_
#define _COP_PLAN_SERVER_H_

#include <ros/ros.h>
#include "cop_lib/cop.h"
#include "cop_lib/typedefs.h"
#include "cop_lib/graph.h"
#include "cop_lib/apsp_floyd_warshall.h"

#include <actionlib/server/simple_action_server.h>
#include <exploration_msgs/PlanCOPAction.h>

namespace ms_planner {
class GlobalPlanServer
{
public:
  GlobalPlanServer(ros::NodeHandle &nh_, ros::NodeHandle &pnh_);
  ~GlobalPlanServer();

    // Action Server goal callback
  void receiveGlobalPlanGoalCB(const exploration_msgs::PlanCOPGoal::ConstPtr &goal);

private:
  typedef actionlib::SimpleActionServer<exploration_msgs::PlanCOPAction> GlobalPlanServerType;

  // Nodehandles
  ros::NodeHandle nh, pnh;

  // Strings
  std::string global_plan_server_topic_;

  // action Server and param:  plan cop server
  std::unique_ptr<GlobalPlanServerType> global_plan_server_ptr_;
};
} // namespace exploration_manager

#endif