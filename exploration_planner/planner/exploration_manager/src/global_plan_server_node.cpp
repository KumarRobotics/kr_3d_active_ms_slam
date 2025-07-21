#include <ros/ros.h>
#include <exploration_manager/global_plan_server.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "global_plan_server_node");
  ros::NodeHandle nh, pnh("~");

  ms_planner::GlobalPlanServer global_plan_node(nh, pnh);

  // ros::AsyncSpinner spinner(4); 
  // Use 4 thread spinner (will allow multiple callback to be called at the same time)
  // spinner.start();
  // ros::waitForShutdown();
  ros::spin();

  return 0;
}