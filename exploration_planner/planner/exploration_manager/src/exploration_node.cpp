#include <ros/ros.h>
#include <exploration_manager/ms_exploration_fsm.h>

using namespace ms_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");

  msExplorationFSM expl_fsm;
  expl_fsm.init(nh);
  
  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  ros::Duration(1.0).sleep();
  spinner.spin(); // spin() will not return until the node has been shutdown

  // ros::spin();

  return 0;
}
