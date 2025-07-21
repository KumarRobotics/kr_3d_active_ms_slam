#include "loop_closure/loopClosureServer.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "loop_closure_server_node");
  ros::NodeHandle n("loop_closure_server_node");
  LoopClosureServer in(n);

  while (ros::ok()) {
    ros::spin();
  }
}