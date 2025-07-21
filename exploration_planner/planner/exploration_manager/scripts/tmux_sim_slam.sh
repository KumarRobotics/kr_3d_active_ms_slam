#!/bin/bash

MAV_TYPE=dragon_ddk
MAV_NAME=ddk
WORLD_FRAME_ID=world
ODOM_TOPIC=ground_truth/odom

echo "MAV name: $MAV_NAME MAV Type: $MAV_TYPE"

MASTER_URI=http://localhost:11311
SETUP_ROS_STRING="export ROS_MASTER_URI=${MASTER_URI}"
SESSION_NAME=sim_exp

CURRENT_DISPLAY=${DISPLAY}
if [ -z ${DISPLAY} ];
then
  echo "DISPLAY is not set"
  CURRENT_DISPLAY=:0
fi

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# Make mouse useful in copy mode
tmux setw -g mouse on


tmux rename-window -t $SESSION_NAME "Core"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roscore" Enter
tmux split-window -t $SESSION_NAME
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Main"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; roslaunch exploration_manager tmux_gazebo_sim.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; roslaunch exploration_manager tmux_spawn.launch x:=3.0 y:=0.5 Y:=3.14" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; roslaunch exploration_manager tmux_control.launch odom_topic:=/gt_odom_perturbed" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; export DISPLAY=${CURRENT_DISPLAY}; rosparam set robot_name ${MAV_NAME}; rosrun rqt_mav_manager rqt_mav_manager" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "SLOAM"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch rgb_sem_segmentation rgb_segmentation_sim.launch odom_topic:=/gt_odom_perturbed" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch scan2shape_launch sim_process_cloud_node.launch odom_topic:=/gt_odom_perturbed" Enter

tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch exploration_manager tmux_sim_sloam_exploration.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch exploration_manager tmux_sim_closure_submap.launch" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Closure"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch scan2shape_launch sim_perturb_odom.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 1; rosrun loop_closure loop_closure_server_node" Enter
tmux select-layout -t $SESSION_NAME tiled


tmux new-window -t $SESSION_NAME -n "COP"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch exploration_manager global_plan_server.launch" Enter
tmux select-layout -t $SESSION_NAME even-horizontal

tmux new-window -t $SESSION_NAME -n "Exp"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 6; roslaunch exploration_manager tmux_exploration.launch" Enter
tmux select-layout -t $SESSION_NAME even-horizontal

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

tmux select-window -t $SESSION_NAME:5
tmux -2 attach-session -t $SESSION_NAME
