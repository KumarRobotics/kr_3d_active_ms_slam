#!/bin/bash

WORLD_FRAME_ID=world
ODOM_TOPIC=/dragonfly67/quadrotor_ukf/control_odom


MASTER_URI=http://10.42.0.1:11311
SETUP_ROS_STRING="export ROS_MASTER_URI=${MASTER_URI}"
SESSION_NAME=experiment

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


tmux new-window -t $SESSION_NAME -n "Main"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 7; roslaunch system_launch system_launch.launch" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "SLOAM"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch rgb_sem_segmentation rgb_segmentation_real_robot.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch scan2shape_launch real_robot_process_cloud_node.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch exploration_manager tmux_real_robot_sloam_exploration.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch exploration_manager tmux_sim_closure_submap.launch" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Closure"
tmux send-keys -t $SESSION_NAME "sleep 1; rosrun loop_closure loop_closure_server_node" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "COP"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 6; roslaunch exploration_manager global_plan_server.launch" Enter
tmux select-layout -t $SESSION_NAME even-horizontal

tmux new-window -t $SESSION_NAME -n "Exp"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 8; roslaunch exploration_manager tmux_exploration_real_robot.launch" Enter
tmux select-layout -t $SESSION_NAME even-horizontal

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

tmux select-window -t $SESSION_NAME:5
tmux -2 attach-session -t $SESSION_NAME
