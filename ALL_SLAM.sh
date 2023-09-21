#!/bin/bash

source /home/nvidia/A_SLAM_3d/SLAM/ws_livox/devel/setup.bash
source /home/nvidia/A_SLAM_3d/SLAM/test/DLO/devel/setup.bash
source /home/nvidia/A_SLAM_2d/devel/setup.bash
# export DISPLAY=192.168.203.63:0.0

{
    source /home/nvidia/A_SLAM_3d/SLAM/ws_livox/devel/setup.bash
    gnome-terminal -x bash -c "roslaunch livox_ros_driver2 MID360.launch;exec bash"
}&
sleep 1s
{
    source /home/nvidia/A_SLAM_3d/SLAM/test/DLO/devel/setup.bash
    gnome-terminal -x bash -c "roslaunch direct_lidar_odometry dlo.launch;exec bash"
}&
sleep 4s
{
    source /home/nvidia/A_SLAM_2d/devel/setup.bash
    gnome-terminal -x bash -c "roslaunch auto_navigation ALL1.launch;exec bash"
}&
sleep 3s
{
    source /home/nvidia/A_SLAM_2d/devel/setup.bash
    gnome-terminal -x bash -c "roslaunch rm_usart move_base_node.launch;exec bash"
}&





