#!/bin/bash
set -e

# Setup ROS1 workspaces
source /root/rosbridge_ws/install/local_setup.bash

# Setup ROS2 workspaces
source /root/perception_ws/install/local_setup.bash

# Welcome information
echo "---------------------"
echo "ZED ROS2 ROS1_bridge Docker Image"
echo "---------------------"

unset ROS_DISTRO
source $ROS1_INSTALL_PATH/setup.bash
echo 'ROS1 distro: ' $ROS_DISTRO
unset ROS_DISTRO

source $ROS2_INSTALL_PATH/setup.bash
echo 'ROS2 distro: ' $ROS_DISTRO

echo 'ROS 1 Workspaces:' $ROS_PACKAGE_PATH
echo 'ROS 2 Workspaces:' $COLCON_PREFIX_PATH

echo 'ROS master URI:' $ROS_MASTER_URI

echo "---------------------" 
echo 'Available ZED packages:'
ros2 pkg list | grep zed
echo "---------------------"    
exec "$@"