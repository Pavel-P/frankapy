#!/bin/bash

control_pc_uname=${1}
control_pc_ip_address=${2}
workstation_ip_address=${3}
control_pc_use_password=${4}
control_pc_password=${5}
workstation_ip_address=${6}

rosmaster_path="bash_scripts/set_rosmaster.sh"
catkin_ws_setup_path="catkin_ws/devel/setup.bash"

if [ "$control_pc_ip_address" = "localhost" ]; then
    cd $HOME
    roslaunch realsense2_camera rs_camera.launch
    bash
else
if [ "$control_pc_use_password" = "0" ]; then
ssh -tt $control_pc_uname@$control_pc_ip_address << EOSSH
export ROS_MASTER_URI="http://${workstation_ip_address}:11311"
roslaunch realsense2_camera rs_camera.launch
bash
EOSSH
else
sshpass -p "$control_pc_password" ssh -tt -o StrictHostKeyChecking=no $control_pc_uname@$control_pc_ip_address << EOSSH
export ROS_MASTER_URI="http://${workstation_ip_address}:11311"
roslaunch realsense2_camera rs_camera.launch
bash
EOSSH
fi
fi
