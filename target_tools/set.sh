#!/bin/bash
echo "Setting enviroment for Rasbery pi....."
mkdir -p /home/kohei/robot/addLibs/BOOST
export LC_ALL="C"
cd /home/kohei/robot/ros_catkin_ws/devel_isolated/
source setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/kohei/robot/addLibs/BOOST
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"

echo "Finished setting up the environment...."

