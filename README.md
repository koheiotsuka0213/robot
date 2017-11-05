# robot

# Build
# cd ros_catkin_ws
#
# [For ARM Rasberry Pi3]
# ./src/catkin/bin/catkin_make_isolated -DCMAKE_TOOLCHAIN_FILE=/home/kohei/robot/rostoolchain.cmake -DPYTHON_EXECUTABLE=/usr/bin/python -DENABLE_PRECOMPILED_HEADERS=OFF
# 
# [For X86]
#./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release



# Rasberry Pi3 set up
# 1. ROS
# http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi

# 2. copy set.sh to target
# 3. bash set.sh
# 4. Copy dependent library 
