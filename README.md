# robot

# Build
#
# cd ros_catkin_ws
#
# [For ARM Rasberry Pi3]
# 1) OpenCV
# 1.1) cmake ../ros_catkin_ws/src/opencv/ -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_TOOLCHAIN_FILE=/home/kohei/robot/rostoolchain.cmake @/home/kohei/robot/opencv_raspi 
# 1.2) make
#
# 2) ROS pkg (original/custom)
# touch src/rviz/CATKIN_IGNORE && ./src/catkin/bin/catkin_make_isolated -DCMAKE_TOOLCHAIN_FILE=/home/kohei/robot/rostoolchain.cmake -DPYTHON_EXECUTABLE=/usr/bin/python -DENABLE_PRECOMPILED_HEADERS=OFF -DCATKIN_ENABLE_TESTING=OFF -DOpenCV_DIR=/home/kohei/robot/opencv_raspi
# 
# [For X86]
# 1) OpenCV
# 1.1) cmake ../ros_catkin_ws/src/opencv/ -DCMAKE_BUILD_TYPE=RELEASE @/home/kohei/robot/opencv
# 1.2) make
#
# 2) ROS pkg (original/custom)
# rm ./src/rviz/CATKIN_IGNORE && ./src/catkin/bin/catkin_make_isolated -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/home/kohei/robot/opencv/ -DQt5_DIR=/home/kohei/robot/Qt5.6.3/5.6.3/gcc_64/lib/cmake/Qt5/
# 2.1) rviz (only x86) Before run rviz(Temporary fix).
# sudo ln -s /home/kohei/robot/Qt5.6.3/5.6.3/gcc_64/plugins/platforms/ /home/kohei/robot/ros_catkin_ws/devel_isolated/rviz/lib/rviz/platforms


# Rasberry Pi3 set up
# 1. ROS
# http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi

# 2. copy devel_isolated, src, set.sh to target /home/kohei/robot/ros_catkin_ws/.
# 3. source set.sh
# 4. Copy dependent missing library if needed to /home/kohei/robot/addLibs/.
# 5. Copy data if needed.
#
