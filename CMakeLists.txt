cmake_minimum_required(VERSION 3.0.2)
project(jetbot_diff_drive)

find_package(catkin REQUIRED)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})