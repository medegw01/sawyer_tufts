cmake_minimum_required(VERSION 2.8.3)
project(sawyer_tufts)

find_package(catkin REQUIRED COMPONENTS
  rospy
  intera_core_msgs
  gazebo_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  intera_core_msgs
  gazebo_msgs
)


#############
## Install ##
#############

install(PROGRAMS
   scripts/pick_and_place_tufts.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch models)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
