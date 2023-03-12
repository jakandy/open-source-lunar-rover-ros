cmake_minimum_required(VERSION 2.8.3)

project(osr)

find_package(catkin REQUIRED
			roscpp
			rospy
			std_msgs
			geometry_msgs
			sensor_msgs
			osr_msgs)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(roslaunch)

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})