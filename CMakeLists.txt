cmake_minimum_required(VERSION 2.8.3)

project(oslr)

find_package(catkin REQUIRED
			roscpp
			rospy
			std_msgs
			geometry_msgs
			sensor_msgs
			message_generation)

add_message_files(
	FILES
	Drive.msg
	Steer.msg
)

generate_messages(
	DEPENDENCIES
	sensor_msgs
	std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(roslaunch)

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})