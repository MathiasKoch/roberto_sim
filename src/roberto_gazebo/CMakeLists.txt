cmake_minimum_required(VERSION 2.8.3)
project(roberto_gazebo)

find_package(catkin 
	REQUIRED COMPONENTS
	roscpp
	gazebo_ros
	control_toolbox
	roberto_msgs)


find_package(gazebo REQUIRED)

catkin_package(
	CATKIN_DEPENDS 
	roberto_msgs
	roscpp
	DEPENDS 
	Boost
	gazebo
	INCLUDE_DIRS include
	LIBRARIES roberto_gazebo_ros_control)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
	            ${GAZEBO_INCLUDE_DIRS})


add_library(roberto_gazebo_ros_control
  plugins/motor_plugin.cpp
  plugins/gazebo_ros_utils.cpp
)

target_link_libraries(roberto_gazebo_ros_control
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)


add_library(roberto_gazebo_mimic_joint_plugin 
	plugins/mimic_joint_plugin.cpp
)

target_link_libraries(roberto_gazebo_mimic_joint_plugin 
	${catkin_LIBRARIES} 
	${GAZEBO_LIBRARIES}
)


add_dependencies(
  roberto_gazebo_ros_control
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS roberto_gazebo_ros_control
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
