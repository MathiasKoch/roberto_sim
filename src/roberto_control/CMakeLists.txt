cmake_minimum_required(VERSION 2.8.3)
project(roberto_control)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	controller_interface
	control_toolbox
	effort_controllers
	gazebo_ros
	gazebo_ros_control
)


find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})
if(${YAML_CPP_VERSION} VERSION_LESS 0.5)
  add_definitions(-DDEPRECATED_YAML_CPP_VERSION)
endif()

find_package(gazebo REQUIRED)


catkin_package(
  CATKIN_DEPENDS
  gazebo_ros_control
  controller_interface
  control_toolbox
  effort_controllers
  DEPENDS 
  yaml-cpp
  gazebo
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)


# FIX!
set(gazebo_libs /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so /usr/lib/x86_64-linux-gnu/libgazebo_common.so /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so /usr/lib/x86_64-linux-gnu/libgazebo_gui.so /usr/lib/x86_64-linux-gnu/libgazebo_gui_building.so /usr/lib/x86_64-linux-gnu/libgazebo_gui_viewers.so /usr/lib/x86_64-linux-gnu/libgazebo_math.so /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so /usr/lib/x86_64-linux-gnu/libgazebo_ode.so /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so /usr/lib/x86_64-linux-gnu/libgazebo_physics.so /usr/lib/x86_64-linux-gnu/libgazebo_physics_ode.so /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so /usr/lib/x86_64-linux-gnu/libgazebo_selection_buffer.so /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so /usr/lib/x86_64-linux-gnu/libgazebo_skyx.so /usr/lib/x86_64-linux-gnu/libgazebo_transport.so /usr/lib/x86_64-linux-gnu/libgazebo_util.so /usr/lib/x86_64-linux-gnu/libgazebo_player.so /usr/lib/x86_64-linux-gnu/libgazebo_rendering_deferred.so optimized /usr/lib/x86_64-linux-gnu/libprotobuf.so debug /usr/lib/x86_64-linux-gnu/libprotobuf.so-lpthread /usr/lib/x86_64-linux-gnu/libsdformat.so /usr/lib/x86_64-linux-gnu/libgazebo.so)



add_library(${PROJECT_NAME} 
  src/roberto_position_controller.cpp
  src/roberto_velocity_controller.cpp
)

target_link_libraries( ${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

add_library(roberto_gazebo_mimic_joint_plugin src/mimic_joint_plugin.cpp)

target_link_libraries(roberto_gazebo_mimic_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
