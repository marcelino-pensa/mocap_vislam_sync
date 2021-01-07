cmake_minimum_required(VERSION 2.8.3)
project(mocap_vislam_sync)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  message_filters
  pensa_msgs
  roscpp
  std_srvs
  std_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(mocap_vislam_sync src/mocap_vislam_sync_class.cpp
                                 src/msg_conversions.cpp
                                 src/vislam_sync.cpp)
target_link_libraries(mocap_vislam_sync ${catkin_LIBRARIES})
add_dependencies(mocap_vislam_sync ${catkin_EXPORTED_TARGETS})

add_executable(odom_pose_stamped_sync src/odom_posestamped_sync.cpp
                                      src/msg_conversions.cpp
                                      src/odom_pose_sync.cpp)
target_link_libraries(odom_pose_stamped_sync ${catkin_LIBRARIES})
add_dependencies(odom_pose_stamped_sync ${catkin_EXPORTED_TARGETS})