cmake_minimum_required(VERSION 2.8.3)
project(path_planning)
set (CMAKE_CXX_STANDARD 11)
## Add support for C++11, supported in ROS Kinetic and newer
 add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(octomap REQUIRED)
find_package(fcl REQUIRED)
find_package(OMPL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planning
  CATKIN_DEPENDS octomap_msgs octomap_ros pcl_conversions pcl_ros roscpp std_msgs
#  DEPENDS octomap PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  ${fcl_INCLUDE_DIRS}
)

link_libraries(${OCTOMAP_LIBRARIES} ${fcl_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(path_planning_node src/path_planning.cpp)
add_executable(old_path_planning src/old_path_planning.cpp)
add_executable(traj_gen src/trajectory_generator.cpp)
add_executable(scale src/scaling.cpp)
add_executable(goal_point_transformer src/goal_point_transformer.cpp)
add_executable(transform_pointcloud src/transform.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(path_planning_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${fcl_LIBRARIES}
   ${OMPL_LIBRARIES}
   fcl
)
add_dependencies(path_planning_node
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(old_path_planning
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${fcl_LIBRARIES}
   ${OMPL_LIBRARIES}
   fcl
)

target_link_libraries(traj_gen
   ${catkin_LIBRARIES}
)
add_dependencies(traj_gen
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(scale
   ${catkin_LIBRARIES}
)
add_dependencies(scale
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(goal_point_transformer
   ${catkin_LIBRARIES}
)
add_dependencies(goal_point_transformer
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(transform_pointcloud
   ${catkin_LIBRARIES}
)
add_dependencies(transform_pointcloud
  ${catkin_EXPORTED_TARGETS})
