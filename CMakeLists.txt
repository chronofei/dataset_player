cmake_minimum_required(VERSION 3.0.2)
project(dataset_player)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dataset_player
#  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs pcl_ros sensor_msgs std_msgs tf tf2
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(SRC
    lib/common.cpp
    lib/BasicDatasetPlayer.cpp
    lib/KITTIDatasetPlayer.cpp
    lib/TUMDatasetPlayer.cpp
    lib/DatasetPlayer.cpp
)

add_library(${PROJECT_NAME}
  ${SRC}
)

add_executable(${PROJECT_NAME}_node src/dataset_player_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
)