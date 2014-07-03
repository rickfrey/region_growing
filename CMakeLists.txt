cmake_minimum_required(VERSION 2.8.3)
project(region_growing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
)

find_package(PCL REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES region_growing
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS eigen
#  DEPENDS system_lib
)

add_executable(region_growing_node src/region_growing.cpp)
target_link_libraries(region_growing_node
  ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Eigen_LIBRARIES}
    ${VTK_LIBRARIES}
)
