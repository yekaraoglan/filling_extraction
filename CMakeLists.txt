cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(filling_extraction)

add_definitions(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

find_package(PCL 1.8 REQUIRED)
# message(FATAL_ERROR "asd:"${PCL_LIBRARY_DIRS})
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES filling_extraction
CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(filling_extraction
  src/FillingExtractor.cpp)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(filling_extractor_node src/filling_extractor_node.cpp src/FillingExtractor)
target_link_libraries(filling_extractor_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})


