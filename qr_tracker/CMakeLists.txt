cmake_minimum_required(VERSION 2.8.3)
project(qr_tracker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qr_tracker
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#target_link_libraries(qr_tracker ${OpenCV_LIBRARIES})

