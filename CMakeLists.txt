cmake_minimum_required(VERSION 3.0.2)
project(image_stitcher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  message_filters
)
find_package(
        OpenCV 4.2 REQUIRED
)


catkin_package(
        CATKIN_DEPENDS roscpp image_transport sensor_msgs cv_bridge
        DEPENDS OpenCV message_filters
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   src/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_stitcher include/image_stitcher/image_stitcher.h
        src/image_stitcher.cpp)
target_link_libraries(image_stitcher
        ${catkin_LIBRARIES}  # 链接 ROS 库，包括 roscpp
        ${OpenCV_LIBRARIES}
)