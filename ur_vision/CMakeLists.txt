cmake_minimum_required(VERSION 3.0.2)
project(ur_vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge image_transport)
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV REQUIRED )

include_directories(${catkin_INCLUDE_DIRS})

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES Num.msg)
#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(vision src/cameraListener.cpp)
target_link_libraries(vision ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(vision beginner_tutorials_generate_messages_cpp)

