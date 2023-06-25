cmake_minimum_required(VERSION 3.0.2)
project(ur_kinematic)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation geometry_msgs)
find_package(Eigen3 3.3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

add_service_files(
  DIRECTORY
  srv
  FILES
  generic_float.srv
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(motion src/motion.cpp)
target_link_libraries(motion ${catkin_LIBRARIES})
add_dependencies(motion ur_kinematic_generate_messages_cpp)
install(TARGETS motion
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# add_executable(prova src/prova.cpp)
# target_link_libraries(prova ${catkin_LIBRARIES})
# add_dependencies(prova ur_kinematic_generate_messages_cpp)
# install(TARGETS prova
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )