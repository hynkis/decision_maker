cmake_minimum_required(VERSION 2.8.12)
project(decision_maker)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

find_package(
  catkin REQUIRED COMPONENTS
  autoware_msgs
  dynamic_reconfigure
  geometry_msgs
  jsk_recognition_msgs
  jsk_rviz_plugins
  roscpp
  state_machine_lib
  std_msgs
  tf
  ackermann_msgs
)

catkin_package()

include_directories(
  include/decision_maker
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  nodes/decision_maker/decision_maker_core.cpp
  nodes/decision_maker/decision_maker_node.cpp

  nodes/decision_maker/car_mode.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS 
      ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  FILES
  cfg/car_mode.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

