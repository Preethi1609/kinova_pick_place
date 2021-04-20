cmake_minimum_required(VERSION 3.1.3)
project(kinova_pick_and_place)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)



catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_ros_planning_interface
  DEPENDS
    EIGEN3
)

include_directories( ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/kinova_pick_and_place_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${PROJECT_NAME}_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

