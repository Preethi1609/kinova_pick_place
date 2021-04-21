cmake_minimum_required(VERSION 3.1.3)
project(kinova_pick_and_place)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
)

catkin_package(
  CATKIN_DEPENDS
    moveit_ros_planning_interface
)

include_directories( ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/kinova_pick_and_place_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

