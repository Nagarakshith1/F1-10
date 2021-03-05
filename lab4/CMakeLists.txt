cmake_minimum_required(VERSION 3.0.2)
project(lab4)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)
include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)
catkin_package()

add_executable(reactive_gap_follow src/reactive_gap_follow.cpp)
add_dependencies(reactive_gap_follow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reactive_gap_follow ${catkin_LIBRARIES})
