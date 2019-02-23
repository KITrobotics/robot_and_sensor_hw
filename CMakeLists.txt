cmake_minimum_required(VERSION 2.8.3)
project(robot_and_sensor_hw)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  pluginlib
  roscpp
  controller_manager
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS hardware_interface pluginlib roscpp controller_manager
)

add_library(${PROJECT_NAME}
  src/robot_and_sensor_hw.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/robot_and_sensor_hw_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
