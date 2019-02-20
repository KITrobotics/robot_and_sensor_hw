cmake_minimum_required(VERSION 2.8.3)
project(robot_and_sensor_hw)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  pluginlib
  roscpp
  controller_manager
  controller_manager_tests
  
  force_torque_sensor
  kuka_rsi_hw_interface
  kuka_rsi_simulator
  testing_controller
  iirob_kuka_bringup
  kuka_kr5_support
  r5_ikfast_arm_plugin
  r5_moveit_config
  iirob_powerball_ikfast_arm_plugin
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS hardware_interface pluginlib roscpp
  
  force_torque_sensor
  kuka_rsi_hw_interface
  kuka_rsi_simulator
  testing_controller
  iirob_kuka_bringup
  kuka_kr5_support
  r5_ikfast_arm_plugin
)

add_library(${PROJECT_NAME}
  src/robot_and_sensor_hw.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testing_dummy_app src/dummy_app.cpp)
target_link_libraries(testing_dummy_app ${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
