cmake_minimum_required(VERSION 2.8.3)
project(youbot_kinematics_rviz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  dynamic_reconfigure
  ik_solver_service
  std_msgs
)

 generate_dynamic_reconfigure_options(
   cfg/arm_joint_state.cfg
   cfg/work_space_state.cfg
 )

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs dynamic_reconfigure std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_executable(arm_state_publish src/joint_state_publisher.cpp)
add_dependencies(arm_state_publish ${PROJECT_NAME}_gencfg)

target_link_libraries(arm_state_publish
   ${catkin_LIBRARIES}
)
   
add_executable(ik_arm_state_publish src/ik_joint_state_publisher.cpp)
add_dependencies(ik_arm_state_publish ${PROJECT_NAME}_gencfg)

target_link_libraries(ik_arm_state_publish
   ${catkin_LIBRARIES}
)

