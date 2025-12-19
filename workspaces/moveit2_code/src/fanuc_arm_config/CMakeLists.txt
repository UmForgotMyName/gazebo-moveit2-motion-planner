cmake_minimum_required(VERSION 3.22)
project(fanuc_arm_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/moveit-rviz.launch.py
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/gazebo_moveit_integration.launch.py
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/gazebo_moveit_octomap.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF/xacro and meshes
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/fanuc200ic5l_w_sg2.urdf
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/fanuc200ic5l_w_sg2.xacro
  DESTINATION share/${PROJECT_NAME}/urdf
)
install(DIRECTORY urdf/meshes/
  DESTINATION share/${PROJECT_NAME}/urdf/meshes
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
