cmake_minimum_required(VERSION 3.0.2)
project(x30_navigation)

find_package(catkin REQUIRED COMPONENTS x30_utils)
catkin_python_setup()
catkin_package()

catkin_install_python(
  PROGRAMS scripts/planner_node scripts/path_loader_node scripts/path_follower_node
           scripts/obstacle_avoidance_node scripts/docking_node scripts/train_dune
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  x30_add_pytests(test)
endif()
