cmake_minimum_required(VERSION 3.0.2)
project(x30_bringup)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(x30_utils REQUIRED)
  find_package(rostest REQUIRED)
  roslaunch_add_file_check(launch)
  x30_add_pytests(test/test_robot_profile.py)
  add_rostest(test/stack_base.test)
endif()
