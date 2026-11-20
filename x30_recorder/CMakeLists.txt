cmake_minimum_required(VERSION 3.0.2)
project(x30_recorder)

find_package(catkin REQUIRED COMPONENTS x30_utils)
catkin_python_setup()
catkin_package()

if(CATKIN_ENABLE_TESTING)
  x30_add_pytests(test)
endif()
