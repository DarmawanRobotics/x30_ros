cmake_minimum_required(VERSION 3.0.2)
project(x30_utils)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package(CFG_EXTRAS x30_utils-extras.cmake)

if(CATKIN_ENABLE_TESTING)
  include(cmake/x30_utils-extras.cmake)
  x30_add_pytests(test/unit)
  x30_add_pytests(test/ros)
  x30_add_pytests(test/test_style.py)
endif()
