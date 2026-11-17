# x30_add_pytests(<test directory or file>): run it with pytest under catkin's run_tests,
# results in the package's test_results (catkin_test_results reads them).
function(x30_add_pytests path)
  if(NOT CATKIN_ENABLE_TESTING)
    return()
  endif()
  get_filename_component(_x30_path "${path}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  if(NOT EXISTS "${_x30_path}")
    message(FATAL_ERROR "x30_add_pytests: ${path} does not exist")
  endif()
  string(REPLACE "/" "." _x30_name "${path}")
  set(_x30_xml "pytest-${_x30_name}.xml")
  set(_x30_out "${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/${_x30_xml}")
  catkin_run_tests_target("pytest" "${_x30_name}" "${_x30_xml}"
    COMMAND "${PYTHON_EXECUTABLE} -m pytest ${_x30_path} -p no:cacheprovider --junitxml=${_x30_out}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endfunction()
