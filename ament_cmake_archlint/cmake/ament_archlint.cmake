# Copyright 2025 PAL Robotics
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Add a test to check manifests of ROS skills.
#
# :param TESTNAME: the name of the test, default: "archlint"
# :type TESTNAME: string
# :param ARGN: the files or directories to check
# :type ARGN: list of strings
#
# @public
#
function(ament_archlint)
  cmake_parse_arguments(ARG "" "TESTNAME" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "archlint")
  endif()

  find_program(ament_archlint_BIN NAMES "ament_archlint")
  if(NOT ament_archlint_BIN)
    message(FATAL_ERROR "ament_archlint() could not find program 'ament_archlint'")
  endif()

  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xunit.xml")
  set(cmd "${ament_archlint_BIN}" "--xunit-file" "${result_file}" ".")
  list(APPEND cmd ${ARG_UNPARSED_ARGUMENTS})

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_archlint")
  ament_add_test(
    "${ARG_TESTNAME}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_archlint/${ARG_TESTNAME}.txt"
    RESULT_FILE "${result_file}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  set_tests_properties(
    "${ARG_TESTNAME}"
    PROPERTIES
    LABELS "archlint;linter"
  )
endfunction()
