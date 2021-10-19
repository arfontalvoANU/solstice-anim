# Copyright (C) CNRS 2016-2017
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.1)
project(solstice-anim C)
enable_testing()

set(SANIM_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../src)
option(NO_TEST "Do not build tests" OFF)

################################################################################
# Check dependencies
################################################################################
find_package(RCMake 0.4 REQUIRED)
find_package(RSys 0.10 REQUIRED)
find_package(OpenMP 1.2 REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${RCMAKE_SOURCE_DIR})
include(rcmake)
include(rcmake_runtime)

include_directories(
  ${RSys_INCLUDE_DIR})

rcmake_append_runtime_dirs(_runtime_dirs RSys)

################################################################################
# Configure and define targets
################################################################################
set(VERSION_MAJOR 0)
set(VERSION_MINOR 2)
set(VERSION_PATCH 4)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

set(SANIM_FILES_SRC
  sanim_node.c)

set(SANIM_FILES_INC_API
  sanim.h)

set(SANIM_FILES_INC
  sanim_node_c.h)

set(SANIM_FILES_DOC COPYING README.md)

# Prepend each file in the `SANIM_FILES_<SRC|INC>' list by `SANIM_SOURCE_DIR'
rcmake_prepend_path(SANIM_FILES_SRC ${SANIM_SOURCE_DIR})
rcmake_prepend_path(SANIM_FILES_INC ${SANIM_SOURCE_DIR})
rcmake_prepend_path(SANIM_FILES_INC_API ${SANIM_SOURCE_DIR})
rcmake_prepend_path(SANIM_FILES_DOC ${PROJECT_SOURCE_DIR}/../)

add_library(solstice-anim SHARED ${SANIM_FILES_SRC} ${SANIM_FILES_INC} ${SANIM_FILES_INC_API})
target_link_libraries(solstice-anim RSys)

if(CMAKE_COMPILER_IS_GNUCC)
  target_link_libraries(solstice-anim m)
endif()

set_target_properties(solstice-anim PROPERTIES
  DEFINE_SYMBOL SANIM_SHARED_BUILD
  COMPILE_FLAGS ${OpenMP_C_FLAGS}
  VERSION ${VERSION}
  SOVERSION ${VERSION_MAJOR})

if(CMAKE_COMPILER_IS_GNUCC)
  set_target_properties(solstice-anim PROPERTIES
  LINK_FLAGS ${OpenMP_C_FLAGS})
endif()

rcmake_setup_devel(solstice-anim SolAnim ${VERSION} solstice/sanim_version.h)

################################################################################
# Add tests
################################################################################
if(NOT NO_TEST)
  function(new_test _name)
    add_executable(${_name} ${SANIM_SOURCE_DIR}/${_name}.c)
    target_link_libraries(${_name} solstice-anim-test solstice-anim RSys)
    add_test(${_name} ${_name})
    rcmake_set_test_runtime_dirs(${_name} _runtime_dirs)
  endfunction()

  add_library(solstice-anim-test STATIC
    ${SANIM_SOURCE_DIR}/test_sanim_utils.h
    ${SANIM_SOURCE_DIR}/test_sanim_utils.c)

  new_test(test_sanim_node)
  new_test(test_sanim_node_transform)
  new_test(test_sanim_node_pivot)
  new_test(test_sanim_search)
  new_test(test_sanim_visit)
endif()

################################################################################
# Define output & install directories
################################################################################
install(TARGETS solstice-anim
  ARCHIVE DESTINATION bin
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES ${SANIM_FILES_INC_API} DESTINATION include/solstice)
install(FILES ${SANIM_FILES_DOC} DESTINATION share/doc/solstice-anim)

