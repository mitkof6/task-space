CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
# project
project(TaskSpace VERSION 1.0.0)

# modules
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# ctest
include(CTest)

option(BUILD_DOCUMENTATION
  "Create and install the HTML based API documentation (requires Doxygen)" OFF)
# for rdm (emacs)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# for grouping into folders
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
# c++11, -g option is used to export debug symbols for gdb
list(APPEND CMAKE_CXX_FLAGS "-std=c++11 -g ${CMAKE_CXX_FLAGS}")

# configure OpenSim
find_package(OpenSim REQUIRED)
if(OpenSim_FOUND)
  message(STATUS "OpenSim found at ${OpenSim_DIR}")
  include_directories(${OpenSim_INCLUDE_DIRS})
  link_directories(${OpenSim_LIB_DIR})
else()
  message(FATAL_ERROR "Can't find OpenSim at ${OpenSim_DIR}")
endif()

# submodules
add_subdirectory(src)
add_subdirectory(doc)
