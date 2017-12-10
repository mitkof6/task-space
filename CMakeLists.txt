CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

project(OpenSimTaskSpace VERSION 1.0.0)

# for rdm (emacs)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# c++11, -g option is used to export debug symbols for gdb
list(APPEND CMAKE_CXX_FLAGS "-std=c++11 -g ${CMAKE_CXX_FLAGS}")
# for grouping into folders
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

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
add_subdirectory(task_space)
