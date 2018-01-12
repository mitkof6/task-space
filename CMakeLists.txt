CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

################################################################################

# project
project(TaskSpace VERSION 1.0.0)

# modules
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# ctest
include(CTest)

option(BUILD_DOCUMENTATION
  "Create and install the HTML based API documentation (requires Doxygen)" ON)

# for rdm (emacs)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# for grouping into folders
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# c++11, -g option is used to export debug symbols for gdb
if(${CMAKE_CXX_COMPILER_ID} MATCHES GNU OR
    ${CMAKE_CXX_COMPILER_ID} MATCHES Clang)
  # Using C++11 on OSX requires using libc++ instead of libstd++.
  # libc++ is an implementation of the C++ standard library for OSX.
  if(APPLE)
    if(XCODE)
      set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
      set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
    endif()
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
  endif()
endif()

# report platform
if(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
  set(PLATFORM_ABI x64)
else()
  set(PLATFORM_ABI x86)
endif()
set(BUILD_PLATFORM "${CMAKE_HOST_SYSTEM_NAME}:${PLATFORM_ABI}" CACHE STRING
"This is the platform and ABI we're building for." FORCE)

# install layout
# Layout. This works for all platforms:
#   * <prefix>/lib/cmake/<PROJECT-NAME>
#   * <prefix>/lib/
#   * <prefix>/include/
#   * <prefix>/bin/
#   * <prefix>/doc/
#   * <prefix>/scripts/
set(CMAKE_INSTALL_BINDIR bin)
set(CMAKE_INSTALL_INCLUDEDIR include)
set(CMAKE_INSTALL_LIBDIR lib)
set(CMAKE_INSTALL_DOCDIR ${CMAKE_INSTALL_PREFIX}/docs)
set(CMAKE_INSTALL_CONFIG ${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME})
set(CMAKE_INSTALL_SCRIPTS ${CMAKE_INSTALL_PREFIX}/scripts)

# define library postfix so that different builds will produce
# distinguished libraries
set(CMAKE_RELEASE_POSTFIX "_r" CACHE string "Release postfix")
set(CMAKE_DEBUG_POSTFIX "_d" CACHE string "Debug postfix")
set(CMAKE_RELWITHDEBINFO_POSTFIX "_rd" CACHE string
  "Release with debug info postfix")
set(CMAKE_MINSIZEREL_POSTFIX "_mr" CACHE string
  "Minimum size release postfix")
mark_as_advanced(CMAKE_RELEASE_POSTFIX)
mark_as_advanced(CMAKE_DEBUG_POSTFIX)
mark_as_advanced(CMAKE_RELWITHDEBINFO_POSTFIX)
mark_as_advanced(CMAKE_MINSIZEREL_POSTFIX)

################################################################################

# configure OpenSim
find_package(OpenSim REQUIRED)
if(OpenSim_FOUND)
  message(STATUS "OpenSim found at ${OpenSim_DIR}")
  include_directories(${OpenSim_INCLUDE_DIRS})
  link_directories(${OpenSim_LIB_DIR})
else()
  message(FATAL_ERROR "Can't find OpenSim at ${OpenSim_DIR}")
endif()

################################################################################

# submodules
add_subdirectory(src)
add_subdirectory(doc)

################################################################################

# installation (https://github.com/forexample/package-example)

# include module with function 'write_basic_package_version_file'
include(CMakePackageConfigHelpers)

# configuration
set(TARGET_EXPORT_NAME "${PROJECT_NAME}-targets" CACHE
  INTERNAL "Internal variable that is referenced by library generation")
set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(VERSION_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(PROJECT_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}Config.cmake")

# Configure '<PROJECT-NAME>ConfigVersion.cmake'
# Note: PROJECT_VERSION is used as a VERSION
write_basic_package_version_file(
  "${VERSION_CONFIG}" COMPATIBILITY SameMajorVersion
  )

# Configure '<PROJECT-NAME>Config.cmake'
configure_package_config_file(
  "cmake/Config.cmake.in"
  "${PROJECT_CONFIG}"
  INSTALL_DESTINATION "${CMAKE_INSTALL_CONFIG}"
  )

# intall configuration
#   * <prefix>/lib/cmake/Foo/FooConfig.cmake
#   * <prefix>/lib/cmake/Foo/FooConfigVersion.cmake
install(
  FILES "${PROJECT_CONFIG}" "${VERSION_CONFIG}"
  DESTINATION "${CMAKE_INSTALL_CONFIG}"
  )

# install target configuration
#   * <prefix>/lib/cmake/Foo/FooTargets.cmake
install(
  EXPORT ${TARGET_EXPORT_NAME}
  DESTINATION "${CMAKE_INSTALL_CONFIG}"
  )

# scripts directory
install(
  DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/scripts/
  DESTINATION "${CMAKE_INSTALL_SCRIPTS}"
  )
# others dirs can be copied to the install dir
