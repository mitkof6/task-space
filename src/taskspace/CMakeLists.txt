file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
file(GLOB tests tests/*.cpp)
file(GLOB examples examples/*.cpp)

include_directories(test/)
include_directories(include/)

# TaskSpace
set(target TaskSpace)
add_library(${target} SHARED ${sources} ${includes})
target_link_libraries (${target} ${OpenSim_LIBRARIES})
set_target_properties(${target}
  PROPERTIES
  PROJECT_LABEL "TaskSpace"
  FOLDER "Libraries")

# install library
install(
  TARGETS ${target}
  EXPORT ${TARGET_EXPORT_NAME}
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")

# install headers
set(INCLUDE_PREFIX "${CMAKE_INSTALL_INCLUDEDIR}")
install(FILES ${includes}
  DESTINATION ${INCLUDE_PREFIX}/${target})

# examples
foreach(example_program ${examples})
  # NAME_WE stands for "name without extension"
  get_filename_component(EXAMPLE_NAME ${example_program} NAME_WE)
  add_executable(${EXAMPLE_NAME} ${example_program})
  target_link_libraries(${EXAMPLE_NAME}  ${OpenSim_LIBRARIES} ${target})
  set_target_properties(${EXAMPLE_NAME}
    PROPERTIES
    PROJECT_LABEL "Example - ${EXAMPLE_NAME}"
    FOLDER "Examples")
endforeach()

set(ADDITIONAL_FILES
  "examples/arm26.osim"
  "examples/arm26_ideal_muscles.osim" 
  "examples/model_02-Scaled_to_subject.osim"
  "examples/hat_ribs_scap.obj"
  "examples/l_foot_surface_coarse.obj"
  "examples/r_foot_surface_coarse.obj"
  "examples/r_foot_surface_x1.0.obj"
  "examples/r_foot_surface_x1.1.obj"
  "examples/r_foot_surface_x1.2.obj"
  "examples/r_foot_surface_x1.3.obj"
  "examples/r_foot_surface_x1.25.obj"
  "examples/r_foot_surface_x2.0.obj"
  "examples/checkered_floor.vtp"
  )

foreach(dataFile ${ADDITIONAL_FILES})
  file(COPY "${dataFile}" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
endforeach()

# tests
if(BUILD_TESTING)
  enable_testing()
  foreach(test_program ${tests})
    # NAME_WE stands for "name without extension"
    get_filename_component(TEST_NAME ${test_program} NAME_WE)
    add_executable(${TEST_NAME} ${test_program})
    target_link_libraries(${TEST_NAME}  ${OpenSim_LIBRARIES} ${target})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    set_target_properties(${TEST_NAME}
      PROPERTIES
      PROJECT_LABEL "Test - ${TEST_NAME}"
      FOLDER "Tests")
  endforeach()
endif()
