# TaskSpaceProjection
set(target TaskSpaceProjection)

file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
file(GLOB tests test/*.cpp)

include_directories(include/)

# task space projection module
add_library(${target} ${sources} ${includes})
target_link_libraries (${target} ${OpenSim_LIBRARIES})
set_target_properties(${target}
  PROPERTIES
  PROJECT_LABEL "TaskSpaceProjection"
  FOLDER "Module")

# tests
foreach(test_program ${tests})
  # NAME_WE stands for "name without extension"
  get_filename_component(TEST_NAME ${test_program} NAME_WE)
  add_executable(${TEST_NAME} ${test_program}
    ${ADDTESTS_SOURCES})
  target_link_libraries(${TEST_NAME}  ${OpenSim_LIBRARIES} ${target})
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
  set_target_properties(${TEST_NAME}
    PROPERTIES
    PROJECT_LABEL "Test - ${TEST_NAME}"
    FOLDER "Tests"
    )
endforeach()
