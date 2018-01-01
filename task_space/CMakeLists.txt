# TaskSpaceProjection
set(target TaskSpaceProjection)

file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
file(GLOB tests tests/*.cpp)
file(GLOB examples examples/*.cpp)

include_directories(test/)
include_directories(include/)

# task space projection module
add_library(${target} ${sources} ${includes})
target_link_libraries (${target} ${OpenSim_LIBRARIES})
set_target_properties(${target}
  PROPERTIES
  PROJECT_LABEL "TaskSpaceProjection"
  FOLDER "Library")

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
