cmake_minimum_required(VERSION 3.30.4)

project(nes_emulator_cpu_system_tests CXX)

file(GLOB_RECURSE sources *.c *.cpp *.h)

# Add the test executable
add_executable(${PROJECT_NAME} ${sources})

add_compile_definitions("CURRENT_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}")
add_compile_definitions("RESOURCES_DIR=${CMAKE_BINARY_DIR}/tests/system/resources")

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/tests/system/resources
    ${CMAKE_BINARY_DIR}/tests/system/resources
)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(${PROJECT_NAME} PRIVATE GTest::gtest_main nes_emulator_lib)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME} )