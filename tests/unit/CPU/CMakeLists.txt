cmake_minimum_required(VERSION 3.30.4)

project(nes_emulator_cpu_unit_tests CXX)

file(GLOB_RECURSE sources *.c *.cpp *.h)

# Add the test executable
add_executable(${PROJECT_NAME} ${sources})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(${PROJECT_NAME} PRIVATE GTest::gtest_main nes_emulator_lib)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})