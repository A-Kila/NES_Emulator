cmake_minimum_required(VERSION 3.30.4)

file(GLOB_RECURSE sources *.c *.cpp *.h)

enable_testing()

# Add the test executable
add_executable(nes_emulator_cpu_system_tests ${sources})

add_compile_definitions("CURRENT_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}")
add_compile_definitions("RESOURCES_DIR=${CMAKE_BINARY_DIR}/tests/system/resources")

add_custom_command(
    TARGET nes_emulator_cpu_system_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/tests/system/resources
    ${CMAKE_BINARY_DIR}/tests/system/resources
)

target_include_directories(nes_emulator_cpu_system_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(nes_emulator_cpu_system_tests PRIVATE GTest::gtest_main nes_emulator_lib)

include(GoogleTest)
gtest_discover_tests(nes_emulator_cpu_system_tests)