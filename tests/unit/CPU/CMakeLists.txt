cmake_minimum_required(VERSION 3.30.4)

file(GLOB_RECURSE sources *.c *.cpp *.h)

enable_testing()

# Add the test executable
add_executable(nes_emulator_cpu_unit_tests ${sources})

target_include_directories(nes_emulator_cpu_unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(nes_emulator_cpu_unit_tests PRIVATE GTest::gtest_main nes_emulator_lib)

include(GoogleTest)
gtest_discover_tests(nes_emulator_cpu_unit_tests)