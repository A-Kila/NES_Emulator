cmake_minimum_required(VERSION 3.30.4)

file(GLOB_RECURSE sources *.c *.cpp *.h)

add_executable(nes_emulator_tests ${sources})

target_include_directories(nes_emulator_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)