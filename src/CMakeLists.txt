cmake_minimum_required(VERSION 3.30.4)

file(GLOB_RECURSE sources *.c *.cpp *.h)

add_executable(nes_emulator  ${sources})
add_library(nes_emulator_lib STATIC ${sources})

target_include_directories(nes_emulator PRIVATE ${CMAKE_SOURCE_DIR}/dependencies/SDL/include)
target_include_directories(nes_emulator_lib PRIVATE ${CMAKE_SOURCE_DIR}/dependencies/SDL/include)
target_link_libraries(nes_emulator PRIVATE SDL3::SDL3)
target_link_libraries(nes_emulator_lib PRIVATE SDL3::SDL3)