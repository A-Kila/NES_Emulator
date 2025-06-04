cmake_minimum_required(VERSION 3.30.4)

file(GLOB_RECURSE sources *.c *.cpp *.h)

# Initilize googletest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/7da55820cc32dedd6c1b048f2d4e13fdde5e8237.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_subdirectory(unit)
add_subdirectory(system)

set(
  ALL_TESTS
  nes_emulator_cpu_system_tests 
  nes_emulator_cpu_unit_tests
)

add_custom_target(build_all_tests DEPENDS ${ALL_TESTS})
add_custom_target(run_all_tests
  COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIG> --output-on-failure
  DEPENDS build_all_tests
)