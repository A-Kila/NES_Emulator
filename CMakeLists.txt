cmake_minimum_required(VERSION 3.30.4)
project(NES_Emulator)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(dependencies)
add_subdirectory(src)
add_subdirectory(tests)
