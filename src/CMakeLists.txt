cmake_minimum_required(VERSION 3.30.4)

file(GLOB_RECURSE sources *.c *.cpp *.h)

add_executable(nes_emulator ${sources})