cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(test LANGUAGES CXX C)

add_subdirectory(bgfx.cmake)

add_custom_command(
  OUTPUT vs_instancing.bin fs_instancing.bin
  COMMAND shader_compile.bat
  DEPENDS vs_instancing.sc fs_instancing.sc
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
add_executable(test Source.cpp vs_instancing.bin fs_instancing.bin)
target_link_libraries(test bgfx example-common)
set_target_properties(test PROPERTIES CXX_STANDARD 14)
