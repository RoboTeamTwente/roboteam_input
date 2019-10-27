cmake_minimum_required(VERSION 3.10)
project(roboteam_input)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(SDL2 REQUIRED sdl2)
include_directories(${SDL2_INCLUDE_DIRS} ${SDL2IMAGE_INCLUDE_DIRS})
add_executable(joystick src/joystick/JoystickManager.cpp src/joystick/JoystickHandler.cpp src/joystick/JoystickHandler.h src/joystick/JoystickState.cpp src/joystick/JoystickState.h)
target_link_libraries(joystick ${SDL2_LIBRARIES} roboteam_proto roboteam_utils)
