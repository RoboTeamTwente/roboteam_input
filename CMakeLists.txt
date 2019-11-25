cmake_minimum_required(VERSION 3.10)
project(roboteam_input)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(SDL2 REQUIRED sdl2)
include_directories(${SDL2_INCLUDE_DIRS} ${SDL2IMAGE_INCLUDE_DIRS})

set (JOYSTICK_SOURCES
        src/joystick/JoystickHandler.cpp
        src/joystick/JoystickManager.cpp
        )

add_executable(joystick
        ${JOYSTICK_SOURCES}
        )

target_include_directories(joystick
        PRIVATE include/roboteam_input
        )

target_link_libraries(joystick
        ${SDL2_LIBRARIES}
        roboteam_proto
        roboteam_utils
        )

