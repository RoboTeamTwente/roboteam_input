cmake_minimum_required(VERSION 2.8.3)
project(roboteam_input)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  joy
  std_msgs
  sensor_msgs
  roboteam_msgs # for message definition
  roboteam_utils
  # roboteam_tactics
)

## Need to install:
# sudo apt-get install libsdl2-dev
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(SDL2)
include_directories(${SDL2_INCLUDE_DIR})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES roboteam_control
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
    CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    message_runtime
    roboteam_msgs
    roboteam_utils
    # roboteam_tactics
)

###########
## Build ##
###########

add_definitions(-std=c++11)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  src
  boost-process/include
)



add_executable(joystick_input src/joystick_input.cpp src/joystick_enums.h)
target_link_libraries(joystick_input ${catkin_LIBRARIES})
add_dependencies(joystick_input ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if (${SDL2_FOUND})

    MESSAGE("-- Building keyboard controller with SDL2")

    add_executable(keyboard_controller 
        src/keyboard_controller.cpp
        )
    target_link_libraries( keyboard_controller 
        ${catkin_LIBRARIES}
        ${SDL2_LIBRARY}
        )
    add_dependencies(keyboard_controller 
        ${${PROJECT_NAME}_EXPORTED_TARGETS} 
        ${catkin_EXPORTED_TARGETS}
        )

else (${SDL2_FOUND})

    MESSAGE("-- SDL2 not found, so cannot build keyboard_controller! Have you installed SDL2?")

endif(${SDL2_FOUND})




#############
## Testing ##
#############

catkin_add_gtest(input_test
    test/main.cpp
	test/SimpleTest.cpp
)
target_link_libraries(input_test ${catkin_LIBRARIES})


