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
# sudo apt-get install libsdl2-dev libsdl2-ttf-dev
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(SDL2)
find_package(SDL2TTF)
include_directories(${SDL2_INCLUDE_DIR} ${SDL2TTF_INCLUDE_DIR})

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



add_executable(joystick
        src/joystick/joystick.cpp
        src/joystick/joystick_enums.h
        src/joystick/joystick_manager.h
        src/joystick/joystick_profiles.h
        src/joystick/joystick_manager.cpp
        src/constants.h)

target_link_libraries(joystick ${catkin_LIBRARIES})
add_dependencies(joystick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if (${SDL2_FOUND})
  if (${SDL2TTF_FOUND})

    MESSAGE("-- Building keyboard controller with SDL2")

    add_executable(keyboard
        src/keyboard/keyboard.cpp
        src/keyboard/keyboard_manager.h
        src/keyboard/keyboard_manager.cpp
        src/input_interface.h
        src/input_interface.cpp
            src/constants.h
        )
    target_link_libraries( keyboard
        ${catkin_LIBRARIES}
        ${SDL2_LIBRARY}
        ${SDL2TTF_LIBRARY}

        )
    add_dependencies(keyboard
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
else (${SDL2TTF_FOUND})
MESSAGE("-- SDL2TTF not found, so cannot build keyboard_controller! Have you installed SDL2TTF?")

endif(${SDL2TTF_FOUND})

else (${SDL2_FOUND})

    MESSAGE("-- SDL2 not found, so cannot build keyboard_controller! Have you installed SDL2?")

endif(${SDL2_FOUND})

#############
## Testing ##
#############
catkin_add_gtest(input_test
        test/main.cpp
        test/KeyboardManagerTest.cpp
        src/keyboard/keyboard_manager.h
        src/keyboard/keyboard_manager.cpp
        test/JoystickTest.cpp
        src/constants.h
        )

target_link_libraries(input_test ${catkin_LIBRARIES}
        ${SDL2_LIBRARY}
        ${SDL2TTF_LIBRARY})