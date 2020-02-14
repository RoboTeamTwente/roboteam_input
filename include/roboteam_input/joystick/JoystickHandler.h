//
// Created by luukkn on 23-10-19.
//

#ifndef RTT_JOYSTICKHANDLER_H
#define RTT_JOYSTICKHANDLER_H
#include <SDL2/SDL.h>
#include <cstdio>
#include <SDL2/SDL_joystick.h>
#include <iostream>

#include "JoystickState.h"
#include "../../../../roboteam_ai/include/roboteam_ai/control/RobotCommand.h"

namespace rtt {
namespace input {

class JoystickHandler {
private:

    proto::RobotCommand command;
    JoystickState joystickState;
    bool notPressedL = true;
    bool notPressedR = true;
    float robotAngle = 0.0;
    int robotId = -1;
    int dribbler_vel = 10;

public:
    JoystickHandler();
    void tick();
    void handleEvent(SDL_Event &event);
    void handleJoystickMotion(SDL_Event &event);
    void handleJoystickButton(SDL_Event &event);
    proto::RobotCommand getCommand();

    void updateVelocity();
    void updateOrientation();
    void doKick();
    void doChip();
    void tuneDribbler();
    void toggleDribbler();
    void changeRobotID();
    JoystickState getJoystickState();
};

} // namespace input
} // namespace rtt

#endif //RTT_JOYSTICKHANDLER_H