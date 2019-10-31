//
// Created by luukkn on 23-10-19.
//

#ifndef RTT_JOYSTICKHANDLER_H
#define RTT_JOYSTICKHANDLER_H
#include <SDL.h>
#include <stdio.h>
#include <SDL_joystick.h>
#include <iostream>

#include "JoystickState.h"

#include "../../../roboteam_ai/include/roboteam_ai/control/RobotCommand.h"

namespace rtt {
namespace input {

class JoystickHandler {
private:
    roboteam_proto::RobotCommand command;
    JoystickState joystickState;
    float robotAngle = 0.0;
    int robotId = -1;

public:
    JoystickHandler();
    void tick();
    void handleEvent(SDL_Event& event);
    void handleJoystickMotion(SDL_Event &event);
    void handleJoystickButton(SDL_Event &event);
    roboteam_proto::RobotCommand getCommand();

    void updateVelocity();
    void updateOrientation();
    void doKick();
    void doChip();
    void toggleDribbler();
    void changeRobotID();
};

} // namespace input
} // namespace rtt

#endif //RTT_JOYSTICKHANDLER_H