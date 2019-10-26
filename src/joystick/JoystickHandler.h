//
// Created by luukkn on 23-10-19.
//

#ifndef RTT_JOYSTICKHANDLER_H
#define RTT_JOYSTICKHANDLER_H
#include <SDL.h>
#include <stdio.h>
#include <SDL_joystick.h>
#include <iostream>

#include "../../../roboteam_ai/include/roboteam_ai/control/RobotCommand.h"

using namespace std;

class JoystickHandler {
private:
    roboteam_proto::RobotCommand command;
    rtt::Vector2 Drive_Angle_Vector;
public:
    JoystickHandler();
    void tick();
    void handleEvent(const SDL_Event& event);
    void handleJoystickMotion(const SDL_Event &event);
    void handleJoystickButton(const SDL_Event &event);
    roboteam_proto::RobotCommand getCommand();

    void handleXMovement(const SDL_Event &event);
    void handleYMovement(const SDL_Event &event);
    void handleXRotation(const SDL_Event &event);
    void handleYRotation(const SDL_Event &event);
    void doSlowShot(const SDL_Event &event);
    void doHardShot(const SDL_Event &event);
    void doChip(const SDL_Event &event);
    void useDribbler(const SDL_Event &event);

};



#endif //RTT_JOYSTICKHANDLER_H
