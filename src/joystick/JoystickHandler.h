//
// Created by luukkn on 23-10-19.
//

#ifndef RTT_JOYSTICKHANDLER_H
#define RTT_JOYSTICKHANDLER_H
#include <SDL.h>
#include <stdio.h>
#include <SDL_joystick.h>
#include <iostream>
#include "roboteam_proto/Publisher.h"
#include "roboteam_proto/RobotCommand.pb.h"
#include <thread>
#include <roboteam_utils/Vector2.h>
using namespace std;

class JoystickHandler {
public:
    roboteam_proto::RobotCommand command;
    rtt::Vector2 Drive_Angle_Vector;
    void tick();
    void handleEvent(const SDL_Event& event);
    JoystickHandler();
};



#endif //RTT_JOYSTICKHANDLER_H
