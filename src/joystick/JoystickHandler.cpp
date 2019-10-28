//
// Created by luukkn on 23-10-19.
//
#include <iostream>
#include "JoystickHandler.h"
#include <SDL.h>

namespace rtt {
namespace input {

JoystickHandler::JoystickHandler() {
    std::cout << "[JoystickHandler] New JoystickHandler" << std::endl;
};

void JoystickHandler::tick(){
    command.mutable_vel()->set_y(-joystickState.stickLeft.x / 32768.0);
    command.mutable_vel()->set_x(-joystickState.stickLeft.y / 32768.0);

    float dAngle = -joystickState.stickRight.x / 32768.0;
    robotAngle += dAngle * 0.05;
    command.set_w(command.w() + dAngle);
    command.set_w(dAngle*5);

    if (joystickState.A){
        joystickState.bumperRight = false;
        command.set_chip_kick_vel(3.0);
        command.set_kicker(true);
        command.set_chip_kick_forced(true);
    }else{
        command.set_chip_kick_vel(0.0);
        command.set_kicker(false);
        command.set_chip_kick_forced(false);
    }

    if(joystickState.back){
        if(joystickState.bumperLeft){
            if (robotId >= -1) {
                joystickState.dpadLeft = false;
                robotId--;
                std::cout << "Current robot ID" << std::endl;
            }
            else
                std::cout << "No robots with lower ID available" << std::endl;
        }
        if(joystickState.bumperRight){
            if (robotId <= 6) {
                joystickState.dpadRight = false;
                robotId++;
                std::cout << "Current robot ID" << std::endl;
            }
            else
                std::cout << "No robots with higher ID available" << std::endl;
        }
        command.set_id(robotId);
    }
}


/* Receives event data and checks the event type.
 * Then calls function to process it either as joystick motion or a button press
 * */
void JoystickHandler::handleEvent(SDL_Event& event) {
    switch (event.type) {
        case SDL_JOYAXISMOTION: /* Handle Axis motion*/
            handleJoystickMotion(event);
            break;
        case SDL_JOYBUTTONUP:
            handleJoystickButton(event);
            break;
        case SDL_JOYBUTTONDOWN:
            handleJoystickButton(event);
            break;
    }
}

/* Processes the joystick motion */
void JoystickHandler::handleJoystickMotion(SDL_Event &event){

    /* Check if values are outside of the deadzone */
    if (-16000 < event.jaxis.value && event.jaxis.value < 16000) {
        event.jaxis.value = 0;
    }

    switch(event.jaxis.axis){
        case 0 : joystickState.stickLeft.x = event.jaxis.value; break;
        case 1 : joystickState.stickLeft.y = event.jaxis.value; break;
        case 2 : joystickState.triggerLeft = event.jaxis.value; break;
        case 3 : joystickState.stickRight.x = event.jaxis.value; break;
        case 4 : joystickState.stickRight.y = event.jaxis.value; break;
        case 5 : joystickState.triggerRight = event.jaxis.value; break;
    }

    std::cout << joystickState.stickLeft << " ";
    std::cout << joystickState.stickRight << "  ";
    std::cout << joystickState.triggerLeft << " " << joystickState.triggerRight;
    std::cout << std::endl;

}

/* Processes the button press as either a shot, soft shot, chip or dribbler switch*/
void JoystickHandler::handleJoystickButton(SDL_Event &event) {
//    std::cout << "[JoystickHandler][handleJoystickButton] " << (int) event.jbutton.button << " " << (int)event.jbutton.state << std::endl;
    bool pressed = (int) event.jbutton.state == 1;
    switch(event.jbutton.button){
        case  0 : joystickState.A = pressed; break;
        case  1 : joystickState.B = pressed; break;
        case  2 : joystickState.X = pressed; break;
        case  3 : joystickState.Y = pressed; break;
        case  4 : joystickState.bumperLeft = pressed; break;
        case  5 : joystickState.bumperRight = pressed; break;
        case  6 : joystickState.back = pressed; break;
        case  7 : joystickState.start = pressed; break;
        case  8 : joystickState.guide = pressed; break;
        case  9 : joystickState.stickLeftBtn = pressed; break;
        case 10 : joystickState.stickRightBtn = pressed; break;
        case 11 : joystickState.dpadLeft = pressed; break;
        case 12 : joystickState.dpadRight = pressed; break;
        case 13 : joystickState.dpadUp = pressed; break;
        case 14 : joystickState.dpadDown = pressed; break;
    }
}

roboteam_proto::RobotCommand JoystickHandler::getCommand(){
    return command;
}

} // namespace input
} // namespace rtt