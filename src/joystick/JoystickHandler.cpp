//
// Created by luukkn on 23-10-19.
//
#include <iostream>
#include "JoystickHandler.h"
#include <SDL.h>
using namespace std;
JoystickHandler::JoystickHandler() {
cout << "New Joyhandler made" << endl;
};

void JoystickHandler::tick(){
    cout << "tick" << endl;
}

void JoystickHandler::handleEvent(const SDL_Event& event) {
    switch (event.type) {
        case SDL_JOYAXISMOTION:
            if ((event.jaxis.value < -3200) || (event.jaxis.value > 3200)) {
                switch (event.jaxis.axis) {
                    case (0):
                    {
                        cout << "left stick horizontal: ";
                        cout << event.jaxis.value << endl;
                        Drive_Angle_Vector.x = event.jaxis.value;
                        cout << Drive_Angle_Vector.normalize() << endl;
                        command.mutable_vel()->set_x(Drive_Angle_Vector.normalize().x);
                        break;
                    }
                    case (1):
                    {
                        cout << "left stick vertical: ";
                        cout << event.jaxis.value << endl;
                        Drive_Angle_Vector.y = event.jaxis.value;
                        cout << Drive_Angle_Vector.normalize() << endl;
                        command.mutable_vel()->set_x(Drive_Angle_Vector.normalize().y);
                        break;
                    }
                    case (3):
                    {
                        cout << "right stick horizontal: ";
                        cout << event.jaxis.value << endl;
                        Drive_Angle_Vector.x = event.jaxis.value;
                        cout << Drive_Angle_Vector.normalize() << endl;
                        Drive_Angle_Vector = Drive_Angle_Vector.rotate(Drive_Angle_Vector.angle());
                        command.mutable_vel()->set_x(Drive_Angle_Vector.normalize().x);               //Do I have to? If I dont it only updates the movement after every update of the left joystick
                        command.mutable_vel()->set_y(Drive_Angle_Vector.normalize().y);
                        break;
                    }
                    case (4):
                    {
                        cout << "right stick vertical: ";
                        cout << event.jaxis.value << endl;
                        Drive_Angle_Vector.y = event.jaxis.value;
                        cout << Drive_Angle_Vector.normalize() << endl;
                        Drive_Angle_Vector = Drive_Angle_Vector.rotate(Drive_Angle_Vector.angle());
                        command.mutable_vel()->set_x(Drive_Angle_Vector.normalize().x);
                        command.mutable_vel()->set_y(Drive_Angle_Vector.normalize().y);
                        break;
                    }
                    default:
                        cout << "No joystick motion" << endl;
//              case(2)         Left trigger
//              case(5)         Right trigger
                }
            }
        case SDL_KEYDOWN:
            /* handle keyboard stuff here */
            break;

        case SDL_QUIT:
            /* Set whatever flags are necessary to */
            /* end the main game loop here */
            break;

        case SDL_JOYBUTTONDOWN:  /* Handle Joystick Button Presses */

            switch (event.jbutton.button) {
                case (0): {
                    cout << "A button: ";
                    cout << event.jaxis.value << endl;
                    command.set_kicker(true);
                    command.set_chip_kick_vel(0.01);
                    /* Soft shot code goes here */
                    break;
                }
                case (1): {
                    cout << "B button: ";
                    cout << event.jaxis.value << endl;
                    command.clear_chipper();
                    command.set_kicker(true);
                    command.set_chip_kick_vel(0.1);
                    /* Hard shot command */
                    break;
                }
                case (2): {
                    cout << "Z button: ";
                    cout << event.jaxis.value << endl;
                    command.clear_chipper();
                    command.set_kicker(false);
                    command.set_chip_kick_vel(0.1);
                    /* Chip command */
                    break;
                }
                case (4): {
                    cout << "L1 button: ";
                    cout << event.jaxis.value << endl;
                    command.clear_kicker();
                    command.set_dribbler(true);
                    /* Turn dribbler on */
                    break;
                }
                case (8): {
                    cout << "Xbox button: ";
                    cout << event.jaxis.value << endl;
                    /* Turn controller off? */
                    break;
                }
                default:
                    cout << "No correct button input" << endl;
//          case(3)      Y-Button
//          case(5)      R1-Button
//          case(6)      Back-Button
//          case(7)      Start-Button
//          case(9)      Left JoyHat
//          case(10)     Right JoyHat
            }


    }
}