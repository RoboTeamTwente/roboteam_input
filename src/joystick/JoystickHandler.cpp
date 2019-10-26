//
// Created by luukkn on 23-10-19.
//
#include <iostream>
#include "JoystickHandler.h"
#include <SDL.h>


using namespace std;
JoystickHandler::JoystickHandler() {
    cout << "New Joyhandler made" << endl;
    command.set_chip_kick_forced(true);         //// Not sure of correct position for this. Forced has to be true throughout since we do not use the ball sensor.
};

void JoystickHandler::tick(){
//    cout << "tick" << endl;
}


/*Receives event data and checks the event type. Then calls function to process it either as joystick motion or a button press*/
void JoystickHandler::handleEvent(const SDL_Event& event) {
    switch (event.type) {
        case SDL_JOYAXISMOTION: /* Handle Axis motion*/
            handleJoystickMotion(event);
        case SDL_JOYBUTTONDOWN:  /* Handle Joystick Button Presses */
            handleJoystickButton(event);
    }
}

/*Processes the joystick motion*/
void JoystickHandler::handleJoystickMotion(const SDL_Event &event)
{
    if ((event.jaxis.value < -3200) || (event.jaxis.value > 3200))
    {
        switch (event.jaxis.axis)
        {
            case (0): {handleXMovement(event);      break;}
            case (1): {handleYMovement(event);      break;}
            case (3): {handleXRotation(event);      break;}     //// TODO Not satisfied with name but will change.
            case (4): {handleYRotation(event);      break;}     //// TODO Not satisfied with name but will change.

            default:
                cout << "No correct joystick input" << endl;
//              case(2)         Left trigger
//              case(5)         Right trigger
        }
    }
    else
    {
        if(event.jaxis.axis == 0) {
            cout << "x_velocity 0" << endl;
            command.mutable_vel()->set_x(0);    //// If the axis value is between -3200 and 3200 set velocity back to 0.
        }
        if(event.jaxis.axis == 1) {
            cout << "y_velocity 0" << endl;
            command.mutable_vel()->set_y(0);    //// If the axis value is between -3200 and 3200 set velocity back to 0.
        }
        // TODO something similar with the rotation axis
    }

}

/* Processes the button press as either a shot, soft shot, chip or dribbler switch*/
void JoystickHandler::handleJoystickButton(const SDL_Event &event) {
    switch (event.jbutton.button) {
        case (0):        {doSlowShot(event);  break;}       //TODO rethink names
        case (1):        {doHardShot(event);  break;}
        case (2):        {doChip(event);      break;}
        case (4):        {useDribbler(event); break;}
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


//// Assigns a horizontal speed to the command depending on deviation from centre.
void JoystickHandler::handleXMovement(const SDL_Event &event)
{
    cout << "left stick horizontal: ";
    cout << event.jaxis.value << endl;
    Drive_Angle_Vector.x = event.jaxis.value;
    cout << Drive_Angle_Vector.normalize() << endl;
    command.mutable_vel()->set_x(-Drive_Angle_Vector.normalize().x); //// Apparently left and right are reversed.
}

//// Assigns a vertical speed to the command depending on deviation from centre.
void JoystickHandler::handleYMovement(const SDL_Event &event) {
    cout << "left stick vertical: ";
    cout << event.jaxis.value << endl;
    Drive_Angle_Vector.y = event.jaxis.value;
    cout << Drive_Angle_Vector.normalize() << endl;
    command.mutable_vel()->set_y(Drive_Angle_Vector.normalize().y);
}

//// Adapts the unit vector on which to project the movement vector in the direction of the right joystick.
//// Think there is something wrong with reference frames.
void JoystickHandler::handleXRotation(const SDL_Event &event)
{
    cout << "right stick horizontal: ";
    cout << event.jaxis.value << endl;
    Drive_Angle_Vector.x = event.jaxis.value;
    cout << Drive_Angle_Vector.normalize() << endl;
    Drive_Angle_Vector = Drive_Angle_Vector.rotate(Drive_Angle_Vector.angle());
}

//// Adapts the unit vector on which to project the movement vector in the direction of the right joystick.
//// Think there is something wrong with reference frames.
void JoystickHandler::handleYRotation(const SDL_Event &event)
{
    cout << "right stick vertical: ";
    cout << event.jaxis.value << endl;
    Drive_Angle_Vector.y = event.jaxis.value;
    cout << Drive_Angle_Vector.normalize() << endl;
    Drive_Angle_Vector = Drive_Angle_Vector.rotate(Drive_Angle_Vector.angle());
}

//// Do a soft shot to possibly play around the other robot.
void JoystickHandler::doSlowShot(const SDL_Event &event)
{
    cout << "A button: ";
    cout << event.jbutton.state << endl;
    command.clear_chipper();
    command.set_kicker(true);
    command.set_chip_kick_vel(0.1);         //TODO find out values
    /* Soft shot code goes here */
};

//// Do a hard shot to for example shoot on goal.
void JoystickHandler::doHardShot(const SDL_Event &event)
{
    cout << "B button: ";
    cout << event.jbutton.state << endl;
    command.clear_chipper();
    command.set_kicker(true);
    command.set_chip_kick_vel(1);           //TODO find out values
    /* Hard shot command */
};

void JoystickHandler::doChip(const SDL_Event &event)
{
    cout << "Z button: ";
    cout << event.jbutton.state << endl;
    command.clear_kicker();
    command.set_chipper(true);
    command.set_chip_kick_vel(0.1);
    /* Chip command */
};

//// Turn dribbler on if off and vice versa.
void JoystickHandler::useDribbler(const SDL_Event &event)
{
    cout << "L1 button: ";
    command.set_dribbler(command.dribbler() == 0);
    cout << command.dribbler() << endl;
    /* Turn dribbler on */
};

roboteam_proto::RobotCommand JoystickHandler::getCommand(){
    return command;
}
