/*
This class creates a manager for keyboard control of the robots.
created by mrlukasbos on 04-09-2018

Multiple parameters are distilled from the keyboard controls and converted into a robotCommand.
*/

#include "roboteam_msgs/RobotCommand.h"
#include <boost/optional.hpp>
#include "roboteam_utils/Vector2.h"
#include <iostream>
#include <SDL.h>
#include "gtest/gtest.h"

namespace b = boost;

class KeyboardManager {
public:
    KeyboardManager();
    void handleSDLEvents(SDL_Event const & e);
    roboteam_msgs::RobotCommand GetRobotCommand();
    int getCurrentGenevaState() const;
    double getCurrentVel() const;
    int getCurrentKick() const;
    int getCurrentId() const;
    double getRotationSpeed() const;
    void setCurrentId(int currentId);

private:
    // Limits
    double const MAX_VEL = 6;
    double const MAX_W = 2048.0 / 360.0 * (2 * M_PI);
    int const MIN_GENEVA_STATE = 1;
    int const MAX_GENEVA_STATE = 5;

    // robot Values
    double rotationSpeed = 1;
    int currentGenevaState = 3;
    double currentVel = 1;
    int currentKick = 1;
    int currentId = 0;
    int xDirection = 0;
    int yDirection = 0;
    int rotationDirection = 0;
    double w = 0;
    double rotation = 0;
    bool doKick = false;
    bool enKick = false;
    bool doChip = false;
    bool doDribble = false;
    int orientationOffset = 0;
    double const STEP_VEL = 0.1;
    double const STEP_W = 0.5;
    const double MAX_ROTATION_SPEED = 5;
    const double MIN_ROTATION_SPEED = 1;
    const double STEP_ROTATION_SPEED = 0.2;
    std::vector<SDL_Keycode> const arrowKeys = {
            SDLK_UP,
            SDLK_DOWN,
            SDLK_LEFT,
            SDLK_RIGHT,
            SDLK_z,
            SDLK_x
    };

    // define more descriptive names for the keys
    static const SDL_Keycode KEY_INCREASE_VEL    = SDLK_KP_6;
    static const SDL_Keycode KEY_DECREASE_VEL    = SDLK_KP_4;
    static const SDL_Keycode KEY_INCREASE_ROTATION_SPEED  = SDLK_KP_3;
    static const SDL_Keycode KEY_DECREASE_ROTATION_SPEED  = SDLK_KP_1;
    static const SDL_Keycode KEY_INCREASE_KICK   = SDLK_KP_9;
    static const SDL_Keycode KEY_DECREASE_KICK   = SDLK_KP_7;
    static const SDL_Keycode KEY_STRAFE_LEFT     = SDLK_z;
    static const SDL_Keycode KEY_STRAFE_RIGHT    = SDLK_x;
    static const SDL_Keycode KEY_DRIBBLE         = SDLK_SPACE;
    static const SDL_Keycode KEY_KICK            = SDLK_v;
    static const SDL_Keycode KEY_CHIP            = SDLK_n;
    static const SDL_Keycode KEY_GENEVA_LEFT     = SDLK_PAGEUP;
    static const SDL_Keycode KEY_GENEVA_RIGHT    = SDLK_PAGEDOWN;
};
