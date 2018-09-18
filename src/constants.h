//
// Created by mrlukasbos on 18-9-18.
//

#include <iostream>
#include <SDL.h>

#ifndef ROBOTEAM_INPUT_CONSTANTS_H
#define ROBOTEAM_INPUT_CONSTANTS_H

namespace constants {
    const int MAX_GENEVA_STATE = 5;
    const int MIN_GENEVA_STATE = 1;
    const int MAX_ROBOT_VELOCITY = 6; // m/s
    const double MAX_ROTATION_SPEED = 5;
    const double MIN_ROTATION_SPEED = 1;
    double const MAX_W = 2048.0 / 360.0 * (2 * M_PI);

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
}

#endif //ROBOTEAM_INPUT_CONSTANTS_H
