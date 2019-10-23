//
// Created by emiel on 21-10-19.
//

#ifndef ROBOTEAM_INPUT_JOYSTICKMANAGER_H
#define ROBOTEAM_INPUT_JOYSTICKMANAGER_H

#include <memory>
#include <map>
#include "JoystickHandler.h"
#include <SDL.h>
#include <SDL_joystick.h>
#include <unistd.h>


class JoystickManager {
public:
    JoystickManager();

private:
    const int TICK_INTERVAL = 500;
    std::map<int, JoystickHandler*> joystickHandlers;

    void init();
    void loop();
    void handleEvent();
    void handleJoystickAdded(const SDL_Event& evt);
    void handleJoystickRemoved(const SDL_Event& evt);
    void tick();

};


#endif //ROBOTEAM_INPUT_JOYSTICKMANAGER_H
