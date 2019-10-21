//
// Created by emiel on 21-10-19.
//

#ifndef ROBOTEAM_INPUT_JOYSTICKMANAGER_H
#define ROBOTEAM_INPUT_JOYSTICKMANAGER_H

#include <memory>
#include <map>

#include <SDL.h>
#include <SDL_joystick.h>
#include <unistd.h>

/* Simple test class to emulate a JoystickHandler */
class Test {
public:
    int id, iEvents;
    Test(int _id){
        id = _id;
        iEvents = 0;
        std::cout << "Test created! id=" << id << std::endl;
    }
    ~Test(){ std::cout << "Test destroyed! id=" << id << std::endl;}
    void tick(){
        std::cout << "Test tick " << id << " " << iEvents << std::endl;
        usleep(200*1000);
    }
    void handleEvent(const SDL_Event& evt){
        iEvents ++;
    }
};

class JoystickManager {
public:
    JoystickManager();

private:
    const int TICK_INTERVAL = 500;
    std::map<int, Test*> joystickHandlers;

    void init();
    void loop();
    void handleEvent();
    void handleJoystickAdded(const SDL_Event& evt);
    void handleJoystickRemoved(const SDL_Event& evt);
    void tick();

};


#endif //ROBOTEAM_INPUT_JOYSTICKMANAGER_H
