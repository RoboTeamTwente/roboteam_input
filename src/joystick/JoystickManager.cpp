#include <iostream>
#include <chrono>
#include <thread>
#include <SDL.h>
#include <SDL_joystick.h>
#include "roboteam_proto/Publisher.h"
#include "roboteam_proto/RobotCommand.pb.h"
#include "JoystickManager.h"

using namespace std::chrono;

namespace rtt {
namespace input {

JoystickManager::JoystickManager() {
    std::cout << "[JoystickManager] New JoystickManager" << std::endl;
    init();
    loop();
}

/**
 * Inits SDL and JoystickHandler-map
 */
void JoystickManager::init() {
    std::cout << "[JoystickManager][init] Initializing.." << std::endl;
    if(SDL_InitSubSystem(SDL_INIT_JOYSTICK))
        std::cout << "SDL_INIT_JOYSTICK failed : " << SDL_GetError() << std::endl;
    SDL_JoystickEventState(SDL_ENABLE);
}

/**
 * Handles events, handles and ticks JoystickHandlers
 */
void JoystickManager::loop() {
    int iTicks = 0;
    int iEvents = 0;
    SDL_Event event;

    steady_clock::time_point tTickNext = steady_clock::now() + milliseconds(TICK_INTERVAL);

    // Creates a publisher in order to send robot commands
    std::unique_ptr<roboteam_proto::Publisher> pub = std::make_unique<roboteam_proto::Publisher>("tcp://127.0.0.1:5556");

    while(iTicks < 9999999999999) {
        /* Tick all JoystickHandlers */
        iTicks++;
        int refTick;                                             //  This is used simply for the stop condition
        for (const auto &joystickHandler : joystickHandlers) {
            joystickHandler.second->tick();
            pub->send("robotcommands", joystickHandler.second->getCommand().SerializeAsString());
        }

        /* Wait for an event or until it is time for the next tick */
        int msToNextTick = (int) duration_cast<milliseconds>(tTickNext - steady_clock::now()).count();
        while (SDL_WaitEventTimeout(&event, msToNextTick)) {
            iEvents++;

            /* Catch device events if needed, else forward event to corresponding JoystickHandler */
            switch (event.type) {
                case SDL_JOYDEVICEADDED:
                    handleJoystickAdded(event);
                    break;
                case SDL_JOYDEVICEREMOVED:
                    handleJoystickRemoved(event);
                    break;
                default:
                    joystickHandlers.at(event.jdevice.which)->handleEvent(event);
                    break;
            }

            /* Check if it is time for another tick */
            msToNextTick = (int) duration_cast<milliseconds>(tTickNext - steady_clock::now()).count();
            if (msToNextTick <= 0)
                break;
        }

        tTickNext += milliseconds(TICK_INTERVAL);
    }
}

/**
 * Takes an SDL_Event and adds a new JoystickHandler to the map of JoystickHandlers.
 */

void JoystickManager::handleJoystickAdded(const SDL_Event& evt) {
    std::cout << "[JoystickManager][handleJoystickAdded] Adding joystick " << evt.jdevice.which << std::endl;

    SDL_Joystick* joystick = SDL_JoystickOpen(evt.jdevice.which);
    if(!joystick){
        std::cout << "[JoystickManager][handleJoystickAdded] Error! Could not open joystick!" << std::endl;
        return;
    }

    int instanceId = SDL_JoystickInstanceID(joystick);
    JoystickHandler* handler = new JoystickHandler();
    joystickHandlers.insert({instanceId, handler});
    std::cout << "[JoystickManager][handleJoystickAdded] Added joystick with InstanceID " << instanceId << std::endl;
}


/**
 * Takes an SDL_Event and deletes and removes the correct JoystickHandler from the map of JoystickHandlers.
 */
void JoystickManager::handleJoystickRemoved(const SDL_Event& evt){
    std::cout << "[JoystickManager][handleJoystickRemoved] Removing joystick " << evt.jdevice.which << std::endl;
    delete joystickHandlers.at(evt.jdevice.which);
    joystickHandlers.erase(evt.jdevice.which);
    std::cout << "[JoystickManager][handleJoystickAdded] Removed joystick with InstanceID " << evt.jdevice.which << std::endl;
}

}
}

int main(int argc, char **argv) {
    rtt::input::JoystickManager manager;
    return 0;
}
