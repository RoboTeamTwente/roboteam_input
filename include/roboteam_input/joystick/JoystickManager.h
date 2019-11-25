//
// Created by emiel on 21-10-19.
//

#ifndef ROBOTEAM_INPUT_JOYSTICKMANAGER_H
#define ROBOTEAM_INPUT_JOYSTICKMANAGER_H

#include <memory>
#include <map>
#include <unistd.h>
#include <mutex>
#include <SDL2/SDL.h>
#include <SDL2/SDL_joystick.h>
#include "JoystickHandler.h"
#include "../../../../roboteam_ai/include/roboteam_ai/io/IOManager.h"
#include "roboteam_proto/Publisher.h"


namespace rtt {
namespace input {

class JoystickManager {
public:
    explicit JoystickManager(ai::io::IOManager* ioManager);  // Used to start up JoystickManager via roboteam_ai interface
    JoystickManager();  // Used to start up JoystickManager standalone
    bool run();
    void activate();
    void deactivate();
    void stop();

private:
    const int TICK_INTERVAL = 20;
    std::map<int, JoystickHandler*> joystickHandlers;

    // TODO Check if these can be non-static
    std::mutex runningLock;
    std::mutex activeLock;
    // Indicates whether the loop should stop
    bool running = true;
    // Indicates whether packets should be handled and joystickHandlers ticked
    bool active = true;

    bool useIoManager = false; // Indicates whether to use the ioManager or publisher
    ai::io::IOManager * ioManager = nullptr; // Used when given by eg roboteam_ai
    std::unique_ptr<proto::Publisher<proto::RobotCommand>> pub; // Used when standalone

    bool init();
    void loop();
    bool isRunning();
    bool isActive();
    void handleJoystickAdded(const SDL_Event& event);
    void handleJoystickRemoved(const SDL_Event& event);
    void tickJoystickHandlers();
    void handleEvent(SDL_Event& event);
};

}
}

#endif //ROBOTEAM_INPUT_JOYSTICKMANAGER_H
