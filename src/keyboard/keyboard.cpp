/*
This class manages the keyboard controls, and draw the parameters to the screen using InputInterface.
*/

#include <SDL.h>
#include <iostream>
#include <ros/ros.h>
#include <boost/optional.hpp>
#include <std_msgs/Bool.h>
#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"
#include "input_interface.h"
#include "keyboard_manager.h"

namespace b = boost;


// This vector described 0 to 15 in hex numbers
std::vector<SDL_Keycode> const robotIDKeys = {
    SDLK_0, SDLK_1,SDLK_2,SDLK_3,SDLK_4,SDLK_5,SDLK_6,SDLK_7,SDLK_8,SDLK_9,
    SDLK_a,SDLK_b,SDLK_c,SDLK_d,SDLK_e,SDLK_f
};

void displayInfoMessage() {
  std::cout << R"--(
RTT Keyboard Joystick

Controls:10
  Arrow keys to steer/drive
  z/x to strafe left/right
  v to kick
  n to chip
  space to dribble
  0-9, a-f to select an ID
  Numpad 7/9 to decrease/increase kicker velocity
  Numpad 4/6 or F4/F6 to decrease/increase x velocity
  Numpad 1/3 or F1/F3 to decrease/increase angular velocity
  PageDown and PageUp to rotate the kicker
)--";
}

void printHelpMessage() {
  std::cout << R"--([Keyboard controller]
Args:
--help, -h      Display this help
-id             Set the starting ID (default: 5)
)--";
}

int initializeSDL() {
  if (SDL_Init( SDL_INIT_EVERYTHING ) != 0) {
      // Something failed, print error and exit.
      std::cout << " Failed to initialize SDL : " << SDL_GetError() << std::endl;
      return -1;
  }
}

b::optional<std::string> getCmdOption(const std::vector<std::string>& args, const std::string & option) {
  auto it = std::find(args.begin(), args.end(), option);
  if (it != args.end() && (it + 1) != args.end()) {
    return *(it + 1);
  }
  return b::none;
}

bool cmdOptionExists(const std::vector<std::string>& args, const std::string& option) {
  return std::find(args.begin(), args.end(), option) != args.end();
}

b::optional<int> stringToID(std::string idStr) {
    if (idStr.length() < 1) return b::none;
    if (idStr[0] >= 'A' && idStr[0] <= 'F') {
        return idStr[0] - 'A' + 10;
    } else if (idStr[0] >= 'a' && idStr[0] <= 'f') {
        return idStr[0] - 'a' + 10;
    } else if (idStr[0] >= '0' && idStr[0] <= '9') {
        return idStr[0] - '0';
    } else {
        return b::none;
    }
}


bool isCtrlPressed(SDL_Event const e) {
    if (e.type == SDL_KEYDOWN) {
        if ((e.key.keysym.mod & KMOD_CTRL) > 0) {
            return true;
        }
    }
    return false;
}


int main(int argc, char* argv[]) {
    std::vector<std::string> args(argv + 1, argv + argc);

    if (cmdOptionExists(args, "--help") || cmdOptionExists(args, "-h") || cmdOptionExists (args, "man")) {
        printHelpMessage();
        return 0;
    }

    displayInfoMessage();
    initializeSDL();

    KeyboardManager keyManager = KeyboardManager();
    InputInterface interface = InputInterface();

    if (auto idOpt = getCmdOption(args, "-id")) {
        try {
            keyManager.currentId = std::stoi(*idOpt);
        } catch (...) {
            std::cerr << "Could not convert id \"" << *idOpt << "\" to integer. Keeping 5.";
        }
    }

    // Ros stuff
    ros::init(argc, argv, "keyboard_controller", ros::InitOption::AnonymousName);
    ros::NodeHandle n;
    auto robotCommandPub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 100);

    // TODO: Maybe base this on role_iterations_per_second?
    ros::Rate fpsRate(60);



    // Event loop
    bool quit = false;
    SDL_Event event;
    while(!quit && ros::ok()) {
        while(SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) {
                quit = true;
            } else if (event.type == SDL_KEYDOWN) {
                auto key = event.key.keysym.sym;

                // quit when ctrc-c escape or q is pressed.
                bool ctrlCIsPressed = isCtrlPressed(event) && key == SDLK_c;
                bool escapeIsPressed = key == SDLK_ESCAPE;
                bool qIsPressed = key == SDLK_q;
                quit = (ctrlCIsPressed || escapeIsPressed || qIsPressed);

                // Handle ID switching
                if (std::find(robotIDKeys.begin(), robotIDKeys.end(), key) != robotIDKeys.end()) {
                    if (auto currentIDOpt = stringToID(SDL_GetKeyName(key))) {
                        keyManager.currentId = *currentIDOpt;
                    } else {
                        std::cout << "Bad ID key pressed, retaining original ID.";
                    }
                }
            }

            keyManager.handleSDLEvents(event);
        }
        // Publish a robot instruction
        robotCommandPub.publish(keyManager.GetRobotCommand());

        // Wait s.t. we don't burn cycles
        ros::spinOnce();
        fpsRate.sleep();

        // Draw the gui and refresh the screen
        interface.drawGui(keyManager.currentKick, keyManager.currentVel, keyManager.currentW, keyManager.currentGenevaState, keyManager.currentId);
    }
    return 0;
}
