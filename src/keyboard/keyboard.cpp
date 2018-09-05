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

namespace b = boost;

InputInterface interface = InputInterface();
KeyboardManager keyManager = KeyboardManager();

void displayInfoMessage();
void printHelpMessage();
void initializeSDL();

int main(int argc, char* argv[]) {
    std::vector<std::string> args(argv + 1, argv + argc);

    if (cmdOptionExists(args, "--help") || cmdOptionExists(args, "-h")) {
        printHelpMessage();
        return 0;
    }

    displayInfoMessage();
    initializeSDL();

    int currentID = 0;
    if (auto idOpt = getCmdOption(args, "-id")) {
        try {
            currentID = std::stoi(*idOpt);
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
    SDL_Event e;
    while(!quit && ros::ok()) {
        while(SDL_PollEvent(&e) != 0) {

            if (e.type == SDL_QUIT) {
                quit = true;
            } else if (e.type == SDL_KEYDOWN) {
                auto key = e.key.keysym.sym;

                // quit when ctrc-c escape or q is pressed.
                bool ctrlCIsPressed = isCtrlPressed(e) && key == SDLK_c;
                bool escapeIsPressed = key == SDLK_ESCAPE;
                bool qIsPressed = key == SDLK_q;
                quit = (ctrlCIsPressed || escapeIsPressed || qIsPressed);

                // Handle ID switching
                if (std::find(robotIDKeys.begin(), robotIDKeys.end(), key) != robotIDKeys.end()) {
                    if (!(key == SDLK_c && isCtrlPressed(e))) {
                        if (auto currentIDOpt = stringToID(SDL_GetKeyName(key))) {
                            currentID = *currentIDOpt;
                        } else {
                            std::cout << "Bad ID key pressed, retaining original ID.";
                        }
                    }
                }
            }
          handleSDLEvent();
        }

        // Publish a robot instruction
        robotCommandPub.publish(makeRobotCommand(currentID, speed, direction));

        // Wait s.t. we don't burn cycles
        ros::spinOnce();
        fpsRate.sleep();

        // Draw the gui and refresh the screen
        interface.drawGui(renderer, speed.currentKick, speed.currentVel, speed.currentW, direction.currentGenevaState, currentID);
    }
    return 0;
}

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

void initializeSDL() {
  int posX = 100;
  int posY = 200;
  int sizeX = 300;
  int sizeY = 400;
  SDL_Window* window;
  SDL_Renderer* renderer;

  if (SDL_Init( SDL_INIT_EVERYTHING ) != 0) {
      // Something failed, print error and exit.
      std::cout << " Failed to initialize SDL : " << SDL_GetError() << std::endl;
      return -1;
  }

  window = SDL_CreateWindow("RTT Keyboard Joystick", posX, posY, sizeX, sizeY, 0);

  if (window == nullptr) {
      std::cout << "Failed to create window : " << SDL_GetError();
      return -1;
  }

  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  if (renderer == nullptr) {
      std::cout << "Failed to create renderer : " << SDL_GetError();
      return -1;
  }
}

namespace {
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
}
