#include <SDL.h>
#include <iostream>
#include <ros/ros.h>
#include <boost/optional.hpp>

#include "roboteam_msgs/RobotCommand.h"

namespace b = boost;

namespace {

std::vector<SDL_Keycode> const robotIDNumKeys = {
    SDLK_0,  
    SDLK_1,  
    SDLK_2,  
    SDLK_3,  
    SDLK_4,  
    SDLK_5,  
    SDLK_6,  
    SDLK_7,  
    SDLK_8,  
    SDLK_9
};

std::vector<SDL_Keycode> const robotIDLetterKeys = {
    SDLK_a,  
    SDLK_b,  
    SDLK_c,  
    SDLK_d,  
    SDLK_e,  
    SDLK_f
};

std::vector<SDL_Keycode> const arrowKeys = {
    SDLK_UP,
    SDLK_DOWN,
    SDLK_LEFT,
    SDLK_RIGHT
};

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

}

int main(int argc, char* argv[]) {
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

    window = SDL_CreateWindow("RTT Keyboard controller", posX, posY, sizeX, sizeY, 0);

    if (window == nullptr) {
        std::cout << "Failed to create window : " << SDL_GetError();
        return -1;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    if (renderer == nullptr) {
        std::cout << "Failed to create renderer : " << SDL_GetError();
        return -1;
    }

    // Render red
    SDL_RenderSetLogicalSize( renderer, sizeX, sizeY );
    SDL_SetRenderDrawColor( renderer, 255, 0, 0, 255 );
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);

    // Control variables
    auto robotIDAllKeys = robotIDNumKeys;
    robotIDAllKeys.insert(robotIDAllKeys.end(), robotIDLetterKeys.begin(), robotIDLetterKeys.end());

    int currentID = 5;
    double currentVel = 2;
    double currentW = 2;

    // Ros stuff
    ros::init(argc, argv, "keyboard_controller");
    ros::NodeHandle n;
    auto robotCommandPub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 100);
    ros::Rate fps30(30);

    // Event loop
    bool quit = false;
    SDL_Event e;

    while(!quit && ros::ok()) {
        while(SDL_PollEvent(&e) != 0) {
            if ( e.type == SDL_QUIT ) {
                quit = true;
            } else if (e.type == SDL_KEYDOWN) {
                auto key = e.key.keysym.sym;

                if ((e.key.keysym.mod & KMOD_CTRL) > 0) {
                    if (key == SDLK_c) {
                        quit = true;
                    }
                } else if (std::find(robotIDAllKeys.begin(), robotIDAllKeys.end(), key) != robotIDAllKeys.end()) {
                    // ID key!
                    if (auto currentIDOpt = stringToID(SDL_GetKeyName(key))) {
                        currentID = *currentIDOpt;
                        std::cout << "Current ID: " << currentID << "\n";
                    } else {
                        std::cout << "Bad ID key pressed, retaining original ID.";
                    };
                } else if (std::find(arrowKeys.begin(), arrowKeys.end(), key) != arrowKeys.end()) {
                    // Arrow key!
                    roboteam_msgs::RobotCommand r;

                    r.id = currentID;

                    if (key == SDLK_UP) {
                        r.y_vel = currentVel;
                    } else if (key == SDLK_DOWN) {
                        r.y_vel = -currentVel;
                    } else if (key == SDLK_LEFT) {
                        r.w = currentW;
                    } else if (key == SDLK_RIGHT) {
                        r.w = -currentW;
                    }

                    robotCommandPub.publish(r);
                } else if (key == SDLK_ESCAPE || key == SDLK_q) {
                    quit = true;
                }
            } else if (e.type == SDL_KEYUP) {
                auto key = e.key.keysym.sym;

                if (std::find(arrowKeys.begin(), arrowKeys.end(), key) != arrowKeys.end()) {
                    // Arrow key!
                    roboteam_msgs::RobotCommand r;

                    r.id = currentID;

                    robotCommandPub.publish(r);
                }
            }
        }

        ros::spinOnce();
        fps30.sleep();
    }

    return 0;
}
