#include <SDL.h>
#include <iostream>
#include <ros/ros.h>
#include <boost/optional.hpp>

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

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
    SDLK_RIGHT,
    SDLK_z,
    SDLK_x
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

double const MAX_VEL = 6;
double const MAX_W = 2048.0 / 360.0 * (2 * M_PI);
int const MAX_ID = 16;

void drawGui(SDL_Renderer *renderer, double currentVel, double currentW, int currentID) {
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    int offsetX = 10;

    int startX = offsetX + 30;
    int startY = 10;
    int barHeight = 20;
    int barWidth = 100;
    int spacing = 20;

    int pictoStartX = offsetX + 2;
    int pictoWidth = 20;
    int pictoEndX = pictoStartX + pictoWidth;

    // Speed
    int pictoHalfY = startY + barHeight / 2;
    SDL_RenderDrawLine(renderer, pictoStartX, pictoHalfY, pictoEndX, pictoHalfY);
    SDL_RenderDrawLine(renderer, pictoEndX, pictoHalfY, pictoStartX + pictoWidth / 2, startY);
    SDL_RenderDrawLine(renderer, pictoEndX, pictoHalfY, pictoStartX + pictoWidth / 2, startY + barHeight);

    SDL_Rect velRect;
    velRect.x = startX;
    velRect.y = startY;
    velRect.w = currentVel / MAX_VEL * barWidth;
    velRect.h = barHeight;
    SDL_RenderFillRect(renderer, &velRect);

    // Angle
    int pictoStartY = startY + barHeight + spacing;
    int pictoEndY = pictoStartY + barHeight;
    SDL_RenderDrawLine(renderer, pictoStartX, pictoEndY, pictoEndX, pictoStartY);
    SDL_RenderDrawLine(renderer, pictoStartX, pictoEndY, pictoEndX, pictoEndY);

    SDL_Rect wRect;
    wRect.x = startX;
    wRect.y = startY + barHeight + spacing;
    wRect.w = currentW / MAX_W * barWidth;
    wRect.h = barHeight;
    SDL_RenderFillRect(renderer, &wRect);

    // ID
    pictoStartY = startY + barHeight + spacing + barHeight + spacing;
    pictoEndY = pictoStartY + barHeight;

    SDL_RenderDrawLine(renderer, pictoStartX, pictoStartY, pictoEndX, pictoStartY);
    SDL_RenderDrawLine(renderer, pictoStartX, pictoEndY, pictoEndX, pictoEndY);
    SDL_RenderDrawLine(renderer, (pictoStartX + pictoEndX) / 2, pictoStartY, (pictoStartX + pictoEndX) / 2, pictoEndY);

    int boxSize = barHeight;
    int boxSpacing = spacing;
    int boxesPerRow = 4;
    int rows = 4;

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < boxesPerRow; ++x) {
            int id = y * boxesPerRow + x;

            SDL_Rect outlineRect;
            outlineRect.x = startX + x * (boxSize + boxSpacing);
            outlineRect.y = y * (boxSize + boxSpacing) + (startY + barHeight + spacing + barHeight + spacing);
            outlineRect.w = boxSize;
            outlineRect.h = boxSize;

            if (id <= currentID) {
                SDL_RenderFillRect(renderer, &outlineRect);
            } else {
                SDL_RenderDrawRect(renderer, &outlineRect);
            }
        }
    }
}

}

int main(int argc, char* argv[]) {
    std::cout << R"--(
RTT Keyboard Joystick

Controls:
    Arrow keys to steer/drive
    z/x to strafe left/right
    v to kick
    n to chip
    space to dribble
    0-9, a-f to select an ID
    Numpad 4/6 to decrease/increase x velocity
    Numpad 1/3 to decrease/increase angular velocity
)--";

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

    // TODO: Keep vel in a vector and update accordingly!

    // Ros stuff
    ros::init(argc, argv, "keyboard_controller", ros::InitOption::AnonymousName);
    ros::NodeHandle n;
    auto robotCommandPub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 100);
    // TODO: Maybe base this on role_iterations_per_second?
    ros::Rate fpsRate(20);

    // Event loop
    bool quit = false;
    SDL_Event e;

    int x_vel = 0;
    int y_vel = 0;
    int w = 0;
    bool doKick = false;
    bool doChip = false;
    bool doDribble = false;

    while(!quit && ros::ok()) {

        while(SDL_PollEvent(&e) != 0) {
            if ( e.type == SDL_QUIT ) {
                quit = true;
            } else if (e.type == SDL_KEYDOWN && e.key.repeat == 0) {
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

                    if (key == SDLK_UP) {
                        x_vel += 1;
                    } else if (key == SDLK_DOWN) {
                        x_vel -= 1;
                    } else if (key == SDLK_LEFT) {
                        w += 1;
                    } else if (key == SDLK_RIGHT) {
                        w -= 1;
                    } else if (key == SDLK_z) {
                        y_vel += 1;
                    } else if (key == SDLK_x) {
                        y_vel -= 1;
                    }
                } else if (key == SDLK_ESCAPE || key == SDLK_q) {
                    quit = true;
                } else if (key == SDLK_KP_4) {
                    currentVel -= 0.1;
                } else if (key == SDLK_KP_6) {
                    currentVel += 0.1;
                } else if (key == SDLK_KP_1) {
                    currentW -= 0.3;
                } else if (key == SDLK_KP_3) {
                    currentW += 0.3;
                } else if (key == SDLK_v) {
                    doKick = true;
                } else if (key == SDLK_n) {
                    doChip = true;
                } else if (key == SDLK_SPACE) {
                    doDribble = true;
                }
            } else if (e.type == SDL_KEYDOWN) {
                auto key = e.key.keysym.sym;

                if (key == SDLK_KP_4) {
                    currentVel -= 0.1;
                } else if (key == SDLK_KP_6) {
                    currentVel += 0.1;
                } else if (key == SDLK_KP_1) {
                    currentW -= 0.3;
                } else if (key == SDLK_KP_3) {
                    currentW += 0.3;
                }
            } else if (e.type == SDL_KEYUP) {
                auto key = e.key.keysym.sym;

                if (key == SDLK_UP) {
                    x_vel -= 1;
                } else if (key == SDLK_DOWN) {
                    x_vel += 1;
                } else if (key == SDLK_LEFT) {
                    w -= 1;
                } else if (key == SDLK_RIGHT) {
                    w += 1;
                } else if (key == SDLK_z) {
                    y_vel -= 1;
                } else if (key == SDLK_x) {
                    y_vel += 1;
                } else if (key == SDLK_SPACE) {
                    doDribble = false;
                }
            }
        }

        if (currentVel > MAX_VEL) currentVel = MAX_VEL;
        if (currentVel < 0) currentVel = 0;
        if (currentW > MAX_W) currentW = MAX_W;
        if (currentW < 0) currentW = 0;

        roboteam_msgs::RobotCommand r;
        r.id = currentID;
        r.x_vel = x_vel * currentVel;
        r.y_vel = y_vel * currentVel;
        r.w = w * currentW;

        if (doKick) {
            r.kicker = true;
            r.kicker_vel = roboteam_msgs::RobotCommand::MAX_KICKER_VEL;
            r.kicker_forced = true;
        } else if (doChip) {
            r.chipper = true;
            r.chipper_vel = roboteam_msgs::RobotCommand::MAX_CHIPPER_VEL;
            r.chipper_forced= true;
        }

        doKick = false;
        doChip = false;

        r.dribbler = doDribble;

        robotCommandPub.publish(r);

        ros::spinOnce();
        fpsRate.sleep();

        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_RenderClear(renderer);

        drawGui(renderer, currentVel, currentW, currentID);
        SDL_RenderPresent(renderer);
    }

    return 0;
}
