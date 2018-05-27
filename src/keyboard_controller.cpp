#include <SDL.h>
#include <iostream>
#include <ros/ros.h>
#include <boost/optional.hpp>
#include <std_msgs/Bool.h>

#include "roboteam_msgs/RobotCommand.h"
#include "roboteam_utils/Vector2.h"

namespace b = boost;

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

int const MIN_GENEVA_STATE = -2;
int const MAX_GENEVA_STATE = 2;

void drawGui(SDL_Renderer *renderer, int currentKick, double currentVel, double currentW, int currentGenevaState, int currentID) {
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

    // Kick
    {
        int pictoHalfY = startY + barHeight / 2;
        int spacing = 10;

        SDL_RenderDrawLine(renderer, pictoStartX, pictoHalfY, pictoEndX, pictoHalfY);
        SDL_RenderDrawLine(renderer, pictoEndX, startY, pictoEndX, startY + barHeight);

        int boxSize = barHeight;

        for (int i = 0; i < 8; ++i) {
            SDL_Rect kickRect;
            kickRect.x = startX + i * (boxSize + spacing);
            kickRect.y = startY;
            kickRect.w = boxSize;
            kickRect.h = boxSize;

            int kickPowerHere = i + 1;

            if (kickPowerHere <= currentKick) {
                SDL_RenderFillRect(renderer, &kickRect);
            } else {
                SDL_RenderDrawRect(renderer, &kickRect);
            }
        }
    }

    // Speed
    int pictoHalfY = startY + barHeight + spacing + barHeight / 2;
    SDL_RenderDrawLine(renderer, pictoStartX, pictoHalfY, pictoEndX, pictoHalfY);
    SDL_RenderDrawLine(renderer, pictoEndX, pictoHalfY, pictoStartX + pictoWidth / 2, startY + barHeight + spacing);
    SDL_RenderDrawLine(renderer, pictoEndX, pictoHalfY, pictoStartX + pictoWidth / 2, startY + barHeight + spacing + barHeight);

    SDL_Rect velRect;
    velRect.x = startX;
    velRect.y = startY + barHeight + spacing;
    velRect.w = currentVel / MAX_VEL * barWidth;
    velRect.h = barHeight;
    SDL_RenderFillRect(renderer, &velRect);

    // Angle Robot
    int pictoStartY = startY + barHeight + spacing + barHeight + spacing;
    int pictoEndY = pictoStartY + barHeight;
    SDL_RenderDrawLine(renderer, pictoStartX, pictoEndY, pictoEndX, pictoStartY);
    SDL_RenderDrawLine(renderer, pictoStartX, pictoEndY, pictoEndX, pictoEndY);

    SDL_Rect wRect;
    wRect.x = startX;
    wRect.y = startY + barHeight + spacing + barHeight + spacing;
    wRect.w = currentW / MAX_W * barWidth;
    wRect.h = barHeight;
    SDL_RenderFillRect(renderer, &wRect);

    // Geneva Drive
    {
        int extraSpacing = barHeight + spacing + barHeight + spacing + barHeight + spacing;
        int pictoStartY = startY + extraSpacing;
        int pictoEndY = pictoStartY + barHeight;
        int spacing = 10;

        SDL_RenderDrawLine(renderer, pictoStartX, pictoStartY, pictoStartX + pictoWidth / 2, pictoEndY);
        SDL_RenderDrawLine(renderer, pictoStartX + pictoWidth / 2, pictoEndY, pictoEndX, pictoStartY);
        SDL_RenderDrawLine(renderer, pictoStartX, pictoStartY, pictoStartX, pictoStartY + 10);
        SDL_RenderDrawLine(renderer, pictoStartX, pictoStartY, pictoStartX + 7, pictoStartY + 6);
        SDL_RenderDrawLine(renderer, pictoEndX - 7, pictoStartY + 6, pictoEndX, pictoStartY);
        SDL_RenderDrawLine(renderer, pictoEndX, pictoStartY, pictoEndX, pictoStartY + 10);

        int boxSize = barHeight;
        for (int i = 0; i < 5; ++i) {
            //genevaState can be -2 through 2
            int genevaState = i - 2;
            SDL_Rect genevaRect;
            genevaRect.x = startX + i * (boxSize + spacing);
            genevaRect.y = startY + extraSpacing;
            genevaRect.w = boxSize;
            genevaRect.h = boxSize;
            if (genevaState == currentGenevaState) {
                SDL_RenderFillRect(renderer, &genevaRect);
            } else {
                SDL_RenderDrawRect(renderer, &genevaRect);
            }
        }
    }

    // ID
    int extraSpacing = barHeight + spacing + barHeight + spacing + barHeight + spacing + barHeight + spacing + barHeight;
    pictoStartY = startY + extraSpacing;
    pictoEndY = pictoStartY + barHeight;

    SDL_RenderDrawLine(renderer, pictoStartX, pictoStartY, pictoEndX, pictoStartY);
    SDL_RenderDrawLine(renderer, pictoStartX, pictoEndY, pictoEndX, pictoEndY);
    SDL_RenderDrawLine(renderer, (pictoStartX + pictoEndX) / 2, pictoStartY, (pictoStartX + pictoEndX) / 2, pictoEndY);

    int boxSize = barHeight;
    int boxSpacing = spacing;
    int boxesPerRow = 4;
    int rows = 4;

    int exclamationSpacing = 5;
    int exclamationWidth = 5;
    int exclamationBarHeight = 12;
    int exclamationDotSize = 3;

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < boxesPerRow; ++x) {
            int id = y * boxesPerRow + x;

            SDL_Rect outlineRect;
            outlineRect.x = startX + x * (boxSize + boxSpacing);
            outlineRect.y = y * (boxSize + boxSpacing) + (startY + extraSpacing);
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

struct Direction {
    void handleEvent(SDL_Event const & e) {
        if ((e.type == SDL_KEYDOWN  || e.type == SDL_KEYUP) && e.key.repeat == 0) {
            int modifierInt = 0;
            bool modifierBool = false;
            if (e.type == SDL_KEYDOWN) {
                modifierInt = 1;
                modifierBool = true;
            } else {
                modifierInt = -1;
                modifierBool = false;
            }

            auto key = e.key.keysym.sym;

            if (key == SDLK_UP) {
                x_vel += modifierInt;
            } else if (key == SDLK_DOWN) {
                x_vel -= modifierInt;
            } else if (key == SDLK_LEFT) {
                w += modifierInt;
            } else if (key == SDLK_RIGHT) {
                w -= modifierInt;
            } else if (key == KEY_STRAFE_LEFT) {
                y_vel += modifierInt;
            } else if (key == KEY_STRAFE_RIGHT) {
                y_vel -= modifierInt;
            } else if (key == KEY_DRIBBLE) {
                doDribble = modifierBool;
            } else if (key == KEY_KICK) {
                doKick = modifierBool;
            } else if (key == KEY_CHIP) {
                doChip = modifierBool;
            } else if (key == KEY_ROTATE_LEFT && e.type == SDL_KEYDOWN) {
                currentGenevaState--;
            } else if (key == KEY_ROTATE_RIGHT && e.type == SDL_KEYDOWN) {
                currentGenevaState++;
            }
        }
        if (currentGenevaState<MIN_GENEVA_STATE) currentGenevaState = MIN_GENEVA_STATE;
        if (currentGenevaState>MAX_GENEVA_STATE) currentGenevaState = MAX_GENEVA_STATE;
    }

    int x_vel = 0;
    int y_vel = 0;
    int w = 0;
    bool doKick = false;
    bool enKick = false;
    bool doChip = false;
    bool doDribble = false;
    int currentGenevaState = 0;

    SDL_Keycode const KEY_STRAFE_LEFT = SDLK_z;
    SDL_Keycode const KEY_STRAFE_RIGHT = SDLK_x;
    SDL_Keycode const KEY_DRIBBLE = SDLK_SPACE;
    SDL_Keycode const KEY_KICK = SDLK_v;
    SDL_Keycode const KEY_CHIP = SDLK_n;
    SDL_Keycode const KEY_ROTATE_LEFT = SDLK_PAGEUP;
    SDL_Keycode const KEY_ROTATE_RIGHT = SDLK_PAGEDOWN;
} ;

struct Speed {
    void handleEvent(SDL_Event const & e) {
        if (e.type == SDL_KEYDOWN) {
            auto key = e.key.keysym.sym;

            if (key == KEY_INCREASE_VEL) {
                currentVel += STEP_VEL;
            } else if (key == KEY_DECREASE_VEL) {
                currentVel -= STEP_VEL;
            } else if (key == KEY_INCREASE_W) {
                currentW += STEP_W;
            } else if (key == KEY_DECREASE_W) {
                currentW -= STEP_W;
            } else if (key == KEY_INCREASE_KICK) {
                currentKick++;
            } else if (key == KEY_DECREASE_KICK) {
                currentKick--;
            }
        }

        if (currentVel > MAX_VEL) currentVel = MAX_VEL;
        if (currentVel < 0) currentVel = 0;
        if (currentW > MAX_W) currentW = MAX_W;
        if (currentW < 0) currentW = 0;
        if (currentKick > roboteam_msgs::RobotCommand::MAX_KICKER_VEL) currentKick = roboteam_msgs::RobotCommand::MAX_KICKER_VEL;
        if (currentKick < 0) currentKick = 0;
    }

    double currentVel = 1;
    double currentW = 5;
    int currentKick = 1;

    double const STEP_VEL = 0.1;
    double const STEP_W = 0.1;

    SDL_Keycode const KEY_INCREASE_VEL = SDLK_KP_6;
    SDL_Keycode const KEY_DECREASE_VEL = SDLK_KP_4;
    SDL_Keycode const KEY_INCREASE_W = SDLK_KP_3;
    SDL_Keycode const KEY_DECREASE_W = SDLK_KP_1;
    SDL_Keycode const KEY_INCREASE_KICK = SDLK_KP_9;
    SDL_Keycode const KEY_DECREASE_KICK = SDLK_KP_7;
} ;

roboteam_msgs::RobotCommand prevRobotCommand;
roboteam_msgs::RobotCommand makeRobotCommand(int const currentID, Speed const & speed, Direction const & direction) {
    roboteam_msgs::RobotCommand r;
    r.id = currentID;
    r.x_vel = direction.x_vel * speed.currentVel;
    r.y_vel = direction.y_vel * speed.currentVel;
    r.w = direction.w * speed.currentW;
    r.use_angle = true;

    // double velIncrement = 0.03;
    // double wIncrement = 0.03;
    // if ((fabs(r.x_vel) - fabs(prevRobotCommand.x_vel)) > velIncrement) {
    //     r.x_vel = (fabs(prevRobotCommand.x_vel) + velIncrement) * r.x_vel / fabs(r.x_vel);
    // }
    // if ((fabs(r.y_vel) - fabs(prevRobotCommand.y_vel)) > velIncrement) {
    //     r.y_vel = (fabs(prevRobotCommand.y_vel) + velIncrement) * r.y_vel / fabs(r.y_vel);
    // }
    // if ((fabs(r.w) - fabs(prevRobotCommand.w)) > wIncrement) {
    //     r.w = (fabs(prevRobotCommand.w) + wIncrement) * r.w / fabs(r.w);
    // }

    if (direction.doKick) {
        r.kicker = true;
        r.kicker_forced = true;
        r.kicker_vel = speed.currentKick;
        // r.kicker_forced = true;
    } else if (direction.doChip) {
        r.chipper = true;
        r.chipper_forced = true;
        r.chipper_vel = speed.currentKick;
        // r.chipper_forced= true;
    }

    r.dribbler = direction.doDribble;
    r.geneva_state = direction.currentGenevaState;
    prevRobotCommand = r;

    return r;
}

bool isCtrlPressed(SDL_Event const e) {
    if (e.type == SDL_KEYDOWN) {
        if ((e.key.keysym.mod & KMOD_CTRL) > 0) {
            return true;
        }
    }

    return false;
}

}

int main(int argc, char* argv[]) {
    std::vector<std::string> args(argv + 1, argv + argc);

    if (cmdOptionExists(args, "--help") || cmdOptionExists(args, "-h")) {
        std::cout << R"--([Keyboard controller]

Args:
    --help, -h      Display this help
    -id             Set the starting ID (default: 5)
)--";
        return 0;
    }

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

    int currentID = 0;

    if (auto idOpt = getCmdOption(args, "-id")) {
        try {
            currentID = std::stoi(*idOpt);
        } catch (...) {
            std::cerr << "Could not convert id \"" << *idOpt << "\" to integer. Keeping 5.";
        }
    }

    Speed speed;
    Direction direction;

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
            if ( e.type == SDL_QUIT ) {
                quit = true;
            } else if (e.type == SDL_KEYDOWN) {
                auto key = e.key.keysym.sym;

                // Handle ctrl-c
                if (isCtrlPressed(e) && key == SDLK_c) {
                    quit = true;
                }

                // Handle escape & q
                if (key == SDLK_ESCAPE || key == SDLK_q) {
                    quit = true;
                }

                // Handle ID switching
                if (std::find(robotIDAllKeys.begin(), robotIDAllKeys.end(), key) != robotIDAllKeys.end()) {
                    if (!(key == SDLK_c && isCtrlPressed(e))) {
                        if (auto currentIDOpt = stringToID(SDL_GetKeyName(key))) {
                            currentID = *currentIDOpt;
                        } else {
                            std::cout << "Bad ID key pressed, retaining original ID.";
                        }
                    }
                }
            }

            // Handle speed & direction
            speed.handleEvent(e);
            direction.handleEvent(e);
        }

        // Publish a robot instruction
        robotCommandPub.publish(makeRobotCommand(currentID, speed, direction));

        // Wait s.t. we don't burn cycles
        ros::spinOnce();
        fpsRate.sleep();

        // Make the screen red
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_RenderClear(renderer);

        // Draw the gui and refresh the screen
        drawGui(renderer, speed.currentKick, speed.currentVel, speed.currentW, direction.currentGenevaState, currentID);
        SDL_RenderPresent(renderer);
    }

    return 0;
}
