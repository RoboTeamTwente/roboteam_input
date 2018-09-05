//
// Created by mrlukasbos on 8/28/18.
//

#ifndef ROBOTEAM_INPUT_INTERFACE_H
#define ROBOTEAM_INPUT_INTERFACE_H

#include <math.h>
#include <SDL.h>
#include <iostream>
#include <SDL_ttf.h>

class InputInterface {
private:
    // System values
    const double MAX_ANGULAR_VELOCITY = 2048.0/36.0 * 2 * M_PI; // max angular velocity value in radians
    const double MAX_VEL = 6.0; // m/s
    const int MIN_GENEVA_STATE = 1;
    const int MAX_GENEVA_STATE = 5;
    SDL_Renderer *renderer = nullptr;

    // Margins
    const int OFFSET_X = 10;
    const int BAR_HEIGHT = 20;
    const int BAR_WIDTH = 200;
    const int SPACING = 20;
    int drawHeight = 0;

    // Colors
    const SDL_Color BACKGROUND_COLOR {255, 0, 0, 255}; // Red
    const SDL_Color ITEM_COLOR { 255, 255, 255, 255 }; // White
    const SDL_Color TEXT_COLOR { 255, 255, 255, 255 }; // White

    // Functions
    void showVelocity(double velocity);
    void showAngle(double currentAngularVelocity);
    void showKickPower(int kickPower);
    void showGeneva(int currentGenevaState);
    void showId(int id);

    // Utilities
    TTF_Font * font = nullptr;
    void drawText(std::string text, int x, int y);
    SDL_Rect drawRect(int x, int y, int w, int h, bool isFilled);

public:
    explicit InputInterface();
    ~InputInterface();
    void drawGui(SDL_Renderer * renderer, int kickPower, double currentVelocity, double currentAngularVelocity, int currentGenevaState, int currentId);
};

#endif //ROBOTEAM_INPUT_INTERFACE_H
