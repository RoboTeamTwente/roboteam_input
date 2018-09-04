//
// Created by mrlukasbos on 8/28/18.
//

#ifndef ROBOTEAM_INPUT_INTERFACE_H
#define ROBOTEAM_INPUT_INTERFACE_H

#include <math.h>
#include <SDL.h>

class InputInterface {
private:
    // System values
    const double MAX_ANGULAR_VELOCITY = 2048.0/36.0 * 2 * M_PI; // max angular velocity value in radians
    const double MAX_VEL = 6.0; // m/s
    const int MIN_GENEVA_STATE = 1;
    const int MAX_GENEVA_STATE = 5;
    SDL_Renderer *renderer = nullptr;
    int offsetX = 10;
    int startX = offsetX + 30;
    int startY = 10;
    int barHeight = 20;
    int barWidth = 100;
    int spacing = 20;
    int pictoStartX = offsetX + 2;
    int pictoWidth = 20;
    int pictoEndX = pictoStartX + pictoWidth;
    int extraSpacing = barHeight + spacing + barHeight + spacing + barHeight + spacing;
    int pictoStartY = startY + extraSpacing;
    int pictoEndY = pictoStartY + barHeight;

    // Colors
    struct Color { uint8_t r, g, b, a; };
    Color BACKGROUND_COLOR {0, 0, 0, 255}; // Black
    Color ITEM_COLOR { 255, 255, 255, 255 }; // White

    // Functions
    void showVelocity(double velocity);
    void showAngle(double currentAngularVelocity);
    void showKickPower(int kickPower);
    void showGeneva(int currentGenevaState);
    void showId(int id);

public:
    explicit InputInterface(SDL_Renderer * renderer);
    void drawGui(SDL_Renderer * renderer, int kickPower, double currentVelocity, double currentAngularVelocity, int currentGenevaState, int currentId);
};

#endif //ROBOTEAM_INPUT_INTERFACE_H
