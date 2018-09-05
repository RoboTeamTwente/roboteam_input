//
// Created by mrlukasbos on 8/28/18.
// This class draws a visulization of the parameters used to manually control a robot
//

#include "input_interface.h"

InputInterface::InputInterface() : renderer(nullptr){
  // load the font
  if (TTF_Init() < 0) {
      std::cout << "TTF library could not be initialized!!";
  }
  font = TTF_OpenFont("/usr/share/fonts/truetype/freefont/FreeMono.ttf",14);
}

InputInterface::~InputInterface() {
  TTF_CloseFont(font);
  TTF_Quit();
}

void InputInterface::drawGui(SDL_Renderer * renderer, int kickPower, double velocity, double angularVelocity, int genevaState, int id) {
    this->renderer = renderer;

    // set background
    SDL_SetRenderDrawColor(renderer, BACKGROUND_COLOR.r, BACKGROUND_COLOR.g, BACKGROUND_COLOR.b, BACKGROUND_COLOR.a);
    SDL_RenderClear(renderer);

    // set color of the drawing items
    SDL_SetRenderDrawColor(renderer, ITEM_COLOR.r, ITEM_COLOR.g, ITEM_COLOR.b, ITEM_COLOR.a);

    // Drawheight is incremented  by each function to indicate the height of that certain part of the interface.
    drawHeight = 0;

    // Draw the parameters
    showId(id);
    showVelocity(velocity);
    showAngle(angularVelocity);
    showKickPower(kickPower);
    showGeneva(genevaState);

    // render to screen
    SDL_RenderPresent(renderer);
}

void InputInterface::drawText(std::string text, int x, int y) {
  SDL_Color textColor = TEXT_COLOR;
  SDL_Surface* surfaceMessage = TTF_RenderText_Solid(font, text.c_str(), textColor);
  if(surfaceMessage == NULL) {
     printf("Unable to render text surface: %s\n",TTF_GetError());
  }
  SDL_Texture* message = SDL_CreateTextureFromSurface(renderer,surfaceMessage);
  SDL_FreeSurface(surfaceMessage);
  SDL_Rect textRect{x, y, surfaceMessage->w, surfaceMessage->h};
  SDL_RenderCopy(renderer,message,NULL,&textRect);
  SDL_DestroyTexture(message);
}

SDL_Rect InputInterface::drawRect(int x, int y, int w, int h, bool isFilled) {
  SDL_Rect rect;
  rect.x = x;
  rect.y = y;
  rect.w = w;
  rect.h = h;

  if(isFilled){
    SDL_RenderFillRect(renderer, &rect);
  } else {
    SDL_RenderDrawRect(renderer, &rect);
  }
}

void InputInterface::showVelocity(double currentVelocity) {
    drawText("Velocity (keypad 5-7)", OFFSET_X, drawHeight);
    drawRect(OFFSET_X, drawHeight + SPACING, currentVelocity/MAX_VEL*BAR_WIDTH, BAR_HEIGHT, true);
    drawHeight+= 80;
}

void InputInterface::showAngle(double currentAngularVelocity) {
    drawText("Angular velocity", OFFSET_X, drawHeight);
    drawRect(OFFSET_X, drawHeight + SPACING, currentAngularVelocity/MAX_ANGULAR_VELOCITY*BAR_WIDTH, BAR_HEIGHT, true);
    drawHeight+=80;
}

void InputInterface::showKickPower(int kickPower) {
    drawText("Kickpower (keypad 4-6)", OFFSET_X, drawHeight);
    int boxSize = BAR_HEIGHT;
    for (int i = 0; i < 8; ++i) {
      int kickPowerHere = i + 1;
      drawRect(OFFSET_X + i * (boxSize + OFFSET_X), drawHeight + SPACING, boxSize, boxSize, kickPowerHere <= kickPower);
    }
    drawHeight += 80;
}

void InputInterface::showGeneva(int currentGenevaState) {
    drawText("Geneva (Pageup/Pagedown)", OFFSET_X, drawHeight);

    int boxSize = BAR_HEIGHT;
    for (int i = 0; i < 5; ++i) {
        //genevaState can be -2 through 2
        int genevaState = i + 1;
        drawRect(OFFSET_X + i * (boxSize + OFFSET_X), drawHeight + SPACING, boxSize, boxSize, genevaState == currentGenevaState);
    }
    drawHeight += 80;
}

void InputInterface::showId(int id) {
    drawText("Robot: " + std::to_string(id), OFFSET_X, drawHeight);
    drawHeight+=20;
}
