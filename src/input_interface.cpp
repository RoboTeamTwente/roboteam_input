//
// Created by mrlukasbos on 8/28/18.
// This class draws a visul
//

#include "input_interface.h"

InputInterface::InputInterface() : renderer(nullptr){}

void InputInterface::drawGui(SDL_Renderer * renderer, int kickPower, double velocity, double angularVelocity, int genevaState, int id) {
    this->renderer = renderer;

    // set background color
    SDL_SetRenderDrawColor(renderer, BACKGROUND_COLOR.r, BACKGROUND_COLOR.g, BACKGROUND_COLOR.b, BACKGROUND_COLOR.a);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, ITEM_COLOR.r, ITEM_COLOR.g, ITEM_COLOR.b, ITEM_COLOR.a);

    showKickPower(kickPower);
    showAngle(angularVelocity);
    showGeneva(genevaState);
    showVelocity(velocity);
    showId(id);

    // initialize the font
    TTF_Font * font = loadFont();
drawText(font, "hallo", 20, 20, 20);
drawText(font, "doei", 120, 20, 20);

    TTF_CloseFont(font);
    TTF_Quit();

    SDL_RenderPresent(renderer);

}

TTF_Font* InputInterface::loadFont() {
  if (TTF_Init() < 0) {
      std::cout << "ttf library not initialized!!";
  }

  TTF_Font* font = TTF_OpenFont("/usr/share/fonts/truetype/freefont/FreeMono.ttf",16);
     if(font == NULL) {
         printf("TTF_OpenFont: %s\n",TTF_GetError());
     }
}

void InputInterface::drawText(TTF_Font * font, std::string text, int x, int y, int size) {
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

void InputInterface::showVelocity(double currentVelocity) {
    int pictoHalfY = startY + barHeight + spacing + barHeight / 2;
    SDL_RenderDrawLine(renderer, pictoStartX, pictoHalfY, pictoEndX, pictoHalfY);
    SDL_RenderDrawLine(renderer, pictoEndX, pictoHalfY, pictoStartX + pictoWidth / 2, startY + barHeight + spacing);
    SDL_RenderDrawLine(renderer, pictoEndX, pictoHalfY, pictoStartX + pictoWidth / 2, startY + barHeight + spacing + barHeight);

    SDL_Rect velRect;
    velRect.x = startX;
    velRect.y = startY + barHeight + spacing;
    velRect.w = currentVelocity / MAX_VEL * barWidth;
    velRect.h = barHeight;
    SDL_RenderFillRect(renderer, &velRect);
}

void InputInterface::showAngle(double currentAngularVelocity) {
    int pictoStartY = startY + barHeight + spacing + barHeight + spacing;
    int pictoEndY = pictoStartY + barHeight;
    SDL_RenderDrawLine(renderer, pictoStartX, pictoEndY, pictoEndX, pictoStartY);
    SDL_RenderDrawLine(renderer, pictoStartX, pictoEndY, pictoEndX, pictoEndY);

    SDL_Rect wRect;
    wRect.x = startX;
    wRect.y = startY + barHeight + spacing + barHeight + spacing;
    wRect.w = currentAngularVelocity / MAX_ANGULAR_VELOCITY * barWidth;
    wRect.h = barHeight;
    SDL_RenderFillRect(renderer, &wRect);
}

void InputInterface::showKickPower(int kickPower) {
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
        if (kickPowerHere <= kickPower) {
            SDL_RenderFillRect(renderer, &kickRect);
        } else {
            SDL_RenderDrawRect(renderer, &kickRect);
        }
    }
}

void InputInterface::showGeneva(int currentGenevaState) {
    SDL_RenderDrawLine(renderer, pictoStartX, pictoStartY, pictoStartX + pictoWidth / 2, pictoEndY);
    SDL_RenderDrawLine(renderer, pictoStartX + pictoWidth / 2, pictoEndY, pictoEndX, pictoStartY);
    SDL_RenderDrawLine(renderer, pictoStartX, pictoStartY, pictoStartX, pictoStartY + 10);
    SDL_RenderDrawLine(renderer, pictoStartX, pictoStartY, pictoStartX + 7, pictoStartY + 6);
    SDL_RenderDrawLine(renderer, pictoEndX - 7, pictoStartY + 6, pictoEndX, pictoStartY);
    SDL_RenderDrawLine(renderer, pictoEndX, pictoStartY, pictoEndX, pictoStartY + 10);

    int boxSize = barHeight;
    for (int i = 0; i < 5; ++i) {
        //genevaState can be -2 through 2
        int genevaState = i + 1;
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

void InputInterface::showId(int id) {
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

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < boxesPerRow; ++x) {
            int _id = y * boxesPerRow + x;

            SDL_Rect outlineRect;
            outlineRect.x = startX + x * (boxSize + boxSpacing);
            outlineRect.y = y * (boxSize + boxSpacing) + (startY + extraSpacing);
            outlineRect.w = boxSize;
            outlineRect.h = boxSize;

            if (_id <= id) {
                SDL_RenderFillRect(renderer, &outlineRect);
            } else {
                SDL_RenderDrawRect(renderer, &outlineRect);
            }
        }
    }
}
