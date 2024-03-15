/**
 * \file
 * Description of UI.h file.
 */

#include "UI.h"


UI::UI(int sizeX, int sizeY) : sizeX(sizeX), sizeY(sizeY) {
    SDL_Init(SDL_INIT_EVERYTHING);

    // Create a Window in the middle of the screen
    window = SDL_CreateWindow("Implicit contour", SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED, sizeX, sizeY, SDL_WINDOW_SHOWN);

    renderer = SDL_CreateRenderer(window, -1,
            SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
}


/// Destructor fully de-initializes the UI, including closing the main window.
UI::~UI() {
    if (renderer)
        SDL_DestroyRenderer(renderer);
    if (window)
        SDL_DestroyWindow(window);
    SDL_Quit();
}

/// Clears the draw buffer (black).
void UI::clear()
{
    setDrawColor(0, 0, 0, 255);
    SDL_RenderClear(renderer);
    setDrawColor(255, 255, 255, 255);
}

/// Presents the draw buffer to the screen.
void UI::present()
{
    SDL_RenderPresent(renderer);
}

/// Draw pixel, coordinate (0,0) is in the middle.
void UI::drawPixel(int x, int y)
{
    SDL_RenderDrawPoint(renderer, x + sizeX / 2, y + sizeY / 2);
}

void UI::setDrawColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    SDL_SetRenderDrawColor(renderer, r, g, b, a);
}

