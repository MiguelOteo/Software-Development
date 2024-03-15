/**
 * \file
 * Description of UI.h file.
 */

#ifndef SRC_UI_H_
#define SRC_UI_H_

#include <SDL2/SDL.h>

#include <cstdint>

/**
 * Description of UI class
 */
class UI
{
  public:
    const int sizeX;
    const int sizeY;

    /// Constructor initializes the UI fully.
    ///
    /// Creates a main (SDL) window for rendering.
    ///
    /// \param Dimensions (pixels) of the window to be created.
    UI(int sizeX, int sizeY);

    // UI objects should not be copied or moved.
    UI(const UI&) = delete;
    UI(const UI&&) = delete;
    UI &operator=(const UI &) = delete;

    /// Destructor fully de-initializes the UI, including closing the main window.
    ~UI();

    /// Clears the draw buffer (black).
    void clear();

    /// Presents the draw buffer to the screen.
    void present();

    /// Draw pixel, coordinate (0,0) is in the middle.
    void drawPixel(int x, int y);

    void setDrawColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a);

  private:
    /// Main UI window.
    SDL_Window *window;
    /// SDL Renderer to draw items onto #window.
    SDL_Renderer *renderer;
};

#endif /* SRC_UI_H_ */
