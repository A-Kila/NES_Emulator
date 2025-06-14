#pragma once

#include "../nes/graphics.h"

#include "SDL3/SDL.h"

namespace SDL {

class sdl_graphics_t : public NES::i_graphics_t
{
public:
    sdl_graphics_t(uint32_t width, uint32_t height);
    ~sdl_graphics_t();

    void update_frame(uint32_t *pixel_data) override;

private:
    SDL_Window *window_;
    SDL_Renderer *renderer_;
    SDL_Texture *texture_;

    uint32_t texture_width_;
    uint32_t texture_height_;
};

} // namespace SDL