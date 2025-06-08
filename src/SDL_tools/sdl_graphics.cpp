#include "sdl_graphics.h"
#include "SDL3/SDL_pixels.h"
#include "SDL3/SDL_render.h"
#include "SDL3/SDL_surface.h"
#include <algorithm>

namespace SDL {

sdl_graphics_t::sdl_graphics_t(uint32_t width, uint32_t height)
{
    assert(
        SDL_CreateWindowAndRenderer("NES Emulator",
            2 * width, 2 * height,
            SDL_WINDOW_RESIZABLE,
            &window_, &renderer_)
        && "Window and render could not be created"
    );

    SDL_SetRenderLogicalPresentation(renderer_, width, height, SDL_LOGICAL_PRESENTATION_LETTERBOX);

    texture_ = SDL_CreateTexture(
        renderer_,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        width, height);

    SDL_SetTextureScaleMode(texture_, SDL_SCALEMODE_PIXELART);
}

sdl_graphics_t::~sdl_graphics_t()
{
    SDL_DestroyTexture(texture_);
    SDL_DestroyRenderer(renderer_);
    SDL_DestroyWindow(window_);
}

void sdl_graphics_t::update_frame(uint32_t *pixel_data)
{
    SDL_UpdateTexture(texture_, nullptr, pixel_data, 200 * sizeof(uint32_t));
    SDL_RenderClear(renderer_);
    SDL_RenderTexture(renderer_, texture_, NULL, NULL);
    SDL_RenderPresent(renderer_);
}

} // namespace SDL