#include "sdl_init.h"
#include "SDL3/SDL_messagebox.h"

#include <SDL3/SDL.h>
#include <cassert>

namespace SDL {

sdl_init_t::sdl_init_t()
{
    assert(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) && "SDL could not be initialized");

}

sdl_init_t::~sdl_init_t()
{
    SDL_Quit();
}

} // namespace SDL