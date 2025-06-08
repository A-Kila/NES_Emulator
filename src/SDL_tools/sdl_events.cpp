#include "sdl_events.h"

namespace SDL {

sdl_events_t::sdl_events_t() :
    event_()
{
}

sdl_events_t::~sdl_events_t()
{
}

NES::nes_event sdl_events_t::get_event()
{
    SDL_PollEvent(&event_);

    switch (event_.type)
    {
    case SDL_EVENT_QUIT:
        return NES::nes_event::QUIT;

    default:
        return NES::nes_event::NO_EVENT;
    }
}

} // namespace SDL