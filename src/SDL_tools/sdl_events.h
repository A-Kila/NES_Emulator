#pragma once

#include "../nes/events.h"

#include "SDL3/SDL.h"

namespace SDL {

class sdl_events_t : public NES::i_event_t
{
public:
    sdl_events_t();
    ~sdl_events_t();

    NES::nes_event get_event() override;

private:
    SDL_Event event_;
};

} // namespace SDL