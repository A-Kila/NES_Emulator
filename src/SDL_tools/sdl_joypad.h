#pragma once

#include <map>
#include "SDL3/SDL.h"
#include  "../nes/joypad.h"

namespace SDL {

class sdl_joypad_t : public NES::joypad_t
{
public:
    sdl_joypad_t();
    ~sdl_joypad_t() override;

    void update_keys() override;

private:
    std::map<SDL_Scancode, keys> keybinds;
};

} // namespace SDL