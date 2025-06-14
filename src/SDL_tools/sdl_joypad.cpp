#include "sdl_joypad.h"

#include "SDL3/SDL_keyboard.h"
#include "../nes/utils.h"

namespace SDL {

sdl_joypad_t::sdl_joypad_t() : joypad_t()
{
    keybinds = {
        { SDL_SCANCODE_J,      NES::joypad_t::A },
        { SDL_SCANCODE_K,      NES::joypad_t::B },
        { SDL_SCANCODE_ESCAPE, NES::joypad_t::SELECT },
        { SDL_SCANCODE_RETURN, NES::joypad_t::START },
        { SDL_SCANCODE_W,      NES::joypad_t::UP },
        { SDL_SCANCODE_S,      NES::joypad_t::DOWN },
        { SDL_SCANCODE_A,      NES::joypad_t::LEFT },
        { SDL_SCANCODE_D,      NES::joypad_t::RIGHT },
    };
}

sdl_joypad_t::~sdl_joypad_t()
{
}

void sdl_joypad_t::update_keys()
{
    const bool *keyboard = SDL_GetKeyboardState(nullptr);

    uint8_t pressed_keys;
    for (auto it = keybinds.begin(); it != keybinds.end(); it++)
    {
        NES::utils::set_flag(pressed_keys, it->second, keyboard[it->first]);
    }

    joypads_[0] = pressed_keys; // for now only one controller is implemented
}

} // namespace SDL