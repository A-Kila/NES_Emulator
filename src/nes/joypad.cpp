#include "joypad.h"

#include "utils.h"

namespace NES {

joypad_t::joypad_t() :
    joypads_(),
    shift_regs_()
{
}

joypad_t::~joypad_t()
{
}

void joypad_t::update_joypad(const bool is_second)
{
    shift_regs_[is_second] = joypads_[is_second];
}

bool joypad_t::get_input(const bool is_second)
{
    bool is_pressed = (joypads_[is_second] & 0x80) > 0;
    joypads_[is_second] <<= 1;

    return is_pressed;
}

} // namespace NES