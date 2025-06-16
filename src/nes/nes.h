
#pragma once

#include "events.h"
#include "graphics.h"
#include "cartridge/cartridge.h"
#include "main_bus.h"
#include "ppu_bus.h"
#include "cpu.h"
#include "ppu.h"

namespace NES {

class nes_t
{
public:
    nes_t(const std::string &filename);
    ~nes_t();

    void reset();
    void run();

    // might be useful for testing
    void set_reset_vector(uint16_t address);

private:
    void _clock();

    void _get_screen_argb(uint32_t *screen, uint8_t *pixels);

private:
    graphics_ref_t graphics_;
    events_ref_t events_;
    joypad_ref_t joypad_;

    cartridge_ref_t cartridge_;
    bus_ref_t ppu_bus_;
    ppu_ref_t ppu_;
    bus_ref_t main_bus_;
    cpu_t cpu_;

    uint32_t cycle_count_;

private:
    static constexpr uint32_t make_argb(uint8_t r, uint8_t g, uint8_t b)
    {
        return 0xFF000000u | (r << 16) | (g << 8) | b;
    };
};

} // namespace NES