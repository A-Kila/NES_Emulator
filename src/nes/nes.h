
#pragma once

#include "events.h"
#include "graphics.h"
#include "cartridge/cartridge.h"
#include "main_bus.h"
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
    void clock();

private:
    graphics_ref_t graphics_;
    events_ref_t events_;

    cartridge_ref_t cartridge_;
    bus_ref_t main_bus_;
    cpu_t cpu_;
    ppu_t ppu_;
};

} // namespace NES