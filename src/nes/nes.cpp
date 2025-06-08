#include "nes.h"

#include "../SDL_tools/sdl_graphics.h"
#include "../SDL_tools/sdl_events.h"
#include "ppu.h"

namespace NES {

const uint32_t TEMP_WIDTH = 320;
const uint32_t TEMP_HEIGHT = 240;

nes_t::nes_t(const std::string &filename) :
    graphics_(std::make_shared<SDL::sdl_graphics_t>(ppu_t::PICTURE_WIDTH, ppu_t::PICTURE_HEIGHT)),
    events_(std::make_shared<SDL::sdl_events_t>()),
    cartridge_(std::make_shared<cartridge_t>(filename)),
    main_bus_(std::make_shared<main_bus_t>(cartridge_)),
    cpu_(main_bus_),
    ppu_()
{
    reset();
}

nes_t::~nes_t()
{
}

void nes_t::reset()
{
    cpu_.reset();
}

uint32_t make_argb(uint8_t r, uint8_t g, uint32_t b)
{
    return 0xFF000000 | (r < 16) | (g << 8) | b;
}

void nes_t::run()
{
    bool quit = false;
    while (!quit)
    {
        while (auto event = events_->get_event())
        {
            switch (event)
            {
            case nes_event::QUIT:
                quit = true;
                break;

            default:
                break;
            }
        }

        clock();

        static uint32_t palette[] =
        {
            make_argb(84,  84,  84),    make_argb(0,  30, 116),    make_argb(8,  16, 144),    make_argb(48,   0, 136),   make_argb(68,   0, 100),   make_argb(92,   0,  48),   make_argb(84,   4,   0),   make_argb(60,  24,   0),   make_argb(32,  42,   0),   make_argb(8,  58,   0),   make_argb(0,  64,   0),    make_argb(0,  60,   0),    make_argb(0,  50,  60),    make_argb(0,   0,   0),   make_argb(0, 0, 0), make_argb(0, 0, 0),
            make_argb(152, 150, 152),   make_argb(8,  76, 196),    make_argb(48,  50, 236),   make_argb(92,  30, 228),   make_argb(136,  20, 176),  make_argb(160,  20, 100),  make_argb(152,  34,  32),  make_argb(120,  60,   0),  make_argb(84,  90,   0),   make_argb(40, 114,   0),  make_argb(8, 124,   0),    make_argb(0, 118,  40),    make_argb(0, 102, 120),    make_argb(0,   0,   0),   make_argb(0, 0, 0), make_argb(0, 0, 0),
            make_argb(236, 238, 236),   make_argb(76, 154, 236),   make_argb(120, 124, 236),  make_argb(176,  98, 236),  make_argb(228,  84, 236),  make_argb(236,  88, 180),  make_argb(236, 106, 100),  make_argb(212, 136,  32),  make_argb(160, 170,   0),  make_argb(116, 196,   0), make_argb(76, 208,  32),   make_argb(56, 204, 108),   make_argb(56, 180, 204),   make_argb(60,  60,  60),  make_argb(0, 0, 0), make_argb(0, 0, 0),
            make_argb(236, 238, 236),   make_argb(168, 204, 236),  make_argb(188, 188, 236),  make_argb(212, 178, 236),  make_argb(236, 174, 236),  make_argb(236, 174, 212),  make_argb(236, 180, 176),  make_argb(228, 196, 144),  make_argb(204, 210, 120),  make_argb(180, 222, 120), make_argb(168, 226, 144),  make_argb(152, 226, 180),  make_argb(160, 214, 228),  make_argb(160, 162, 160), make_argb(0, 0, 0), make_argb(0, 0, 0)
        };

        graphics_->update_frame(ppu_.get_picture());
    }
}

void nes_t::set_reset_vector(uint16_t address)
{
    cpu_.set_reset_vector(address);
}

void nes_t::clock()
{
    cpu_.clock();
    ppu_.clock();
}

} // namespace NES