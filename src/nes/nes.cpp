#include "nes.h"

#include "../SDL_tools/sdl_graphics.h"
#include "../SDL_tools/sdl_events.h"
#include "../SDL_tools/sdl_joypad.h"
#include "ppu.h"

namespace NES {

const uint32_t TEMP_WIDTH = 320;
const uint32_t TEMP_HEIGHT = 240;

nes_t::nes_t(const std::string &filename) :
    graphics_(std::make_shared<SDL::sdl_graphics_t>(ppu_t::PICTURE_WIDTH, ppu_t::PICTURE_HEIGHT)),
    events_(std::make_shared<SDL::sdl_events_t>()),
    joypad_(std::make_shared<SDL::sdl_joypad_t>()),
    cartridge_(std::make_shared<cartridge_t>(filename)),
    ppu_bus_(std::make_shared<ppu_bus_t>(cartridge_)),
    ppu_(std::make_shared<ppu_t>(ppu_bus_)),
    main_bus_(std::make_shared<main_bus_t>(ppu_, cartridge_, joypad_)),
    cpu_(main_bus_)
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

void nes_t::run()
{
    uint32_t screen[ppu_t::PICTURE_WIDTH * ppu_t::PICTURE_HEIGHT];

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

            case nes_event::KEY_CHANGED:
                joypad_->update_keys();

            default:
                break;
            }
        }

        _clock();

        uint8_t *pixel_data = ppu_->get_picture();
        if (pixel_data)
        {
            _get_screen_argb(screen, pixel_data);
            graphics_->update_frame(screen);
        }
    }
}

void nes_t::set_reset_vector(uint16_t address)
{
    cpu_.set_reset_vector(address);
}

void nes_t::_clock()
{
    for (int i = 0; i < 3; i++) ppu_->clock(); // PPU runs 3x faster than CPU

    cpu_.clock();

    if (ppu_->get_nmi())
    {
        ppu_->clear_nmi();
        cpu_.nonmaskable_interrupt();
    }
}

void nes_t::_get_screen_argb(uint32_t *screen, uint8_t *pixels)
{
    static constexpr uint32_t pallete[] = {
        make_argb(84,  84,  84),    make_argb(0,  30, 116),    make_argb(8,  16, 144),    make_argb(48,   0, 136),   make_argb(68,   0, 100),   make_argb(92,   0,  48),   make_argb(84,   4,   0),   make_argb(60,  24,   0),   make_argb(32,  42,   0),   make_argb(8,  58,   0),   make_argb(0,  64,   0),    make_argb(0,  60,   0),    make_argb(0,  50,  60),    make_argb(0,   0,   0),   make_argb(0, 0, 0), make_argb(0, 0, 0),
        make_argb(152, 150, 152),   make_argb(8,  76, 196),    make_argb(48,  50, 236),   make_argb(92,  30, 228),   make_argb(136,  20, 176),  make_argb(160,  20, 100),  make_argb(152,  34,  32),  make_argb(120,  60,   0),  make_argb(84,  90,   0),   make_argb(40, 114,   0),  make_argb(8, 124,   0),    make_argb(0, 118,  40),    make_argb(0, 102, 120),    make_argb(0,   0,   0),   make_argb(0, 0, 0), make_argb(0, 0, 0),
        make_argb(236, 238, 236),   make_argb(76, 154, 236),   make_argb(120, 124, 236),  make_argb(176,  98, 236),  make_argb(228,  84, 236),  make_argb(236,  88, 180),  make_argb(236, 106, 100),  make_argb(212, 136,  32),  make_argb(160, 170,   0),  make_argb(116, 196,   0), make_argb(76, 208,  32),   make_argb(56, 204, 108),   make_argb(56, 180, 204),   make_argb(60,  60,  60),  make_argb(0, 0, 0), make_argb(0, 0, 0),
        make_argb(236, 238, 236),   make_argb(168, 204, 236),  make_argb(188, 188, 236),  make_argb(212, 178, 236),  make_argb(236, 174, 236),  make_argb(236, 174, 212),  make_argb(236, 180, 176),  make_argb(228, 196, 144),  make_argb(204, 210, 120),  make_argb(180, 222, 120), make_argb(168, 226, 144),  make_argb(152, 226, 180),  make_argb(160, 214, 228),  make_argb(160, 162, 160), make_argb(0, 0, 0), make_argb(0, 0, 0)
    };

    for (uint32_t i = 0; i < ppu_t::PICTURE_HEIGHT; i++)
        for (uint32_t j = 0; j < ppu_t::PICTURE_WIDTH; j++)
        {
            const uint32_t index = i * ppu_t::PICTURE_WIDTH + j;
            screen[index] = pallete[pixels[index]];
        }
}


} // namespace NES