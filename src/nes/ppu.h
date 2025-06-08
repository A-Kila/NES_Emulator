#pragma once

#include "main_bus.h"

namespace NES {

class ppu_t
{
public:
    ppu_t();
    ~ppu_t();

    void reset();
    void clock();

    uint32_t *get_picture();

public:
    static constexpr uint16_t PICTURE_WIDTH = 320;
    static constexpr uint16_t PICTURE_HEIGHT = 240;

private:
    bus_ref_t main_bus;

    uint32_t picture_[PICTURE_WIDTH * PICTURE_HEIGHT];

private:
    static constexpr uint8_t SCAN_PADDING = 21;

    // encoded in ARGB, alpha is always maximum
    static constexpr uint32_t PALLETE[] = {
        // 0x0x
        0xFF626262, 0xFF001FB2, 0xFF2404C8, 0xFF5200B2, 0xFF730076, 0xFF800024, 0xFF730B00, 0xFF522800,
        0xFF244400, 0xFF005700, 0xFF005C00, 0xFF005324, 0xFF003C76, 0xFF000000, 0xFF000000, 0xFF000000,
        // 0x1x
        0xFFABABAB, 0xFF0D57FF, 0xFF4B30FF, 0xFF8A14FF, 0xFFBC08D6, 0xFFD21269, 0xFFC72E00, 0xFF9D5400,
        0xFF607B00, 0xFF209800, 0xFF00A300, 0xFF009942, 0xFF007DB4, 0xFF000000, 0xFF000000, 0xFF000000,
        // 0x2x
        0xFFFFFFFF, 0xFF53AEFF, 0xFF9085FF, 0xFFD365FF, 0xFFFF57FF, 0xFFFF5DCF, 0xFFFF7757, 0xFFFA9E00,
        0xFFBdC700, 0xFF7AE700, 0xFF43F611, 0xFF26EF7E, 0xFF2CD5F6, 0xFF4E4E4E, 0xFF000000, 0xFF000000,
        // 0x3x
        0xFFFFFFFF, 0xFFB6E1FF, 0xFFCeD1FF, 0xFFE9C3FF, 0xFFFFBCFF, 0xFFFFBdF4, 0xFFFFC6C3, 0xFFFFD59A,
        0xFFE9E681, 0xFFCEF481, 0xFFB6FB9A, 0xFFA9FAC3, 0xFFA9F0F4, 0xFFB8B8B8, 0xFF000000, 0xFF000000,
    };
};

} // namespace NES