#pragma once

#include <memory>
#include "bus.h"

namespace NES {

class ppu_t
{
public:
    ppu_t(bus_ref_t ppu_bus);
    ~ppu_t();

    void clock();
    bool get_nmi();
    void clear_nmi();

    void write(uint16_t addr, uint8_t data);
    uint8_t read(uint16_t addr);

    uint8_t *get_picture();

public:
    static constexpr uint16_t PICTURE_WIDTH = 256;
    static constexpr uint16_t PICTURE_HEIGHT = 240;

private:
    void _fetch_next_tile_id();
    void _fetch_next_attribute();
    void _fetch_next_lsbits();
    void _fetch_next_msbits();

    void _scroll_x();
    void _scroll_y();
    void _temp_to_active_x();
    void _temp_to_active_y();

    void _load_shifters();
    void _update_shifters();

    void _update_pixel();

private:
    enum ppu_register_addr
    {
        controller = 0,
        mask,
        status,
        oam_address,
        oam_data,
        scroll,
        address,
        data,
        oam_dma = -1, // Special register with different address // FIXME: no idea yet
    };
    static constexpr uint8_t NUM_REGISTERS = ppu_register_addr::data + 1;

    struct
    {
        uint8_t controller;
        uint8_t mask;
        uint8_t status;
        uint16_t oam_address;
        uint8_t scroll;
        uint16_t address;
    } external_regs_;

    enum controller_flag
    {
        nametable_x = (1 << 0), // 0 = 0x2000, 1 = 0x2400, 2 = 0x2800, 3 = 0x2C00
        nametable_y = (1 << 1),
        vram_addr_increment = (1 << 2), // 0: add 1, going across; 1: add 32, going down
        sprite_pattern_addr = (1 << 3), // 0: $0000; 1: $1000; ignored in 8x16 mode
        bg_pattern_addr = (1 << 4), // 0: $0000; 1: $1000
        sprite_size = (1 << 5), // 0: 8x8 pixels; 1: 8x16 pixels
        master_slave_select = (1 << 6), // 0: read backdrop from EXT pins; 1: output color on EXT pins
        generate_nmi = (1 << 7) // 0: off, 1: on
    };

    enum mask_flag
    {
        grayscale = (1 << 0),
        show_background_left = (1 << 1),
        show_sprite_left = (1 << 2),
        render_background = (1 << 3),
        render_sprites = (1 << 4),
        red_emphasize = (1 << 5),
        green_emphasize = (1 << 6),
        blue_emphasize = (1 << 7)
    };

    enum status_flag
    {
        // first 5 bytes are unused
        sprite_overflow = (1 << 5),
        sprite_0_hit = (1 << 6),
        vblank = (1 << 7) // documentation says !!!UNRELIABLE!!!
    };
    struct
    {
        union
        {
            struct
            {
                uint16_t coarse_x : 5;
                uint16_t coarse_y : 5;
                uint16_t nametable_x : 1;
                uint16_t nametable_y : 1;
                uint16_t fine_y : 3;
                uint16_t unused : 1;
            };

            uint16_t reg;
        } temp_reg, active_reg;

        uint8_t fine_x;
        bool write_toggle;
    } internal_regs_;

    struct
    {
        uint8_t tile_id;
        uint8_t attribute;
        uint8_t bg_lsbits;
        uint8_t bg_msbits;
    } bg_next_info_;

    struct
    {
        uint16_t attrib_lsbits;
        uint16_t attrib_msbits;
        uint16_t pattern_lsbits;
        uint16_t pattern_msbits;
    } shifters_;


    uint8_t read_buffer_; // PPUDATA read is a delayed operation, it returns data from internal buffer updating it
    bool nmi_needed_;
    bool should_render_;

private:
    bus_ref_t ppu_bus_;

    uint16_t scanline_h_, scanline_v_;
    uint8_t picture_[PICTURE_WIDTH * PICTURE_HEIGHT];

private:
    static constexpr uint16_t SCANLINES_H = 341;
    static constexpr uint16_t SCANLINES_V = 261;

    static constexpr uint16_t PALLETE_ADDR_START = 0x3F00;

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

typedef std::shared_ptr<ppu_t> ppu_ref_t;

} // namespace NES