#include "ppu.h"
#include "bus.h"
#include "utils.h"
#include <algorithm>
#include <cassert>

namespace NES {

ppu_t::ppu_t(bus_ref_t ppu_bus) :
    ppu_bus_(ppu_bus),
    scanline_h_(), scanline_v_(),
    picture_(),
    external_regs_(),
    internal_regs_(),
    bg_next_info_(),
    shifters_(),
    read_buffer_(),
    nmi_needed_(),
    should_render_()
{
    for (int i = 0; i < PICTURE_WIDTH * PICTURE_HEIGHT; i++)
        picture_[i] = 0;
}

ppu_t::~ppu_t()
{
}

/*
 * Full details of what ppu does on each cycle is provided here:
 *
 * -------- https://www.nesdev.org/w/images/default/4/4f/Ppu.svg
 */
void ppu_t::clock()
{
    // first bit is skipped. FIXME: maybe need to skip on even frames
    if (scanline_v_ == 0 && scanline_h_ == 0)
    {
        scanline_h_++;
        should_render_ = false;
    }

    if (scanline_v_ < PICTURE_HEIGHT || scanline_v_ == SCANLINES_V)
    {
        if ((scanline_h_ > 0 && scanline_h_ < PICTURE_WIDTH) || ((scanline_h_ >= 321) & (scanline_h_ < 338)))
        {
            _update_shifters();

            switch ((scanline_h_ - 1) % 8)
            {
            case 0:
                _load_shifters();
                _fetch_next_tile_id();
                break;
            case 2: _fetch_next_attribute(); break;
            case 4: _fetch_next_lsbits(); break;
            case 6: _fetch_next_msbits(); break;
            case 7: _scroll_x(); break;
            }
        }

        if (scanline_h_ == PICTURE_WIDTH) _scroll_y();
        if (scanline_h_ == PICTURE_WIDTH + 1)
        {
            _load_shifters();
            _temp_to_active_x();
        }

        if (scanline_h_ == 339) _fetch_next_tile_id();

        if (scanline_v_ == SCANLINES_V && scanline_h_ >= 280 && scanline_h_ <= 304)
            _temp_to_active_y();

        // TODO: unused/ignored fetches
    }

    if (scanline_v_ == PICTURE_HEIGHT + 1 && scanline_h_ == 1)
    {
        utils::set_flag(external_regs_.status, status_flag::vblank, true);

        if (utils::get_flag(external_regs_.controller, controller_flag::generate_nmi))
            nmi_needed_ = true;
    }

    if (scanline_v_ == SCANLINES_V && scanline_h_ == 1) external_regs_.status = 0; // clear all the flags

    _update_pixel();

    scanline_h_++;

    if (scanline_h_ == SCANLINES_H)
    {
        scanline_h_ = 0;
        scanline_v_++;

        if (scanline_v_ > SCANLINES_V)
        {
            scanline_v_ = 0; // scanline 261 is pre-render scanline
            should_render_ = true;
        }
    }
}

bool ppu_t::get_nmi()
{
    return nmi_needed_;
}

void ppu_t::clear_nmi()
{
    nmi_needed_ = false;
}

void ppu_t::write(uint16_t addr, uint8_t data)
{
    uint8_t register_addr = addr % NUM_REGISTERS;

    switch (register_addr)
    {
        bool increment_mode; // cant skip over variable decleration in switch in cpp;

    case ppu_register_addr::controller:
        external_regs_.controller = data;

        internal_regs_.temp_reg.nametable_x =
            utils::get_flag(external_regs_.controller, controller_flag::nametable_x);

        internal_regs_.temp_reg.nametable_y =
            utils::get_flag(external_regs_.controller, controller_flag::nametable_y);
        break;

    case ppu_register_addr::mask:
        external_regs_.mask = data;
        break;

    case ppu_register_addr::oam_address:
        // TODO: implemet
        break;

    case ppu_register_addr::oam_data:
        // TODO: implement
        break;

    case ppu_register_addr::scroll:
        if (internal_regs_.write_toggle)
        {
            internal_regs_.temp_reg.coarse_y = (data >> 3) & 0x3F;
            internal_regs_.temp_reg.fine_y = data & 0x07;
        }
        else
        {
            internal_regs_.temp_reg.coarse_x = (data >> 3) & 0x3F;
            internal_regs_.fine_x = data & 0x07;
        }

        internal_regs_.write_toggle = !internal_regs_.write_toggle;
        break;

    case ppu_register_addr::address:
        internal_regs_.temp_reg.reg =
            internal_regs_.write_toggle ?
            ((internal_regs_.temp_reg.reg & 0xFF00) | data) :
            (((data << 8) & 0x3F00) | (internal_regs_.temp_reg.reg & 0x00FF));

        if (internal_regs_.write_toggle) internal_regs_.active_reg = internal_regs_.temp_reg;

        internal_regs_.write_toggle = !internal_regs_.write_toggle;
        break;

    case ppu_register_addr::data:
        ppu_bus_->write(internal_regs_.active_reg.reg, data);

        increment_mode = utils::get_flag(external_regs_.controller, controller_flag::vram_addr_increment);
        internal_regs_.active_reg.reg += increment_mode ? (PICTURE_WIDTH / 8) : 1;
        break;

    default:
        assert(false && "CPU trying to write to a read-only memory");
    }
}

uint8_t ppu_t::read(uint16_t addr)
{
    uint8_t register_addr = addr % NUM_REGISTERS;

    uint8_t return_data;
    switch (register_addr)
    {
        bool increment_mode; // cant skip over variable decleration in switch in cpp;

    case ppu_register_addr::status:
        return_data = external_regs_.status | (read_buffer_ & 0x1F); // unused satus gets filled with read_buffer value
        utils::set_flag(external_regs_.status, status_flag::vblank, false);

        internal_regs_.write_toggle = false;
        break;

    case ppu_register_addr::oam_address:
        // TODO: implemet
        break;

    case ppu_register_addr::oam_data:
        // TODO: implement
        break;

    case ppu_register_addr::data:
        // delayed address return (unless it is a pallete read)
        return_data = read_buffer_;
        read_buffer_ = ppu_bus_->read(internal_regs_.active_reg.reg);
        if (internal_regs_.active_reg.reg >= PALLETE_ADDR_START) return_data = read_buffer_; // updated read_buffer

        increment_mode = utils::get_flag(external_regs_.controller, controller_flag::vram_addr_increment);
        internal_regs_.active_reg.reg += increment_mode ? (PICTURE_WIDTH / 8) : 1; // 8 is tile width

        break;
    default:
        assert(false && "CPU trying to read a write-only memory");
    }

    return return_data;
}

uint8_t *ppu_t::get_picture()
{
    return should_render_ ? picture_ : nullptr;
}

void ppu_t::_fetch_next_tile_id()
{
    bg_next_info_.tile_id = ppu_bus_->read(0x2000 | (internal_regs_.active_reg.reg & 0x0FFF));
}

void ppu_t::_fetch_next_attribute()
{
    static constexpr uint16_t ATTIRBUTE_MEMORY_START = 0x23C0;

    const uint16_t nametable_addr = internal_regs_.active_reg.reg & 0x0C00;// nametable_x and nametable_y

    const uint16_t tile_pos = (internal_regs_.active_reg.coarse_x >> 2)
        | ((internal_regs_.active_reg.coarse_y >> 2) << 3);  // last two bits give sections in a byte

    uint8_t attribute_byte = ppu_bus_->read(ATTIRBUTE_MEMORY_START | nametable_addr | tile_pos); // 0xaabbccdd

    // we need to isolate the subtile value
    const uint8_t quarter_id = (internal_regs_.active_reg.coarse_y & 0x02)
        | ((internal_regs_.active_reg.coarse_x & 0x02) >> 1);

    const uint8_t quarter_mask = 0xFF << (quarter_id * 2);
    bg_next_info_.attribute = (attribute_byte & quarter_mask) >> (quarter_id * 2);
}

void ppu_t::_fetch_next_lsbits()
{
    const bool is_background = utils::get_flag(external_regs_.controller, controller_flag::bg_pattern_addr);
    const uint16_t pattern_addr = is_background ? 0x1000 : 0x0000;

    const uint16_t lsbyte_plane_addr = (((uint16_t)bg_next_info_.tile_id << 4) | internal_regs_.active_reg.fine_y);
    bg_next_info_.bg_lsbits = ppu_bus_->read(pattern_addr + lsbyte_plane_addr);
}

void ppu_t::_fetch_next_msbits()
{
    const bool is_background = utils::get_flag(external_regs_.controller, controller_flag::bg_pattern_addr);
    const uint16_t pattern_addr = is_background ? 0x1000 : 0x0000;

    const uint16_t lsbyte_plane_addr = (((uint16_t)bg_next_info_.tile_id << 4) | internal_regs_.active_reg.fine_y);
    bg_next_info_.bg_msbits = ppu_bus_->read(pattern_addr + lsbyte_plane_addr + 0x0008);
}

// scroll functions are implemented based on wiki pseudo-code: https://www.nesdev.org/wiki/PPU_scrolling
void ppu_t::_scroll_x()
{
    if (!utils::get_flag(external_regs_.mask, mask_flag::render_background) &&
        !utils::get_flag(external_regs_.mask, mask_flag::render_sprites))
        return;

    if (internal_regs_.active_reg.coarse_x == 31)
    {
        internal_regs_.active_reg.coarse_x = 0;
        internal_regs_.active_reg.nametable_x = ~internal_regs_.active_reg.nametable_x;
    }
    else internal_regs_.active_reg.coarse_x++;
}

void ppu_t::_scroll_y()
{
    if (!utils::get_flag(external_regs_.mask, mask_flag::render_background) &&
        !utils::get_flag(external_regs_.mask, mask_flag::render_sprites))
        return;

    if (internal_regs_.active_reg.fine_y < 7)
    {
        internal_regs_.active_reg.fine_y++;
        return;
    }

    internal_regs_.active_reg.fine_y = 0;
    if (internal_regs_.active_reg.coarse_y == 29)
    {
        internal_regs_.active_reg.coarse_y = 0;
        internal_regs_.active_reg.nametable_y = ~internal_regs_.active_reg.nametable_y;
    }
    else if (internal_regs_.active_reg.coarse_y == 31) internal_regs_.active_reg.coarse_y = 0;
    else internal_regs_.active_reg.coarse_y++;
}

void ppu_t::_temp_to_active_x()
{
    if (!utils::get_flag(external_regs_.mask, mask_flag::render_background) &&
        !utils::get_flag(external_regs_.mask, mask_flag::render_sprites))
        return;

    internal_regs_.active_reg.coarse_x = internal_regs_.temp_reg.coarse_x;
    internal_regs_.active_reg.nametable_x = internal_regs_.temp_reg.nametable_x;
}

void ppu_t::_temp_to_active_y()
{
    if (!utils::get_flag(external_regs_.mask, mask_flag::render_background) &&
        !utils::get_flag(external_regs_.mask, mask_flag::render_sprites))
        return;

    internal_regs_.active_reg.coarse_y = internal_regs_.temp_reg.coarse_y;
    internal_regs_.active_reg.fine_y = internal_regs_.temp_reg.fine_y;
    internal_regs_.active_reg.nametable_y = internal_regs_.temp_reg.nametable_y;
}

void ppu_t::_load_shifters()
{
    shifters_.pattern_lsbits = (shifters_.pattern_lsbits & 0xFF00) | bg_next_info_.bg_lsbits;
    shifters_.pattern_msbits = (shifters_.pattern_msbits & 0xFF00) | bg_next_info_.bg_msbits;
    shifters_.attrib_lsbits = (shifters_.attrib_lsbits & 0xFF00) | ((bg_next_info_.attribute & 0x0001) ? 0xFF : 0x00);
    shifters_.attrib_msbits = (shifters_.attrib_msbits & 0xFF00) | ((bg_next_info_.attribute & 0x0002) ? 0xFF : 0x00);
}

void ppu_t::_update_shifters()
{
    if (!utils::get_flag(external_regs_.mask, mask_flag::render_background)) return;

    shifters_.attrib_lsbits <<= 1;
    shifters_.attrib_msbits <<= 1;
    shifters_.pattern_lsbits <<= 1;
    shifters_.pattern_msbits <<= 1;
}

void ppu_t::_update_pixel()
{
    if (scanline_h_ == 0) return; // scanline_h = 0 is idle
    if (scanline_v_ >= PICTURE_HEIGHT || scanline_h_ > PICTURE_WIDTH) return;

    uint16_t color_address = 0x3F00; // palette ram bg address

    if (utils::get_flag(external_regs_.mask, mask_flag::render_background))
    {
        uint16_t bit_mask = 0x8000 >> internal_regs_.fine_x;

        uint8_t lsb_pixel = (shifters_.pattern_lsbits & bit_mask) > 0;
        uint8_t msb_pixel = (shifters_.pattern_msbits & bit_mask) > 0;
        uint8_t pixel = (msb_pixel << 1) | lsb_pixel;

        uint8_t lsb_palette = (shifters_.attrib_lsbits & bit_mask) > 0;
        uint8_t msb_palette = (shifters_.attrib_msbits & bit_mask) > 0;
        uint8_t palette = (msb_palette << 1) | lsb_palette;

        color_address += (palette << 2) + pixel;
    }

    uint8_t color = ppu_bus_->read(color_address);
    picture_[scanline_v_ * PICTURE_WIDTH + (scanline_h_ - 1)] = color;
}

}