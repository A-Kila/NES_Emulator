#pragma once

#include <utility>
#include "bus.h"
#include "cartridge/cartridge.h"

namespace NES {

class ppu_bus_t : public i_bus_t
{
public:
    ppu_bus_t(cartridge_ref_t cartridge);
    ~ppu_bus_t();

    void write(const uint16_t addr, const uint8_t data) override;
    uint8_t read(const uint16_t addr) override;

private:
    void _nametable_write(const uint16_t addr, const uint8_t data);
    uint8_t _nametable_read(const uint16_t addr);
    uint16_t _get_nametable_index(const uint16_t addr);

private:
    static constexpr auto CHR_ROM_ADDR_RANGE = std::make_pair(0x0000, 0x1FFF);
    static constexpr auto NAMETABLES_ADDR_RANGE = std::make_pair(0x2000, 0x3EFF);
    static constexpr auto PALETTE_RAM_ADDR_RANGE = std::make_pair(0x3F00, 0x3FFF);
    static constexpr uint16_t PALETTE_RAM_MIRROR_MASK = 0x001F;
    static constexpr uint16_t NAMETABLE_MIRROR_MASK = 0x03FF;
    static constexpr uint16_t NAMETABLE_SIZE = 0x0400;

private:
    cartridge_ref_t cartridge_;

    uint8_t palette_ram[PALETTE_RAM_MIRROR_MASK + 1];
    uint8_t nametables[2][NAMETABLE_SIZE];
};

} // namespace NES