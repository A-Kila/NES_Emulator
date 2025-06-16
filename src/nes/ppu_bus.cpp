#include "ppu_bus.h"
#include "cartridge/cartridge.h"
#include "utils.h"
#include <cassert>

namespace NES {

ppu_bus_t::ppu_bus_t(cartridge_ref_t cartridge) :
    cartridge_(cartridge),
    palette_ram_(),
    nametables_()
{
}

ppu_bus_t::~ppu_bus_t()
{
}

void ppu_bus_t::write(const uint16_t addr, const uint8_t data)
{
    assert(cartridge_ && "Cartridge must be initialized before writing to the bus");

    auto masked_addr = addr & PALETTE_RAM_ADDR_RANGE.second;

    if (utils::is_addr_in_range(masked_addr, CHR_ROM_ADDR_RANGE)) cartridge_->write_ppu(masked_addr, data);
    else if (utils::is_addr_in_range(masked_addr, NAMETABLES_ADDR_RANGE)) _nametable_write(masked_addr, data);
    else if (utils::is_addr_in_range(masked_addr, PALETTE_RAM_ADDR_RANGE))
    {
        masked_addr &= PALETTE_RAM_MIRROR_MASK;
        const uint16_t palette_addr = ((masked_addr & 0x03) == 0) ? masked_addr & 0x0F : masked_addr;
        palette_ram_[palette_addr] = data;
    }
}

uint8_t ppu_bus_t::read(const uint16_t addr)
{
    assert(cartridge_ && "Cartridge must be initialized before writing to the bus");

    auto masked_addr = addr & PALETTE_RAM_ADDR_RANGE.second;

    if (utils::is_addr_in_range(masked_addr, CHR_ROM_ADDR_RANGE)) return cartridge_->read_ppu(masked_addr);
    else if (utils::is_addr_in_range(masked_addr, NAMETABLES_ADDR_RANGE)) return _nametable_read(masked_addr);
    else if (utils::is_addr_in_range(masked_addr, PALETTE_RAM_ADDR_RANGE))
    {
        masked_addr &= PALETTE_RAM_MIRROR_MASK;
        const uint16_t palette_addr = ((masked_addr & 0x03) == 0) ? masked_addr & 0x0F : masked_addr;
        return palette_ram_[palette_addr];
    }

    return 0;
}

void ppu_bus_t::_nametable_write(const uint16_t addr, const uint8_t data)
{
    const uint16_t addr_in_nametable = addr & NAMETABLE_MIRROR_MASK;
    uint16_t nametable_index = _get_nametable_index(addr);

    nametables_[nametable_index][addr_in_nametable] = data;
}

uint8_t ppu_bus_t::_nametable_read(const uint16_t addr)
{
    const uint16_t addr_in_nametable = addr & NAMETABLE_MIRROR_MASK;
    uint16_t nametable_index = _get_nametable_index(addr);

    return nametables_[nametable_index][addr_in_nametable];
}

uint16_t ppu_bus_t::_get_nametable_index(const uint16_t addr)
{
    const uint16_t addr_vram = addr - NAMETABLES_ADDR_RANGE.first; // 0 - 2000
    const uint8_t nametable_id = (addr_vram & 0x0FFF) / NAMETABLE_SIZE; // 0 - 3

    switch (cartridge_->mirroring_mode)
    {
    case cartridge_t::mirroring_mode::HORIZONTAL_MIRRORING:
        return nametable_id / 2; // nt[0] = nt[1]; nt[2] = nt[3]

    case cartridge_t::mirroring_mode::VERTICAL_MIRRORING:
        return nametable_id % 2; // nt[0] = nt[2]; nt[1] = nt[3]
    }

    assert(false && "Mirroring mode not supported");
    return -1;
}

} // namespace NES