#include "main_bus.h"
#include "ram.h"
#include <cassert>

namespace NES {

main_bus_t::main_bus_t(cartridge_ref_t cartridge) :
    ram_(),
    cartridge_(cartridge)
{
}

main_bus_t::~main_bus_t()
{
}

void main_bus_t::write(uint16_t addr, uint8_t data)
{
    if (_in_addr_range(addr, RAM_ADDR_RANGE)) ram_.write(addr, data);
    else if (_in_addr_range(addr, PPU_ADDR_RANGE)); // TODO: Fill after PPU implementation
    else if (_in_addr_range(addr, APU_IO_ADDR_RANGE)); // TODO: Fill after APU/IO implementation
    else if (_in_addr_range(addr, CARTRIDGE_ADDR_RANGE)); // TODO: Fill after cartridge implementation
    else assert(false && "Address out of range in main bus write operation");
}

uint8_t main_bus_t::read(uint16_t addr)
{
    if (_in_addr_range(addr, RAM_ADDR_RANGE)) return ram_.read(addr);
    else if (_in_addr_range(addr, PPU_ADDR_RANGE)); // TODO: Fill after PPU implementation
    else if (_in_addr_range(addr, APU_IO_ADDR_RANGE)); // TODO: Fill after APU/IO implementation
    else if (_in_addr_range(addr, CARTRIDGE_ADDR_RANGE)); // TODO: Fill after cartridge implementation 

    assert(false && "Address out of range in main bus read operation");
    return 0;
}

bool main_bus_t::_in_addr_range(const uint16_t addr, const std::pair<uint16_t, uint16_t> &range) const
{
    return addr >= range.first && addr <= range.second;
}

} // namespace NES
