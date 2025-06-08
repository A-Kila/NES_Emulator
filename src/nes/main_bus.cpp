#include "main_bus.h"
#include "ram.h"
#include "utils.h"
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
    assert(cartridge_ && "Cartridge must be initialized before writing to the bus");

    if (utils::is_addr_in_range(addr, RAM_ADDR_RANGE)) ram_.write(addr, data);
    else if (utils::is_addr_in_range(addr, PPU_ADDR_RANGE)); // TODO: Fill after PPU implementation
    else if (utils::is_addr_in_range(addr, APU_IO_ADDR_RANGE)); // TODO: Fill after APU/IO implementation
    else if (utils::is_addr_in_range(addr, CARTRIDGE_ADDR_RANGE)) cartridge_->write_cpu(addr, data);
    else assert(false && "Address out of range in main bus write operation");
}

uint8_t main_bus_t::read(uint16_t addr)
{
    assert(cartridge_ && "Cartridge must be initialized before reading from the bus");

    if (utils::is_addr_in_range(addr, RAM_ADDR_RANGE)) return ram_.read(addr);
    else if (utils::is_addr_in_range(addr, PPU_ADDR_RANGE)) return 0; // TODO: Fill after PPU implementation
    else if (utils::is_addr_in_range(addr, APU_IO_ADDR_RANGE)) return 0xFF; // TODO: Fill after APU/IO implementation
    else if (utils::is_addr_in_range(addr, CARTRIDGE_ADDR_RANGE)) return cartridge_->read_cpu(addr);

    assert(false && "Address out of range in main bus read operation");
    return 0;
}

} // namespace NES
