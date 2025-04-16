#include "bus.h"

namespace NES {

bus_t::bus_t()
{
    for (uint32_t i = 0; i < RAM_SIZE; i++)
        ram_[i] = 0x00;
}

bus_t::~bus_t()
{
}

void bus_t::write(uint16_t addr, uint8_t data)
{
    ram_[addr] = data;
}

uint8_t bus_t::read(uint16_t addr)
{
    return ram_[addr];
}

} // namespace NES
