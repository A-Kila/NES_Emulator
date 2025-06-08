#include "ram.h"
#include <cassert>

ram_t::ram_t()
{
    // Ram is initilized to 0x00
    for (uint16_t i = 0; i < RAM_SIZE; i++)
        memory[i] = 0x00;
}

ram_t::~ram_t()
{
}

uint8_t ram_t::read(const uint16_t addr) const
{
    assert(addr <= RAM_SIZE_MIRRORS && "Address out of range in RAM read operation");

    return memory[addr % RAM_SIZE];
}

void ram_t::write(const uint16_t addr, const uint8_t data)
{
    assert(addr <= RAM_SIZE_MIRRORS && "Address out of range in RAM write operation");

    memory[addr % RAM_SIZE] = data;
}
