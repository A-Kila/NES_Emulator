#pragma once

#include <vector>
#include "bus.h"

namespace NES {

class ppu_bus_t : public i_bus_t
{
public:
    ppu_bus_t();
    ~ppu_bus_t();

    void write(uint16_t addr, uint8_t data) override;
    uint8_t read(uint16_t addr) override;

private:
    std::vector<uint8_t> chr_rom_;

private:
    static constexpr uint16_t CHR_ROM_SIZE = 0x2000;
    static constexpr uint16_t NAMETABLES_SIZE = 0x1000;
    static constexpr uint16_t PALETTE_RAM_SIZE = 0x0020;
};

} // namespace NES