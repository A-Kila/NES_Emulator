#pragma once

#include <utility>
#include "bus.h" 
#include "ram.h"
#include "cartridge/cartridge.h"

namespace NES {

class main_bus_t : public i_bus_t
{
public:
    main_bus_t(cartridge_ref_t cartridge);
    virtual ~main_bus_t();

    void write(uint16_t addr, uint8_t data) override;
    uint8_t read(uint16_t addr) override;

private:
    static constexpr auto RAM_ADDR_RANGE = std::make_pair(0x0000, 0x1FFF);
    static constexpr auto PPU_ADDR_RANGE = std::make_pair(0x2000, 0x3FFF);
    static constexpr auto APU_IO_ADDR_RANGE = std::make_pair(0x4000, 0x401F);
    static constexpr auto CARTRIDGE_ADDR_RANGE = std::make_pair(0x4020, 0xFFFF);

private:
    ram_t ram_;
    cartridge_ref_t cartridge_;
};

} // namespace NES