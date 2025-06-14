#pragma once

#include <utility>
#include "bus.h" 
#include "ram.h"
#include "cartridge/cartridge.h"
#include "ppu.h"
#include "joypad.h"

namespace NES {

class main_bus_t : public i_bus_t
{
public:
    main_bus_t(ppu_ref_t ppu, cartridge_ref_t cartridge, joypad_ref_t joypad);
    virtual ~main_bus_t();

    void write(uint16_t addr, uint8_t data) override;
    uint8_t read(uint16_t addr) override;

private:
    static constexpr auto RAM_ADDR_RANGE = std::make_pair(0x0000, 0x1FFF);
    static constexpr auto PPU_ADDR_RANGE = std::make_pair(0x2000, 0x3FFF);
    static constexpr auto APU_ADDR_RANGE = std::make_pair(0x4000, 0x4015);
    static constexpr auto IO_ADDR_RANGE = std::make_pair(0x4016, 0x4017);
    static constexpr auto TEST_ADDR_RANGE = std::make_pair(0x4018, 0x401F);
    static constexpr auto CARTRIDGE_ADDR_RANGE = std::make_pair(0x4020, 0xFFFF);

private:
    ram_t ram_;
    ppu_ref_t ppu_;
    cartridge_ref_t cartridge_;
    joypad_ref_t joypad_;
};

} // namespace NES