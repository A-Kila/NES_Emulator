#pragma once

#include "bus.h" 

namespace NES {

class nes_bus_t : public i_bus_t
{
public:
    nes_bus_t();
    virtual ~nes_bus_t();

    void write(uint16_t addr, uint8_t data) override;
    uint8_t read(uint16_t addr) override;

private:
    static constexpr uint32_t RAM_SIZE = 64 * 1024;

private:
    uint8_t ram_[RAM_SIZE]; // 64KB of RAM
};

} // namespace NES