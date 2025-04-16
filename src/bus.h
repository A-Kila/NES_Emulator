#pragma once

#include <array>
#include <memory>

namespace NES {

class bus_t
{
public:
    bus_t();
    ~bus_t();

    void write(uint16_t addr, uint8_t data);
    uint8_t read(uint16_t addr);

private:
    static const uint32_t RAM_SIZE = 64 * 1024;

private:
    uint8_t ram_[RAM_SIZE]; // 64KB of RAM

};

typedef std::shared_ptr<bus_t> bus_ref_t;

} // namespace NES