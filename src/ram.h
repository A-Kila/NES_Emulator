#pragma once

#include <stdint.h>

class ram_t
{
public:
    ram_t();
    ~ram_t();

    uint8_t read(const uint16_t addr) const;
    void write(const uint16_t addr, const uint8_t data);

private:
    static constexpr uint16_t RAM_SIZE = 0x800; // 2KB of RAM
    static constexpr uint16_t RAM_SIZE_MIRRORS = 0x2000; // 8KB of RAM with mirroring

    uint8_t memory[RAM_SIZE]; // Array to hold the RAM data
};
