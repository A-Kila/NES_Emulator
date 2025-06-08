#pragma once

#include <memory>
#include <vector>
#include "mapper.h"

namespace NES {

class cartridge_t
{
public:
    cartridge_t();
    cartridge_t(const std::string &filename);
    ~cartridge_t();

    bool load(const std::string &filename);
    bool is_valid() const { return is_valid_; }

    uint8_t read_cpu(uint16_t addr) const;
    void write_cpu(uint16_t addr, uint8_t data);
    uint8_t read_ppu(uint16_t addr) const;
    void write_ppu(uint16_t addr, uint8_t data);

public: // FIXME: needs to be private, just a temp
    bool is_valid_;

    mapper_ref_t mapper_;
    std::vector<uint8_t> prg_rom_; // ROM for the CPU
    std::vector<uint8_t> chr_rom_; // ROM used by the PPU

    struct header_t
    {
        uint8_t head[4]; // "NES\x1A"
        uint8_t prg_rom_banks; // Number of 16KB PRG-ROM banks
        uint8_t chr_rom_banks; // Number of 8KB CHR-ROM banks
        uint8_t flags_6; // Mapper and mirroring information
        uint8_t flags_7; // Mapper, VS Unisystem, and PlayChoice-10 information
        uint8_t flags_8; // Size of PRG RAM in 8KB units
        uint8_t flags_9; // TV system
        uint8_t flags_10; // TV system and PRG RAM presence
        uint8_t unused[5]; // Unused bytes
    } header_;

public:
    // Mapper needs this information as well
    static constexpr uint16_t PRG_ROM_BANK_SIZE = 0x4000; // 16KB
    static constexpr uint16_t CHR_ROM_BANK_SIZE = 0x2000; // 8KB

private:
    static constexpr uint16_t TRAINER_SIZE = 0x200; // 512 bytes
};

typedef std::shared_ptr<cartridge_t> cartridge_ref_t;

} // namespace NES