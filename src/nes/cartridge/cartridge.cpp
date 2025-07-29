#include "cartridge.h"
#include "mapper.h"
#include "mappers/nrom_mapper.h"

#include <array>
#include <fstream>

namespace NES {

cartridge_t::cartridge_t() :
    is_valid_(false),
    mapper_(),
    prg_rom_(),
    chr_rom_()
{
}

cartridge_t::cartridge_t(const std::string &filename)
    : is_valid_(false), mapper_()
{
    load(filename);
}

cartridge_t::~cartridge_t()
{
}

bool cartridge_t::load(const std::string &filename)
{
    std::ifstream cartridge_file(filename, std::ios::in);

    if (!cartridge_file.is_open())
    {
        is_valid_ = false;
        return is_valid_;
    }

    is_valid_ = true;

    cartridge_file.read((char *)&header_, sizeof(header_));

    bool is_trainer_present = header_.flags_6 & 0x04;
    if (is_trainer_present)
    {
        // Skip trainer data for now
        uint8_t *trainer_data[TRAINER_SIZE];
        cartridge_file.read((char *)trainer_data, TRAINER_SIZE);
    }

    mirroring_mode = (header_.flags_6 & 0x01) ? VERTICAL_MIRRORING : HORIZONTAL_MIRRORING;

    prg_rom_.resize(header_.prg_rom_banks * PRG_ROM_BANK_SIZE);
    cartridge_file.read((char *)prg_rom_.data(), prg_rom_.size());

    chr_rom_.resize(header_.chr_rom_banks * CHR_ROM_BANK_SIZE);
    if (header_.chr_rom_banks == 0) chr_rom_.resize(CHR_ROM_BANK_SIZE); // If no CHR ROM, allocate CHR_RAMs    
    cartridge_file.read((char *)chr_rom_.data(), chr_rom_.size());

    cartridge_file.close();

    uint16_t mapper_id = (header_.flags_6 >> 4) | (header_.flags_7 & 0xF0);
    switch (mapper_id)
    {
    case 0: // NROM
        mapper_ = std::make_shared<mapper_impl::nrom_mapper_t>(
            header_.prg_rom_banks, header_.chr_rom_banks);
        break;

    default:
        assert(false && "Mapper for this cartridge is not implemented yet");
        is_valid_ = false;
        break;
    }

    return is_valid_;
}

uint8_t cartridge_t::read_cpu(uint16_t addr) const
{
    assert(is_valid_ && mapper_ && "Cartridge is not valid");

    return prg_rom_[mapper_->map_cpu_read(addr)];
}

void cartridge_t::write_cpu(uint16_t addr, uint8_t data)
{
    assert(is_valid_ && mapper_ && "Cartridge is not valid");

    prg_rom_[mapper_->map_cpu_write(addr)] = data;
}

uint8_t cartridge_t::read_ppu(uint16_t addr) const
{
    assert(is_valid_ && "Cartridge is not valid");

    return chr_rom_[mapper_->map_ppu_read(addr)];
}

void cartridge_t::write_ppu(uint16_t addr, uint8_t data)
{
    assert(is_valid_ && "Cartridge is not valid");

    chr_rom_[mapper_->map_ppu_write(addr)] = data;
}

} // namespace NES