#pragma once

#include <memory>
#include "../definitions.h"

namespace NES {

INTERFACE i_mapper_t
{
    virtual uint16_t map_cpu_read(uint16_t addr) const = 0;
    virtual uint16_t map_cpu_write(uint16_t addr) const = 0;
    virtual uint16_t map_ppu_read(uint16_t addr) const = 0;
    virtual uint16_t map_ppu_write(uint16_t addr) const = 0;
};

class base_mapper_t : public i_mapper_t
{
protected:
    base_mapper_t(uint16_t num_prg_banks, uint16_t num_chr_banks)
        : num_prg_banks_(num_prg_banks), num_chr_banks_(num_chr_banks)
    {
    }
    virtual ~base_mapper_t() = default;

protected:
    const uint16_t num_prg_banks_; // Number of 16KB PRG-ROM banks
    const uint16_t num_chr_banks_; // Number of 8KB CHR-ROM banks
};

typedef std::shared_ptr<i_mapper_t> mapper_ref_t;

} // namespace NES