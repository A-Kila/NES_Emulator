#include "nrom_mapper.h"
#include "../../utils.h"
#include "../cartridge.h"
#include <cassert>

namespace NES {
namespace mapper_impl {

nrom_mapper_t::nrom_mapper_t(uint16_t num_pgr_banks, uint16_t num_chr_banks) :
    base_mapper_t(num_pgr_banks, num_chr_banks)
{
    assert(num_prg_banks_ && num_prg_banks_ <= 2 && "NROM supports only 1 or 2 PRG banks");
    assert(num_chr_banks_ == 1 && "NROM supports only 1 CHR bank");
}

nrom_mapper_t::~nrom_mapper_t()
{
}

uint16_t nrom_mapper_t::map_cpu_read(uint16_t addr) const
{
    return _map_addr(
        addr,
        cartridge_t::PRG_ROM_BANK_SIZE,
        num_prg_banks_,
        PRG_ADDRESS_RANGE
    );
}

uint16_t nrom_mapper_t::map_cpu_write(uint16_t addr) const
{
    return _map_addr(
        addr,
        cartridge_t::PRG_ROM_BANK_SIZE,
        num_prg_banks_,
        PRG_ADDRESS_RANGE
    );
}

uint16_t nrom_mapper_t::map_ppu_read(uint16_t addr) const
{
    return _map_addr(
        addr,
        cartridge_t::CHR_ROM_BANK_SIZE,
        num_chr_banks_,
        CHR_ADDRESS_RANGE
    );
}

uint16_t nrom_mapper_t::map_ppu_write(uint16_t addr) const
{
    return _map_addr(
        addr,
        cartridge_t::CHR_ROM_BANK_SIZE,
        num_chr_banks_,
        CHR_ADDRESS_RANGE
    );
}

uint16_t nrom_mapper_t::_map_addr(
    const uint16_t addr,
    const uint16_t bank_size,
    const uint16_t num_banks,
    const std::pair<uint16_t, uint16_t> &address_range
) const
{
    assert(utils::is_addr_in_range(addr, address_range) && "Address out of range for mapping");

    return addr & (bank_size * num_banks - 1);
}


} // namespace mapper_impl
} // namespace NES