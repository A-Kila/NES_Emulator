#pragma once

#include <utility>
#include "../mapper.h"

namespace NES {
namespace mapper_impl {

class nrom_mapper_t : public base_mapper_t
{
public:
    nrom_mapper_t(uint16_t num_pgr_banks = 1, uint16_t num_chr_banks = 1);
    ~nrom_mapper_t() override;

    uint16_t map_cpu_read(uint16_t addr) const override;
    uint16_t map_cpu_write(uint16_t addr) const override;
    uint16_t map_ppu_read(uint16_t addr) const override;
    uint16_t map_ppu_write(uint16_t addr) const override;

private:
    uint16_t _map_addr(
        const uint16_t addr,
        const uint16_t bank_size,
        const uint16_t num_banks,
        const std::pair<uint16_t, uint16_t> &address_range
    ) const;

private:
    static constexpr auto PRG_ADDRESS_RANGE = std::make_pair(0x8000, 0xFFFF);
    static constexpr auto CHR_ADDRESS_RANGE = std::make_pair(0x0000, 0x1FFF);
};

} // namespace mapper_impl
} // namespace NES