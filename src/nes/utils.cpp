#include "utils.h"

namespace NES {
namespace utils {


bool is_addr_in_range(const uint16_t addr, const std::pair<uint16_t, uint16_t> &range)
{
    return addr >= range.first && addr <= range.second;
}

bool get_flag(const uint8_t &status, const uint8_t flag)
{
    return status & flag;
}

void set_flag(uint8_t &status, const uint8_t flag, const bool value)
{
    if (value) status |= flag;
    else status &= ~flag;
}

} // namespace utils
} // namespace NES