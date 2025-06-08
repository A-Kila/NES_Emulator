#include "utils.h"

namespace NES {
namespace utils {


bool is_addr_in_range(const uint16_t addr, const std::pair<uint16_t, uint16_t> &range)
{
    return addr >= range.first && addr <= range.second;
}

} // namespace utils
} // namespace NES