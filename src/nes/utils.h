#pragma once

#include <utility>
#include <cstdint>

namespace NES {
namespace utils {

bool is_addr_in_range(const uint16_t addr, const std::pair<uint16_t, uint16_t> &range);

bool get_flag(const uint8_t &status, const uint8_t flag);
void set_flag(uint8_t &status, const uint8_t flag, const bool value);

} // namespace utils
} // namespace NES