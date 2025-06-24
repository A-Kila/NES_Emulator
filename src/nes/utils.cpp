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

timer_t::timer_t()
{
    last_time_ = std::chrono::high_resolution_clock::now();
}

timer_t::~timer_t()
{
}

float timer_t::get_elapsed_ms()
{
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<float>(end - last_time_).count() * 1000.0f;
}

void timer_t::print_elapsed_ms()
{
    printf("MS %f\n", get_elapsed_ms());
}

void timer_t::reset()
{
    last_time_ = std::chrono::high_resolution_clock::now();
}

} // namespace utils
} // namespace NES