#pragma once

#include <utility>
#include <cstdint>
#include <chrono>

namespace NES {
namespace utils {

bool is_addr_in_range(const uint16_t addr, const std::pair<uint16_t, uint16_t> &range);

bool get_flag(const uint8_t &status, const uint8_t flag);
void set_flag(uint8_t &status, const uint8_t flag, const bool value);

class timer_t
{
public:
    timer_t();
    ~timer_t();

    float get_elapsed_ms();
    void print_elapsed_ms();

    void reset();
private:
    std::chrono::high_resolution_clock::time_point last_time_;
};

} // namespace utils
} // namespace NES