#pragma once

#include <cstdint>
#include <memory>

namespace NES {

class joypad_t
{
public:
    joypad_t();
    virtual ~joypad_t();

    void update_joypad(const bool is_second);
    bool get_input(const bool is_second);

    virtual void update_keys() = 0;

protected:
    uint8_t joypads_[2];
    uint8_t shift_regs_[2];

public:
    enum keys
    {
        RIGHT = (1 << 0),
        LEFT = (1 << 1),
        DOWN = (1 << 2),
        UP = (1 << 3),
        START = (1 << 4),
        SELECT = (1 << 5),
        B = (1 << 6),
        A = (1 << 7)
    };
};

typedef std::shared_ptr<joypad_t> joypad_ref_t;

} // namespace NES