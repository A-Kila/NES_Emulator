#pragma once

#include <memory>
#include "definitions.h"

namespace NES {

INTERFACE i_bus_t
{
    virtual void write(uint16_t addr, uint8_t data) = 0;
    virtual uint8_t read(uint16_t addr) = 0;
};

typedef std::shared_ptr<i_bus_t> bus_ref_t;

} // namespace NES