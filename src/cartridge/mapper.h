#pragma once

#include <memory>
#include "../definitions.h"

namespace NES {

INTERFACE i_mapper_t
{
    virtual uint16_t map_address(uint16_t addr) const = 0;
};

typedef std::shared_ptr<i_mapper_t> mapper_ref_t;

} // namespace NES