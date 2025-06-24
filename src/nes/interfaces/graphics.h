#pragma once

#include <memory>
#include "definitions.h"

namespace NES {

INTERFACE i_graphics_t
{
    virtual void update_frame(uint32_t * pixel_data) = 0;
};

typedef std::shared_ptr<i_graphics_t> graphics_ref_t;

} // namespace NES