#pragma once

#include <memory>
#include "definitions.h"

namespace NES {

enum nes_event
{
    NO_EVENT = 0,
    KEY_CHANGED = 1,
    QUIT
};

INTERFACE i_event_t
{
    virtual nes_event get_event() = 0;
};

typedef std::shared_ptr<i_event_t> events_ref_t;

} // namespace NES