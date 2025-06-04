#pragma once

#include <memory>
#include "mapper.h"

namespace NES {

class cartridge_t
{
public:
    cartridge_t();
    ~cartridge_t();

private:
    mapper_ref_t mapper_;

};

typedef std::shared_ptr<cartridge_t> cartridge_ref_t;

} // namespace NES