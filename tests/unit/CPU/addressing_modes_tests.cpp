#include <gtest/gtest.h>
#include "cpu.h"
#include "bus.h"

// TODO: implement tests for all addressing modes
TEST(AddressingModesTests, ImplicitMode)
{
    NES::bus_ref_t bus = std::make_shared<NES::bus_t>();
    NES::cpu_t cpu(bus);
}