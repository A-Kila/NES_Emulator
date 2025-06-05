#include <gtest/gtest.h>
#include <fstream>
#include "cpu.h"
#include "main_bus.h"
#include "cartridge/cartridge.h"
#include "../utils.h"

#define STRING(x) #x
#define XSTRING(x) STRING(x)

namespace NES_test {

uint16_t get_program_size(std::ifstream &cartridge_file)
{
    uint8_t header[16];
    cartridge_file.read(reinterpret_cast<char *>(header), sizeof(header));

    return header[4] * 0x4000; // how many 16KB PRG-ROM banks
}

TEST(NestestSystemTest, TestCpuState)
{
    // This test load a ROM file by Kevin Horton
    // The ROM file is designed to test the CPU state after executing a series of instructions.
    // If the ROM tests do not pass, there will be error code written in RAM at address 0x0002 and 0x0003.
    // The execution wil start at address 0xC000.
    constexpr uint16_t RESET_VECTOR = 0xFFFC;
    constexpr uint16_t MAX_CYCLES = 26555; // Maximum cycles to run the CPU
    const char *ROM_FILE_PATH = XSTRING(RESOURCES_DIR) "/nestest.nes";
    const char *LOG_FILE_PATH = XSTRING(CURRENT_BINARY_DIR) "/my_nes.log";

    auto cartridge = std::make_shared<NES::cartridge_t>(ROM_FILE_PATH);
    ASSERT_TRUE(cartridge->is_valid()) << "Failed to load cartridge from file: " << ROM_FILE_PATH;

    auto bus = std::make_shared<NES::main_bus_t>(cartridge);
    NES::cpu_t cpu(bus);

    bus->write(RESET_VECTOR, 0x00); // Set the reset vector to 0xC000
    bus->write(RESET_VECTOR + 1, 0xC0); // Set the reset vector to 0xC000
    cpu.reset(); // Reset the CPU

    std::ofstream log_file(LOG_FILE_PATH);
    ASSERT_TRUE(log_file.is_open()) << "Failed to open log file";
    for (uint16_t i = 0; i < MAX_CYCLES; i++)
    {
        const char *log = cpu._log_debug();
        if (log) log_file << log << std::endl;

        cpu.clock(); // Execute one CPU cycle
    }
    log_file.close();

    ASSERT_EQ(0x00, bus->read(0x0002)); // Error code low byte
    ASSERT_EQ(0x00, bus->read(0x0003)); // Error code high byte
}

} // namespace NES_test