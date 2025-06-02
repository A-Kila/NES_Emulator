#include <gtest/gtest.h>
#include <fstream>
#include "cpu.h"
#include "../../utils.h"

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

    auto bus = std::make_shared<bus_stub_t>();
    NES::cpu_t cpu(bus);

    bus->write(0xFFFC, 0x00); // Reset vector low byte
    bus->write(0xFFFD, 0xC0); // Reset vector high byte

    cpu.reset(); // Reset the CPU

    std::ifstream cartrige_file(ROM_FILE_PATH, std::ios::in);
    ASSERT_TRUE(cartrige_file.is_open()) << "Failed to open ROM file: " << ROM_FILE_PATH;

    uint16_t program_size = get_program_size(cartrige_file);
    std::vector<uint8_t> program_data(program_size);
    cartrige_file.read(reinterpret_cast<char *>(program_data.data()), program_size);
    ASSERT_EQ(program_size, cartrige_file.gcount()) << "Failed to read the entire ROM file";

    cartrige_file.close();

    ASSERT_TRUE(bus->load_cartige(program_data, 0xC000)) << "Failed to load ROM data into bus";

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