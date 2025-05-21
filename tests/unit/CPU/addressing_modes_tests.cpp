#include <gtest/gtest.h>
#include "cpu.h"
#include "utils.h"

#include <optional>

namespace NES_test {

class AddressingModesTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        srand(time(0));

        bus = std::make_shared<bus_stub_t>();
        cpu = std::make_shared<NES::cpu_t>(bus);
    }

    void load_instruction(uint8_t opcode, uint8_t operand1, std::optional<uint8_t> operand2 = std::nullopt)
    {
        bus->write(local_pc++, opcode);
        bus->write(local_pc++, operand1);
        if (operand2) bus->write(local_pc++, *operand2);
    }

    void run_cpu(uint16_t cycles)
    {
        for (int i = 0; i < cycles; i++)
            cpu->clock();
    }

    uint8_t read_A()
    {
        // STA #STA_ADDRESS
        bus->write(local_pc++, 0x85); // STA zero page
        bus->write(local_pc++, STA_ADDRESS);

        run_cpu(3); // execute instruction (STA ZP needs 3 cycles)

        return bus->read(10);
    }

    uint8_t read_X()
    {
        // STX #STX_ADDRESS
        bus->write(local_pc++, 0x86); // STX zero page
        bus->write(local_pc++, STX_ADDRESS);

        run_cpu(3); // execute instruction (STX ZP needs 3 cycles)
        return bus->read(11);
    }

    void TearDown() override
    {
    }

protected:
    NES::bus_ref_t bus;
    cpu_ref_t cpu;
    uint16_t local_pc = 0;

    static constexpr uint16_t STA_ADDRESS = 10;
    static constexpr uint16_t STX_ADDRESS = 11;
};

// No need to implement tests for Implicit and Accumulator modes, they don't do anything

// Start testing with zero page addressing mode, text fixure also uses it with STA
TEST_F(AddressingModesTest, ZeroPage)
{
    uint8_t load_value = rand() % 0xFF;
    uint8_t address = 0; while (address < 0x04) address = rand() % 0xFF; // avoid instrutions  

    bus->write(address, load_value);

    load_instruction(0xA5, address); // LDA zero page
    run_cpu(3);

    ASSERT_EQ(load_value, read_A());
}

TEST_F(AddressingModesTest, Immediate)
{
    uint8_t load_value = rand() % 0xFF;

    load_instruction(0xA9, load_value); // LDA immediate
    run_cpu(2);

    ASSERT_EQ(load_value, read_A());
}

TEST_F(AddressingModesTest, ZeroPageX)
{
    uint8_t load_value = rand() % 0xFF;

    uint8_t address = 0, offset = 0;
    // avoid instrutions
    while (uint8_t(address + offset) < 0x06)
    {
        address = rand() % 0xFF;
        offset = rand() % 0xFF;
    }

    bus->write(uint8_t(address + offset), load_value);

    load_instruction(0xA2, offset); // LDX immediate
    load_instruction(0xB5, address); // LDA zero page X
    run_cpu(6); // LDX immediate - 2 cycles, LDA zero page X - 4 cycles

    ASSERT_EQ(load_value, read_A());
}

TEST_F(AddressingModesTest, ZeroPageY)
{
    uint8_t load_value = rand() % 0xFF;

    uint8_t address = 0, offset = 0;
    // avoid instrutions
    while (uint8_t(address + offset) < 0x06)
    {
        address = rand() % 0xFF;
        offset = rand() % 0xFF;
    }

    bus->write(uint8_t(address + offset), load_value);

    load_instruction(0xA0, offset); // LDY immediate
    load_instruction(0xB6, address); // LDX zero page Y
    run_cpu(6); // LDY immediate - 2 cycles, LDX zero page Y - 4 cycles

    ASSERT_EQ(load_value, read_X());
}

TEST_F(AddressingModesTest, Absolute)
{
    uint8_t load_value = rand() % 0xFF;
    uint16_t address = 0; while (address < 0x04) address = rand() % 0xFFFF; // avoid instrutions

    bus->write(address, load_value);

    load_instruction(0xAD, address & 0x00FF, address >> 8); // LDA absolute
    run_cpu(4);

    ASSERT_EQ(load_value, read_A());
}

TEST_F(AddressingModesTest, AbsoluteX)
{
    uint8_t load_value = rand() % 0xFF;

    // avoid instrutions
    uint16_t address = 0, offset = 0;
    while (uint16_t(address + offset) < 0x09)
    {
        address = rand() % 0xFFFF;
        offset = rand() % 0xFF;
    }

    bus->write(uint16_t(address + offset), load_value);

    /* To check for correct clock cycles:
           Add an extra instruction that will only be executed
           if the cpu doensn't add necessary extra cycle.

           If we then try to add the extra cycle and if the instruction
           is still not executed, then the cpu added the extra cycle
           when it wasn't supposed to.
    */
    uint8_t extra_cycle = (address & 0xFF00) < ((address + offset) & 0xFF00);
    bus->write(7, 0x86); // STX zero page
    bus->write(8, STA_ADDRESS);

    load_instruction(0xA2, offset); // LDX immediate
    load_instruction(0xBD, address & 0x00FF, address >> 8); // LDA absolute X
    run_cpu(6 + extra_cycle); // LDX immediate - 2 cycles, LDA absolute X - 4* cycles

    ASSERT_EQ(load_value, read_A());

    // execute failsafe instruction
    run_cpu(1);
    ASSERT_NE(load_value, read_A());
}

TEST_F(AddressingModesTest, AbsoluteY)
{
    uint8_t load_value = rand() % 0xFF;

    // avoid instrutions
    uint16_t address = 0, offset = 0;
    while (uint16_t(address + offset) < 0x09)
    {
        address = rand() % 0xFFFF;
        offset = rand() % 0xFF;
    }

    bus->write(uint16_t(address + offset), load_value);

    uint8_t extra_cycle = (address & 0xFF00) < ((address + offset) & 0xFF00);
    bus->write(7, 0x86);
    bus->write(8, STA_ADDRESS);

    load_instruction(0xA0, offset); // LDY immediate
    load_instruction(0xB9, address & 0x00FF, address >> 8); // LDA absolute Y
    run_cpu(6 + extra_cycle); // LDY immediate - 2 cycles, LDA absolute Y - 4* cycles

    ASSERT_EQ(load_value, read_A());

    // execute failsafe instruction
    run_cpu(1);
    ASSERT_NE(load_value, read_A());
}

TEST_F(AddressingModesTest, Indirect)
{
    uint8_t load_value = rand() % 0xFF;

    // avoid overlaping
    uint16_t actual_addr = 0, indirect_addr = 0;
    while (actual_addr < 0x02 || indirect_addr < 0x02
        || (actual_addr <= indirect_addr + 1 && actual_addr + 4 >= indirect_addr))
    {
        actual_addr = rand() % 0xFFFF;
        indirect_addr = rand() % 0xFFFF;
    }

    // test for wrong increment bug
    uint16_t indirect_addr_incr_bug = (indirect_addr & 0xFF00) | uint8_t(indirect_addr + 1);

    bus->write(actual_addr, 0xA9); // LDA Immediate
    bus->write(actual_addr + 1, load_value);

    bus->write(indirect_addr, actual_addr & 0x00FF);
    bus->write(indirect_addr_incr_bug, actual_addr >> 8);

    // pc = 0, local_pc = 0
    load_instruction(0x6C, indirect_addr & 0x00FF, indirect_addr >> 8); // JMP (indirect)
    run_cpu(5);

    // pc = actual_addr, local_pc = 3
    run_cpu(2); // LDA immediate - 2 cycles

    // pc = actual_addr + 2, locl_pc = 3
    local_pc = actual_addr + 2;

    // pc = actual_addr + 2, local_pc = actual_addr + 2
    ASSERT_EQ(load_value, read_A());
}

TEST_F(AddressingModesTest, IndexedIndirectX)
{
    uint8_t load_value = rand() % 0xFF;

    // avoid instrutions
    uint16_t actual_addr = 0;
    uint8_t indirect_addr = 0, offset = 0;
    while (uint8_t(indirect_addr + offset) < 0x06 || actual_addr < 0x06)
    {
        actual_addr = rand() % 0xFFFF;
        indirect_addr = rand() % 0xFF;
        offset = rand() % 0xFF;
    }

    bus->write(actual_addr, load_value);
    bus->write(uint8_t(indirect_addr + offset), actual_addr & 0x00FF);
    bus->write(uint8_t(indirect_addr + offset + 1), actual_addr >> 8);

    load_instruction(0xA2, offset); // LDX immediate
    load_instruction(0xA1, indirect_addr); // LDA indexed indirect X
    run_cpu(8); // LDX immediate - 2 cycles, LDA indexed indirect X - 6 cycles

    ASSERT_EQ(load_value, read_A());
}

TEST_F(AddressingModesTest, IndirectIndexedY)
{
    uint8_t load_value = rand() % 0xFF;

    // avoid instrutions
    uint16_t actual_addr = 0;
    uint8_t indirect_addr = 0, offset = 0;
    while (indirect_addr < 0x06 || actual_addr + offset < 0x06)
    {
        actual_addr = rand() % 0xFFFF;
        indirect_addr = rand() % 0xFF;
        offset = rand() % 0xFF;
    }

    // Check for correct clock cycles
    uint8_t extra_cycle = (actual_addr & 0xFF00) < ((actual_addr + offset) & 0xFF00);
    bus->write(6, 0x86);
    bus->write(7, STA_ADDRESS);

    bus->write(actual_addr + offset, load_value);
    bus->write(indirect_addr, actual_addr & 0x00FF);
    bus->write(uint8_t(indirect_addr + 1), actual_addr >> 8);

    load_instruction(0xA0, offset); // LDY immediate
    load_instruction(0xB1, indirect_addr); // LDA indirect indexed Y
    run_cpu(7 + extra_cycle); // LDY immediate - 2 cycles, LDA indirect indexed Y - 5* cycles

    ASSERT_EQ(load_value, read_A());

    // execute failsafe instruction
    run_cpu(1);
    ASSERT_NE(load_value, read_A());
}

TEST_F(AddressingModesTest, Relative)
{
    uint8_t load_value = rand() & 0xFE; // make sure load_value is not 0x01

    load_instruction(0xA9, 0xFF); run_cpu(2); // LDA immediate negative
    load_instruction(0x10, -20); run_cpu(2); // BPL -20
    load_instruction(0xA9, 0x01); run_cpu(2); // LDA immediate positive

    ASSERT_EQ(0x01, read_A()); // make sure LDA positive was executed

    load_instruction(0x10, 127); run_cpu(3); // BPL +127
    local_pc += 127;
    load_instruction(0x10, 127); run_cpu(4); // BPL +127, page crossed
    local_pc += 127;
    load_instruction(0x10, -127); run_cpu(4); // BPL -127, page crossed
    local_pc -= 127;

    // LDA imediate, no need to run clock: read_A() will run clock 3 times, LDA needs 2.
    // if at least one extra clock cycle gets added, next read_A() will not execute.
    load_instruction(0xA9, load_value);
    ASSERT_EQ(load_value, read_A());
}

} // namspace NES_test