#include <gtest/gtest.h>
#include "cpu.h"
#include "utils.h"

namespace NES_test {

class InstructionTests : public ::testing::Test
{
protected:
    // Redefine status flag enum in tests not to break private access
    enum status_flag
    {
        carry = (1 << 0),
        zero = (1 << 1),
        no_interrupts = (1 << 2),
        decimal = (1 << 3),
        b = (1 << 4),
        unused = (1 << 5),
        overflow = (1 << 6),
        negative = (1 << 7)
    };

protected:
    void SetUp() override
    {
        srand(time(0));

        bus = std::make_shared<bus_stub_t>();
        cpu = std::make_shared<NES::cpu_t>(bus);

        cpu->reset(); // pc will be set to 0, bus[RESET_VECTOR] = 0
        run_cpu(8);
    }

    void load_instruction(
        uint8_t opcode,
        std::optional<uint8_t> operand1 = std::nullopt,
        std::optional<uint8_t> operand2 = std::nullopt)
    {
        bus->write(local_pc++, opcode);
        if (operand1) bus->write(local_pc++, *operand1);
        if (operand2) bus->write(local_pc++, *operand2);
    }

    void load_accumulator(uint8_t value)
    {
        load_instruction(0xA9, value); // LDA immediate
        run_cpu(2);
    }

    uint8_t store_accumulator()
    {
        load_instruction(0x85, STA_ADDRESS); // STA zero page
        run_cpu(3);

        return bus->read(STA_ADDRESS);
    }

    void run_cpu(uint16_t cycles)
    {
        for (int i = 0; i < cycles; i++)
            cpu->clock();
    }

    bool check_flag(status_flag flag)
    {
        load_instruction(0x08); // Push status to stack
        run_cpu(3); // Push status to stack - 3 cycles

        uint8_t status = bus->read(local_sp--);

        return status & flag;
    }

    void TearDown() override
    {
    }

protected:
    NES::bus_ref_t bus;
    cpu_ref_t cpu;
    uint16_t local_pc = 0;
    uint16_t local_sp = 0x1FD; // Stack pointer after reset

    static constexpr uint16_t STA_ADDRESS = 0xF0;
    static constexpr uint16_t STX_ADDRESS = 0xF1;
    static constexpr uint16_t STY_ADDRESS = 0xF2;
};

// Start testing with status_flag, load and store instructions to use them in other tests

TEST_F(InstructionTests, clc_sec)
{
    EXPECT_FALSE(check_flag(carry));

    load_instruction(0x38); // SEC
    run_cpu(2); // SEC - 2 cycles
    EXPECT_TRUE(check_flag(carry));

    load_instruction(0x18); // CLC
    run_cpu(2); // CLC - 2 cycles
    EXPECT_FALSE(check_flag(carry));
}

TEST_F(InstructionTests, cld_sed)
{
    EXPECT_FALSE(check_flag(decimal));

    load_instruction(0xF8); // SED
    run_cpu(2); // SED - 2 cycles
    EXPECT_TRUE(check_flag(decimal));

    load_instruction(0xD8); // CLD
    run_cpu(2); // CLD - 2 cycles
    EXPECT_FALSE(check_flag(decimal));
}

TEST_F(InstructionTests, cli_sei)
{
    EXPECT_TRUE(check_flag(no_interrupts)); // reset sets no_interrupts to true

    load_instruction(0x58); // CLI
    run_cpu(2); // CLI - 2 cycles
    EXPECT_FALSE(check_flag(no_interrupts));

    load_instruction(0x78); // SEI
    run_cpu(2); // SEI - 2 cycles
    EXPECT_TRUE(check_flag(no_interrupts));
}

// since there is no set overflow instruction, we will test clear instruction later

TEST_F(InstructionTests, php)
{
    // check flag pretty much tested this, I will just check reset flags
    EXPECT_TRUE(check_flag(unused));
    EXPECT_TRUE(check_flag(no_interrupts));
}

TEST_F(InstructionTests, sta_lda)
{
    uint8_t load_value = 0;

    load_accumulator(load_value);
    EXPECT_EQ(load_value, store_accumulator());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_value = 0xFF; // -1
    load_accumulator(load_value);
    EXPECT_EQ(load_value, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));

    load_value = 1;
    load_accumulator(load_value);
    EXPECT_EQ(load_value, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
}

TEST_F(InstructionTests, stx_ldx)
{
    uint8_t load_value = 0;

    load_instruction(0xA2, load_value); // LDX immediate
    load_instruction(0x86, STX_ADDRESS); // STX zero page
    run_cpu(5); // LDX immediate - 2 cycles, STX zero page - 3 cycles

    EXPECT_EQ(load_value, bus->read(STX_ADDRESS));
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_value = 0xFF; // -1
    load_instruction(0xA2, load_value); // LDX immediate
    load_instruction(0x86, STX_ADDRESS); // STX zero page
    run_cpu(5);

    EXPECT_EQ(load_value, bus->read(STX_ADDRESS));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));

    load_value = 1;
    load_instruction(0xA2, load_value); // LDX immediate
    load_instruction(0x86, STX_ADDRESS); // STX zero page
    run_cpu(5);

    EXPECT_EQ(load_value, bus->read(STX_ADDRESS));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
}

TEST_F(InstructionTests, sty_ldy)
{
    uint8_t load_value = 0;

    load_instruction(0xA0, load_value); // LDY immediate
    load_instruction(0x84, STY_ADDRESS); // STY zero page
    run_cpu(5); // LDY immediate - 2 cycles, STY zero page - 3 cycles

    EXPECT_EQ(load_value, bus->read(STY_ADDRESS));
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_value = 0xFF; // -1
    load_instruction(0xA0, load_value); // LDY immediate
    load_instruction(0x84, STY_ADDRESS); // STY zero page
    run_cpu(5);

    EXPECT_EQ(load_value, bus->read(STY_ADDRESS));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));

    load_value = 1;
    load_instruction(0xA0, load_value); // LDY immediate
    load_instruction(0x84, STY_ADDRESS); // STY zero page
    run_cpu(5);

    EXPECT_EQ(load_value, bus->read(STY_ADDRESS));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
}

// ------------------------------------------------------------------------

TEST_F(InstructionTests, adc)
{
    const uint8_t add_value_1 = 0xFF; // triggers negative
    const uint8_t add_value_2 = 0x01; // triggers carry, zero and overflow

    load_instruction(0x69, add_value_1); run_cpu(2); // ADC immediate - 2 cycles
    EXPECT_EQ(add_value_1, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
    EXPECT_FALSE(check_flag(carry));
    EXPECT_FALSE(check_flag(overflow));

    load_instruction(0x69, add_value_2); run_cpu(2); // ADC immediate - 2 cycles
    EXPECT_EQ(uint8_t(add_value_1 + add_value_2), store_accumulator());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_TRUE(check_flag(carry));
    EXPECT_TRUE(check_flag(overflow));

    // add with carry
    load_instruction(0x69, 0); run_cpu(2); // ADC immediate - 2 cycles
    EXPECT_EQ(uint8_t(add_value_1 + add_value_2 + 1), store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_FALSE(check_flag(carry));
    EXPECT_FALSE(check_flag(overflow));
}

TEST_F(InstructionTests, and_)
{
    const uint8_t and_value_1 = 0xF0; // triggers negative
    const uint8_t and_value_2 = 0x40; // triggers nothing
    const uint8_t and_value_3 = 0x0F; // triggers zero

    load_accumulator(0xFF);

    load_instruction(0x29, and_value_1); run_cpu(2); // AND immediate - 2 cycles
    EXPECT_EQ(0xFF & and_value_1, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));

    load_instruction(0x29, and_value_2); run_cpu(2); // AND immediate - 2 cycles
    EXPECT_EQ(and_value_1 & and_value_2, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_instruction(0x29, and_value_3); run_cpu(2); // AND immediate - 2 cycles
    EXPECT_EQ(and_value_2 & and_value_3, store_accumulator());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
}

TEST_F(InstructionTests, asl)
{
    load_accumulator(1 << 5); // 0010 0000

    load_instruction(0x0A); run_cpu(2); // ASL accumulator - 2 cycles
    EXPECT_EQ(0x40, store_accumulator()); // 0100 0000
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_FALSE(check_flag(carry));

    load_instruction(0x0A); run_cpu(2); // ASL accumulator - 2 cycles
    EXPECT_EQ(0x80, store_accumulator()); // 1000 0000
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
    EXPECT_FALSE(check_flag(carry));

    load_instruction(0x0A); run_cpu(2); // ASL accumulator - 2 cycles
    EXPECT_EQ(0x00, store_accumulator()); // 0000 0000
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_TRUE(check_flag(carry));
}

// TODO: implement branch instructions
TEST_F(InstructionTests, bcc)
{
}

TEST_F(InstructionTests, bcs)
{
}

TEST_F(InstructionTests, beq)
{
}

TEST_F(InstructionTests, bit)
{
    const uint8_t mem_loc = STY_ADDRESS + 1;

    load_accumulator(0x3F);

    bus->write(mem_loc, 0);
    load_instruction(0x24, mem_loc); run_cpu(3); // BIT zero page - 3 cycles
    EXPECT_EQ(0x3F, store_accumulator());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_FALSE(check_flag(overflow));

    bus->write(mem_loc, 0xF0);
    load_instruction(0x24, mem_loc); run_cpu(3); // BIT zero page - 3 cycles
    EXPECT_EQ(0x3F, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
    EXPECT_TRUE(check_flag(overflow));
}

TEST_F(InstructionTests, bmi)
{
}

TEST_F(InstructionTests, bne)
{
}

TEST_F(InstructionTests, bpl)
{
}

TEST_F(InstructionTests, brk)
{
}

TEST_F(InstructionTests, bvc)
{
}

TEST_F(InstructionTests, bvs)
{
}

TEST_F(InstructionTests, clv)
{
}

TEST_F(InstructionTests, cmp)
{
}

TEST_F(InstructionTests, cpx)
{
}

TEST_F(InstructionTests, cpy)
{
}

TEST_F(InstructionTests, dec)
{
}

TEST_F(InstructionTests, dex)
{
}

TEST_F(InstructionTests, dey)
{
}

TEST_F(InstructionTests, eor)
{
}

TEST_F(InstructionTests, inc)
{
}

TEST_F(InstructionTests, inx)
{
}

TEST_F(InstructionTests, iny)
{
}

TEST_F(InstructionTests, jmp)
{
}

TEST_F(InstructionTests, jsr)
{
}

TEST_F(InstructionTests, lsr)
{
}

TEST_F(InstructionTests, nop)
{
}

TEST_F(InstructionTests, ora)
{
}

TEST_F(InstructionTests, pha)
{
}

TEST_F(InstructionTests, pla)
{
}

TEST_F(InstructionTests, plp)
{
}

TEST_F(InstructionTests, rol)
{
}

TEST_F(InstructionTests, ror)
{
}

TEST_F(InstructionTests, rti)
{
}

TEST_F(InstructionTests, rts)
{
}

TEST_F(InstructionTests, sbc)
{
}

TEST_F(InstructionTests, tax)
{
}

TEST_F(InstructionTests, tay)
{
}

TEST_F(InstructionTests, tsx)
{
}

TEST_F(InstructionTests, txa)
{
}

TEST_F(InstructionTests, txs)
{
}

TEST_F(InstructionTests, tya)
{
}

} // namespace NES_test
