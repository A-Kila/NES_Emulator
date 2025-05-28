#include <_types/_uint8_t.h>
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

    void load_accumulator(const uint8_t value)
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

    void load_x(const uint8_t value)
    {
        load_instruction(0xA2, value); // LDX immediate
        run_cpu(2);
    }

    uint8_t store_x()
    {
        load_instruction(0x86, STX_ADDRESS); // STX zero page
        run_cpu(3);

        return bus->read(STX_ADDRESS);
    }

    void load_y(const uint8_t value)
    {
        load_instruction(0xA0, value); // LDY immediate
        run_cpu(2);
    }

    uint8_t store_y()
    {
        load_instruction(0x84, STY_ADDRESS); // STY zero page
        run_cpu(3);

        return bus->read(STY_ADDRESS);
    }

    void run_cpu(const uint16_t cycles)
    {
        for (int i = 0; i < cycles; i++)
            cpu->clock();
    }

    bool check_flag(const status_flag flag)
    {
        load_instruction(0x08); // Push status to stack
        run_cpu(3); // Push status to stack - 3 cycles

        uint8_t status = bus->read(STACK_BASE + local_sp--);

        return status & flag;
    }

    void TearDown() override
    {
    }

protected:
    NES::bus_ref_t bus;
    cpu_ref_t cpu;
    uint16_t local_pc = 0;
    uint8_t local_sp = 0xFD; // Stack pointer after reset

    static constexpr uint16_t STACK_BASE = 0x100;
    static constexpr uint16_t RESET_VECTOR = 0xFFFC;
    static constexpr uint16_t IRQ_VECTOR = 0xFFFE;
    static constexpr uint16_t NMI_VECTOR = 0xFFFA;

    static constexpr uint8_t STA_ADDRESS = 0xF0;
    static constexpr uint8_t STX_ADDRESS = 0xF1;
    static constexpr uint8_t STY_ADDRESS = 0xF2;
};


// ------------------------------------------------------------------------
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
    // common practice is to clear carry before ADC
    load_instruction(0x18); run_cpu(2); // CLC - 2 cycles

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

// page crossing and cycle counts are tested in addressing mode tests.
TEST_F(InstructionTests, bcc)
{
    load_instruction(0x38); run_cpu(2); // SEC - 2 cycles

    // if branch is taken, accumulator will not be set to 0x01
    load_instruction(0x90, 0x02); // BCC relative - 2 cycles
    load_accumulator(0x01); run_cpu(2); // run both cycles
    EXPECT_EQ(0x01, store_accumulator());

    load_instruction(0x18); run_cpu(2); // CLC - 2 cycles

    // if branch is not taken, accumulator will be not be set to 0x02
    load_instruction(0x90, 0x01); // BCC relative - 3 cycles (taken)
    load_instruction(0xEA); // NOP - 2 cycles
    load_accumulator(0x02); run_cpu(3); // run both cycles
    EXPECT_EQ(0x02, store_accumulator());
}

TEST_F(InstructionTests, bcs)
{
    load_instruction(0x18); run_cpu(2); // CLC - 2 cycles

    // if branch is not taken, accumulator will be not be set to 0x01
    load_instruction(0xB0, 0x02); // BCS relative - 2 cycles
    load_accumulator(0x01); run_cpu(2); // run both cycles
    EXPECT_EQ(0x01, store_accumulator());

    load_instruction(0x38); run_cpu(2); // SEC - 2 cycles

    // if branch is taken, accumulator will not be set to 0x02
    load_instruction(0xB0, 0x01); // BCS relative - 3 cycles (taken)
    load_instruction(0xEA); // NOP - 2 cycles
    load_accumulator(0x02); run_cpu(3); // run both cycles
    EXPECT_EQ(0x02, store_accumulator());
}

TEST_F(InstructionTests, beq)
{
    load_accumulator(0x01); // unset zero flag

    load_instruction(0xF0, 0x02); // BEQ relative - 2 cycles
    load_accumulator(0x01); run_cpu(2); // run both cycles
    EXPECT_EQ(0x01, store_accumulator());

    load_accumulator(0x00); // set zero flag

    load_instruction(0xF0, 0x01); // BEQ relative - 3 cycles (taken)
    load_instruction(0xEA); // NOP - 2 cycles
    load_accumulator(0x02); run_cpu(3); // run both cycles
    EXPECT_EQ(0x02, store_accumulator());
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
    load_accumulator(0x00); // unset negative flag

    load_instruction(0x30, 0x02); // BMI relative - 2 cycles
    load_accumulator(0x01); run_cpu(2); // run both cycles
    EXPECT_EQ(0x01, store_accumulator());

    load_accumulator(0x80); // set negative flag

    load_instruction(0x30, 0x01); // BMI relative - 3 cycles (taken)
    load_instruction(0xEA); // NOP - 2 cycles
    load_accumulator(0x02); run_cpu(3); // run both cycles
    EXPECT_EQ(0x02, store_accumulator());
}

TEST_F(InstructionTests, bne)
{
    load_accumulator(0x00); // set zero flag

    load_instruction(0xD0, 0x02); // BNE relative - 2 cycles
    load_accumulator(0x01); run_cpu(2); // run both cycles
    EXPECT_EQ(0x01, store_accumulator());

    load_accumulator(0x01); // unset zero flag

    load_instruction(0xD0, 0x01); // BNE relative - 3 cycles (taken)
    load_instruction(0xEA); // NOP - 2 cycles
    load_accumulator(0x02); run_cpu(3); // run both cycles
    EXPECT_EQ(0x02, store_accumulator());
}

TEST_F(InstructionTests, bpl)
{
    load_accumulator(0x80); // set negative flag

    load_instruction(0x10, 0x02); // BPL relative - 2 cycles
    load_accumulator(0x01); run_cpu(2); // run both cycles
    EXPECT_EQ(0x01, store_accumulator());

    load_accumulator(0x00); // unset negative flag

    load_instruction(0x10, 0x01); // BPL relative - 3 cycles (taken)
    load_instruction(0xEA); // NOP - 2 cycles
    load_accumulator(0x02); run_cpu(3); // run both cycles
    EXPECT_EQ(0x02, store_accumulator());
}

TEST_F(InstructionTests, brk)
{
    load_accumulator(0xF0); // set negative flag
    load_instruction(0x58); run_cpu(2); // clear no interrupts flag

    bus->write(IRQ_VECTOR, 0x00); // next instruction location low (0x1000)
    bus->write(IRQ_VECTOR + 1, 0x10); // next instruction location high (0x1000)

    bus->write(0x1000, 0xA9); // LDA immediate
    bus->write(0x1001, 0x10);

    const uint8_t pc_saved = local_pc + 2; // 2 bytes from brk and 
    load_instruction(0x00); run_cpu(7); // BRK - 7 cycles
    run_cpu(2); // LDA immediate - 2 cycles

    local_pc = 0x1002; local_sp -= 3;
    EXPECT_EQ(0x10, store_accumulator()); // LDA immediate
    EXPECT_EQ(pc_saved, bus->read(STACK_BASE + local_sp + 2)); // PC low

    // check saved flags
    EXPECT_TRUE(bus->read(STACK_BASE + local_sp + 1) & status_flag::b);
    EXPECT_FALSE(bus->read(STACK_BASE + local_sp + 1) & status_flag::zero);
    EXPECT_TRUE(bus->read(STACK_BASE + local_sp + 1) & status_flag::negative);
    EXPECT_FALSE(bus->read(STACK_BASE + local_sp + 1) & status_flag::no_interrupts);

    EXPECT_TRUE(check_flag(no_interrupts)); // check current flags
}

TEST_F(InstructionTests, bvc)
{
    // trigger overflow
    load_accumulator(0xFF);
    load_instruction(0x69, 0x01); run_cpu(2); // ADC immediate - 2 cycles

    load_instruction(0x50, 0x02); // BVC relative - 2 cycles
    load_accumulator(0x01); run_cpu(2); // run both cycles
    EXPECT_EQ(0x01, store_accumulator());

    // after loading 0x01, overflow flag is cleared
    load_instruction(0x50, 0x01); // BVC relative - 3 cycles (taken)
    load_instruction(0xEA); // NOP - 2 cycles
    load_accumulator(0x02); run_cpu(3); // run both cycles
    EXPECT_EQ(0x02, store_accumulator());
}

TEST_F(InstructionTests, bvs)
{
    load_accumulator(0x00); // unset overflow flag

    load_instruction(0x70, 0x02); // BVS relative - 2 cycles
    load_accumulator(0x01); run_cpu(2); // run both cycles
    EXPECT_EQ(0x01, store_accumulator());

    // trigger overflow
    load_accumulator(0xFF);
    load_instruction(0x69, 0x01); run_cpu(2); // ADC immediate - 2 cycles

    load_instruction(0x70, 0x01); // BVS relative - 3 cycles (taken)
    load_instruction(0xEA); // NOP - 2 cycles
    load_accumulator(0x02); run_cpu(3); // run both cycles
    EXPECT_EQ(0x02, store_accumulator());
}

TEST_F(InstructionTests, clv)
{
    // trigger overflow
    load_accumulator(0xFF);
    load_instruction(0x69, 0x01); run_cpu(2); // ADC immediate - 2 cycles

    EXPECT_TRUE(check_flag(overflow));

    load_instruction(0xB8); run_cpu(2); // CLV - 2 cycles

    EXPECT_FALSE(check_flag(overflow));
}

TEST_F(InstructionTests, cmp)
{
    const uint8_t a_value = 0x02;
    load_accumulator(a_value);

    uint8_t cmp_value = 0x01; // triggers zero
    load_instruction(0xC9, cmp_value); run_cpu(2); // CMP immediate - 2 cycles
    EXPECT_TRUE(check_flag(carry));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    cmp_value = 0x02; // triggers zero
    load_instruction(0xC9, cmp_value); run_cpu(2); // CMP immediate - 2 cycles
    EXPECT_TRUE(check_flag(carry));
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    cmp_value = 0x03; // triggers negative
    load_instruction(0xC9, cmp_value); run_cpu(2); // CMP immediate - 2 cycles
    EXPECT_FALSE(check_flag(carry));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
}

TEST_F(InstructionTests, cpx)
{
    const uint8_t x_value = 0x02;
    load_x(x_value);

    uint8_t cmp_value = 0x01; // triggers zero
    load_instruction(0xE0, cmp_value); run_cpu(2); // CMP immediate - 2 cycles
    EXPECT_TRUE(check_flag(carry));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    cmp_value = 0x02; // triggers zero
    load_instruction(0xE0, cmp_value); run_cpu(2); // CMP immediate - 2 cycles
    EXPECT_TRUE(check_flag(carry));
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    cmp_value = 0x03; // triggers negative
    load_instruction(0xE0, cmp_value); run_cpu(2); // CMP immediate - 2 cycles
    EXPECT_FALSE(check_flag(carry));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
}

TEST_F(InstructionTests, cpy)
{
    const uint8_t y_value = 0x02;
    load_y(y_value);

    uint8_t cmp_value = 0x01; // triggers zero
    load_instruction(0xC0, cmp_value); run_cpu(2); // CMP immediate - 2 cycles
    EXPECT_TRUE(check_flag(carry));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    cmp_value = 0x02; // triggers zero
    load_instruction(0xC0, cmp_value); run_cpu(2); // CMP immediate - 2 cycles
    EXPECT_TRUE(check_flag(carry));
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    cmp_value = 0x03; // triggers negative
    load_instruction(0xC0, cmp_value); run_cpu(2); // CMP immediate - 2 cycles
    EXPECT_FALSE(check_flag(carry));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
}

TEST_F(InstructionTests, dec)
{
    const uint16_t dec_addr = STY_ADDRESS + 1;
    bus->write(dec_addr, 0x02);

    load_instruction(0xC6, dec_addr); run_cpu(5); // DEC zero page - 5 cycles
    EXPECT_EQ(0x01, bus->read(dec_addr));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_instruction(0xC6, dec_addr); run_cpu(5); // DEC zero page - 5 cycles
    EXPECT_EQ(0x00, bus->read(dec_addr));
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_instruction(0xC6, dec_addr); run_cpu(5); // DEC zero page - 5 cycles
    EXPECT_EQ(0xFF, bus->read(dec_addr));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
}

TEST_F(InstructionTests, dex)
{
    load_x(0x02);

    load_instruction(0xCA); run_cpu(2); // DEX - 2 cycles
    EXPECT_EQ(0x01, store_x());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_instruction(0xCA); run_cpu(2); // DEX - 2 cycles
    EXPECT_EQ(0x00, store_x());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_instruction(0xCA); run_cpu(2); // DEX - 2 cycles
    EXPECT_EQ(0xFF, store_x());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
}

TEST_F(InstructionTests, dey)
{
    load_y(0x02);

    load_instruction(0x88); run_cpu(2); // DEY - 2 cycles
    EXPECT_EQ(0x01, store_y());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_instruction(0x88); run_cpu(2); // DEY - 2 cycles
    EXPECT_EQ(0x00, store_y());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_instruction(0x88); run_cpu(2); // DEY - 2 cycles
    EXPECT_EQ(0xFF, store_y());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
}

TEST_F(InstructionTests, eor)
{
    load_accumulator(0xFF);

    const uint8_t or_value_1 = 0xFF; // triggers zero
    load_instruction(0x49, or_value_1); run_cpu(2); // EOR immediate - 2 cycles
    EXPECT_EQ(0, store_accumulator());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    const uint8_t or_value_2 = 0x0F; // triggers nothing
    load_instruction(0x49, or_value_2); run_cpu(2); // EOR immediate - 2 cycles
    EXPECT_EQ(or_value_2, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    const uint8_t or_value_3 = 0x8F; // triggers negative
    load_instruction(0x49, or_value_3); run_cpu(2); // EOR immediate - 2 cycles
    EXPECT_EQ(or_value_2 ^ or_value_3, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
}

TEST_F(InstructionTests, inc)
{
    const uint16_t inc_addr = STY_ADDRESS + 1;
    bus->write(inc_addr, 0xFE);

    load_instruction(0xE6, inc_addr); run_cpu(5); // DEC zero page - 5 cycles
    EXPECT_EQ(0xFF, bus->read(inc_addr));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));

    load_instruction(0xE6, inc_addr); run_cpu(5); // DEC zero page - 5 cycles
    EXPECT_EQ(0x00, bus->read(inc_addr));
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_instruction(0xE6, inc_addr); run_cpu(5); // DEC zero page - 5 cycles
    EXPECT_EQ(0x1, bus->read(inc_addr));
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
}

TEST_F(InstructionTests, inx)
{
    load_x(0xFE);

    load_instruction(0xE8); run_cpu(2); // INX - 2 cycles
    EXPECT_EQ(0xFF, store_x());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));

    load_instruction(0xE8); run_cpu(2); // INX - 2 cycles
    EXPECT_EQ(0x00, store_x());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_instruction(0xE8); run_cpu(2); // INX - 2 cycles
    EXPECT_EQ(0x01, store_x());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
}

TEST_F(InstructionTests, iny)
{
    load_y(0xFE);

    load_instruction(0xC8); run_cpu(2); // INY - 2 cycles
    EXPECT_EQ(0xFF, store_y());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));

    load_instruction(0xC8); run_cpu(2); // INY - 2 cycles
    EXPECT_EQ(0x00, store_y());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_instruction(0xC8); run_cpu(2); // INY - 2 cycles
    EXPECT_EQ(0x01, store_y());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
}

TEST_F(InstructionTests, jmp)
{
    uint16_t jump_address = 0x01FF;

    load_instruction(0x4C, jump_address & 0x00FF, jump_address >> 8); // JMP absolute
    local_pc = jump_address;
    load_instruction(0xA2, 0x12); // LDX immediate
    run_cpu(5); // JMP - 3 cycles, LDX - 2 cycles

    ASSERT_EQ(0x12, store_x());
}

TEST_F(InstructionTests, jsr)
{
    uint16_t jump_address = 0x12FF;
    uint16_t local_pc_temp = local_pc + 2; // location that will be pushed to stack

    load_instruction(0x20, jump_address & 0x00FF, jump_address >> 8); // JSR absolute
    local_pc = jump_address;
    load_instruction(0xA2, 0x12); // LDX immediate
    run_cpu(6); // JSR - 6 cycles, LDX - 2 cycles

    local_sp -= 2;

    ASSERT_EQ(0x12, store_x());
    ASSERT_EQ(local_pc_temp & 0x00FF, bus->read(STACK_BASE + local_sp + 1)); // return address low
    ASSERT_EQ(local_pc_temp >> 8, bus->read(STACK_BASE + local_sp + 2)); // return address high
}

TEST_F(InstructionTests, lsr)
{
    load_accumulator(1 << 1); // 0000 0010

    // since lsr moves 0 to bit #7 it will never trigger negative

    load_instruction(0x4A); run_cpu(2); // ASL accumulator - 2 cycles
    EXPECT_EQ(0x1, store_accumulator()); // 0000 0001
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_FALSE(check_flag(carry));

    load_instruction(0x4A); run_cpu(2); // ASL accumulator - 2 cycles
    EXPECT_EQ(0x00, store_accumulator()); // 0000 0000
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_TRUE(check_flag(carry));

    load_instruction(0x4A); run_cpu(2); // ASL accumulator - 2 cycles
    EXPECT_EQ(0x00, store_accumulator()); // 0000 0000
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_FALSE(check_flag(carry));
}

TEST_F(InstructionTests, ora)
{
    load_accumulator(0x00);

    const uint8_t or_value_1 = 00; // triggers zero
    load_instruction(0x09, or_value_1); run_cpu(2); // EOR immediate - 2 cycles
    EXPECT_EQ(or_value_1, store_accumulator());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    const uint8_t or_value_2 = 0x0F; // triggers nothing
    load_instruction(0x09, or_value_2); run_cpu(2); // EOR immediate - 2 cycles
    EXPECT_EQ(or_value_1 | or_value_2, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    const uint8_t or_value_3 = 0x80; // triggers negative
    load_instruction(0x09, or_value_3); run_cpu(2); // EOR immediate - 2 cycles
    EXPECT_EQ(or_value_1 | or_value_2 | or_value_3, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
}

TEST_F(InstructionTests, pha)
{
    load_accumulator(0x69);
    load_instruction(0x48); run_cpu(3); // PHA - 3 cycles

    EXPECT_EQ(0x69, bus->read(STACK_BASE + local_sp--));
}

TEST_F(InstructionTests, pla)
{
    local_sp++;
    load_instruction(0x68); run_cpu(4); // PLA - 4 cycles
    EXPECT_EQ(0x00, store_accumulator());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    bus->write(STACK_BASE + ++local_sp, 0x69); // push value to stack
    load_instruction(0x68); run_cpu(4); // PLA - 4 cycles
    EXPECT_EQ(0x69, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    bus->write(STACK_BASE + ++local_sp, 0xF0); // push negative value to stack
    load_instruction(0x68); run_cpu(4); // PLA - 4 cycles
    EXPECT_EQ(0xF0, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
}

TEST_F(InstructionTests, plp)
{
    // status flag b is set by pushing status to stack
    const uint8_t expected_status = status_flag::unused | status_flag::no_interrupts | status_flag::b;

    // save status
    load_instruction(0x08); run_cpu(3); // PushStack - 3 cycles
    EXPECT_EQ(expected_status, bus->read(STACK_BASE + local_sp--));

    // modify status flags
    load_instruction(0x38); run_cpu(2); // SEC
    load_instruction(0xF8); run_cpu(2); // SED
    load_instruction(0x58); run_cpu(2); // CLI

    // restore status by pulling from stack
    local_sp++;
    load_instruction(0x28); run_cpu(4); // PLP - 4 cycles

    bus->write(STACK_BASE + local_sp, 0); // reset sp value to make sure new push overwrites value
    load_instruction(0x08); run_cpu(3); // PushStack - 3 cycle
    EXPECT_EQ(expected_status, bus->read(STACK_BASE + local_sp--));

}

TEST_F(InstructionTests, rol)
{
    load_accumulator(1 << 5); // 0010 0000

    load_instruction(0x2A); run_cpu(2); // ROL accumulator - 2 cycles
    EXPECT_EQ(0x40, store_accumulator()); // 0100 0000
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_FALSE(check_flag(carry));

    load_instruction(0x2A); run_cpu(2); // ROL accumulator - 2 cycles
    EXPECT_EQ(0x80, store_accumulator()); // 1000 0000
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
    EXPECT_FALSE(check_flag(carry));

    load_instruction(0x2A); run_cpu(2); // ROL accumulator - 2 cycles
    EXPECT_EQ(0x00, store_accumulator()); // 0000 0000
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_TRUE(check_flag(carry));

    load_instruction(0x2A); run_cpu(2); // ROL accumulator - 2 cycles
    EXPECT_EQ(0x01, store_accumulator()); // 0000 0001
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_FALSE(check_flag(carry));
}

TEST_F(InstructionTests, ror)
{
    load_accumulator(1 << 1); // 0000 0010

    load_instruction(0x6A); run_cpu(2); // ROR accumulator - 2 cycles
    EXPECT_EQ(0x01, store_accumulator()); // 0000 0001
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_FALSE(check_flag(carry));

    load_instruction(0x6A); run_cpu(2); // ROR accumulator - 2 cycles
    EXPECT_EQ(0x00, store_accumulator()); // 0000 0000
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_TRUE(check_flag(carry));

    load_instruction(0x6A); run_cpu(2); // ROR accumulator - 2 cycles
    EXPECT_EQ(0x80, store_accumulator()); // 1000 0000
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
    EXPECT_FALSE(check_flag(carry));
}

TEST_F(InstructionTests, rti)
{
    bus->write(IRQ_VECTOR, 0xFF);
    bus->write(IRQ_VECTOR + 1, 0x01);
    load_accumulator(0xF0); // set negative flag
    ASSERT_TRUE(check_flag(negative));

    load_instruction(0x00); // BRK - 7 cycles
    uint16_t local_pc_temp = local_pc + 1;
    local_pc += 4; // 1 will be used by break, 2 will be used by set_accumulator(), 1 by check_flag
    load_instruction(0xA2, 0x01); // LDX immediate

    run_cpu(7); // run BRK

    local_pc = 0x01FF; // set PC to the end of the instruction
    load_instruction(0xA2, 0x02); run_cpu(2); // LDX immediate
    ASSERT_FALSE(check_flag(negative));
    load_instruction(0x68); run_cpu(4); local_sp--; // PLA - 4 cycles, pop check_flag from stack
    load_instruction(0x40); run_cpu(6); // RTI - 6 cycles

    local_pc = local_pc_temp; // restore PC
    EXPECT_EQ(0x02, store_x()); // check accumulator
    EXPECT_TRUE(check_flag(negative)); // check negative flag

    run_cpu(2); // run LDA immediate

    local_pc += 2; // used by LDA
    EXPECT_EQ(0x01, store_x()); // check accumulator
}

TEST_F(InstructionTests, rts)
{
    load_instruction(0x20, 0xFF, 0x01);
    uint16_t local_pc_temp = local_pc;
    local_pc += 2; // will be used by set_accumulator()
    load_instruction(0xA9, 0x01); // LDA immediate

    run_cpu(6); // run JSR

    local_pc = 0x01FF; // set PC to the end of the instruction
    load_instruction(0xA9, 0x02); run_cpu(2); // LDA immediate
    load_instruction(0x60); run_cpu(6); // RTS - 6 cycles

    // Make sure accumulator got set after jump
    local_pc = local_pc_temp; // restore PC
    EXPECT_EQ(0x02, store_accumulator()); // check accumulator

    run_cpu(2); // run LDA immediate
    local_pc += 2; // used by LDA

    EXPECT_EQ(0x01, store_accumulator()); // check accumulator
}

TEST_F(InstructionTests, sbc)
{
    // set carry flag, common practice is to set it before subtraction
    // carry flag works in reverse for subtraction
    load_instruction(0x38); run_cpu(2); // SEC - 2 cycles

    load_accumulator(0x01);
    const uint8_t sub_value_1 = 0x01; // triggers carry (it is reverse of ADC), zero and negative
    const uint8_t sub_value_2 = 0x70; // getting ready for overflow
    const uint8_t sub_value_3 = 0x0F; // triggers overflow (underflow)

    load_instruction(0xE9, sub_value_1); run_cpu(2); // SBC immediate - 2 cycles
    EXPECT_EQ(0x00, store_accumulator());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_TRUE(check_flag(carry)); // no unsigned underflow means carry = 1
    EXPECT_FALSE(check_flag(overflow));

    load_instruction(0xE9, sub_value_1); run_cpu(2); // SBC immediate - 2 cycles
    EXPECT_EQ(0xFF, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
    EXPECT_FALSE(check_flag(carry)); // unsigned underflow means carry = 0
    EXPECT_FALSE(check_flag(overflow));

    // subtract with carry (when carry is 0)
    load_instruction(0xE9, sub_value_2); run_cpu(2); // SBC immediate - 2 cycles
    EXPECT_EQ(0xFE - sub_value_2, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
    EXPECT_TRUE(check_flag(carry));
    EXPECT_FALSE(check_flag(overflow));

    load_instruction(0xE9, sub_value_3); run_cpu(2); // SBC immediate - 2 cycles
    EXPECT_EQ(uint8_t(0xFE - sub_value_2 - sub_value_3), store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
    EXPECT_TRUE(check_flag(carry));
    EXPECT_TRUE(check_flag(overflow)); // signed underflow
}

TEST_F(InstructionTests, tax)
{
    load_accumulator(0x69);
    load_instruction(0xAA); run_cpu(2); // TAX - 2 cycles
    EXPECT_EQ(0x69, store_x());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_accumulator(0x00);
    load_instruction(0xAA); run_cpu(2); // TAX - 2 cycles
    EXPECT_EQ(0x00, store_x());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_accumulator(0xF0);
    load_instruction(0xAA); run_cpu(2); // TAX - 2 cycles
    EXPECT_EQ(0xF0, store_x());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
}

TEST_F(InstructionTests, tay)
{
    load_accumulator(0x69);
    load_instruction(0xA8); run_cpu(2); // TAY - 2 cycles
    EXPECT_EQ(0x69, store_y());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_accumulator(0x00);
    load_instruction(0xA8); run_cpu(2); // TAY - 2 cycles
    EXPECT_EQ(0x00, store_y());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_accumulator(0xF0);
    load_instruction(0xA8); run_cpu(2); // TAY - 2 cycles
    EXPECT_EQ(0xF0, store_y());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
}

TEST_F(InstructionTests, tsx)
{
    load_instruction(0xBA); run_cpu(2); // TSX - 2 cycles
    EXPECT_EQ(local_sp, store_x());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));

    for (; (int8_t)local_sp != 0; local_sp++)
    {
        // pull from stack until sp is 0
        load_instruction((0x68)); run_cpu(4); // PLA - 4 cycles
    }

    load_instruction(0xBA); run_cpu(2); // TSX - 2 cycles
    EXPECT_EQ(local_sp, store_x());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    for (; (int8_t)local_sp <= 0; local_sp++)
    {
        // pull from stack until sp is 0
        load_instruction((0x68)); run_cpu(4); // PLA - 4 cycles
    }

    load_instruction(0xBA); run_cpu(2); // TSX - 2 cycles
    EXPECT_EQ(local_sp, store_x());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));
}

TEST_F(InstructionTests, txa)
{
    load_x(0x69);
    load_instruction(0x8A); run_cpu(2); // TXA - 2 cycles
    EXPECT_EQ(0x69, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_x(0x00);
    load_instruction(0x8A); run_cpu(2); // TXA - 2 cycles
    EXPECT_EQ(0x00, store_accumulator());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_x(0xF0);
    load_instruction(0x8A); run_cpu(2); // TXA - 2 cycles
    EXPECT_EQ(0xF0, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
}

TEST_F(InstructionTests, txs)
{
    load_x(0x69);
    load_instruction(0x9A); run_cpu(2); // TXS - 2 cycles

    // reset x
    load_x(0x00);
    load_instruction(0xBA); run_cpu(2); // TXS - 2 cycles

    EXPECT_EQ(0x69, store_x());
}

TEST_F(InstructionTests, tya)
{
    load_y(0x69);
    load_instruction(0x98); run_cpu(2); // TYA - 2 cycles
    EXPECT_EQ(0x69, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_y(0x00);
    load_instruction(0x98); run_cpu(2); // TYA - 2 cycles
    EXPECT_EQ(0x00, store_accumulator());
    EXPECT_TRUE(check_flag(zero));
    EXPECT_FALSE(check_flag(negative));

    load_y(0xF0);
    load_instruction(0x98); run_cpu(2); // TYA - 2 cycles
    EXPECT_EQ(0xF0, store_accumulator());
    EXPECT_FALSE(check_flag(zero));
    EXPECT_TRUE(check_flag(negative));
}
TEST_F(InstructionTests, interrupt)
{
    load_instruction(0x58); run_cpu(2); // clear no interrupts flag
    load_accumulator(0xF0); // set negative flag

    bus->write(IRQ_VECTOR, 0x00); // next instruction location low (0x1000)
    bus->write(IRQ_VECTOR + 1, 0x10); // next instruction location high (0x1000)

    bus->write(0x1000, 0xA9); // LDA immediate
    bus->write(0x1001, 0x10);

    const uint8_t pc_saved = local_pc; // 2 bytes from interrupt 
    cpu->interrupt(); run_cpu(7); // trigger IRQ interrupt    
    run_cpu(2); // LDA immediate - 2 cycles

    local_pc = 0x1002; local_sp -= 3;
    EXPECT_EQ(0x10, store_accumulator()); // LDA immediate
    EXPECT_EQ(pc_saved, bus->read(STACK_BASE + local_sp + 2)); // PC low

    // check saved flags
    EXPECT_FALSE(bus->read(STACK_BASE + local_sp + 1) & status_flag::b);
    EXPECT_FALSE(bus->read(STACK_BASE + local_sp + 1) & status_flag::zero);
    EXPECT_TRUE(bus->read(STACK_BASE + local_sp + 1) & status_flag::negative);
    EXPECT_FALSE(bus->read(STACK_BASE + local_sp + 1) & status_flag::no_interrupts);

    EXPECT_TRUE(check_flag(no_interrupts)); // check current flags

    load_instruction(0x78); run_cpu(2); // set no iterrupts flag

    cpu->interrupt(); run_cpu(7); // trigger another interrupt (should not work, will call brk (0x00))

    load_instruction(0x28); run_cpu(4); // PLP - 4 cycles
    EXPECT_TRUE(check_flag(b)); // b flag gets set by brk
}

TEST_F(InstructionTests, non_maskable_interrupt)
{
    load_instruction(0x58); run_cpu(2); // clear no interrupts flag
    load_accumulator(0xF0); // set negative flag

    bus->write(NMI_VECTOR, 0x00); // next instruction location low (0x1000)
    bus->write(NMI_VECTOR + 1, 0x10); // next instruction location high (0x1000)

    bus->write(0x1000, 0xA9); // LDA immediate
    bus->write(0x1001, 0x10);

    const uint8_t pc_saved = local_pc; // 2 bytes from interrupt 
    cpu->nonmaskable_interrupt(); run_cpu(7); // trigger NMI interrupt    
    run_cpu(2); // LDA immediate - 2 cycles

    local_pc = 0x1002; local_sp -= 3;
    EXPECT_EQ(0x10, store_accumulator()); // LDA immediate
    EXPECT_EQ(pc_saved, bus->read(STACK_BASE + local_sp + 2)); // PC low

    // check saved flags
    EXPECT_FALSE(bus->read(STACK_BASE + local_sp + 1) & status_flag::b);
    EXPECT_FALSE(bus->read(STACK_BASE + local_sp + 1) & status_flag::zero);
    EXPECT_TRUE(bus->read(STACK_BASE + local_sp + 1) & status_flag::negative);
    EXPECT_FALSE(bus->read(STACK_BASE + local_sp + 1) & status_flag::no_interrupts);

    EXPECT_TRUE(check_flag(no_interrupts)); // check current flags

    load_instruction(0x78); run_cpu(2); // set no iterrupts flag

    cpu->interrupt(); // trigger another interrupt (This should still work)

    load_instruction(0x28); run_cpu(4); // PLP - 4 cycles
    EXPECT_FALSE(check_flag(b)); // b flag gets set by brk
}

TEST_F(InstructionTests, reset)
{
    load_accumulator(0x01);
    load_x(0x01);
    load_y(0x01);

    // set some flag like decimal
    load_instruction(0xF8); run_cpu(2); // SED - 2 cycles

    cpu->reset(); // reset CPU

    EXPECT_EQ(0x00, store_accumulator()); // accumulator should be reset to 0
    EXPECT_EQ(0x00, store_x()); // x should be reset to 0
    EXPECT_EQ(0x00, store_y()); // y should be reset to 0
    EXPECT_FALSE(check_flag(decimal)); // zero flag should be set
}

} // namespace NES_test
