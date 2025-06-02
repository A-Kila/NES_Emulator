#pragma once

#include <stdint.h>
#include <vector>
#include "bus.h"

namespace NES {

class cpu_t
{
public:
    cpu_t(bus_ref_t bus);
    virtual ~cpu_t();

public:
    // Signals
    void clock();
    void reset();
    void interrupt();
    void nonmaskable_interrupt();

private:
    struct registers_t
    {
        registers_t() :
            a(),
            x(),
            y(),
            pc(),
            sp(),
            status()
        {
        }

        uint8_t a; // Accumulator
        uint8_t x;
        uint8_t y;
        uint16_t pc; // Program counter
        uint8_t sp;  // Stack Pointer
        uint8_t status;
    };

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

    struct instruction_t
    {
        const char *name;
        bool (cpu_t:: *operation)(const uint16_t address);
        bool (cpu_t:: *addr_mode)(uint16_t &address);
        uint8_t cycles;
    };

protected:
    // Addressing modes (returns true if additional clock cycles are needed)
    bool implicit(uint16_t &address);
    bool immediate(uint16_t &address);
    bool accumulator(uint16_t &address);
    bool zero_page(uint16_t &address);
    bool zero_page_x(uint16_t &address);
    bool zero_page_y(uint16_t &address);
    bool absolute(uint16_t &address);
    bool absolute_x(uint16_t &address);
    bool absolute_y(uint16_t &address);
    bool indirect(uint16_t &address);
    bool indexed_indirect_x(uint16_t &address);
    bool indirect_indexed_y(uint16_t &address);
    bool relative(uint16_t &address);

    // Instructions (returns true if additional clock cycles are needed)
    bool adc(const uint16_t address);  // Add with carry, Type: Arithmetic
    bool and_(const uint16_t address); // Bitwise AND, Type: Bitwise   (and is a keyword, thus we use and_)
    bool asl(const uint16_t address);  // Arithmetic Shift Left, Type: Shift
    bool bcc(const uint16_t address);  // Branch if Carry Clear, Type: Branch
    bool bcs(const uint16_t address);  // Branch if Carry Set, Type: Branch
    bool beq(const uint16_t address);  // Branch if Equal, Type: Branch
    bool bit(const uint16_t address);  // Bit Test, Type: Bitwise
    bool bmi(const uint16_t address);  // Branch if Minus, Type: Branch
    bool bne(const uint16_t address);  // Branch if Not Equal, Type: Branch
    bool bpl(const uint16_t address);  // Branch if Plus, Type: Branch
    bool brk(const uint16_t address);  // Force Break, Type: Jump
    bool bvc(const uint16_t address);  // Branch if Overflow Clear, Type: Branch
    bool bvs(const uint16_t address);  // Branch if Overflow Set, Type: Branch
    bool clc(const uint16_t address);  // Clear Carry Flag, Type: Flags
    bool cld(const uint16_t address);  // Clear Decimal, Type: Flags
    bool cli(const uint16_t address);  // Clear Interrupt Disable, Type: Flags
    bool clv(const uint16_t address);  // Clear Overflow, Type: Flags
    bool cmp(const uint16_t address);  // Compare A, Type: Compare
    bool cpx(const uint16_t address);  // Compare X, Type: Compare
    bool cpy(const uint16_t address);  // Compare Y, Type: Compare
    bool dec(const uint16_t address);  // Decrement Memory, Type: Arithmetic
    bool dex(const uint16_t address);  // Decrement X, Type: Arithmetic
    bool dey(const uint16_t address);  // Decrement Y, Type: Arithmetic
    bool eor(const uint16_t address);  // Bitwise Exclusive OR, Type: Bitwise
    bool inc(const uint16_t address);  // Increment Memory, Type: Arithmetic
    bool inx(const uint16_t address);  // Increment X, Type: Arithmetic
    bool iny(const uint16_t address);  // Increment Y, Type: Arithmetic
    bool jmp(const uint16_t address);  // Jump to Address, Type: Jump
    bool jsr(const uint16_t address);  // Jump to Subroutine, Type: Jump
    bool lda(const uint16_t address);  // Load A, Type: Access
    bool ldx(const uint16_t address);  // Load X, Type: Access
    bool ldy(const uint16_t address);  // Load Y, Type: Access
    bool lsr(const uint16_t address);  // Logical Shift Right, Type: Shift
    bool nop(const uint16_t address);  // No Operation, Type: Other
    bool ora(const uint16_t address);  // Bitwise OR, Type: Bitwise
    bool pha(const uint16_t address);  // Push A, Type: Stack
    bool php(const uint16_t address);  // Push Processor Status, Type: Stack
    bool pla(const uint16_t address);  // Pull A, Type: Stack
    bool plp(const uint16_t address);  // Pull Processor Status, Type: Stack
    bool rol(const uint16_t address);  // Rotate Left, Type: Shift
    bool ror(const uint16_t address);  // Rotate Right, Type: Shift
    bool rti(const uint16_t address);  // Return from Interrupt, Type: Jump
    bool rts(const uint16_t address);  // Return from Subroutine, Type: Jump
    bool sbc(const uint16_t address);  // Subtract with Carry, Type: Arithmetic
    bool sec(const uint16_t address);  // Set Carry, Type: Flags
    bool sed(const uint16_t address);  // Set Decimal, Type: Flags
    bool sei(const uint16_t address);  // Set Interrupt Disable, Type: Flags
    bool sta(const uint16_t address);  // Store A, Type: Access
    bool stx(const uint16_t address);  // Store X, Type: Access
    bool sty(const uint16_t address);  // Store Y, Type: Access
    bool tax(const uint16_t address);  // Transfer A to X, Type: Transfer
    bool tay(const uint16_t address);  // Transfer A to Y, Type: Transfer
    bool tsx(const uint16_t address);  // Transfer Stack Pointer to X, Type: Stack
    bool txa(const uint16_t address);  // Transfer X to A, Type: Transfer
    bool txs(const uint16_t address);  // Transfer X to Stack Pointer, Type: Stack
    bool tya(const uint16_t address);  // Transfer Y to A, Type: Transfer

    bool xxx(const uint16_t _); // Illegal opcodes (not implemented), Type: Other

    // Illegal opcodes (not all)
    bool _dcp(const uint16_t address); // (DCM) Decrement + Compare, Type: Other
    bool _isb(const uint16_t address); // (ISC, INS) Increment + Subtract with Carry, Type: Other
    bool _lax(const uint16_t address); // Load A + Load X, Type: Other
    bool _rla(const uint16_t address); // Rotate Left + AND, Type: Other
    bool _rra(const uint16_t address); // Rotate Right + Add with Carry, Type: Other
    bool _sax(const uint16_t address); // (AXS, AAX) Store A + X, Type: Other
    bool _slo(const uint16_t address); // (ASO) Shift Left + OR, Type: Other
    bool _sre(const uint16_t address); // (LSE) Logical Shift Right + Exclusive OR, Type: Other

private:
    // Helper functions
    bool _get_flag(status_flag flag);
    void _set_flag(const status_flag flag, const bool value);

    void _push_stack(const uint8_t value);
    uint8_t _pop_stack();

    void _basic_interrupt(const uint16_t vector, const uint8_t cycles);

    void _zero_page_add(uint16_t &address, const uint8_t register_value);
    bool _absolute_add(uint16_t &address, const uint8_t register_value);

    bool _relative_jump(const bool condition, const uint16_t address);

public:
    // Debugging
    char *_log_debug();

private:
    bus_ref_t bus_; // Pointer to the bus

    registers_t registers_; // CPU registers
    uint8_t cycles_; // Cycles remaining
    bool is_addressing_accumulator_; // Addressing mode is accumulator

    std::vector<instruction_t> instruction_lookup_;

private:
    static constexpr uint16_t STACK_BASE = 0x0100;

    static constexpr uint16_t IRQ_VECTOR = 0xFFFE;
    static constexpr uint16_t NMI_VECTOR = 0xFFFA;
    static constexpr uint16_t RESET_VECTOR = 0xFFFC;
};

} // namespace NES
