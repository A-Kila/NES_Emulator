#pragma once

#include <stdint.h>
#include <vector>
#include "bus.h"

namespace NES {

class cpu_t
{
public:
    cpu_t(bus_ref_t bus);
    ~cpu_t();

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
        breaks = (1 << 4),
        unused = (1 << 5),
        overflow = (1 << 6),
        negative = (1 << 7)
    };

    struct instruction_t
    {
        const char *name;
        bool (cpu_t:: *operation)(uint16_t address);
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
    bool adc(uint16_t address);  // Add with carry, Type: Arithmetic
    bool and_(uint16_t address); // Bitwise AND, Type: Bitwise   (and is a keyword, thus we use and_)
    bool asl(uint16_t address);  // Arithmetic Shift Left, Type: Shift
    bool bcc(uint16_t address);  // Branch if Carry Clear, Type: Branch
    bool bcs(uint16_t address);  // Branch if Carry Set, Type: Branch
    bool beq(uint16_t address);  // Branch if Equal, Type: Branch
    bool bit(uint16_t address);  // Bit Test, Type: Bitwise
    bool bmi(uint16_t address);  // Branch if Minus, Type: Branch
    bool bne(uint16_t address);  // Branch if Not Equal, Type: Branch
    bool bpl(uint16_t address);  // Branch if Plus, Type: Branch
    bool brk(uint16_t address);  // Force Break, Type: Jump
    bool bvc(uint16_t address);  // Branch if Overflow Clear, Type: Branch
    bool bvs(uint16_t address);  // Branch if Overflow Set, Type: Branch
    bool clc(uint16_t address);  // Clear Carry Flag, Type: Flags
    bool cld(uint16_t address);  // Clear Decimal, Type: Flags
    bool cli(uint16_t address);  // Clear Interrupt Disable, Type: Flags
    bool clv(uint16_t address);  // Clear Overflow, Type: Flags
    bool cmp(uint16_t address);  // Compare A, Type: Compare
    bool cpx(uint16_t address);  // Compare X, Type: Compare
    bool cpy(uint16_t address);  // Compare Y, Type: Compare
    bool dec(uint16_t address);  // Decrement Memory, Type: Arithmetic
    bool dex(uint16_t address);  // Decrement X, Type: Arithmetic
    bool dey(uint16_t address);  // Decrement Y, Type: Arithmetic
    bool eor(uint16_t address);  // Bitwise Exclusive OR, Type: Bitwise
    bool inc(uint16_t address);  // Increment Memory, Type: Arithmetic
    bool inx(uint16_t address);  // Increment X, Type: Arithmetic
    bool iny(uint16_t address);  // Increment Y, Type: Arithmetic
    bool jmp(uint16_t address);  // Jump to Address, Type: Jump
    bool jsr(uint16_t address);  // Jump to Subroutine, Type: Jump
    bool lda(uint16_t address);  // Load A, Type: Access
    bool ldx(uint16_t address);  // Load X, Type: Access
    bool ldy(uint16_t address);  // Load Y, Type: Access
    bool lsr(uint16_t address);  // Logical Shift Right, Type: Shift
    bool nop(uint16_t address);  // No Operation, Type: Other
    bool ora(uint16_t address);  // Bitwise OR, Type: Bitwise
    bool pha(uint16_t address);  // Push A, Type: Stack
    bool php(uint16_t address);  // Push Processor Status, Type: Stack
    bool pla(uint16_t address);  // Pull A, Type: Stack
    bool plp(uint16_t address);  // Pull Processor Status, Type: Stack
    bool rol(uint16_t address);  // Rotate Left, Type: Shift
    bool ror(uint16_t address);  // Rotate Right, Type: Shift
    bool rti(uint16_t address);  // Return from Interrupt, Type: Jump
    bool rts(uint16_t address);  // Return from Subroutine, Type: Jump
    bool sbc(uint16_t address);  // Subtract with Carry, Type: Arithmetic
    bool sec(uint16_t address);  // Set Carry, Type: Flags
    bool sed(uint16_t address);  // Set Decimal, Type: Flags
    bool sei(uint16_t address);  // Set Interrupt Disable, Type: Flags
    bool sta(uint16_t address);  // Store A, Type: Access
    bool stx(uint16_t address);  // Store X, Type: Access
    bool sty(uint16_t address);  // Store Y, Type: Access
    bool tax(uint16_t address);  // Transfer A to X, Type: Transfer
    bool tay(uint16_t address);  // Transfer A to Y, Type: Transfer
    bool tsx(uint16_t address);  // Transfer Stack Pointer to X, Type: Stack
    bool txa(uint16_t address);  // Transfer X to A, Type: Transfer
    bool txs(uint16_t address);  // Transfer X to Stack Pointer, Type: Stack
    bool tya(uint16_t address);  // Transfer Y to A, Type: Transfer

    bool xxx(uint16_t _); // Illegal opcode, Type: Other

private:
    // Helper functions
    bool _get_flag(status_flag flag);
    void _set_flag(status_flag flag, bool value);

    void _zero_page_add(uint16_t &address, uint8_t register_value);
    bool _absolute_add(uint16_t &address, uint8_t register_value);

private:
    bus_ref_t bus_; // Pointer to the bus

    registers_t registers_; // CPU registers
    uint8_t opcode_; // Current opcode
    uint8_t cycles_; // Cycles remaining

    std::vector<instruction_t> instruction_lookup_;
};

} // namespace NES
