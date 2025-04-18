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
        bool (cpu_t:: *operation)();
        bool (cpu_t:: *addr_mode)();
        uint8_t cycles;
    };

private:
    // Addressing modes (returns true if additional clock cycles are needed)
    bool implicit();
    bool immediate();
    bool accumulator();
    bool zero_page();
    bool zero_page_x();
    bool zero_page_y();
    bool absolute();
    bool absolute_x();
    bool absolute_y();
    bool indirect();
    bool indexed_indirect_x();
    bool indirect_indexed_y();
    bool relative();

    // Instructions (returns true if additional clock cycles are needed)
    bool adc();  // Add with carry, Type: Arithmetic
    bool and_(); // Bitwise AND, Type: Bitwise
    bool asl();  // Arithmetic Shift Left, Type: Shift
    bool bcc();  // Branch if Carry Clear, Type: Branch
    bool bcs();  // Branch if Carry Set, Type: Branch
    bool beq();  // Branch if Equal, Type: Branch
    bool bit();  // Bit Test, Type: Bitwise
    bool bmi();  // Branch if Minus, Type: Branch
    bool bne();  // Branch if Not Equal, Type: Branch
    bool bpl();  // Branch if Plus, Type: Branch
    bool brk();  // Force Break, Type: Jump
    bool bvc();  // Branch if Overflow Clear, Type: Branch
    bool bvs();  // Branch if Overflow Set, Type: Branch
    bool clc();  // Clear Carry Flag, Type: Flags
    bool cld();  // Clear Decimal, Type: Flags
    bool cli();  // Clear Interrupt Disable, Type: Flags
    bool clv();  // Clear Overflow, Type: Flags
    bool cmp();  // Compare A, Type: Compare
    bool cpx();  // Compare X, Type: Compare
    bool cpy();  // Compare Y, Type: Compare
    bool dec();  // Decrement Memory, Type: Arithmetic
    bool dex();  // Decrement X, Type: Arithmetic
    bool dey();  // Decrement Y, Type: Arithmetic
    bool eor();  // Bitwise Exclusive OR, Type: Bitwise
    bool inc();  // Increment Memory, Type: Arithmetic
    bool inx();  // Increment X, Type: Arithmetic
    bool iny();  // Increment Y, Type: Arithmetic
    bool jmp();  // Jump to Address, Type: Jump
    bool jsr();  // Jump to Subroutine, Type: Jump
    bool lda();  // Load A, Type: Access
    bool ldx();  // Load X, Type: Access
    bool ldy();  // Load Y, Type: Access
    bool lsr();  // Logical Shift Right, Type: Shift
    bool nop();  // No Operation, Type: Other
    bool ora();  // Bitwise OR, Type: Bitwise
    bool pha();  // Push A, Type: Stack
    bool php();  // Push Processor Status, Type: Stack
    bool pla();  // Pull A, Type: Stack
    bool plp();  // Pull Processor Status, Type: Stack
    bool rol();  // Rotate Left, Type: Shift
    bool ror();  // Rotate Right, Type: Shift
    bool rti();  // Return from Interrupt, Type: Jump
    bool rts();  // Return from Subroutine, Type: Jump
    bool sbc();  // Subtract with Carry, Type: Arithmetic
    bool sec();  // Set Carry, Type: Flags
    bool sed();  // Set Decimal, Type: Flags
    bool sei();  // Set Interrupt Disable, Type: Flags
    bool sta();  // Store A, Type: Access
    bool stx();  // Store X, Type: Access
    bool sty();  // Store Y, Type: Access
    bool tax();  // Transfer A to X, Type: Transfer
    bool tay();  // Transfer A to Y, Type: Transfer
    bool tsx();  // Transfer Stack Pointer to X, Type: Stack
    bool txa();  // Transfer X to A, Type: Transfer
    bool txs();  // Transfer X to Stack Pointer, Type: Stack
    bool tya();  // Transfer Y to A, Type: Transfer

    bool xxx(); // Illegal opcode, Type: Other

    // Helper functions
    bool get_flag(status_flag flag);
    void set_flag(status_flag flag, bool value);

    void zero_page_add(uint8_t register_value);
    bool absolute_add(uint8_t register_value);

private:
    bus_ref_t bus_; // Pointer to the bus

    registers_t registers_; // CPU registers
    uint16_t addr_absolute_; // Absolute address
    uint16_t addr_relative_; // Relative address
    uint8_t opcode_; // Current opcode
    uint8_t cycles_; // Cycles remaining

    std::vector<instruction_t> instruction_lookup_;
};

} // namespace NES