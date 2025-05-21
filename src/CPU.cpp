#include "cpu.h"
#include "bus.h"
#include <_types/_uint16_t.h>
#include <_types/_uint8_t.h>
#include <vector>

namespace NES {


// TODO: maybe refactor in a map
cpu_t::cpu_t(bus_ref_t bus) :
    bus_(bus),
    registers_(),
    cycles_()
{
    instruction_t instructions[] = {
        // 0x0*
        {"BRK", &cpu_t::brk, &cpu_t::immediate, 7},    {"ORA", &cpu_t::ora, &cpu_t::indexed_indirect_x, 6},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::zero_page_x, 8},
        {"???", &cpu_t::nop, &cpu_t::implicit, 3},     {"ORA", &cpu_t::ora, &cpu_t::zero_page, 3},
        {"ASL", &cpu_t::asl, &cpu_t::zero_page, 5},    {"???", &cpu_t::xxx, &cpu_t::implicit, 5},
        {"PHP", &cpu_t::php, &cpu_t::implicit, 3},     {"ORA", &cpu_t::ora, &cpu_t::immediate, 2},
        {"ASL", &cpu_t::asl, &cpu_t::accumulator, 2},  {"???", &cpu_t::xxx, &cpu_t::implicit, 2},
        {"???", &cpu_t::nop, &cpu_t::implicit, 4},     {"ORA", &cpu_t::ora, &cpu_t::absolute, 4},
        {"ASL", &cpu_t::asl, &cpu_t::absolute, 6},     {"???", &cpu_t::xxx, &cpu_t::implicit, 6},

        // 0x1*
        {"BPL", &cpu_t::bpl, &cpu_t::relative, 2},     {"ORA", &cpu_t::ora, &cpu_t::indirect_indexed_y, 5},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 8},
        {"???", &cpu_t::nop, &cpu_t::implicit, 4},     {"ORA", &cpu_t::ora, &cpu_t::zero_page_x, 4},
        {"ASL", &cpu_t::asl, &cpu_t::zero_page_x, 6},  {"???", &cpu_t::xxx, &cpu_t::implicit, 6},
        {"CLC", &cpu_t::clc, &cpu_t::implicit, 2},     {"ORA", &cpu_t::ora, &cpu_t::absolute_y, 4},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 7},
        {"???", &cpu_t::nop, &cpu_t::implicit, 4},     {"ORA", &cpu_t::ora, &cpu_t::absolute_x, 4},
        {"ASL", &cpu_t::asl, &cpu_t::absolute_x, 7},   {"???", &cpu_t::xxx, &cpu_t::implicit, 7},

        // 0x2*
        {"JSR", &cpu_t::jsr, &cpu_t::absolute, 6},     {"AND", &cpu_t::and_, &cpu_t::indexed_indirect_x, 6},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 8},
        {"BIT", &cpu_t::bit, &cpu_t::zero_page, 3},    {"AND", &cpu_t::and_, &cpu_t::zero_page, 3},
        {"ROL", &cpu_t::rol, &cpu_t::zero_page, 5},    {"???", &cpu_t::xxx, &cpu_t::implicit, 5},
        {"PLP", &cpu_t::plp, &cpu_t::implicit, 4},     {"AND", &cpu_t::and_, &cpu_t::immediate, 2},
        {"ROL", &cpu_t::rol, &cpu_t::accumulator, 2},  {"???", &cpu_t::xxx, &cpu_t::implicit, 2},
        {"BIT", &cpu_t::bit, &cpu_t::absolute, 4},     {"AND", &cpu_t::and_, &cpu_t::absolute, 4},
        {"ROL", &cpu_t::rol, &cpu_t::absolute, 6},     {"???", &cpu_t::xxx, &cpu_t::implicit, 6},

        // 0x3*
        {"BMI", &cpu_t::bmi, &cpu_t::relative, 2},     {"AND", &cpu_t::and_, &cpu_t::indirect_indexed_y, 5},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 8},
        {"???", &cpu_t::nop, &cpu_t::implicit, 4},     {"AND", &cpu_t::and_, &cpu_t::zero_page_x, 4},
        {"ROL", &cpu_t::rol, &cpu_t::zero_page_x, 6},  {"???", &cpu_t::xxx, &cpu_t::implicit, 6},
        {"SEC", &cpu_t::sec, &cpu_t::implicit, 2},     {"AND", &cpu_t::and_, &cpu_t::absolute_y, 4},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 7},
        {"???", &cpu_t::nop, &cpu_t::implicit, 4},     {"AND", &cpu_t::and_, &cpu_t::absolute_x, 4},
        {"ROL", &cpu_t::rol, &cpu_t::absolute_x, 7},   {"???", &cpu_t::xxx, &cpu_t::implicit, 7},

        // 0x4*
        {"RTI", &cpu_t::rti, &cpu_t::implicit, 6},     {"EOR", &cpu_t::eor, &cpu_t::indexed_indirect_x, 6},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::zero_page_x, 8},
        {"???", &cpu_t::nop, &cpu_t::implicit, 3},     {"EOR", &cpu_t::eor, &cpu_t::zero_page, 3},
        {"LSR", &cpu_t::lsr, &cpu_t::zero_page, 5},    {"???", &cpu_t::xxx, &cpu_t::implicit, 5},
        {"PHA", &cpu_t::pha, &cpu_t::implicit, 3},     {"EOR", &cpu_t::eor, &cpu_t::immediate, 2},
        {"LSR", &cpu_t::lsr, &cpu_t::accumulator, 2},  {"???", &cpu_t::xxx, &cpu_t::implicit, 2},
        {"JMP", &cpu_t::jmp, &cpu_t::absolute, 3},     {"EOR", &cpu_t::eor, &cpu_t::absolute, 4},
        {"LSR", &cpu_t::lsr, &cpu_t::absolute, 6},     {"???", &cpu_t::xxx, &cpu_t::implicit, 6},

        // 0x5*
        {"BVC", &cpu_t::bvc, &cpu_t::relative, 2},     {"EOR", &cpu_t::eor, &cpu_t::indirect_indexed_y, 5},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 8},
        {"???", &cpu_t::nop, &cpu_t::implicit, 4},     {"EOR", &cpu_t::eor, &cpu_t::zero_page_x, 4},
        {"LSR", &cpu_t::lsr, &cpu_t::zero_page_x, 6},  {"???", &cpu_t::xxx, &cpu_t::implicit, 6},
        {"CLI", &cpu_t::cli, &cpu_t::implicit, 2},     {"EOR", &cpu_t::eor, &cpu_t::absolute_y, 4},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 7},
        {"???", &cpu_t::nop, &cpu_t::implicit, 4},     {"EOR", &cpu_t::eor, &cpu_t::absolute_x, 4},
        {"LSR", &cpu_t::lsr, &cpu_t::absolute_x, 7},   {"???", &cpu_t::xxx, &cpu_t::implicit, 7},

        // 0x6*
        {"RTS", &cpu_t::rts, &cpu_t::implicit, 6},     {"ADC", &cpu_t::adc, &cpu_t::indexed_indirect_x, 6},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::zero_page_x, 8},
        {"???", &cpu_t::nop, &cpu_t::implicit, 3},     {"ADC", &cpu_t::adc, &cpu_t::zero_page, 3},
        {"ROR", &cpu_t::ror, &cpu_t::zero_page, 5},    {"???", &cpu_t::xxx, &cpu_t::implicit, 5},
        {"PLA", &cpu_t::pla, &cpu_t::implicit, 4},     {"ADC", &cpu_t::adc, &cpu_t::immediate, 2},
        {"ROR", &cpu_t::ror, &cpu_t::accumulator, 2},  {"???", &cpu_t::xxx, &cpu_t::implicit, 2},
        {"JMP", &cpu_t::jmp, &cpu_t::indirect, 5},     {"ADC", &cpu_t::adc, &cpu_t::absolute, 4},
        {"ROR", &cpu_t::ror, &cpu_t::absolute, 6},     {"???", &cpu_t::xxx, &cpu_t::implicit, 6},

        // 0x7*
        {"BVS", &cpu_t::bvs, &cpu_t::relative, 2},     {"ADC", &cpu_t::adc, &cpu_t::indirect_indexed_y, 5},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 8},
        {"???", &cpu_t::nop, &cpu_t::implicit, 4},     {"ADC", &cpu_t::adc, &cpu_t::zero_page_x, 4},
        {"ROR", &cpu_t::ror, &cpu_t::zero_page_x, 6},  {"???", &cpu_t::xxx, &cpu_t::implicit, 6},
        {"SEI", &cpu_t::sei, &cpu_t::implicit, 2},     {"ADC", &cpu_t::adc, &cpu_t::absolute_y, 4},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 7},
        {"???", &cpu_t::nop, &cpu_t::implicit, 4},     {"ADC", &cpu_t::adc, &cpu_t::absolute_x, 4},
        {"ROR", &cpu_t::ror, &cpu_t::absolute_x, 7},   {"???", &cpu_t::xxx, &cpu_t::implicit, 7},

        // 0x8*
        {"???", &cpu_t::nop, &cpu_t::implicit, 2},     {"STA", &cpu_t::sta, &cpu_t::indexed_indirect_x, 6},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::zero_page_x, 6},
        {"STY", &cpu_t::sty, &cpu_t::zero_page, 3},    {"STA", &cpu_t::sta, &cpu_t::zero_page, 3},
        {"STX", &cpu_t::stx, &cpu_t::zero_page, 3},    {"???", &cpu_t::xxx, &cpu_t::implicit, 3},
        {"DEY", &cpu_t::dey, &cpu_t::implicit, 2},     {"???", &cpu_t::nop, &cpu_t::implicit, 2},
        {"TXA", &cpu_t::txa, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 2},
        {"STY", &cpu_t::sty, &cpu_t::absolute, 4},     {"STA", &cpu_t::sta, &cpu_t::absolute, 4},
        {"STX", &cpu_t::stx, &cpu_t::absolute, 4},     {"???", &cpu_t::xxx, &cpu_t::implicit, 4},

        // 0x9*
        {"BCC", &cpu_t::bcc, &cpu_t::relative, 2},     {"STA", &cpu_t::sta, &cpu_t::indirect_indexed_y, 6},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 6},
        {"STY", &cpu_t::sty, &cpu_t::zero_page_x, 4},  {"STA", &cpu_t::sta, &cpu_t::zero_page_x, 4},
        {"STX", &cpu_t::stx, &cpu_t::zero_page_y, 4},  {"???", &cpu_t::xxx, &cpu_t::implicit, 4},
        {"TYA", &cpu_t::tya, &cpu_t::implicit, 2},     {"STA", &cpu_t::sta, &cpu_t::absolute_y, 5},
        {"TXS", &cpu_t::txs, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 5},
        {"???", &cpu_t::nop, &cpu_t::implicit, 5},     {"STA", &cpu_t::sta, &cpu_t::absolute_x, 5},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 5},     {"???", &cpu_t::xxx, &cpu_t::implicit, 5},

        // 0xA* 
        {"LDY", &cpu_t::ldy, &cpu_t::immediate, 2},    {"LDA", &cpu_t::lda, &cpu_t::indexed_indirect_x, 6},
        {"LDX", &cpu_t::ldx, &cpu_t::immediate, 2},    {"???", &cpu_t::xxx, &cpu_t::zero_page_x, 6},
        {"LDY", &cpu_t::ldy, &cpu_t::zero_page, 3},    {"LDA", &cpu_t::lda, &cpu_t::zero_page, 3},
        {"LDX", &cpu_t::ldx, &cpu_t::zero_page, 3},    {"???", &cpu_t::xxx, &cpu_t::implicit, 3},
        {"TAY", &cpu_t::tay, &cpu_t::implicit, 2},     {"LDA", &cpu_t::lda, &cpu_t::immediate, 2},
        {"TAX", &cpu_t::tax, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 2},
        {"LDY", &cpu_t::ldy, &cpu_t::absolute, 4},     {"LDA", &cpu_t::lda, &cpu_t::absolute, 4},
        {"LDX", &cpu_t::ldx, &cpu_t::absolute, 4},     {"???", &cpu_t::xxx, &cpu_t::implicit, 4},

        // 0xB*
        {"BCS", &cpu_t::bcs, &cpu_t::relative, 2},     {"LDA", &cpu_t::lda, &cpu_t::indirect_indexed_y, 5},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 5},
        {"LDY", &cpu_t::ldy, &cpu_t::zero_page_x, 4},  {"LDA", &cpu_t::lda, &cpu_t::zero_page_x, 4},
        {"LDX", &cpu_t::ldx, &cpu_t::zero_page_y, 4},  {"???", &cpu_t::xxx, &cpu_t::implicit, 4},
        {"CLV", &cpu_t::clv, &cpu_t::implicit, 2},     {"LDA", &cpu_t::lda, &cpu_t::absolute_y, 4},
        {"TSX", &cpu_t::tsx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 4},
        {"LDY", &cpu_t::ldy, &cpu_t::absolute_x, 4},   {"LDA", &cpu_t::lda, &cpu_t::absolute_x, 4},
        {"LDX", &cpu_t::ldx, &cpu_t::absolute_y, 4},   {"???", &cpu_t::xxx, &cpu_t::implicit, 4},

        // 0xC*
        {"CPY", &cpu_t::cpy, &cpu_t::immediate, 2},    {"CMP", &cpu_t::cmp, &cpu_t::indexed_indirect_x, 6},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::zero_page_x, 8},
        {"CPY", &cpu_t::cpy, &cpu_t::zero_page, 3},    {"CMP", &cpu_t::cmp, &cpu_t::zero_page, 3},
        {"DEC", &cpu_t::dec, &cpu_t::zero_page, 5},    {"???", &cpu_t::xxx, &cpu_t::implicit, 5},
        {"INY", &cpu_t::iny, &cpu_t::implicit, 2},     {"CMP", &cpu_t::cmp, &cpu_t::immediate, 2},
        {"DEX", &cpu_t::dex, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 2},
        {"CPY", &cpu_t::cpy, &cpu_t::absolute, 4},     {"CMP", &cpu_t::cmp, &cpu_t::absolute, 4},
        {"DEC", &cpu_t::dec, &cpu_t::absolute, 6},     {"???", &cpu_t::xxx, &cpu_t::implicit, 6},

        // 0xD*
        {"BNE", &cpu_t::bne, &cpu_t::relative, 2},     {"CMP", &cpu_t::cmp, &cpu_t::indirect_indexed_y, 5},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 8},
        {"???", &cpu_t::nop, &cpu_t::implicit, 4},     {"CMP", &cpu_t::cmp, &cpu_t::zero_page_x, 4},
        {"DEC", &cpu_t::dec, &cpu_t::zero_page_x, 6},  {"???", &cpu_t::xxx, &cpu_t::implicit, 6},
        {"CLD", &cpu_t::cld, &cpu_t::implicit, 2},     {"CMP", &cpu_t::cmp, &cpu_t::absolute_y, 4},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 7},
        {"???", &cpu_t::nop, &cpu_t::implicit, 4},     {"CMP", &cpu_t::cmp, &cpu_t::absolute_x, 4},
        {"DEC", &cpu_t::dec, &cpu_t::absolute_x, 7},   {"???", &cpu_t::xxx, &cpu_t::implicit, 7},

        // 0xE*
        {"CPX", &cpu_t::cpx, &cpu_t::immediate, 2},    {"SBC", &cpu_t::sbc, &cpu_t::indexed_indirect_x, 6},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::zero_page_x, 8},
        {"CPX", &cpu_t::cpx, &cpu_t::zero_page, 3},    {"SBC", &cpu_t::sbc, &cpu_t::zero_page, 3},
        {"INC", &cpu_t::inc, &cpu_t::zero_page, 5},    {"???", &cpu_t::xxx, &cpu_t::implicit, 5},
        {"INX", &cpu_t::inx, &cpu_t::implicit, 2},     {"SBC", &cpu_t::sbc, &cpu_t::immediate, 2},
        {"NOP", &cpu_t::nop, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 2},
        {"CPX", &cpu_t::cpx, &cpu_t::absolute, 4},     {"SBC", &cpu_t::sbc, &cpu_t::absolute, 4},
        {"INC", &cpu_t::inc, &cpu_t::absolute, 6},     {"???", &cpu_t::xxx, &cpu_t::implicit, 6},

        // 0xF*
        {"BEQ", &cpu_t::beq, &cpu_t::relative, 2},     {"SBC", &cpu_t::sbc, &cpu_t::indirect_indexed_y, 5},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 8},
        {"???", &cpu_t::nop, &cpu_t::implicit, 4},     {"SBC", &cpu_t::sbc, &cpu_t::zero_page_x, 4},
        {"INC", &cpu_t::inc, &cpu_t::zero_page_x, 6},  {"???", &cpu_t::xxx, &cpu_t::implicit, 6},
        {"SED", &cpu_t::sed, &cpu_t::implicit, 2},     {"SBC", &cpu_t::sbc, &cpu_t::absolute_y, 4},
        {"???", &cpu_t::xxx, &cpu_t::implicit, 2},     {"???", &cpu_t::xxx, &cpu_t::implicit, 7},
        {"???", &cpu_t::nop, &cpu_t::implicit, 4},     {"SBC", &cpu_t::sbc, &cpu_t::absolute_x, 4},
        {"INC", &cpu_t::inc, &cpu_t::absolute_x, 7},   {"???", &cpu_t::xxx, &cpu_t::implicit, 7}
    };

    instruction_lookup_.reserve(0x100);
    instruction_lookup_.assign(std::begin(instructions), std::end(instructions));
}

cpu_t::~cpu_t()
{
}

void cpu_t::clock()
{
    if (cycles_ == 0)
    {
        const uint8_t opcode = bus_->read(registers_.pc++);

        cycles_ = instruction_lookup_[opcode].cycles;

        is_addressing_accumulator_ = false;

        uint16_t address;
        bool additional_cycle = (this->*instruction_lookup_[opcode].addr_mode)(address);
        additional_cycle &= (this->*instruction_lookup_[opcode].operation)(address);

        cycles_ += (uint8_t)additional_cycle;
    }

    cycles_--;
}

void cpu_t::reset()
{
    registers_ = registers_t();

    // reset works like an interrupt, pushed pc and status to stack but as 0
    // Start SP is 0, after pushin it becomes -3 or 0xFD
    _basic_interrupt(RESET_VECTOR, 8);
}

void cpu_t::interrupt()
{
    if (_get_flag(status_flag::no_interrupts)) return;

    _basic_interrupt(IRQ_VECTOR, 7);
}

void cpu_t::nonmaskable_interrupt()
{
    _basic_interrupt(NMI_VECTOR, 8);
}

// Addressing modes --------------------------------

/* Does not need to read from bus */
bool cpu_t::implicit(uint16_t &address)
{
    return false;
}

bool cpu_t::immediate(uint16_t &address)
{
    address = registers_.pc++;

    return false;
}

/* Instructions operate on the accumulator, no address, works like implicit */
bool cpu_t::accumulator(uint16_t &address)
{
    is_addressing_accumulator_ = true;

    return false;
}

bool cpu_t::zero_page(uint16_t &address)
{
    _zero_page_add(address, 0);
    return false;
}

bool cpu_t::zero_page_x(uint16_t &address)
{
    _zero_page_add(address, registers_.x);
    return false;
}

bool cpu_t::zero_page_y(uint16_t &address)
{
    _zero_page_add(address, registers_.y);
    return false;
}

bool cpu_t::absolute(uint16_t &address)
{
    return _absolute_add(address, 0);
}

bool cpu_t::absolute_x(uint16_t &address)
{
    return _absolute_add(address, registers_.x);
}

bool cpu_t::absolute_y(uint16_t &address)
{
    return _absolute_add(address, registers_.y);
}

bool cpu_t::indirect(uint16_t &address)
{
    const uint8_t direct_addr_low = bus_->read(registers_.pc++);
    const uint8_t direct_addr_high = bus_->read(registers_.pc++);

    const uint16_t direct_addr = (direct_addr_high << 8) + direct_addr_low;

    const uint8_t indirect_addr_low = bus_->read(direct_addr);
    uint8_t indirect_addr_high = bus_->read(direct_addr + 1);

    // Simulate the 6502 bug: 
    // If the low byte is 0xFF, cpu increments it by one but forgets to increment the high byte
    // The address becomes OxHi00 instead of (OxHiLo + 1)
    if (direct_addr_low == 0x00FF) indirect_addr_high = bus_->read(direct_addr & 0xFF00);

    address = (indirect_addr_high << 8) + indirect_addr_low;

    return false;
}

bool cpu_t::indexed_indirect_x(uint16_t &address)
{
    uint8_t direct_zero_page_addr = bus_->read(registers_.pc++);
    direct_zero_page_addr += registers_.x;

    const uint8_t indirect_addr_low = bus_->read(direct_zero_page_addr++); // Since indirect_addr_low is 8bit it will wrap around
    const uint8_t indirect_addr_high = bus_->read(direct_zero_page_addr);

    address = (indirect_addr_high << 8) + indirect_addr_low;

    return false;
}

bool cpu_t::indirect_indexed_y(uint16_t &address)
{
    uint8_t direct_zero_page_addr = bus_->read(registers_.pc++);

    const uint8_t indirect_addr_low = bus_->read(direct_zero_page_addr++); // Since indirect_addr_low is 8bit it will wrap around
    const uint8_t indirect_addr_high = bus_->read(direct_zero_page_addr);

    const uint16_t sum_low = indirect_addr_low + registers_.y;

    address = (indirect_addr_high << 8) + sum_low;

    return sum_low > 0x00FF;
}

bool cpu_t::relative(uint16_t &address)
{
    address = bus_->read(registers_.pc++);

    // If the sign bit is set
    if (address & 0x80) address |= 0xFF00; // Negative address

    // If page crossed, extra cycle needed
    return (registers_.pc & 0xFF00) != ((registers_.pc + address) & 0xFF00);
}

// Instructions --------------------------------------------

bool cpu_t::adc(const uint16_t address)
{
    const uint16_t value = bus_->read(address);
    const uint16_t sum = registers_.a + value + _get_flag(status_flag::carry);

    _set_flag(status_flag::carry, sum > 0xFF);
    _set_flag(status_flag::zero, (sum & 0xFF) == 0);
    _set_flag(status_flag::overflow, ((registers_.a ^ sum) & (value ^ sum)) >> 7);
    _set_flag(status_flag::negative, sum & (1 << 7));

    registers_.a = sum & 0xFF;
    return true;
}

bool cpu_t::and_(const uint16_t address)
{
    registers_.a &= bus_->read(address);

    _set_flag(status_flag::zero, registers_.a == 0);
    _set_flag(status_flag::negative, registers_.a >> 7);

    return true;
}

bool cpu_t::asl(const uint16_t address)
{
    uint8_t value = is_addressing_accumulator_ ? registers_.a : bus_->read(address);

    _set_flag(status_flag::carry, value >> 7);

    value <<= 1;

    _set_flag(status_flag::zero, value == 0);
    _set_flag(status_flag::negative, value >> 7);

    if (is_addressing_accumulator_) registers_.a = value;
    else bus_->write(address, value);

    return false;
}

bool cpu_t::bcc(const uint16_t address)
{
    return _relative_jump(!_get_flag(status_flag::carry), address);
}

bool cpu_t::bcs(const uint16_t address)
{
    return _relative_jump(_get_flag(status_flag::carry), address);
}

bool cpu_t::beq(const uint16_t address)
{
    return _relative_jump(_get_flag(status_flag::zero), address);
}

bool cpu_t::bit(const uint16_t address)
{
    const uint8_t test_value = registers_.a & bus_->read(address);

    _set_flag(status_flag::zero, test_value == 0);
    _set_flag(status_flag::overflow, (bus_->read(address) >> 6) & 0x01);
    _set_flag(status_flag::negative, (bus_->read(address) >> 7));

    return false;
}

bool cpu_t::bmi(const uint16_t address)
{
    return _relative_jump(_get_flag(status_flag::negative), address);
}

bool cpu_t::bne(const uint16_t address)
{
    return _relative_jump(!_get_flag(status_flag::zero), address);
}

bool cpu_t::bpl(const uint16_t address)
{
    return _relative_jump(!_get_flag(status_flag::negative), address);
}

bool cpu_t::brk(const uint16_t address)
{
    registers_.pc++; // BRK skippes a byte, often considered a two byte instruction

    _push_stack(registers_.pc >> 8);
    _push_stack(registers_.pc & 0x00FF);

    _push_stack(registers_.status | status_flag::b | status_flag::unused);

    _set_flag(status_flag::no_interrupts, true);

    registers_.pc = bus_->read(IRQ_VECTOR) + (bus_->read(IRQ_VECTOR + 1) << 8);

    return false;
}

bool cpu_t::bvc(const uint16_t address)
{
    return _relative_jump(!_get_flag(status_flag::overflow), address);
}

bool cpu_t::bvs(const uint16_t address)
{
    return _relative_jump(_get_flag(status_flag::overflow), address);
}

bool cpu_t::clc(const uint16_t address)
{
    _set_flag(status_flag::carry, false);

    return false;
}

bool cpu_t::cld(const uint16_t address)
{
    _set_flag(status_flag::decimal, false);

    return false;
}

bool cpu_t::cli(const uint16_t address)
{
    _set_flag(status_flag::no_interrupts, false);

    return false;
}

bool cpu_t::clv(const uint16_t address)
{
    _set_flag(status_flag::overflow, false);

    return false;
}

bool cpu_t::cmp(const uint16_t address)
{
    const uint8_t result = registers_.a - bus_->read(address);

    _set_flag(status_flag::carry, (int8_t)result >= 0);
    _set_flag(status_flag::zero, result == 0);
    _set_flag(status_flag::negative, result >> 7);

    return true;
}

bool cpu_t::cpx(const uint16_t address)
{
    const uint8_t result = registers_.x - bus_->read(address);

    _set_flag(status_flag::carry, (int8_t)result >= 0);
    _set_flag(status_flag::zero, result == 0);
    _set_flag(status_flag::negative, result >> 7);

    return false;
}

bool cpu_t::cpy(const uint16_t address)
{
    const uint8_t result = registers_.y - bus_->read(address);

    _set_flag(status_flag::carry, (int8_t)result >= 0);
    _set_flag(status_flag::zero, result == 0);
    _set_flag(status_flag::negative, result >> 7);

    return false;
}

bool cpu_t::dec(const uint16_t address)
{
    const uint8_t value = bus_->read(address) - 1;

    _set_flag(status_flag::zero, value == 0);
    _set_flag(status_flag::negative, value >> 7);

    bus_->write(address, value);

    return false;
}

bool cpu_t::dex(const uint16_t address)
{
    registers_.x--;

    _set_flag(status_flag::zero, registers_.x == 0);
    _set_flag(status_flag::negative, registers_.x >> 7);

    return false;
}

bool cpu_t::dey(const uint16_t address)
{
    registers_.y--;

    _set_flag(status_flag::zero, registers_.y == 0);
    _set_flag(status_flag::negative, registers_.y >> 7);

    return false;
}

bool cpu_t::eor(const uint16_t address)
{
    registers_.a ^= bus_->read(address);

    _set_flag(status_flag::zero, registers_.a == 0);
    _set_flag(status_flag::negative, registers_.a >> 7);

    return true;
}

bool cpu_t::inc(const uint16_t address)
{
    const uint8_t value = bus_->read(address) + 1;

    _set_flag(status_flag::zero, value == 0);
    _set_flag(status_flag::negative, value >> 7);

    bus_->write(address, value);

    return false;
}

bool cpu_t::inx(const uint16_t address)
{
    registers_.x++;

    _set_flag(status_flag::zero, registers_.x == 0);
    _set_flag(status_flag::negative, registers_.x >> 7);

    return false;
}

bool cpu_t::iny(const uint16_t address)
{
    registers_.y++;

    _set_flag(status_flag::zero, registers_.y == 0);
    _set_flag(status_flag::negative, registers_.y >> 7);

    return false;
}

bool cpu_t::jmp(const uint16_t address)
{
    registers_.pc = address;

    return false;
}

bool cpu_t::jsr(const uint16_t address)
{
    registers_.pc--; // We need to push pc before next instruction, RTS will increment it

    _push_stack(registers_.pc >> 8);
    _push_stack(registers_.pc & 0x00FF);

    registers_.pc = address;

    return false;
}

bool cpu_t::lda(const uint16_t address)
{
    registers_.a = bus_->read(address);

    _set_flag(status_flag::zero, registers_.a == 0);
    _set_flag(status_flag::negative, registers_.a >> 7);

    return true;
}

bool cpu_t::ldx(const uint16_t address)
{
    registers_.x = bus_->read(address);

    _set_flag(status_flag::zero, registers_.x == 0);
    _set_flag(status_flag::negative, registers_.x >> 7);

    return true;
}

bool cpu_t::ldy(const uint16_t address)
{
    registers_.y = bus_->read(address);

    _set_flag(status_flag::zero, registers_.y == 0);
    _set_flag(status_flag::negative, registers_.y >> 7);

    return true;
}

bool cpu_t::lsr(const uint16_t address)
{
    uint8_t value = is_addressing_accumulator_ ? registers_.a : bus_->read(address);

    _set_flag(status_flag::carry, value & 0x01);

    value = (value >> 1) & 0x7F; // 0 is added to bit 7 

    _set_flag(status_flag::zero, value == 0);
    _set_flag(status_flag::negative, value >> 7);

    if (is_addressing_accumulator_) registers_.a = value;
    else bus_->write(address, value);

    return false;
}

bool cpu_t::nop(const uint16_t address)
{
    return true; // If addressing mode is absolute_x it might need an extra cycle
}

bool cpu_t::ora(const uint16_t address)
{
    registers_.a |= bus_->read(address);

    _set_flag(status_flag::zero, registers_.a == 0);
    _set_flag(status_flag::negative, registers_.a >> 7);

    return true;
}

bool cpu_t::pha(const uint16_t address)
{
    _push_stack(registers_.a);
    return false;
}

bool cpu_t::php(const uint16_t address)
{
    // Documantation says to set unused bit to 1 in stack
    _push_stack(registers_.status | status_flag::b | status_flag::unused);
    return false;
}

bool cpu_t::pla(const uint16_t address)
{
    registers_.a = _pop_stack();

    _set_flag(status_flag::zero, registers_.a == 0);
    _set_flag(status_flag::negative, registers_.a >> 7);

    return false;
}

bool cpu_t::plp(const uint16_t address)
{
    registers_.status = _pop_stack();

    return false;
}

bool cpu_t::rol(const uint16_t address)
{
    uint8_t value = is_addressing_accumulator_ ? registers_.a : bus_->read(address);
    const bool carry = _get_flag(status_flag::carry);

    _set_flag(status_flag::carry, value >> 7);

    value = (value << 1) | carry;

    _set_flag(status_flag::zero, value == 0);
    _set_flag(status_flag::negative, value >> 7);

    if (is_addressing_accumulator_) registers_.a = value;
    else bus_->write(address, value);

    return false;
}

bool cpu_t::ror(const uint16_t address)
{
    uint8_t value = is_addressing_accumulator_ ? registers_.a : bus_->read(address);
    const bool carry = _get_flag(status_flag::carry);

    _set_flag(status_flag::carry, value & 0x01);

    value = (value >> 1) | (carry << 7);

    _set_flag(status_flag::zero, value == 0);
    _set_flag(status_flag::negative, value >> 7);

    if (is_addressing_accumulator_) registers_.a = value;
    else bus_->write(address, value);

    return false;
}

bool cpu_t::rti(const uint16_t address)
{
    registers_.status = _pop_stack();

    registers_.pc = _pop_stack();
    registers_.pc |= _pop_stack() << 8;

    return false;
}

bool cpu_t::rts(const uint16_t address)
{
    registers_.pc = _pop_stack();
    registers_.pc |= _pop_stack() << 8;

    registers_.pc++;

    return false;
}


bool cpu_t::sbc(const uint16_t address)
{
    const uint8_t value = bus_->read(address);
    const int16_t sub = registers_.a - value - !_get_flag(status_flag::carry);

    _set_flag(status_flag::carry, ~(sub < 0));
    _set_flag(status_flag::zero, (sub & 0xFF) == 0);
    _set_flag(status_flag::overflow, ((registers_.a ^ sub) & (value ^ sub)) >> 7);
    _set_flag(status_flag::negative, sub & (1 << 7));

    return true;
}

bool cpu_t::sec(const uint16_t address)
{
    _set_flag(status_flag::carry, true);

    return false;
}

bool cpu_t::sed(const uint16_t address)
{
    _set_flag(status_flag::decimal, true);

    return false;
}

bool cpu_t::sei(const uint16_t address)
{
    _set_flag(status_flag::no_interrupts, true);

    return false;
}

bool cpu_t::sta(const uint16_t address)
{
    bus_->write(address, registers_.a);
    return false;
}

bool cpu_t::stx(const uint16_t address)
{
    bus_->write(address, registers_.x);
    return false;
}

bool cpu_t::sty(const uint16_t address)
{
    bus_->write(address, registers_.y);
    return false;
}

bool cpu_t::tax(const uint16_t address)
{
    registers_.x = registers_.a;

    _set_flag(status_flag::zero, registers_.x == 0);
    _set_flag(status_flag::negative, registers_.x >> 7);

    return false;
}

bool cpu_t::tay(const uint16_t address)
{
    registers_.y = registers_.a;

    _set_flag(status_flag::zero, registers_.y == 0);
    _set_flag(status_flag::negative, registers_.y >> 7);

    return false;
}

bool cpu_t::tsx(const uint16_t address)
{
    registers_.x = registers_.sp;

    _set_flag(status_flag::zero, registers_.x == 0);
    _set_flag(status_flag::negative, registers_.x >> 7);

    return false;
}

bool cpu_t::txa(const uint16_t address)
{
    registers_.a = registers_.x;

    _set_flag(status_flag::zero, registers_.a == 0);
    _set_flag(status_flag::negative, registers_.a >> 7);

    return false;
}

bool cpu_t::txs(const uint16_t address)
{
    registers_.sp = registers_.x;

    _set_flag(status_flag::zero, registers_.sp == 0);
    _set_flag(status_flag::negative, registers_.sp >> 7);

    return false;
}

bool cpu_t::tya(const uint16_t address)
{
    registers_.a = registers_.y;

    _set_flag(status_flag::zero, registers_.a == 0);
    _set_flag(status_flag::negative, registers_.a >> 7);

    return false;
}

bool cpu_t::xxx(const uint16_t _)
{
    return false;
}

// Helper functions --------------------------------
bool cpu_t::_get_flag(const status_flag flag)
{
    return registers_.status & flag;
}

void cpu_t::_set_flag(const status_flag flag, const bool value)
{
    if (value) registers_.status |= flag;
    else registers_.status &= ~flag;
}

void cpu_t::_push_stack(const uint8_t value)
{
    bus_->write(STACK_BASE + registers_.sp--, value);
}

uint8_t cpu_t::_pop_stack()
{
    return bus_->read(STACK_BASE + ++registers_.sp);
}

void cpu_t::_basic_interrupt(const uint16_t vector, const uint8_t cycles)
{
    _push_stack(registers_.pc >> 8);
    _push_stack(registers_.pc & 0x00FF);
    _push_stack(registers_.status);

    registers_.pc = bus_->read(vector) + (bus_->read(vector + 1) << 8);

    _set_flag(status_flag::unused, true);
    _set_flag(status_flag::no_interrupts, true);

    cycles_ = cycles;
}

void cpu_t::_zero_page_add(uint16_t &address, const uint8_t register_value)
{
    uint8_t addr_zero_page = bus_->read(registers_.pc++);
    address = (addr_zero_page + register_value) & 0x00FF;
}

bool cpu_t::_absolute_add(uint16_t &address, const uint8_t register_value)
{
    uint16_t addr_low = bus_->read(registers_.pc++);
    uint16_t addr_high = bus_->read(registers_.pc++);

    uint16_t sum_low = addr_low + register_value;

    address = (addr_high << 8) + sum_low;

    return sum_low > 0x00FF;
}

bool cpu_t::_relative_jump(const bool condition, const uint16_t address)
{
    if (!condition) return false;

    cycles_++;
    registers_.pc += address;

    return true;
}

}