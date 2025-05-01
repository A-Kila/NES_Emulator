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
    opcode_(),
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
        // FIXME: opcode_ var could probably be a local variable instead of a class var
        opcode_ = bus_->read(registers_.pc);

        registers_.pc++;
        cycles_ = instruction_lookup_[opcode_].cycles;

        uint16_t address;
        bool additional_cycle = false;
        additional_cycle |= (this->*instruction_lookup_[opcode_].addr_mode)(address);
        additional_cycle |= (this->*instruction_lookup_[opcode_].operation)(address);

        cycles_ += (uint8_t)additional_cycle;
    }

    cycles_--;
}

void cpu_t::reset()
{
}

void cpu_t::interrupt()
{
}

void cpu_t::nonmaskable_interrupt()
{
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
    uint8_t direct_addr_low = bus_->read(registers_.pc++);
    uint8_t direct_addr_high = bus_->read(registers_.pc++);

    uint16_t direct_addr = (direct_addr_high << 8) + direct_addr_low;

    uint8_t indirect_addr_low = bus_->read(direct_addr);
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

    uint8_t indirect_addr_low = bus_->read(direct_zero_page_addr++); // Since indirect_addr_low is 8bit it will wrap around
    uint8_t indirect_addr_high = bus_->read(direct_zero_page_addr);

    address = (indirect_addr_high << 8) + indirect_addr_low;

    return false;
}

bool cpu_t::indirect_indexed_y(uint16_t &address)
{
    uint8_t direct_zero_page_addr = bus_->read(registers_.pc++);

    uint8_t indirect_addr_low = bus_->read(direct_zero_page_addr++); // Since indirect_addr_low is 8bit it will wrap around
    uint8_t indirect_addr_high = bus_->read(direct_zero_page_addr);

    uint16_t sum_low = indirect_addr_low + registers_.y;

    address = (indirect_addr_high << 8) + sum_low;

    return sum_low > 0x00FF;
}

bool cpu_t::relative(uint16_t &address)
{
    address = bus_->read(registers_.pc++);

    // If the sign bit is set
    if (address & 0x80) address |= 0xFF00; // Negative address

    return false;
}

// Instructions --------------------------------------------

bool cpu_t::adc(uint16_t address)
{
    return false;
}

bool cpu_t::and_(uint16_t address)
{
    return false;
}

bool cpu_t::asl(uint16_t address)
{
    return false;
}

bool cpu_t::bcc(uint16_t address)
{
    return false;
}

bool cpu_t::bcs(uint16_t address)
{
    return false;
}

bool cpu_t::beq(uint16_t address)
{
    return false;
}

bool cpu_t::bit(uint16_t address)
{
    return false;
}

bool cpu_t::bmi(uint16_t address)
{
    return false;
}

bool cpu_t::bne(uint16_t address)
{
    return false;
}

bool cpu_t::bpl(uint16_t address)
{
    return false;
}

bool cpu_t::brk(uint16_t address)
{
    return false;
}

bool cpu_t::bvc(uint16_t address)
{
    return false;
}

bool cpu_t::bvs(uint16_t address)
{
    return false;
}

bool cpu_t::clc(uint16_t address)
{
    return false;
}

bool cpu_t::cld(uint16_t address)
{
    return false;
}

bool cpu_t::cli(uint16_t address)
{
    return false;
}

bool cpu_t::clv(uint16_t address)
{
    return false;
}

bool cpu_t::cmp(uint16_t address)
{
    return false;
}

bool cpu_t::cpx(uint16_t address)
{
    return false;
}

bool cpu_t::cpy(uint16_t address)
{
    return false;
}

bool cpu_t::dec(uint16_t address)
{
    return false;
}

bool cpu_t::dex(uint16_t address)
{
    return false;
}

bool cpu_t::dey(uint16_t address)
{
    return false;
}

bool cpu_t::eor(uint16_t address)
{
    return false;
}

bool cpu_t::inc(uint16_t address)
{
    return false;
}

bool cpu_t::inx(uint16_t address)
{
    return false;
}

bool cpu_t::iny(uint16_t address)
{
    return false;
}

bool cpu_t::jmp(uint16_t address)
{
    return false;
}

bool cpu_t::jsr(uint16_t address)
{
    return false;
}

bool cpu_t::lda(uint16_t address)
{
    registers_.a = bus_->read(address);

    // set_flag()

    return false;
}

bool cpu_t::ldx(uint16_t address)
{
    return false;
}

bool cpu_t::ldy(uint16_t address)
{
    return false;
}

bool cpu_t::lsr(uint16_t address)
{
    return false;
}

bool cpu_t::nop(uint16_t address)
{
    return false;
}

bool cpu_t::ora(uint16_t address)
{
    return false;
}

bool cpu_t::pha(uint16_t address)
{
    return false;
}

bool cpu_t::php(uint16_t address)
{
    return false;
}

bool cpu_t::pla(uint16_t address)
{
    return false;
}

bool cpu_t::plp(uint16_t address)
{
    return false;
}

bool cpu_t::rol(uint16_t address)
{
    return false;
}

bool cpu_t::ror(uint16_t address)
{
    return false;
}

bool cpu_t::rti(uint16_t address)
{
    return false;
}

bool cpu_t::rts(uint16_t address)
{
    return false;
}

bool cpu_t::sbc(uint16_t address)
{
    return false;
}

bool cpu_t::sec(uint16_t address)
{
    return false;
}

bool cpu_t::sed(uint16_t address)
{
    return false;
}

bool cpu_t::sei(uint16_t address)
{
    return false;
}

bool cpu_t::sta(uint16_t address)
{
    return false;
}

bool cpu_t::stx(uint16_t address)
{
    return false;
}

bool cpu_t::sty(uint16_t address)
{
    return false;
}

bool cpu_t::tax(uint16_t address)
{
    return false;
}

bool cpu_t::tay(uint16_t address)
{
    return false;
}

bool cpu_t::tsx(uint16_t address)
{
    return false;
}

bool cpu_t::txa(uint16_t address)
{
    return false;
}

bool cpu_t::txs(uint16_t address)
{
    return false;
}

bool cpu_t::tya(uint16_t address)
{
    return false;
}

bool cpu_t::xxx(uint16_t _)
{
    return false;
}

// Helper functions --------------------------------
bool cpu_t::_get_flag(status_flag flag)
{
    return registers_.status & flag;
}

void cpu_t::_set_flag(status_flag flag, bool value)
{
    if (value) registers_.status |= flag;
    else registers_.status &= ~flag;
}

void cpu_t::_zero_page_add(uint16_t &address, uint8_t register_value)
{
    uint8_t addr_zero_page = bus_->read(registers_.pc++);
    address = (addr_zero_page + register_value) & 0x00FF;
}

bool cpu_t::_absolute_add(uint16_t &address, uint8_t register_value)
{
    uint16_t addr_low = bus_->read(registers_.pc++);
    uint16_t addr_high = bus_->read(registers_.pc++);

    uint16_t sum_low = addr_low + register_value;

    address = (addr_high << 8) + sum_low;

    return sum_low > 0x00FF;
}

}
