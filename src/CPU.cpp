#include "cpu.h"
#include "bus.h"
#include <vector>

namespace NES {


// TODO: maybe refactor in a map
cpu_t::cpu_t(bus_ref_t bus) :
    bus_(bus),
    registers_(),
    fetched_(),
    addr_absolute_(),
    addr_relative_(),
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
        opcode_ = bus_->read(registers_.pc);

        registers_.pc++;
        cycles_ = instruction_lookup_[opcode_].cycles;

        bool additional_cycle = false;
        additional_cycle |= (this->*instruction_lookup_[opcode_].addr_mode)();
        additional_cycle |= (this->*instruction_lookup_[opcode_].operation)();

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
bool cpu_t::implicit()
{
    return false;
}

bool cpu_t::immediate()
{
    return false;
}

bool cpu_t::accumulator()
{
    return false;
}

bool cpu_t::zero_page()
{
    return false;
}

bool cpu_t::zero_page_x()
{
    return false;
}

bool cpu_t::zero_page_y()
{
    return false;
}

bool cpu_t::absolute()
{
    return false;
}

bool cpu_t::absolute_x()
{
    return false;
}

bool cpu_t::absolute_y()
{
    return false;
}

bool cpu_t::indirect()
{
    return false;
}

bool cpu_t::indexed_indirect_x()
{
    return false;
}

bool cpu_t::indirect_indexed_y()
{
    return false;
}

bool cpu_t::relative()
{
    return false;
}

// Instructions --------------------------------------------

bool cpu_t::adc()
{
    return false;
}

bool cpu_t::and_()
{
    return false;
}

bool cpu_t::asl()
{
    return false;
}

bool cpu_t::bcc()
{
    return false;
}

bool cpu_t::bcs()
{
    return false;
}

bool cpu_t::beq()
{
    return false;
}

bool cpu_t::bit()
{
    return false;
}

bool cpu_t::bmi()
{
    return false;
}

bool cpu_t::bne()
{
    return false;
}

bool cpu_t::bpl()
{
    return false;
}

bool cpu_t::brk()
{
    return false;
}

bool cpu_t::bvc()
{
    return false;
}

bool cpu_t::bvs()
{
    return false;
}

bool cpu_t::clc()
{
    return false;
}

bool cpu_t::cld()
{
    return false;
}

bool cpu_t::cli()
{
    return false;
}

bool cpu_t::clv()
{
    return false;
}

bool cpu_t::cmp()
{
    return false;
}

bool cpu_t::cpx()
{
    return false;
}

bool cpu_t::cpy()
{
    return false;
}

bool cpu_t::dec()
{
    return false;
}

bool cpu_t::dex()
{
    return false;
}

bool cpu_t::dey()
{
    return false;
}

bool cpu_t::eor()
{
    return false;
}

bool cpu_t::inc()
{
    return false;
}

bool cpu_t::inx()
{
    return false;
}

bool cpu_t::iny()
{
    return false;
}

bool cpu_t::jmp()
{
    return false;
}

bool cpu_t::jsr()
{
    return false;
}

bool cpu_t::lda()
{
    return false;
}

bool cpu_t::ldx()
{
    return false;
}

bool cpu_t::ldy()
{
    return false;
}

bool cpu_t::lsr()
{
    return false;
}

bool cpu_t::nop()
{
    return false;
}

bool cpu_t::ora()
{
    return false;
}

bool cpu_t::pha()
{
    return false;
}

bool cpu_t::php()
{
    return false;
}

bool cpu_t::pla()
{
    return false;
}

bool cpu_t::plp()
{
    return false;
}

bool cpu_t::rol()
{
    return false;
}

bool cpu_t::ror()
{
    return false;
}

bool cpu_t::rti()
{
    return false;
}

bool cpu_t::rts()
{
    return false;
}

bool cpu_t::sbc()
{
    return false;
}

bool cpu_t::sec()
{
    return false;
}

bool cpu_t::sed()
{
    return false;
}

bool cpu_t::sei()
{
    return false;
}

bool cpu_t::sta()
{
    return false;
}

bool cpu_t::stx()
{
    return false;
}

bool cpu_t::sty()
{
    return false;
}

bool cpu_t::tax()
{
    return false;
}

bool cpu_t::tay()
{
    return false;
}

bool cpu_t::tsx()
{
    return false;
}

bool cpu_t::txa()
{
    return false;
}

bool cpu_t::txs()
{
    return false;
}

bool cpu_t::tya()
{
    return false;
}

bool cpu_t::xxx()
{
    return false;
}

}