#include "cpu.h"

namespace NES_test {

class testable_cpu_t : public NES::cpu_t
{
public:
    testable_cpu_t(NES::bus_ref_t bus) : cpu_t(bus) {}

    void implicit() { cpu_t::implicit(); }
    void immediate() { cpu_t::immediate(); }
    void accumulator() { cpu_t::accumulator(); }
    void zero_page() { cpu_t::zero_page(); }
    void zero_page_x() { cpu_t::zero_page_x(); }
    void zero_page_y() { cpu_t::zero_page_y(); }
    void absolute() { cpu_t::absolute(); }
    void absolute_x() { cpu_t::absolute_x(); }
    void absolute_y() { cpu_t::absolute_y(); }
    void indexed_indirect_x() { cpu_t::indexed_indirect_x(); }
    void indirect_indexed_y() { cpu_t::indirect_indexed_y(); }
    void relative() { cpu_t::relative(); }

};

}
