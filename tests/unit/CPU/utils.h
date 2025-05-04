#include <vector>
#include "bus.h"
#include "cpu.h"

namespace NES_test {

class bus_stub_t : public NES::i_bus_t
{
public:
    bus_stub_t() : size_(sizeof(ram))
    {
        for (int i = 0; i < 0x1000; i++)
            ram[i] = 0x00;
    }

    void write(uint16_t addr, uint8_t data) override
    {
        if (addr < size_)
            ram[addr] = data;
    }

    uint8_t read(uint16_t addr) override
    {
        if (addr < size_)
            return ram[addr];

        return 0x00;
    }

private:
    uint8_t ram[0xFFFF];
    uint16_t size_;
};

typedef std::shared_ptr<NES::cpu_t> cpu_ref_t;

} // namespace NES_test
