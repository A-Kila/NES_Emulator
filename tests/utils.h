#include <vector>
#include "nes/interfaces/bus.h"
#include "nes/cpu.h"
#include "nes/joypad.h"

namespace NES_test {

class bus_stub_t : public NES::i_bus_t
{
public:
    bus_stub_t() : size_(sizeof(ram))
    {
        for (int i = 0; i < size_; i++)
            if (i >= 0x4000 && i < 0x4017) ram[i] = 0xFF; // APU
            else ram[i] = 0x00;
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

    bool load_cartige(const std::vector<uint8_t> &data, const uint16_t start_addr = 0x0000)
    {
        if (start_addr + data.size() > 0x10000)
            return false;

        for (uint32_t i = 0; i < data.size(); i++)
        {
            ram[i + start_addr] = data[i];
        }

        return true;
    }

private:
    uint8_t ram[0x10000];
    uint32_t size_;
};

class joypad_stub_t : public NES::joypad_t
{
    void update_keys() override {}
};

typedef std::shared_ptr<NES::cpu_t> cpu_ref_t;

} // namespace NES_test
