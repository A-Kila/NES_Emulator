#pragma once

#include "interfaces/bus.h"
#include "ppu.h"

namespace NES {

class dma_transfer_t
{
public:
    dma_transfer_t(ppu_ref_t ppu);
    ~dma_transfer_t();

    void set_bus(bus_ref_t bus_);
    void clock(uint32_t cycle);

    void start_dma(uint8_t page);
    bool is_dma_started();

private:
    void _reset();

private:
    bus_weak_ref_t bus_;
    ppu_ref_t ppu_;

    bool dma_sync_cycle_;
    bool is_dma_started_;
    uint8_t oam_addr_;
    uint8_t oam_data_;
    uint8_t bus_page_;
};

typedef std::shared_ptr<dma_transfer_t> dma_transfer_ref_t;

} // namespace NES