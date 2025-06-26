#include "dma_transfer.h"
#include <cassert>

namespace NES {

dma_transfer_t::dma_transfer_t(ppu_ref_t ppu) :
    ppu_(ppu),
    bus_(),
    dma_sync_cycle_(true),
    is_dma_started_(),
    oam_addr_(),
    oam_data_(),
    bus_page_()
{
}

dma_transfer_t::~dma_transfer_t()
{
}

void dma_transfer_t::set_bus(bus_ref_t bus)
{
    bus_ = bus;
}

void dma_transfer_t::clock(uint32_t cycles)
{
    assert(is_dma_started_ && !bus_.expired() && "DMA transfer has not been started or bus not set, should not be clocked");

    // first 1 or 2 cycles are used to sync up
    if (dma_sync_cycle_)
    {
        dma_sync_cycle_ = cycles % 2 == 0; // last cycle is odd
        return;
    }

    if (cycles % 2 == 0)
    {
        oam_data_ = bus_.lock()->read(bus_page_ << 8 | oam_addr_);
        return;
    }

    ppu_->write_oam(oam_addr_++, oam_data_);

    if (oam_addr_ == 0x00) _reset();
}

void dma_transfer_t::start_dma(uint8_t page)
{
    assert(!bus_.expired() && "bus not set");

    is_dma_started_ = true;
    bus_page_ = page;
}

bool dma_transfer_t::is_dma_started()
{
    return is_dma_started_;
}

void dma_transfer_t::_reset()
{
    dma_sync_cycle_ = true;
    is_dma_started_ = false;
    oam_addr_ = 0x00;
    oam_data_ = 0x00;
    bus_page_ = 0x00;
}

} // namespace NES