/*
 * Copyright (c) 2016 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include <libmaple/timer.h>
#include <libmaple/util.h>
//#include <libmaple/rcc.h>

#include "wirish.h"
#include "boards.h"

#define I2S_STATE_IDLE     0
#define I2S_STATE_READY    1
#define I2S_STATE_RECEIVE  2
#define I2S_STATE_TRANSMIT 3

#define SPI_I2SCFGR_DIR_BIT 8

#define i2s_dev_disable(dev) bb_peri_set_bit(&dev->regs->I2SCFGR, SPI_I2SCFGR_I2SE_BIT, 0);
#define i2s_dev_enable(dev) bb_peri_set_bit(&dev->regs->I2SCFGR, SPI_I2SCFGR_I2SE_BIT, 1);

#define i2s_dev_trasmit(dev) bb_peri_set_bit(&dev->regs->I2SCFGR, SPI_I2SCFGR_DIR_BIT, 0);
#define i2s_dev_receive(dev) bb_peri_set_bit(&dev->regs->I2SCFGR, SPI_I2SCFGR_DIR_BIT, 1);
#define armv7m_core_yield() __asm__ volatile ("wfe; sev; wfe");

#if CYCLES_PER_MICROSECOND != 72
/* TODO [0.2.0?] something smarter than this */
#warning "Unexpected clock speed; I2S frequency calculation will be incorrect"
#endif




/*
* Auxiliary functions, may want to move to separarate file.
*/

static const i2s_pins* dev_to_i2s_pins(spi_dev *dev);
static void disable_pwm(const stm32_pin_info *i) ;
static void configure_i2s_gpio(spi_dev *dev, bool as_master);
static uint16_t compute_i2sspr(long sampleRate, int bitsPerSample, bool masterClock);

/* TODO Clean this up, should only run in board with 3SPIs anyway
* Add definitions for the I2S pins in /variants/xxx/board/board.h
* rather than using the SPI definitions.
*/
struct i2s_pins {
    uint8 i2s_ws; //nss;
    uint8 i2s_ck; //sck;
    uint8 i2s_mck; //miso;
    uint8 i2s_sd; //mosi;
};

static const i2s_pins board_i2s_pins[] __FLASH__ = {
    {BOARD_SPI2_NSS_PIN,
     BOARD_SPI2_SCK_PIN,
     PC6, //MCK pin,
     BOARD_SPI2_MOSI_PIN},
    {BOARD_SPI3_NSS_PIN,
     BOARD_SPI3_SCK_PIN,
     PC7, //MCK pin,
     BOARD_SPI3_MOSI_PIN},
};


I2SClass::I2SClass(spi_dev *dev)
{
   _i2s_d(dev);
   switch (_i2s_d) {
   case SPI2:
        _i2sDmaDev = DMA1;
        _i2sTxDmaChannel = DMA_CH5;
        _i2sRxDmaChannel = DMA_CH4;
        _i2s2_this = this;
        break;
   case SPI3:
        _i2sDmaDev = DMA2;
        _i2sTxDmaChannel = DMA_CH2;
        _i2sRxDmaChannel = DMA_CH1;
        _i2s3_this = this;
        break;
   default:
        ASSERT(0);
   }
   
    _xf_active = 0;
    _xf_pending = 0;
    _xf_offset = 0;
    
   _receiveCallback = NULL;
   _transmitCallback = NULL;
   
   _state(I2S_STATE_IDLE);

   //_dmaTransferInProgress(false),
  
/*
  _deviceIndex(deviceIndex),
  _clockGenerator(clockGenerator),
  _sdPin(sdPin),
  _sckPin(sckPin),
  _fsPin(fsPin),

  _state(I2S_STATE_IDLE),
  _dmaChannel(-1),
  _bitsPerSample(0),
  _dmaTransferInProgress(false),

  _receiveCallback (NULL),
  _transmitCallback (NULL)
  ///////////
  
    _sai = sai;

    stm32l4_sai_create(sai, instance, pins, priority, mode);

    _state = I2S_STATE_IDLE;

    _xf_active = 0;
    _xf_pending = 0;
    _xf_offset = 0;

    _receiveCallback = NULL;
    _transmitCallback = NULL;
*/
}

int I2SClass::begin(int mode, long sampleRate, int bitsPerSample, bool masterClock = false)
{

    if (_state != I2S_STATE_IDLE) {
    return 0;
    }
    uint16_t _i2scfgr = SPI_I2SCFGR_I2SMOD_I2S | SPI_I2SCFGR_I2SCFG_MASTER_TX;
    uint16_t _i2spr = 0;
        
    switch (mode) {
    case I2S_PHILIPS_MODE:
    case I2S_RIGHT_JUSTIFIED_MODE:
    case I2S_LEFT_JUSTIFIED_MODE:
    case I2S_PCM_MODE: //  has some errata and doesn't work in all conditions.
        _i2scfgr |= mode;
        break;
    default:
    return 0;
    }
    

    switch (bitsPerSample) {
        case 16:
            _i2scfgr |= SPI_I2SCFGR_DATLEN_16_BIT | SPI_I2SCFGR_CHLEN_16_BIT;
            break;
        case 24:
            _i2scfgr |= SPI_I2SCFGR_DATLEN_24_BIT | SPI_I2SCFGR_CHLEN_32_BIT;
            break;
        case 32:
            _i2scfgr |= SPI_I2SCFGR_DATLEN_32_BIT | SPI_I2SCFGR_CHLEN_32_BIT;
            break;
        default:
        return 0;
    }
    _width = bitsPerSample;

    
// Check to confirm the sampleRate is within range

    if (sampleRate < 8000 || sampleRate > 96000) {
        return 0;
    }

// Compute ideal _i2spr value
    _i2spr = compute_i2sspr(sampleRate, bitsPerSample, masterClock);

// Disable SPI peripheral in case it was enabled (SPI and I2S are the same peripheral, but disable bits are different)
    spi_peripheral_disable(_i2s_d);

/* Todo: Change this so configure_i2s_gpio takes a bool for masteclock*/

    if (masterClock) {
        _i2spr |= SPI_I2SPR_MCKOE; // enable Masterclock signal
        configure_i2s_gpio(_i2s_d, 1);
    } else {
        configure_i2s_gpio(_i2s_d, 0);
    }


	i2s_dev_disable (__i2s_d);
    rcc_clk_enable(_i2s_d->clk_id); // clock setup

    _i2s_d->regs->I2SCFGR = _i2scfgr; // set configuration, default is TX mode, but is idle.
    _i2s_d->regs->I2SPR = _i2spr; // set prescaler
    i2s_dev_enable (__i2s_d);

/*
* Todo, possibly setup most of DMA stuff here, without enabling it.
* Should move the interrupts out of here to only be attached at the start of a send or reception, and detached right after.
*/
    dma_init(_i2sDmaDev);
    switch (_i2s_d) {
    case SPI2:
        dma_attach_interrupt(_i2sDmaDev, _i2sTxDmaChannel, &I2SClass::_i2s2EventCallback);
        break;
    case SPI3:
        dma_attach_interrupt(_i2sDmaDev, _i2sTxDmaChannel, &I2SClass::_i2s3EventCallback);
        break;
    default:
        ASSERT(0);
    }




    _state = I2S_STATE_READY;

    return 1;
}

/* Todo: This one is to be set for slave mode
*/

int I2SClass::begin(int mode, int bitsPerSample)
{

    if (_state != I2S_STATE_IDLE) {
    return 0;
    }
    uint16_t _i2scfgr = SPI_I2SCFGR_I2SMOD_I2S | SPI_I2SCFGR_I2SCFG_SLAVE_TX;
    uint16_t _i2spr = 2; // not usre if needed, need to recheck Datasheet
        
    switch (mode) {
    case I2S_PHILIPS_MODE:
    case I2S_RIGHT_JUSTIFIED_MODE:
    case I2S_LEFT_JUSTIFIED_MODE:
    case I2S_PCM_MODE: // This has some errata and doesn't work in all conditions.
        _i2scfgr |= mode;
        break;
    default:
    return 0;
    }

    spi_peripheral_disable(_i2s_d);

/* Todo: Change this so configure_i2s_gpio takes a bool for masteclock*/

    if (masterClock) {
        _i2spr |= SPI_I2SPR_MCKOE; // enable Masterclock signal
        configure_i2s_gpio(_i2s_d, 1);
    } else {
        configure_i2s_gpio(_i2s_d, 0);
    }
 
 	i2s_dev_disable (__i2s_d);
    rcc_clk_enable(_i2s_d->clk_id); // clock setup
    _i2s_d->regs->I2SCFGR = _i2scfgr; // set configuration, default is TX mode, but is idle.
    _i2s_d->regs->I2SPR = _i2spr; // set prescaler
    i2s_dev_enable (__i2s_d);

    dma_init(_i2sDmaDev);
	
	/* Todo, change these to two variables that are set when the object is created 
	* The just used for the call back, without having to use the switch
	*/
	
    switch (_i2s_d) {
    case SPI2:
        dma_attach_interrupt(_i2sDmaDev, _i2sTxDmaChannel, &I2SClass::_i2s2EventCallback);
        break;
    case SPI3:
        dma_attach_interrupt(_i2sDmaDev, _i2sTxDmaChannel, &I2SClass::_i2s3EventCallback);
        break;
    default:
        ASSERT(0);
    }

    _state = I2S_STATE_READY;

    return 1;

/*
    uint32_t option;

    if (_state != I2S_STATE_IDLE) {
    return 0;
    }

    switch (mode) {
    case I2S_PHILIPS_MODE:
    option = SAI_OPTION_FORMAT_I2S;
    break;
    case I2S_RIGHT_JUSTIFIED_MODE:
    option = SAI_OPTION_FORMAT_RIGHT_JUSTIFIED;
    break;
    case I2S_LEFT_JUSTIFIED_MODE:
    option = SAI_OPTION_FORMAT_LEFT_JUSTIFIED;
    break;
    default:
    return 0;
    }

    switch (bitsPerSample) {
    case 8:
    case 16:
    case 32:
    _width = bitsPerSample;
    break;
    default:
    return 0;
    }

    stm32l4_sai_enable(_sai, bitsPerSample, 0, option, I2SClass::_eventCallback, (void*)this, (SAI_EVENT_RECEIVE_REQUEST | SAI_EVENT_TRANSMIT_REQUEST));

    _state = I2S_STATE_
Y;

    return 1;
*/
}

void I2SClass::end()
{
/*
* Need to finish this up.
* Check the datasheet for all to check before disabling then disable device.
* Then set pins to input mode
*/

    while (_xf_active) {
    armv7m_core_yield();
    }
	/* This should probably be moved to the functions that send and receive data
	* May add more overhead on those, but will allow other things to use the DMA channels when not in use
	* by the i2s peripheral.
	*/
	dma_detach_interrupt(_i2sDmaDev, _i2sTxDmaChannel);
	dma_detach_interrupt(_i2sDmaDev, _i2sRxDmaChannel);


    _state = I2S_STATE_IDLE;

    _xf_active = 0;
    _xf_pending = 0;
    _xf_offset = 0;

/*
    while (_xf_active) {
    armv7m_core_yield();
    }

    stm32l4_sai_disable(_sai);

    _state = I2S_STATE_IDLE;

    _xf_active = 0;
    _xf_pending = 0;
    _xf_offset = 0;
*/
}

int I2SClass::available()
{
    uint32_t xf_offset, xf_index, xf_count;

    switch (_state) {
    case I2S_STATE_READY:
        i2s_dev_receive(_i2s_d);
        _state = I2S_STATE_RECEIVE;
    case I2S_STATE_RECEIVE:
    break;
    default:
    return 0;
    }
/*
    if (!((_state == I2S_STATE_READY) || (_state == I2S_STATE_RECEIVE))) {
    return 0;
    }

    TODO: We need more than this, if the device is not in Receive mode, we need to set it in that mode.
    i2s_dev_receive(_i2s_d); //perhaps this, but needs to check the state first.
    _state = I2S_STATE_RECEIVE;

*/
    if (!_xf_active)
    {
    xf_offset = _xf_offset;
    xf_index  = xf_offset >> 31;
    xf_count  = xf_offset & 0x7fffffff;

    if (xf_count == 0)
    {
        // need to check what this exactly is for
        //if (stm32l4_sai_done(_sai))
        //{
        if (_xf_pending)
        {
            _xf_pending = false;
            xf_count = I2S_BUFFER_SIZE;
        }
        
        _xf_active = true;
        _xf_offset = ((xf_index ^ 1) << 31) | xf_count;
        
        dma_setup_transfer(_i2sDmaDev, _i2sRxDmaChannel, &_i2s_d->regs->DR, DMA_SIZE_16BITS,
                       (uint16_t*)&_xf_data[xf_index][0], DMA_SIZE_16BITS, (DMA_MINC_MODE | DMA_TRNS_CMPLT));
        
        dma_set_num_transfers(_i2sDmaDev, _i2sRxDmaChannel, I2S_BUFFER_SIZE/2);
        
        dma_enable(_i2sDmaDev, _i2sRxDmaChannel);
        
        //stm32l4_sai_receive(_sai, (uint8_t*)&_xf_data[xf_index][0], I2S_BUFFER_SIZE);
        //}
    }
    }

    return (_xf_offset & 0x7fffffff);
}

int I2SClass::peek()
{
    uint32_t xf_offset, xf_index, xf_count;
    
    switch (_state) {
    case I2S_STATE_READY:
        i2s_dev_receive(_i2s_d);
        _state = I2S_STATE_RECEIVE;
    case I2S_STATE_RECEIVE:
    break;
    default:
    return 0;
    }
/*

    if (!((_state == I2S_STATE_READY) || (_state == I2S_STATE_RECEIVE))) {
    return 0;
    }
    
    TODO: We need more than this, if the device is not in Receive mode, we need to set it in that mode.
    i2s_dev_receive(_i2s_d); //perhaps this, but needs to check the state first.
    _state = I2S_STATE_RECEIVE;
*/
    if (!_xf_active)
    {
    xf_offset = _xf_offset;
    xf_index  = xf_offset >> 31;
    xf_count  = xf_offset & 0x7fffffff;

    if (xf_count == 0)
    {
        //if (stm32l4_sai_done(_sai))
       // {
        if (_xf_pending)
        {
            _xf_pending = false;
            xf_count = I2S_BUFFER_SIZE;
        }
        
        _xf_active = true;
        _xf_offset = ((xf_index ^ 1) << 31) | xf_count;
		
		spi_rx_dma_enable(_i2s_d); // function part of the core, don't need own one.
        
        dma_setup_transfer(_i2sDmaDev, _i2sRxDmaChannel, &_i2s_d->regs->DR, DMA_SIZE_16BITS,
                       (uint16_t*)&_xf_data[xf_index][0], DMA_SIZE_16BITS, (DMA_MINC_MODE | DMA_TRNS_CMPLT));
        
        dma_set_num_transfers(_i2sDmaDev, _i2sRxDmaChannel, I2S_BUFFER_SIZE/2);
        
        dma_enable(_i2sDmaDev, _i2sRxDmaChannel);
        
        //stm32l4_sai_receive(_sai, (uint8_t*)&_xf_data[xf_index][0], I2S_BUFFER_SIZE);
        //}
    }
    }

    xf_offset = _xf_offset;
    xf_index  = xf_offset >> 31;
    xf_count  = xf_offset & 0x7fffffff;

    if (_width == 32)
    {
    if (xf_count >= 4) {
        return *((int32_t*)&(((uint8_t*)&_xf_data[xf_index][0])[xf_count - 4]));
    }
    }

    else if (_width == 16)
    {
    if (xf_count >= 2) {
        return *((int16_t*)&(((uint8_t*)&_xf_data[xf_index][0])[xf_count - 2]));
    }
    }
    else
    {
    if (xf_count >= 1) {
        return *((uint8_t*)&(((uint8_t*)&_xf_data[xf_index][0])[xf_count - 1]));
    }
    }

    return 0;
}

int I2SClass::read()
{
    if (_width == 32)      { int32_t temp; if (read(&temp, 4)) { return temp; } }
    else if (_width == 16) { int16_t temp; if (read(&temp, 2)) { return temp; } }

    return 0;
}

int I2SClass::read(void* buffer, size_t size)
{
    uint32_t xf_offset, xf_index, xf_count;

    switch (_state) {
    case I2S_STATE_READY:
        i2s_dev_receive(_i2s_d);
        _state = I2S_STATE_RECEIVE;
    case I2S_STATE_RECEIVE:
    break;
    default:
    return 0;
    }
/*
    if (!((_state == I2S_STATE_READY) || (_state == I2S_STATE_RECEIVE))) {
    return 0;
    }

    TODO: We need more than this, if the device is not in Receive mode, we need to set it in that mode.
    i2s_dev_receive(_i2s_d); //perhaps this, but needs to check the state first.
    _state = I2S_STATE_RECEIVE;
*/
    if (!_xf_active)
    {
    xf_offset = _xf_offset;
    xf_index  = xf_offset >> 31;
    xf_count  = xf_offset & 0x7fffffff;

    if (xf_count == 0)
    {
        //if (stm32l4_sai_done(_sai))
        //{
        if (_xf_pending)
        {
            _xf_pending = false;
            xf_count = I2S_BUFFER_SIZE;
        }
        
        _xf_active = true;
        _xf_offset = ((xf_index ^ 1) << 31) | xf_count;
		
		spi_rx_dma_enable(_i2s_d); // function part of the core, don't need own one.
        
        dma_setup_transfer(_i2sDmaDev, _i2sRxDmaChannel, &_i2s_d->regs->DR, DMA_SIZE_16BITS,
                       (uint16_t*)&_xf_data[xf_index][0], DMA_SIZE_16BITS, (DMA_MINC_MODE | DMA_TRNS_CMPLT));
        
        dma_set_num_transfers(_i2sDmaDev, _i2sRxDmaChannel, I2S_BUFFER_SIZE/2);
        
        dma_enable(_i2sDmaDev, _i2sRxDmaChannel);
        
        //stm32l4_sai_receive(_sai, (uint8_t*)&_xf_data[xf_index][0], I2S_BUFFER_SIZE);

        //}
    }
    }

    xf_offset = _xf_offset;
    xf_index  = xf_offset >> 31;
    xf_count  = xf_offset & 0x7fffffff;

    if (size > xf_count) {
    size = xf_count;
    }

    if (size)
    {
    xf_count -= size;

    memcpy(buffer, &(((uint8_t*)&_xf_data[xf_index][0])[xf_count]), size);
    }

    _xf_offset = (xf_index << 31) | xf_count;

    if (!_xf_active)
    {
    xf_offset = _xf_offset;
    xf_index  = xf_offset >> 31;
    xf_count  = xf_offset & 0x7fffffff;

    if (xf_count == 0)
    {
        //if (stm32l4_sai_done(_sai))
        //{
        if (_xf_pending)
        {
            _xf_pending = false;
            xf_count = I2S_BUFFER_SIZE;
        }
        
        _xf_active = true;
        _xf_offset = ((xf_index ^ 1) << 31) | xf_count;
		
		spi_rx_dma_enable(_i2s_d); // function part of the core, don't need own one.
        
        dma_setup_transfer(_i2sDmaDev, _i2sRxDmaChannel, &_i2s_d->regs->DR, DMA_SIZE_16BITS,
                       (uint16_t*)&_xf_data[xf_index][0], DMA_SIZE_16BITS, (DMA_MINC_MODE | DMA_TRNS_CMPLT));
        
        dma_set_num_transfers(_i2sDmaDev, _i2sRxDmaChannel, I2S_BUFFER_SIZE/2);
        
        dma_enable(_i2sDmaDev, _i2sRxDmaChannel);
        
        //stm32l4_sai_receive(_sai, (uint8_t*)&_xf_data[xf_index][0], I2S_BUFFER_SIZE);
        //}
    }
    }

    return size;
}

void I2SClass::flush()
{
}

size_t I2SClass::availableForWrite()
{

    switch (_state) {
    case I2S_STATE_READY:
        i2s_dev_transmit(_i2s_d);
        _state = I2S_STATE_TRANSMIT;
    case I2S_STATE_TRANSMIT:
    break;
    default:
    return 0;
    }
/*
    if (!((_state == I2S_STATE_READY) || (_state == I2S_STATE_TRANSMIT))) {
    return 0;
    }

*/
    return I2S_BUFFER_SIZE - (_xf_offset & 0x7fffffff);
}

size_t I2SClass::write(uint8_t data)
{
// This is pretty much pointless since the smaller size the F1 can send is 16 bits. 8bits wouldn't do much
    return write((int16_t)data);
}

size_t I2SClass::write(const uint8_t *buffer, size_t size)
{
    return write((const void*)buffer, size);
}

size_t I2SClass::write(int16_t sample)
{
    return write((const void*)&sample, (_width / 8));
}

size_t I2SClass::write(int32_t sample)
{
    size_t sent = 0;
	while (sent < _width / 8)
	sent += write((const void*)(&sample+sent), (_width / 8));
	return sent;
}

size_t I2SClass::write(const void *buffer, size_t size)
{
    uint32_t xf_offset, xf_index, xf_count;
    switch (_state) {
    case I2S_STATE_READY:
        i2s_dev_transmit(_i2s_d);
        _state = I2S_STATE_TRANSMIT;
    case I2S_STATE_TRANSMIT:
    break;
    default:
    return 0;
    }
/*
    if (!((_state == I2S_STATE_READY) || (_state == I2S_STATE_TRANSMIT))) {
    return 0;
    }

    _state = I2S_STATE_TRANSMIT;
*/
    if      (_width == 32) { size &= ~3; }
    else if (_width == 16) { size &= ~1; }
    
    xf_offset = _xf_offset;
    xf_index  = xf_offset >> 31;
    xf_count  = xf_offset & 0x7fffffff;
    
    if ((xf_count + size) > I2S_BUFFER_SIZE) {
    size = I2S_BUFFER_SIZE - xf_count;
    }

    if (size)
    {
        memcpy(&(((uint8_t*)&_xf_data[xf_index][0])[xf_count]), buffer, size);

    xf_count += size;
    }

    _xf_offset = (xf_index << 31) | xf_count;

    if (!_xf_active)
    {
    xf_offset = _xf_offset;
    xf_index  = xf_offset >> 31;
    xf_count  = xf_offset & 0x7fffffff;

    if (xf_count == I2S_BUFFER_SIZE)
    {
        //if (stm32l4_sai_done(_sai))
        //{
        _xf_active = true;
        _xf_offset = (xf_index ^ 1) << 31;
        
        spi_tx_dma_enable(_i2s_d); // function part of the core, don't need own one.
        
        dma_setup_transfer(_i2sDmaDev, _i2sTxDmaChannel, &_i2s_d->regs->DR, DMA_SIZE_16BITS,
                       (const uint16_t*)&_xf_data[xf_index][0], DMA_SIZE_16BITS, (DMA_MINC_MODE |  DMA_FROM_MEM | DMA_TRNS_CMPLT));
        
        dma_set_num_transfers(_i2sDmaDev, _i2sTxDmaChannel, I2S_BUFFER_SIZE/2);
        
        dma_enable(_i2sDmaDev, _i2sTxDmaChannel);
        
        //stm32l4_sai_transmit(_sai, (const uint8_t*)&_xf_data[xf_index][0], I2S_BUFFER_SIZE);
        //}
    }
    }

    return size;
}

void I2SClass::onReceive(void(*callback)(void))
{
    _receiveCallback = callback;
}

void I2SClass::onTransmit(void(*callback)(void))
{
    _transmitCallback = callback;
}

/* Todo: Rewrite this
* Better use the _state variable to decide if it's in xmit or receive mode, similar to SAMD
*/
void I2SClass::EventCallback()
{
    uint32_t xf_offset, xf_index, xf_count;

    xf_offset = _xf_offset;
    xf_index  = xf_offset >> 31;
    xf_count  = xf_offset & 0x7fffffff;

    if (_state == I2S_STATE_RECEIVER)
    {
    if (xf_count == 0)
    {
        _xf_offset = ((xf_index ^ 1) << 31) | I2S_BUFFER_SIZE;
        
        dma_setup_transfer(_i2sDmaDev, _i2sRxDmaChannel, &_i2s_d->regs->DR, DMA_SIZE_16BITS,
                       (uint16_t*)&_xf_data[xf_index][0], DMA_SIZE_16BITS, (DMA_MINC_MODE | DMA_TRNS_CMPLT));
        
        dma_set_num_transfers(_i2sDmaDev, _i2sRxDmaChannel, I2S_BUFFER_SIZE/2);
        
        dma_enable(_i2sDmaDev, _i2sRxDmaChannel);
        
        //stm32l4_sai_receive(_sai, (uint8_t*)&_xf_data[xf_index][0], I2S_BUFFER_SIZE);
    }
    else
    {
    
        // TODO: Perhaps disable interrutps if we are done, right now we disable dma only
        _xf_pending = true;
        _xf_active = false;
        spi_rx_dma_disable(_i2sDmaDev);
        _state = I2S_STATE_READY;
    }

    if (_receiveCallback)
    {
        _receiveCallback();
        //armv7m_pendsv_enqueue((armv7m_pendsv_routine_t)_receiveCallback, NULL, 0);
    }
    }

    if (_state == I2S_STATE_TRANSMIT)
    {
    if (xf_count == I2S_BUFFER_SIZE)
    {
        _xf_offset = (xf_index ^ 1) << 31;
        
        dma_setup_transfer(_i2sDmaDev, _i2sTxDmaChannel, &_i2s_d->regs->DR, DMA_SIZE_16BITS,
                       (const uint16_t*)&_xf_data[xf_index][0], DMA_SIZE_16BITS, (DMA_MINC_MODE |  DMA_FROM_MEM | DMA_TRNS_CMPLT));
        
        dma_set_num_transfers(_i2sDmaDev, _i2sTxDmaChannel, I2S_BUFFER_SIZE/2);
        
        dma_enable(_i2sDmaDev, _i2sTxDmaChannel);
        
        //stm32l4_sai_transmit(_sai, (const uint8_t*)&_xf_data[xf_index][0], I2S_BUFFER_SIZE);
    }
    else
    {
        // TODO: disable interrutps if we are done, or at least disable DMA requests from SPI
        // so other devices can use the DMA channel.
        _xf_active = false;
        spi_tx_dma_disable(_i2sDmaDev);
        _state = I2S_STATE_READY;
    }

    if (_transmitCallback)
    {
        _transmitCallback();
        //armv7m_pendsv_enqueue((armv7m_pendsv_routine_t)_transmitCallback, NULL, 0);
    }
    }
}

/* This is the static ISR, need to discern who called it to then call a different 
* At the moment just set 2 of them, they are short anyway
*/

void I2SClass::_i2s2EventCallback()
{
    _i2s2_this->EventCallback();
    //reinterpret_cast<class I2SClass*>(_i2s2_this)->EventCallback();
}

void I2SClass::_i2s3EventCallback()
{
    _i2s3_this->EventCallback();
    //reinterpret_cast<class I2SClass*>(_i2s3_this)->EventCallback();
}
/*
void I2SClass::_eventCallback(void *context, uint32_t events)
{
    reinterpret_cast<class I2SClass*>(context)->EventCallback(events);
}
*/

#if I2S_INTERFACES_COUNT > 0
/*
extern const stm32l4_sai_pins_t g_SAIPins;
extern const unsigned int g_SAIInstance;
extern const unsigned int g_SAIMode;

static stm32l4_sai_t _SAI;

I2SClass I2S(&_SAI, g_SAIInstance, &g_SAIPins, STM32L4_SAI_IRQ_PRIORITY, g_SAIMode);
*/
#endif

/*
 * Auxiliary functions, similar to libmaple spi.
 * May want to move them to a separate file.
 */

static const i2s_pins* dev_to_i2s_pins(spi_dev *dev) {
    switch (dev->clk_id) {
    case RCC_SPI2: return board_i2s_pins;
    case RCC_SPI3: return board_i2s_pins + 1;
    default:       return NULL;
    }
}

/* This function also exists in the spi library, may need to rename it, or check if it's defined already
*
*/
static void disable_pwm(const stm32_pin_info *i) {
    if (i->timer_device) {
        timer_set_mode(i->timer_device, i->timer_channel, TIMER_DISABLED);
    }
}

static void configure_i2s_gpio(spi_dev *dev, bool as_master) {
    const i2s_pins *pins = dev_to_i2s_pins(dev);

    if (!pins) {
        return;
    }
    

    const stm32_pin_info *i2s_ws = &PIN_MAP[pins->i2s_ws];
    const stm32_pin_info *i2s_ck = &PIN_MAP[pins->i2s_ck];
    const stm32_pin_info *i2s_mck = &PIN_MAP[pins->i2s_mck];
    const stm32_pin_info *i2s_sd = &PIN_MAP[pins->i2s_sd];

    disable_pwm(i2s_ws);
    disable_pwm(i2s_ck);
    disable_pwm(i2s_sd);
    gpio_set_mode(i2s_ws->gpio_device, i2s_ws->gpio_bit, GPIO_AF_OUTPUT_PP);
    gpio_set_mode(i2s_ck->gpio_device, i2s_ck->gpio_bit, GPIO_AF_OUTPUT_PP);
    gpio_set_mode(i2s_sd->gpio_device, i2s_sd->gpio_bit, GPIO_AF_OUTPUT_PP);
    
    if (as_master) {	
        disable_pwm(i2s_mck);
        gpio_set_mode(i2s_mck->gpio_device, i2s_mck->gpio_bit, GPIO_AF_OUTPUT_PP);
    } 

    
}

static uint16_t compute_i2sspr(long sampleRate, int bitsPerSample, bool masterClock) {
/* Todo: Make sourceclock come from the compile options, so it can calculate correctly for other core speeds
*/
    uint16_t i2sdiv = 2, i2sodd = 0, packetlength = 1;
    uint32_t tmp = 0;
    uint32_t sourceclock = 72000000;
    /* Check the frame length (For the Prescaler computing) */
    if(bitspersample == 16)
    {
      /* Packet length is 16 bits */
      packetlength = 1;
    }
    else
    {
      /* Packet length is 32 bits */
      packetlength = 2;
    }
    /* Compute the Real divider depending on the MCLK output state with a floating point */
    if(masterClock)
    {
      /* MCLK output is enabled */
      tmp = (uint16_t)(((((sourceclock / 256) * 10) / sampleRate)) + 5);
    }
    else
    {
      /* MCLK output is disabled */
      tmp = (uint16_t)(((((sourceclock / (32 * packetlength)) *10 ) / sampleRate)) + 5);
    }
    
    /* Remove the floating point */
    tmp = tmp / 10;  
      
    /* Check the parity of the divider */
    i2sodd = (uint16_t)(tmp & (uint16_t)0x0001);
   
    /* Compute the i2sdiv prescaler */
    i2sdiv = (uint16_t)((tmp - i2sodd) / 2);
   
    /* Get the Mask for the Odd bit (SPI_I2SPR[8]) register */
    i2sodd = (uint16_t) (i2sodd << 8);
    return (i2sdiv | i2sodd);
}