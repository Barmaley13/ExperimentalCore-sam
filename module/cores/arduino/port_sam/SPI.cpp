/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "SPI.h"
#include "driver_init.h"
#include "hpl_pmc.h"
#include "variant.h"

// Not sure why it has such a trouble including this macro...
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif // min

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif // max

#define SPI_IMODE_NONE   0
#define SPI_IMODE_EXTINT 1
#define SPI_IMODE_GLOBAL 2

#define SPI_MAX_CLK         min(12500000, (VARIANT_MCK/2))


// Constructors ////////////////////////////////////////////////////////////////
SPISettings::SPISettings() 
{
    init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0);
}
    
SPISettings::SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode)
{
    if (__builtin_constant_p(clock))
    {
        init_AlwaysInline(clock, bitOrder, dataMode);
    }
    else
    {
        init_MightInline(clock, bitOrder, dataMode);
    }
}

SPIClass::SPIClass(Spi *spi, uint32_t pinMOSI, uint32_t pinMISO, uint32_t pinSCK)
{
    _spi = spi;
    _flexcom = (Flexcom *)((uint32_t)_spi - 0x400U);

    _pinMOSI = PinMap[pinMOSI].ulPin;
    _pinMISO = PinMap[pinMISO].ulPin;
    _pinSCK = PinMap[pinSCK].ulPin;

    _pinMOSIMux = PinMap[pinMOSI].ulPinType;
    _pinMISOMux = PinMap[pinMISO].ulPinType;
    _pinSCKMux = PinMap[pinSCK].ulPinType;
    
    _clockId = 0;
}

// Private Methods //////////////////////////////////////////////////////////////
void SPISettings::init_MightInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode)
{
    init_AlwaysInline(clock, bitOrder, dataMode);
}

inline void SPISettings::init_AlwaysInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode)
{
    this->clockFreq = (clock >= SPI_MAX_CLK ? SPI_MAX_CLK : clock);
    this->bitOrder = (bitOrder == MSBFIRST ? MSBFIRST : LSBFIRST);

    switch (dataMode)
    {
        case SPI_MODE0:
            this->dataMode = (uint8_t) CLOCK_MODE_0; break;
        case SPI_MODE1:
            this->dataMode = (uint8_t) CLOCK_MODE_1; break;
        case SPI_MODE2:
            this->dataMode = (uint8_t) CLOCK_MODE_2; break;
        case SPI_MODE3:
            this->dataMode = (uint8_t) CLOCK_MODE_3; break;
        default:
            this->dataMode = (uint8_t) CLOCK_MODE_0; break;
    }
}

void SPIClass::init()
{   
    // Little safety guard
    if (initialized)
        return;

    // Figure out FLEXCOM index
    uint8_t flexcomIndex = 0;
    for (flexcomIndex = 0; flexcomIndex <= FLEXCOM_INST_NUM; flexcomIndex++)
    {
        // Houston, we got a problem!
        if (flexcomIndex >= FLEXCOM_INST_NUM)
            return;
        // Index is found!
        else if (_flexcom == FLEXCOMS[flexcomIndex])
            break;
    }
    
    // Dynamic assignment of IRQ handler
    _irqn = FLEXCOM_IRQNS[flexcomIndex];
    // vectorAssign(_irqn, _irq_handler);

    // Activate Serial peripheral clock
    _clockId = FLEXCOM_IDS[flexcomIndex];
    _pmc_enable_periph_clock(_clockId);
    
    // PIO init
    gpio_set_pin_function(_pinMOSI, _pinMOSI);
    gpio_set_pin_function(_pinMISO, _pinMISO);
    gpio_set_pin_function(_pinSCK, _pinSCK);

    // Config spi in sync fashion
    config(DEFAULT_SPI_SETTINGS);

    // Set private attributes
    interruptMode = SPI_IMODE_NONE;
    interruptSave = 0;
    interruptMask = 0;
    initialized = true;

}

int32_t SPIClass::config(SPISettings settings)
{
    if (this->settings != settings || !initialized) 
    {
        this->settings = settings;
        
        // Disable SPI        
        _spi->SPI_CR = SPI_CR_SPIDIS;
        
        // Extract data from clockMode
        int ncpha, cpol;
        if((settings.dataMode & (0x1ul)) == 0 )
            ncpha = 1;
        else
            ncpha = 0;

        if((settings.dataMode & (0x2ul)) == 0)
            cpol = 0;
        else
            cpol = 1;

        // Init SPI in sync fashion    
	    uint32_t cr = 0;
	    uint32_t mr = 0;
	    uint32_t csr = 0;
         
        // Baud rate register
        uint32_t scbr = (VARIANT_MCK / settings.clockFreq);

        // Delay Before SPCK (ns) <0-255000>
        uint32_t dly_spck = 1000;
        // Delay Between Consecutive Transfers (ns) <0-8160000>
        uint32_t dly_bct = 1000;
        // Calculate delay register values
        uint32_t dlybs = (((VARIANT_MCK / 1000000) * dly_spck) / 1000);
        uint32_t dlybct = (((VARIANT_MCK / 1000000) * dly_bct) / 32000);

        cr = (uint32_t) (0x01 << 31);                   // Disable FIFO
        mr |= (0x01 << SPI_MR_MSTR_Pos);                // SPI Mode (0 == slave, 1 == master)
        mr |= (0x00 << SPI_MR_PS_Pos);                  // Peripheral Select (0 == fixed, 1 == variable)
        mr |= (0x00 << SPI_MR_PCSDEC_Pos);              // Chip select decode (0 == direct)
        csr |= (cpol << SPI_CSR_CPOL_Pos);              // Clock polarity
        csr |= (ncpha << SPI_CSR_NCPHA_Pos);            // Clock phase
        csr |= (SPI_CSR_BITS(SPI_CHAR_SIZE_8_BITS));    // Character size
        csr |= (SPI_CSR_SCBR(scbr));                    // Baud rate register
        csr |= (SPI_CSR_DLYBS(dlybs));                  // Delay settings
        csr |= (SPI_CSR_DLYBCT(dlybct));                // Delay settings
        
        // Write registers       
        _flexcom->FLEXCOM_MR = FLEXCOM_MR_OPMODE_SPI;
        if ((_spi->SPI_SR & SPI_SR_SPIENS) > 0) 
        {
            return ERR_DENIED;
        }
        _spi->SPI_CR = SPI_CR_SWRST;
        _spi->SPI_CR = cr & ~(SPI_CR_SPIEN | SPI_CR_SPIDIS | SPI_CR_SWRST | SPI_CR_LASTXFER);
        _spi->SPI_MR = (mr | SPI_MR_PCS(0x0E) | SPI_MR_MODFDIS) & ~SPI_MR_LLB;
        _spi->SPI_CSR[0] = csr;
        
        // Enable SPI
        _spi->SPI_CR = SPI_CR_SPIEN;
    }
    
    return ERR_NONE;
}

// Public Methods //////////////////////////////////////////////////////////////
void SPIClass::begin()
{
    init();
}

void SPIClass::end()
{
    //// Reset SPI 
    //_spi->SPI_CR = SPI_CR_SWRST;

    // Disable SPI
    _spi->SPI_CR = SPI_CR_SPIDIS;

    initialized = false;
}

#ifndef interruptsStatus
#define interruptsStatus() __interruptsStatus()
static inline unsigned char __interruptsStatus(void) __attribute__((always_inline, unused));
static inline unsigned char __interruptsStatus(void)
{
  unsigned long primask, faultmask;

  asm volatile ("mrs %0, primask" : "=r" (primask));
  if (primask) return 0;

  asm volatile ("mrs %0, faultmask" : "=r" (faultmask));
  if (faultmask) return 0;

  return 1;
}
#endif // interruptsStatus

void SPIClass::usingInterrupt(int interruptNumber)
{

    // TODO: Implement this!
    #if 0
    uint8_t irestore;

    irestore = interruptsStatus();
    noInterrupts();
    if (interruptMode < 16)
    {
        if (interruptNumber > NUM_DIGITAL_PINS)
        {
            interruptMode = 16;
        }
        else
        {
            Pio *pio = PinMap[interruptNumber].pPort;
            uint32_t mask = PinMap[interruptNumber].ulPin;
            if (pio == PIOA) 
            {
                interruptMode |= 1;
                interruptMask[0] |= mask;
            }
            else if (pio == PIOB) 
            {
                interruptMode |= 2;
                interruptMask[1] |= mask;
            } 
            else if (pio == PIOC) 
            {
                interruptMode |= 4;
                interruptMask[2] |= mask;
            } 
            else if (pio == PIOD) 
            {
                interruptMode |= 8;
                interruptMask[3] |= mask;
            } 
            else 
            {
                interruptMode = 16;
            }
        }
    }
    if (irestore) interrupts();
    #endif // 0
}


void SPIClass::notUsingInterrupt(int interruptNumber)
{
    // TODO: Implement this!
}

void SPIClass::beginTransaction(SPISettings settings)
{
    //if (interruptMode != SPI_IMODE_NONE)
    //{
        //if (interruptMode & SPI_IMODE_GLOBAL)
        //{
            //interruptSave = interruptsStatus();
            //noInterrupts();
        //}
        //else if (interruptMode & SPI_IMODE_EXTINT)
        //EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(interruptMask);
    //}

    config(settings);
}

void SPIClass::endTransaction(void)
{
    // Wait till transmit is done
    while (!(_spi->SPI_SR & SPI_SR_TXEMPTY)){};

    //if (interruptMode != SPI_IMODE_NONE)
    //{
        //if (interruptMode & SPI_IMODE_GLOBAL)
        //{
            //if (interruptSave)
            //interrupts();
        //}
        //else if (interruptMode & SPI_IMODE_EXTINT)
        //EIC->INTENSET.reg = EIC_INTENSET_EXTINT(interruptMask);
    //}
#if 0
    uint8_t mode = interruptMode;
    if (mode > 0)
    {
        if (mode < 16)
        {
            if (mode & 1) PIOA->PIO_IER = interruptMask[0];
            if (mode & 2) PIOB->PIO_IER = interruptMask[1];
            if (mode & 4) PIOC->PIO_IER = interruptMask[2];
            if (mode & 8) PIOD->PIO_IER = interruptMask[3];
        } 
        else 
        {
            if (interruptSave) interrupts();
        }
    }
#endif // 0
}

void SPIClass::setBitOrder(BitOrder bitOrder)
{
    settings.bitOrder = bitOrder;
}

void SPIClass::setDataMode(uint8_t dataMode)
{
    SPISettings newSettings = SPISettings(settings.clockFreq, settings.bitOrder, dataMode);
    config(newSettings);
}

void SPIClass::setClockFreq(uint32_t clockFreq)
{
    SPISettings newSettings = SPISettings(clockFreq, settings.bitOrder, settings.dataMode);
    config(newSettings);
}

bool SPIClass::spi_rx(uint8_t *rx_data)
{
    // Check incoming data
    if (!(_spi->SPI_SR & SPI_SR_RDRF))
        return false;
    
    // Read receive register (16 bit value)
    uint32_t _rx_data = (_spi->SPI_RDR & SPI_RDR_RD_Msk) >> SPI_RDR_RD_Pos;    

    // Cast to byte by discarding unused bits
    rx_data =  (uint8_t *) _rx_data;
    return true;
}

bool SPIClass::spi_tx(uint8_t tx_data)
{
    // Wait till hw is ready
    if (!(_spi->SPI_SR & SPI_SR_TDRE))
        return false;
    
    // Write byte to shift register
    _spi->SPI_TDR = tx_data;

    return true;
}

byte SPIClass::transfer(uint8_t tx_data)
{
    uint8_t rx_data = 0;
    
    // Check if spi has been enabled!    
    if (!(_spi->SPI_SR & SPI_SR_SPIENS))
    {
        // Nope!
        return rx_data;
    }

    // Clear incoming register (just in case)
    spi_rx(&rx_data);
    
    // Send data
    while (!(spi_tx(tx_data))){};

    // Check for errors
    if (_spi->SPI_SR & SPI_SR_OVRES)
    {
        // Overflow!
        return rx_data;
    }

    // Receive data
    while (!(spi_rx(&rx_data))){};

	return rx_data;
}

uint16_t SPIClass::transfer16(uint16_t data)
{
    union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } t;

    t.val = data;

    if (settings.bitOrder == LSBFIRST)
    {
        t.lsb = transfer(t.lsb);
        t.msb = transfer(t.msb);
    } 
    else 
    {
        t.msb = transfer(t.msb);
        t.lsb = transfer(t.lsb);
    }

    return t.val;
}

void SPIClass::transfer(void *buf, size_t count)
{
    uint8_t *buffer = reinterpret_cast<uint8_t *>(buf);
    for (size_t i=0; i<count; i++)
    {
        *buffer = transfer(*buffer);
        buffer++;
    }
}

void SPIClass::attachInterrupt(void)
{
  // Should be enableInterrupt()
}

void SPIClass::detachInterrupt(void)
{
  // Should be disableInterrupt()
}
