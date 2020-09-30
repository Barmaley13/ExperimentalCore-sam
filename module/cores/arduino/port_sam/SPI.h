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

#ifndef _ARDUINO_CORE_SPI_HPP_
#define _ARDUINO_CORE_SPI_HPP_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stddef.h>
#include "core_constants.h"
#include "core_variant.h"
//#include <stdio.h>

// SPI_HAS_TRANSACTION means SPI has
//   - beginTransaction()
//   - endTransaction()
//   - usingInterrupt()
//   - SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

// SPI_HAS_EXTENDED_CS_PIN_HANDLING means SPI has automatic
// CS pin handling and provides the following methods:
//   - begin(pin)
//   - end(pin)
//   - setBitOrder(pin, bitorder)
//   - setDataMode(pin, datamode)
//   - setClockDivider(pin, clockdiv)
//   - transfer(pin, data, SPI_LAST/SPI_CONTINUE)
//   - beginTransaction(pin, SPISettings settings) (if transactions are available)
#define SPI_HAS_EXTENDED_CS_PIN_HANDLING 0

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

#define SPI_MODE0           (0x00)
#define SPI_MODE1           (0x01)
#define SPI_MODE2           (0x02)
#define SPI_MODE3           (0x03)

typedef enum
{
    CLOCK_MODE_0 = SPI_MODE0,	    // CPOL : 0  | CPHA : 0
    CLOCK_MODE_1 = SPI_MODE1,		// CPOL : 0  | CPHA : 1
    CLOCK_MODE_2 = SPI_MODE2,		// CPOL : 1  | CPHA : 0
    CLOCK_MODE_3 = SPI_MODE2		// CPOL : 1  | CPHA : 1
} ClockMode;


enum SPITransferMode
{
    SPI_CONTINUE,
    SPI_LAST
};

typedef enum
{
    SPI_CHAR_SIZE_8_BITS,
    SPI_CHAR_SIZE_9_BITS,
    SPI_CHAR_SIZE_10_BITS,
    SPI_CHAR_SIZE_11_BITS,
    SPI_CHAR_SIZE_12_BITS,
    SPI_CHAR_SIZE_13_BITS,
    SPI_CHAR_SIZE_14_BITS,
    SPI_CHAR_SIZE_15_BITS,
    SPI_CHAR_SIZE_16_BITS
} CharSize;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class SPISettings
{
    public:
        SPISettings();
        SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode);
        
        bool operator==(const SPISettings& rhs) const
        {
            if ((this->clockFreq == rhs.clockFreq) &&
            (this->bitOrder == rhs.bitOrder) &&
            (this->dataMode == rhs.dataMode)) {
                return true;
            }
            return false;
        }

        bool operator!=(const SPISettings& rhs) const
        {
            return !(*this == rhs);
        }

        uint32_t getClockFreq() const {return clockFreq;}
        uint8_t getDataMode() const {return dataMode;}
        BitOrder getBitOrder() const {return (bitOrder == MSBFIRST ? MSBFIRST : LSBFIRST);}

    private:
        void init_MightInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode);
        inline void init_AlwaysInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode);
        
        uint32_t clockFreq;
        BitOrder bitOrder;
        uint8_t dataMode;
        friend class SPIClass;
};

const SPISettings DEFAULT_SPI_SETTINGS = SPISettings();

class SPIClass
{
    public:
        SPIClass(Spi *spi, uint32_t pinMOSI, uint32_t pinMISO, uint32_t pinSCK);

        uint8_t transfer(uint8_t data);
        uint16_t transfer16(uint16_t data);
        void transfer(void *buf, size_t count);

        // Transaction Functions
        void usingInterrupt(int interruptNumber);
        void notUsingInterrupt(int interruptNumber);
        void beginTransaction(SPISettings settings);
        void endTransaction(void);

        // SPI Configuration methods
        void attachInterrupt(void);
        void detachInterrupt(void);

        void begin(void);
        void end(void);

        void setBitOrder(BitOrder bitOrder);
        void setDataMode(uint8_t dataMode);
        void setClockFreq(uint32_t clockFreq);

    private:
        void init();
        int32_t config(SPISettings settings);
        bool spi_rx(uint8_t *rx_data);
        bool spi_tx(uint8_t tx_data);

        Spi *_spi;
        Flexcom* _flexcom;
        
        uint32_t _pinMOSI;
        uint32_t _pinMISO;
        uint32_t _pinSCK;
        EGPIOType _pinMOSIMux;
        EGPIOType _pinMISOMux;
        EGPIOType _pinSCKMux;
        
        IRQn_Type _irqn;
        uint8_t _clockId;
        SPISettings settings;

        bool initialized;
        uint8_t interruptMode;
        char interruptSave;
        uint32_t interruptMask[NUM_PORTS];
};

#endif

#endif // _ARDUINO_CORE_SPI_HPP_
