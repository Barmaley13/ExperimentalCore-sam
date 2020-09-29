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

#include "variant.h"
#include "core_constants.h"
#include "utils.h"
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

#define SPI_MODE0           (0x02)
#define SPI_MODE1           (0x00)
#define SPI_MODE2           (0x03)
#define SPI_MODE3           (0x01)

typedef enum
{
    CLOCK_MODE_0 = SPI_MODE0,	    // CPOL : 0  | CPHA : 0
    CLOCK_MODE_1 = SPI_MODE1,		// CPOL : 0  | CPHA : 1
    CLOCK_MODE_2 = SPI_MODE2,		// CPOL : 1  | CPHA : 0
    CLOCK_MODE_3 = SPI_MODE2		// CPOL : 1  | CPHA : 1
} ClockMode;

#define SPI_MAX_CLK         (min(12500000, (VARIANT_MCK/2)))

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


class SPISettings
{
    public:
        SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode)
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

        SPISettings() { init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0); }

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
        void init_MightInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode)
        {
            init_AlwaysInline(clock, bitOrder, dataMode);
        }

        void init_AlwaysInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) __attribute__((__always_inline__))
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

        byte transfer(uint8_t data);
        uint16_t transfer16(uint16_t data);
        void transfer(void *buf, size_t count);

        // Transaction Functions
        void usingInterrupt(IRQn_Type interruptNumber);
        void notUsingInterrupt(IRQn_Type interruptNumber);
        void beginTransaction(SPISettings settings);
        void endTransaction(void);

        // SPI Configuration methods
        void attachInterrupt(void);
        void detachInterrupt(void);

        void begin(void);
        void end(void);

        void setBitOrder(BitOrder bitOrder);
        void setDataMode(uint8_t uc_mode);
        void setClockFreq(uint32_t clockFreq);

    private:
        void init();
        int32_t config(SPISettings settings);

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
        uint32_t interruptMask;
};

#if SPI_INTERFACES_COUNT > 0
extern SPIClass SPI;
#endif
#if SPI_INTERFACES_COUNT > 1
extern SPIClass SPI1;
#endif
#if SPI_INTERFACES_COUNT > 2
extern SPIClass SPI2;
#endif
#if SPI_INTERFACES_COUNT > 3
extern SPIClass SPI3;
#endif
#if SPI_INTERFACES_COUNT > 4
extern SPIClass SPI4;
#endif
#if SPI_INTERFACES_COUNT > 5
extern SPIClass SPI5;
#endif

#endif // _ARDUINO_CORE_SPI_HPP_
