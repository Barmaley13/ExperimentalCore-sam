/*
  Copyright (c) 2015 Thibaut VIARD.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "CoreSerial.hpp"
#include "core_private.h"
#include "core_cortex_vectors.h"
#include "driver_init.h"
#include "hpl_pmc.h"
#include "hpl_usart_async.h"
#include "variant.h"
#include "log.h"


// Constructors ////////////////////////////////////////////////////////////////
SAMSerial::SAMSerial(Usart *usart, uint32_t pinRX, uint32_t pinTX, void (*irq_handler)(void))
{
    _usart = usart;
    _flexcom = (Flexcom *)((uint32_t)_usart - 0x200U);

    _pinRX = PinMap[pinRX].ulPin;
    _pinTX = PinMap[pinTX].ulPin;

    _pinRXMux = PinMap[pinRX].ulPinType;
    _pinTXMux = PinMap[pinTX].ulPinType;
    
    _irqn = HardFault_IRQn;
    _clockId = 0;

    _irq_handler = irq_handler;
}

// Private Methods //////////////////////////////////////////////////////////////
void SAMSerial::init(const uint32_t baudRate, const UARTModes mode)
{
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
    vectorAssign(_irqn, _irq_handler);

    // Activate Serial peripheral clock
    _clockId = FLEXCOM_IDS[flexcomIndex];
    _pmc_enable_periph_clock(_clockId);
    
    // PIO init
    gpio_set_pin_function(_pinRX, _pinRXMux);
    gpio_set_pin_function(_pinTX, _pinTXMux);
    
    // Init USART in async fashion
    struct _usart_async_device dev;
    _usart_async_init(&dev, _flexcom);
    
    // Configure baud rate and UART mode    
    config(baudRate, mode);
}

void SAMSerial::config(const uint32_t baudRate, const UARTModes mode)
{
    uint32_t mr = 0;
    uint32_t brgr = 0;
    uint32_t baud_cd = 0;
    uint32_t baud_fp = 0;

    // Configure mode
    switch ( mode & HARDSER_PARITY_MASK)
    {
        case HARDSER_PARITY_ODD:
        mr|=US_MR_PAR_ODD;
        break;
                
        case HARDSER_PARITY_NONE:
        mr|=US_MR_PAR_NO;
        break;
                
        default:
        mr |= US_MR_PAR_EVEN;
    }
            
    switch ( mode & HARDSER_STOP_BIT_MASK)
    {
        case HARDSER_STOP_BIT_1_5:
        mr|=US_MR_NBSTOP_1_5_BIT;
        break;
                
        case HARDSER_STOP_BIT_2:
        mr|=US_MR_NBSTOP_2_BIT;
        break;
                
        default:
        mr|=US_MR_NBSTOP_1_BIT;
    }

    /* UART has Character Length fixed to 8bits */
    switch ( mode & HARDSER_DATA_MASK)
    {
        case HARDSER_DATA_5:
        mr|=US_MR_CHRL_5_BIT;
        break;
                
        case HARDSER_DATA_6:
        mr|=US_MR_CHRL_6_BIT;
        break;
                
        case HARDSER_DATA_7:
        mr|=US_MR_CHRL_7_BIT;
        break;
                
        default:
        mr|=US_MR_CHRL_8_BIT;
    }

    //// Some extra USART settings
    //mr |= US_MR_USART_MODE(0);
    //mr |= US_MR_USCLKS(0);
    //mr |= (0 << US_MR_SYNC_Pos);
    //mr |= US_MR_CHMODE(0);
    //mr |= (0 << US_MR_MSBF_Pos);
    //mr |= (0 << US_MR_MODE9_Pos);
    //mr |= (0 << US_MR_CLKO_Pos);
    #define CONF_USART_OVER         0
    //mr |= (CONF_USART_OVER << US_MR_OVER_Pos);
    mr |= (1 << US_MR_INACK_Pos);
    //mr |= (0 << US_MR_DSNACK_Pos);
    //mr |= (0 << US_MR_INVDATA_Pos);
    //mr |= US_MR_MAX_ITERATION(0);
    //mr |= (0 << US_MR_FILTER_Pos);
            
    // Calculate baud rate register value
    baud_cd = ((VARIANT_MCK) / baudRate / 8 / (2 - CONF_USART_OVER));
    baud_fp = ((VARIANT_MCK) / baudRate / (2 - CONF_USART_OVER) - 8 * baud_cd);
    brgr |= US_BRGR_CD(baud_cd);
    brgr |= US_BRGR_FP(baud_fp);
            
    // Write registers
    _usart->US_MR = mr;
    _usart->US_BRGR = brgr;

    // Configure interrupts
    _usart->US_IDR = 0xFFFFFFFF;
    _usart->US_IER = US_IER_RXRDY | US_IER_OVRE | US_IER_FRAME;

    // Make sure both ring buffers are initialized back to empty.
    _rx_buffer.clear();
    _tx_buffer.clear();

    // Enable receiver and transmitter
    _usart->US_CR = US_CR_RXEN | US_CR_TXEN;
}

// Public Methods //////////////////////////////////////////////////////////////
void SAMSerial::begin(const uint32_t baudRate)
{
    init(baudRate, SERIAL_8N1);
}

void SAMSerial::begin(const uint32_t baudRate, const UARTModes mode)
{
    init(baudRate, mode);
}

void SAMSerial::end( void )
{
    // Clear any received data
    _rx_buffer.clear();

    // Wait for any outstanding data to be sent
    flush();

    // Disable all UART interrupts
    _usart->US_IER = 0;

    // Disable UART interrupt in NVIC
    NVIC_DisableIRQ(_irqn);
    NVIC_ClearPendingIRQ(_irqn);

    // Dynamic assignment of IRQ handler
    vectorReset(_irqn);

    // Deactivate Serial peripheral clock
    _pmc_disable_periph_clock(_clockId);
    
    // TODO: Deinit pins?
}

void SAMSerial::setInterruptPriority(uint32_t priority)
{
    NVIC_SetPriority(_irqn, priority & 0x0F);
}

uint32_t SAMSerial::getInterruptPriority()
{
    return NVIC_GetPriority(_irqn);
}

int SAMSerial::available(void)
{
    return _rx_buffer.available();
}

int SAMSerial::availableForWrite(void)
{
    int head = _tx_buffer._iHead;
    int tail = _tx_buffer._iTail;

    if (head >= tail)
    {
        return SERIAL_BUFFER_SIZE - 1 - head + tail;
    }
    else
    {
        return tail - head - 1;
    }
}

int SAMSerial::peek( void )
{
    return _rx_buffer.peek();
}

int SAMSerial::read( void )
{
    uint8_t uc = -1;
    
    // if the head isn't ahead of the tail, we don't have any characters
    if ( _rx_buffer._iHead != _rx_buffer._iTail )
    {
        uc = _rx_buffer._aucBuffer[_rx_buffer._iTail];
        _rx_buffer._iTail = (unsigned int)(_rx_buffer._iTail + 1) % SERIAL_BUFFER_SIZE;
    }

    return uc;
}

void SAMSerial::flush( void )
{    
    // Wait for transmit data to be sent
    while (_tx_buffer._iHead != _tx_buffer._iTail){};
    // Wait for transmission to complete
    while (_usart->US_CSR & US_CSR_TXRDY){};
}

size_t SAMSerial::write( const uint8_t uc_data )
{    
    // Spin locks if we're about to overwrite the buffer. This continues once the data is sent
    int l = (_tx_buffer._iHead + 1) % SERIAL_BUFFER_SIZE;
    while (_tx_buffer._iTail == l){};

    _tx_buffer._aucBuffer[_tx_buffer._iHead] = uc_data;
    _tx_buffer._iHead = l;
    
    // Enable TX interrupt
    _usart->US_IER = US_IMR_TXRDY;

  return 1;
}

void SAMSerial::IrqHandler( void )
{
    uint32_t csr = _usart->US_CSR;
    uint32_t imr = _usart->US_IMR;
    
    // Transmitting?
    if (csr & US_CSR_TXRDY && imr & US_IMR_TXRDY) {
        // Clear interrupt    
        _usart->US_IDR = US_IMR_TXRDY;
        
        // Check buffer
        if (_tx_buffer._iTail != _tx_buffer._iHead)
        {
            // Send to buffer and increment index
            _usart->US_THR = _tx_buffer._aucBuffer[_tx_buffer._iTail];
            _tx_buffer._iTail = (unsigned int)(_tx_buffer._iTail + 1) % SERIAL_BUFFER_SIZE;
            // Enable tx interrupt
            _usart->US_IER = US_IMR_TXRDY;
        }
        else
        {
            // Done with tx!
            _usart->US_IER = US_IMR_TXEMPTY;
        }

	}
    // Tx empty?
    else if (csr & US_CSR_TXEMPTY && imr & US_IMR_TXEMPTY) 
    {
        // Clear interrupt
        _usart->US_IDR = US_IMR_TXEMPTY;
	    // Tx done callback can be inserted here...
	} 
    // Receiving?
    else if (csr & US_CSR_RXRDY && imr & US_IMR_RXRDY)
    {
        // Read char    
        uint8_t r_char = _usart->US_RHR;	    
        
        // Error?
        if (csr & (US_CSR_OVRE | US_CSR_FRAME | US_CSR_PARE)) 
        {
            // Report error
            _usart->US_CR = US_CR_RSTSTA;
            // Error handler can be inserted here...
	    }
        else
        {
            // Store char
            _rx_buffer.store_char(r_char);
            // Receive callback can be inserted here...
        }
	}
    // Error
    else if (csr & (US_CSR_OVRE | US_CSR_FRAME | US_CSR_PARE)) 
    {
        // Report error
        _usart->US_CR = US_CR_RSTSTA;
        // Error handler can be inserted here...
    }

}
