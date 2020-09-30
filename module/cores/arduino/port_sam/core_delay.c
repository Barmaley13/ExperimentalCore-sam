/*
  Copyright (c) 2011 Arduino LLC.  All right reserved.

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

#include "core_delay.h"
#include "Arduino.h"
#include "core_reset.h" // for tickReset()

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_delay.h"
#include "hpl_time_measure.h"


void delay(uint32_t ms)
{
    // Calculate remainder and whole
    uint16_t _ms = (uint16_t) (ms & 0x0000FFFF);
    uint32_t cycles = (uint16_t) (ms >> 16);
    
    // Remainder
    delay_ms(_ms);
    
    // Whole cycles
    while(cycles)
    {
        delay_ms(0xFFFF);
        cycles--;  
    }
}    

void delayMicroseconds(uint32_t usec)
{
    // Calculate remainder and whole
    uint16_t _usec = (uint16_t) (usec & 0x0000FFFF);
    uint32_t cycles = (uint16_t) (usec >> 16);
    
    // Remainder
    delay_us(_usec);
    
    // Whole cycles
    while(cycles)
    {
        delay_us(0xFFFF);
        cycles--;
    }
}

// The assumption is that those functions are used (during constant polling)
// User should prefer delay and delayMicroseconds to the following ones
uint32_t millis(void)
{
    static bool initialized = false;
    static uint32_t ms = 0;
    static uint32_t last_ticks = 0;
    static uint32_t ms_ticks, max_ticks;

    bool countflag;
    uint32_t current_ticks = 0;
    
    if (!initialized)
    {
        initialized = true;
        ms_ticks = _get_cycles_for_ms(1);
        max_ticks = SysTick-> LOAD;

        // Read to clear the flag (if needed)
        countflag = SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk;
            
        // Save for the next read
        last_ticks = SysTick->VAL;
    }
    else
    {
        // Read countflag and current_ticks so we can minimize the error
        countflag = (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);
        current_ticks = SysTick->VAL;
        
        // Calculate milliseconds
        if (countflag)
            ms += (last_ticks + max_ticks - current_ticks) / ms_ticks;
        else
            ms += (last_ticks - current_ticks) / ms_ticks;
        
        // Save for the next read
        last_ticks = current_ticks;
    }

    return ms;
}

uint32_t micros( void )
{
    static bool initialized = false;
    static uint32_t us = 0;
    static uint32_t last_ticks = 0;
    static uint32_t us_ticks, max_ticks;

    bool countflag;
    uint32_t current_ticks = 0;
    
    if (!initialized)
    {    
        initialized = true;
        us_ticks = _get_cycles_for_us(1);
        max_ticks = SysTick-> LOAD;
        
        // Read to clear the flag (if needed)
        countflag = SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk;
        
        // Save for the next read
        last_ticks = SysTick->VAL;
    }
    else
    {
        // Read countflag and current_ticks so we can minimize the error
        countflag = (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);
        current_ticks =  SysTick->VAL;
        
        // Calculate micro seconds
        if (countflag)
            us += (last_ticks + max_ticks - current_ticks) / us_ticks;
        else
            us += (last_ticks - current_ticks) / us_ticks;
        
        // Save for the next read
        last_ticks = current_ticks;
    }
    
    return us;
}


//void SysTick_Handler(void)
//{
//if (sysTickHook() != 0)
//{
//return;
//}
//
//SysTick_DefaultHandler();
//}
///** Tick Counter united by ms */
//static volatile uint32_t _ulTickCount=0 ;
//
//uint32_t millis( void )
//{
//// todo: ensure no interrupts
  //return _ulTickCount ;
//}
//
//// Interrupt-compatible version of micros
//// Theory: repeatedly take readings of SysTick counter, millis counter and SysTick interrupt pending flag.
//// When it appears that millis counter and pending is stable and SysTick hasn't rolled over, use these
//// values to calculate micros. If there is a pending SysTick, add one to the millis counter in the calculation.
//uint32_t micros( void )
//{
    //uint32_t ticks, ticks2;
    //uint32_t pend, pend2;
    //uint32_t count, count2;
//
    //ticks2  = SysTick->VAL;
    //pend2   = !!((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)||((SCB->SHCSR & SCB_SHCSR_SYSTICKACT_Msk)))  ;
    //count2  = _ulTickCount;
//
    //do 
	//{
        //ticks=ticks2;
        //pend=pend2;
        //count=count2;
        //ticks2  = SysTick->VAL;
        //pend2   = !!((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)||((SCB->SHCSR & SCB_SHCSR_SYSTICKACT_Msk)))  ;
        //count2  = _ulTickCount;
    //} while ((pend != pend2) || (count != count2) || (ticks < ticks2));
//
    //return ((count+pend) * 1000) + (((SysTick->LOAD  - ticks)*(1048576/(VARIANT_MCK/1000000)))>>20) ;
    //// this is an optimization to turn a runtime division into two compile-time divisions and
    //// a runtime multiplication and shift, saving a few cycles
//}

// original function:
// uint32_t micros( void )
// {
//     uint32_t ticks ;
//     uint32_t count ;
//
//     SysTick->CTRL;
//     do {
//         ticks = SysTick->VAL;
//         count = GetTickCount();
//     } while (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);
//
//     return count * 1000 + (SysTick->LOAD + 1 - ticks) / (SystemCoreClock/1000000) ;
// }


//void delay( uint32_t ms )
//{
  //if ( ms == 0 )
  //{
    //return ;
  //}
//
  //uint32_t start = _ulTickCount ;
//
  //do
  //{
    //yield() ;
  //} while ( _ulTickCount - start < ms ) ;
//}
//
//void SysTick_DefaultHandler(void)
//{
  //// Increment tick count each ms
  //_ulTickCount++;
  //tickReset();
//}

#ifdef __cplusplus
}
#endif
