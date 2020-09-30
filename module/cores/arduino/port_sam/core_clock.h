/*
 * core_clock.h
 *
 * Created: 9/30/2020 10:00:15 AM
 *  Author: Kirill
 */ 


#ifndef CORE_CLOCK_H_
#define CORE_CLOCK_H_

#include "sam.h"
    
static inline void pmc_enable_periph_clock(uint32_t clockId)
{   
    uint32_t clockMask;
    if (clockId < 32)
    {
        clockMask = 1 << clockId;
        if (!(PMC->PMC_PCSR0 & clockMask))
            PMC->PMC_PCER0 = clockMask;
    }
    else
    {
        clockId -= 32;
        clockMask = 1 << clockId;
        if (!(PMC->PMC_PCSR1 & clockMask))
            PMC->PMC_PCER1 = clockMask;
    }
}

static inline void pmc_disable_periph_clock(uint32_t clockId)
{
    uint32_t clockMask;
    if (clockId < 32)
    {
        clockMask = 1 << clockId;
        if (!(PMC->PMC_PCSR0 & clockMask))
            PMC->PMC_PCDR0 = clockMask;
    }
    else
    {
        clockId -= 32;
        clockMask = 1 << clockId;
        if (!(PMC->PMC_PCSR1 & clockMask))
            PMC->PMC_PCDR1 = clockMask;
    }
}


#endif /* CORE_CLOCK_H_ */