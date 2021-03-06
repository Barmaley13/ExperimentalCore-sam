/*
 * core_variant.c
 *
 * Created: 9/27/2020 11:00:33 AM
 *  Author: Kirill
 */ 

#include "core_variant.h"

const uint8_t FlexcomIds[NUM_FLEXCOM] = {
	#if (SAM4S_SERIES || SAM4E_SERIES)
		ID_FLEXCOM0, ID_FLEXCOM1
	#elif SAM3XA_SERIES
		ID_FLEXCOM0, ID_FLEXCOM1, ID_FLEXCOM2
	#elif SAME70_SERIES
		ID_FLEXCOM0, ID_FLEXCOM1, ID_FLEXCOM2, ID_FLEXCOM3, ID_FLEXCOM4
	#elif __SAMG55J19__
		ID_FLEXCOM0, ID_FLEXCOM1, ID_FLEXCOM2, ID_FLEXCOM3, ID_FLEXCOM4, ID_FLEXCOM5, ID_FLEXCOM6, ID_FLEXCOM7
	#elif SAMG55_SERIES
		ID_FLEXCOM0, ID_FLEXCOM1, ID_FLEXCOM2, ID_FLEXCOM3, ID_FLEXCOM4, ID_FLEXCOM5, ID_FLEXCOM6
	#endif
};
