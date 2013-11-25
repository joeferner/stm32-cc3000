#ifndef TIME_H
#define	TIME_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

void time_setup();
volatile uint32_t time_ms();
void time_SysTick_Handler();

#ifdef	__cplusplus
}
#endif

#endif	/* TIME_H */

