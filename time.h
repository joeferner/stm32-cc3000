#ifndef TIME_H
#define	TIME_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

void time_config();
volatile uint32_t time_ms();

#ifdef	__cplusplus
}
#endif

#endif	/* TIME_H */

