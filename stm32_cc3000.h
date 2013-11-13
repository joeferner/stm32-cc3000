
#ifndef STM32_CC3000_H
#define	STM32_CC3000_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

void cc3000_setup();
uint16_t cc3000_get_firmware_version();

#ifdef	__cplusplus
}
#endif

#endif	/* STM32_CC3000_H */

