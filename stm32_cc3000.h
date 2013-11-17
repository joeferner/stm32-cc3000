
#ifndef STM32_CC3000_H
#define	STM32_CC3000_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

  int cc3000_setup(int patchesAvailableAtHost, int useSmartConfigData);
  int cc3000_get_firmware_version(uint8_t *major, uint8_t *minor);
  int cc3000_get_mac_address(uint8_t *addr);
  void cc3000_irq();

  typedef void (*cc3000_spi_rx_handler_t)(void *p);

#ifdef	__cplusplus
}
#endif

#endif	/* STM32_CC3000_H */

