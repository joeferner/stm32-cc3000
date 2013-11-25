
#ifndef STM32_CC3000_H
#define	STM32_CC3000_H

#include <stdint.h>
#include "cc3000-host-driver/wlan.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define WLAN_CONNECT_TIMEOUT_MS 10000
#define MAX_SOCKETS             32
#define MAXSSID					        32
#define MAXLENGTHKEY 		        32
  extern char cc3000_device_name[];

  int cc3000_setup(int patchesAvailableAtHost, int useSmartConfigData);
  int cc3000_get_firmware_version(uint8_t *major, uint8_t *minor);
  int cc3000_get_mac_address(uint8_t *addr);
  void cc3000_irq();
  int cc3000_delete_profiles();
  int cc3000_connect_to_ap(const char *ssid, const char *key, uint8_t secmode);
  int cc3000_check_dhcp();
  int cc3000_get_ip_address(uint32_t *retip, uint32_t *netmask, uint32_t *gateway, uint32_t *dhcpserv, uint32_t *dnsserv);
  int cc3000_is_socket_closed(uint32_t sock);
  void cc3000_display_mac_address();
  void cc3000_display_ifconfig();

  uint32_t iptol(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
  int16_t connect_udp(uint32_t destIP, uint16_t destPort);
  int available(int16_t socket);

  typedef void (*cc3000_spi_rx_handler_t)(void *p);

#ifdef	__cplusplus
}
#endif

#endif	/* STM32_CC3000_H */

