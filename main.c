
#include <string.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include "cc3000-host-driver/socket.h"
#include "cc3000-host-driver/netapp.h"
#include "stm32_cc3000.h"
#include "debug.h"
#include "platform_config.h"
#include "delay.h"
#include "connection_info.h"
#include "time.h"
#include "ntp.h"

char cc3000_device_name[] = "CC3000";

void setup();
void loop();
void assert_failed(uint8_t* file, uint32_t line);

int countdown = 0;

int main(void) {
  setup();
  while (1) {
    loop();
  }
  return 0;
}

void setup() {
  uint8_t cc3000MajorFirmwareVersion, cc3000MinorFirmwareVersion;

  debug_setup();

  cc3000_setup(0, 0);

  cc3000_get_firmware_version(&cc3000MajorFirmwareVersion, &cc3000MinorFirmwareVersion);
  debug_write("major: 0x");
  debug_write_u8(cc3000MajorFirmwareVersion, 16);
  debug_write_line("");
  debug_write("minor: 0x");
  debug_write_u8(cc3000MinorFirmwareVersion, 16);
  debug_write_line("");
  if (cc3000MajorFirmwareVersion != 0x01 || cc3000MinorFirmwareVersion != 0x18) {
    debug_write_line("Wrong firmware version!");
    while (1);
  }

  cc3000_display_mac_address();

  debug_write_line("Deleting old connection profiles");
  if (cc3000_delete_profiles() != 0) {
    debug_write_line("Failed!");
    while (1);
  }

#ifdef STATIC_IP_ADDRESS
  unsigned long aucIP = STATIC_IP_ADDRESS;
  unsigned long aucSubnetMask = STATIC_SUBNET_MASK;
  unsigned long aucDefaultGateway = STATIC_DEFAULT_GATEWAY;
  unsigned long aucDNSServer = STATIC_DNS_SERVER;
  if (netapp_dhcp(&aucIP, &aucSubnetMask, &aucDefaultGateway, &aucDNSServer) != 0) {
    debug_write_line("netapp_dhcp Failed!");
    while (1);
  }
#else
  unsigned long aucIP = 0;
  unsigned long aucSubnetMask = 0;
  unsigned long aucDefaultGateway = 0;
  unsigned long aucDNSServer = 0;
  if (netapp_dhcp(&aucIP, &aucSubnetMask, &aucDefaultGateway, &aucDNSServer) != 0) {
    debug_write_line("netapp_dhcp Failed!");
    while (1);
  }
#endif

  // Attempt to connect to an access point
  char *ssid = WLAN_SSID; /* Max 32 chars */
  debug_write("Attempting to connect to ");
  debug_write_line(ssid);

  // NOTE: Secure connections are not available in 'Tiny' mode!
  if (cc3000_connect_to_ap(WLAN_SSID, WLAN_PASS, WLAN_SECURITY) != 0) {
    debug_write_line("Connect Failed!");
    while (1);
  }

  debug_write_line("Connected!");

#ifndef STATIC_IP_ADDRESS
  // Wait for DHCP to complete
  debug_write_line("Request DHCP");
  while (cc3000_check_dhcp() != 0) {
    delay_ms(100);
  }
#endif

  while(!cc3000_is_connected()) {
    delay_ms(100);
  }
  
  cc3000_display_ipconfig();
}

void loop() {
  if (countdown == 0) {
    uint32_t t = ntp_query_time_server();
    if (t) { // Success?
      debug_write("Current UNIX time: ");
      debug_write_u32(t, 10);
      debug_write_line(" (seconds since 1/1/1970 UTC)");

      countdown = 1;
    }
  } else {
    countdown--;
  }

  delay_ms(15000);
}

void debug_on_rx(uint8_t* data, uint16_t len) {
  debug_write_bytes(data, len);
}

void assert_failed(uint8_t* file, uint32_t line) {
  debug_write("assert_failed: file ");
  debug_write((const char*) file);
  debug_write(" on line ");
  debug_write_u32(line, 10);
  debug_write_line("");

  /* Infinite loop */
  while (1) {
  }
}
