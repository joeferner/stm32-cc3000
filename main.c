
#include <string.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_exti.h>
#include "stm32_cc3000.h"
#include "debug.h"
#include "platform_config.h"
#include "delay.h"
#include "cc3000-host-driver/socket.h"

#define WLAN_SSID      "testap"
#define WLAN_PASS      "test"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY  WLAN_SEC_UNSEC

#define NTP_SERVER     "pool.ntp.org"

char cc3000_device_name[] = "CC3000";

void setup();
void loop();
void assert_failed(uint8_t* file, uint32_t line);
void display_mac_address();
uint32_t query_time_server();
int32_t connect_udp(uint32_t destIP, uint16_t destPort);

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
  debug_write("major: ");
  debug_write_u8(cc3000MajorFirmwareVersion, 16);
  debug_write_line("");
  debug_write("minor: ");
  debug_write_u8(cc3000MinorFirmwareVersion, 16);
  debug_write_line("");
  if (cc3000MajorFirmwareVersion != 0x01 || cc3000MinorFirmwareVersion != 0x18) {
    debug_write_line("Wrong firmware version!");
    while (1);
  }

  display_mac_address();

  debug_write_line("Deleting old connection profiles");
  if (cc3000_delete_profiles() != 0) {
    debug_write_line("Failed!");
    while (1);
  }

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

  // Wait for DHCP to complete
  debug_write_line("Request DHCP");
  while (cc3000_check_dhcp() != 0) {
    delay_ms(100);
  }

  // Display the IP address DNS, Gateway, etc.
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  while (cc3000_get_ip_address(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv) != 0) {
    delay_ms(1000);
  }
  debug_write("IP Addr: ");
  debug_write_ip_le(ipAddress);
  debug_write_line("");

  debug_write("Netmask: ");
  debug_write_ip_le(netmask);
  debug_write_line("");

  debug_write("Gateway: ");
  debug_write_ip_le(gateway);
  debug_write_line("");

  debug_write("DHCPsrv: ");
  debug_write_ip_le(dhcpserv);
  debug_write_line("");

  debug_write("DNSserv: ");
  debug_write_ip_le(dnsserv);
  debug_write_line("");
}

void loop() {
  if (countdown == 0) {
    uint32_t t = query_time_server();
    if (t) { // Success?
      debug_write("Current UNIX time: ");
      debug_write_u32(t / 1000, 10);
      debug_write_line(" (seconds since 1/1/1970 UTC)");

      countdown = 1;
    }
  } else {
    countdown--;
  }

  delay_ms(15000);
}

void display_mac_address() {
  uint8_t macAddress[20];

  if (cc3000_get_mac_address(macAddress) != 0) {
    debug_write_line("Unable to retrieve MAC Address!");
  } else {
    debug_write("MAC Address: ");
    debug_write_u8_array(macAddress, 6);
    debug_write_line("");
  }
}

void debug_on_rx(uint8_t* data, uint16_t len) {
  debug_write_bytes(data, len);
}

void assert_failed(uint8_t* file, uint32_t line) {
  debug_write("assert_failed: file ");
  debug_write((const char*) file);
  debug_write(" on line ");
  debug_write_u32(line, 16); // TODO change to base 10
  debug_write_line("");

  /* Infinite loop */
  while (1) {
  }
}

/* !!! Interrupt handler - Don't change this function name !!! */
void EXTI1_IRQHandler(void) {
  if (EXTI_GetITStatus(CC3000_IRQ_EXTI_LINE) != RESET) {
    cc3000_irq();
    EXTI_ClearITPendingBit(CC3000_IRQ_EXTI_LINE);
  }
}

uint32_t query_time_server() {
  uint8_t buf[48];
  uint32_t ip;
  uint32_t t = 0L;
  int32_t client;

  debug_write_line("Locating time server...");

  // Host name to IP lookup; use NTP pool (rotates through servers)
  if (gethostbyname(NTP_SERVER, strlen(NTP_SERVER), &ip) > 0) {
    debug_write_line("Could not get host by name: pool.ntp.org");
    return 0;
  }

  static const char timeReqA[] = {227, 0, 6, 236};
  static const char timeReqB[] = {49, 78, 49, 52};

  debug_write_line("Attempting connection...");

  client = connect_udp(ip, 123);
  if (client == 0) {
    debug_write_line("Could not connect");
    return 0;
  }

  debug_write_line("connected!");
  debug_write_line("Issuing request...");

  // Assemble and issue request packet
  memset(buf, 0, sizeof (buf));
  memcpy(buf, timeReqA, sizeof (timeReqA));
  memcpy(&buf[12], timeReqB, sizeof (timeReqB));
  send(client, buf, sizeof (buf), 0);

  debug_write_line("Awaiting response...");
  memset(buf, 0, sizeof (buf));

  int r = recv(client, buf, sizeof (buf), 0);
  debug_write("recv: ");
  debug_write_u16(r, 10);
  debug_write_line("bytes");

  t = (((unsigned long) buf[40] << 24) |
          ((unsigned long) buf[41] << 16) |
          ((unsigned long) buf[42] << 8) |
          (unsigned long) buf[43]) - 2208988800UL;
  debug_write_line("OK");

  closesocket(client);

  return t;
}

int32_t connect_udp(uint32_t destIP, uint16_t destPort) {
  sockaddr socketAddress;
  int32_t udp_socket;

  // Create the socket(s)
  // socket   = SOCK_STREAM, SOCK_DGRAM, or SOCK_RAW 
  // protocol = IPPROTO_TCP, IPPROTO_UDP or IPPROTO_RAW
  udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (-1 == udp_socket) {
    debug_write_line("Failed to open socket");
    return 0;
  }

  // Try to open the socket
  memset(&socketAddress, 0x00, sizeof (socketAddress));
  socketAddress.sa_family = AF_INET;
  socketAddress.sa_data[0] = (destPort & 0xFF00) >> 8; // Set the Port Number
  socketAddress.sa_data[1] = (destPort & 0x00FF);
  socketAddress.sa_data[2] = destIP >> 24;
  socketAddress.sa_data[3] = destIP >> 16;
  socketAddress.sa_data[4] = destIP >> 8;
  socketAddress.sa_data[5] = destIP;

  debug_write("Connect to ");
  debug_write_ip_le(destIP);
  debug_write_ch(':');
  debug_write_u16(destPort, 10);
  debug_write_line("");

  if (-1 == connect(udp_socket, &socketAddress, sizeof (socketAddress))) {
    debug_write_line("Connection error");
    closesocket(udp_socket);
    return 0;
  }

  return udp_socket;
}