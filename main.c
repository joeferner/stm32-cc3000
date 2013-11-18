
#include <string.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_exti.h>
#include "cc3000-host-driver/socket.h"
#include "stm32_cc3000.h"
#include "debug.h"
#include "platform_config.h"
#include "delay.h"
#include "connection_info.h"

#define NTP_SERVER     "pool.ntp.org"

char cc3000_device_name[] = "CC3000";

void setup();
void loop();
void assert_failed(uint8_t* file, uint32_t line);
void display_mac_address();
uint32_t query_time_server();
int16_t connect_udp(uint32_t destIP, uint16_t destPort);
int available(int16_t socket);
uint32_t swap_endian(uint32_t val);

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

  //  debug_write_line("Deleting old connection profiles");
  //  if (cc3000_delete_profiles() != 0) {
  //    debug_write_line("Failed!");
  //    while (1);
  //  }

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
  debug_write_ip(ipAddress);
  debug_write_line("");

  debug_write("Netmask: ");
  debug_write_ip(netmask);
  debug_write_line("");

  debug_write("Gateway: ");
  debug_write_ip(gateway);
  debug_write_line("");

  debug_write("DHCPsrv: ");
  debug_write_ip(dhcpserv);
  debug_write_line("");

  debug_write("DNSserv: ");
  debug_write_ip(dnsserv);
  debug_write_line("");
}

void loop() {
  if (countdown == 0) {
    uint32_t t = query_time_server();
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

#define LEAP_INDICATOR_UNKNOWN   0xc0

#define NTP_VERSION_3            0x18

#define NTP_MODE_CLIENT          0x03

typedef struct {
  uint32_t seconds;
  uint32_t fraction;
} ntp_time_t;

typedef struct {
  uint8_t flags;
  uint8_t stratum;
  uint8_t poll_interval;
  uint8_t precision;
  uint32_t root_delay;
  uint32_t root_dispersion;
  uint32_t reference_id;
  ntp_time_t reference_timestamp;
  ntp_time_t origin_timestamp;
  ntp_time_t receive_timestamp;
  ntp_time_t transmit_timestamp;
} ntp_packet_t;

uint32_t query_time_server() {
  ntp_packet_t ntp_packet;
  uint32_t ip;
  uint32_t t = 0L;
  int16_t client;
  int result;

  debug_write_line("Locating time server...");

  // Host name to IP lookup; use NTP pool (rotates through servers)
  result = gethostbyname(NTP_SERVER, strlen(NTP_SERVER), &ip);
  if (result < 0) {
    debug_write("Could not get host by name: " NTP_SERVER " (");
    debug_write_u32(result, 10);
    debug_write_line(")");
    return 0;
  }

  debug_write_line("Attempting connection...");

  client = connect_udp(ip, 123);
  if (client == -1) {
    debug_write_line("Could not connect");
    return 0;
  }

  debug_write_line("connected!");
  debug_write_line("Issuing request...");

  // Assemble and issue request packet
  memset((uint8_t*) & ntp_packet, 0, sizeof (ntp_packet));
  ntp_packet.flags =
          LEAP_INDICATOR_UNKNOWN
          | NTP_VERSION_3
          | NTP_MODE_CLIENT;
  ntp_packet.stratum = 0;
  ntp_packet.poll_interval = 10;
  ntp_packet.precision = 0xfa;
  ntp_packet.root_dispersion = 0x00010290;
  ntp_packet.transmit_timestamp.seconds = 0xc50204ec;
  int r = send(client, (uint8_t*) & ntp_packet, sizeof (ntp_packet), 0);
  debug_write("sent: ");
  debug_write_u16(r, 10);
  debug_write_line("bytes");

  debug_write_line("Awaiting response...");
  memset((uint8_t*) & ntp_packet, 0, sizeof (ntp_packet));

  r = recv(client, (uint8_t*) & ntp_packet, sizeof (ntp_packet), 0);
  debug_write("recv: ");
  debug_write_u16(r, 10);
  debug_write_line("bytes");

  debug_write_u8_array((uint8_t*) & ntp_packet, sizeof (ntp_packet));
  debug_write_line("");

  uint32_t time = swap_endian(ntp_packet.reference_timestamp.seconds);

  debug_write_line("OK");

  closesocket(client);

  return time - 0x83AA7E80; // the seconds from Jan 1, 1900 to Jan 1, 1970
}

int16_t connect_udp(uint32_t destIP, uint16_t destPort) {
  sockaddr socketAddress;
  int16_t udp_socket;

  // Create the socket(s)
  // socket   = SOCK_STREAM, SOCK_DGRAM, or SOCK_RAW 
  // protocol = IPPROTO_TCP, IPPROTO_UDP or IPPROTO_RAW
  udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (udp_socket == -1) {
    debug_write_line("Failed to open socket");
    return -1;
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
  debug_write_ip(destIP);
  debug_write_ch(':');
  debug_write_u16(destPort, 10);
  debug_write_line("");

  if (connect(udp_socket, &socketAddress, sizeof (socketAddress)) != 0) {
    debug_write_line("Connection error");
    closesocket(udp_socket);
    return -1;
  }

  return udp_socket;
}

int available(int16_t socket) {
  // not open!
  if (socket < 0) {
    return 0;
  }

  // do a select() call on this socket
  timeval timeout;
  fd_set fd_read;

  memset(&fd_read, 0, sizeof (fd_read));
  FD_SET(socket, &fd_read);

  timeout.tv_sec = 0;
  timeout.tv_usec = 5000; // 5 millisec

  int s = select(socket + 1, &fd_read, NULL, NULL, &timeout);
  if (s == 1) {
    return 1; // some data is available to read
  } else {
    return 0; // no data is available
  }
}

uint32_t swap_endian(uint32_t val) {
  return ((val << 24) & 0xff000000)
          | ((val << 8) & 0x00ff0000)
          | ((val >> 8) & 0x0000ff00)
          | ((val >> 24) & 0x000000ff);
}
