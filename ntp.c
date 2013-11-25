
#include <stdint.h>
#include <string.h>
#include "ntp.h"
#include "connection_info.h"
#include "debug.h"
#include "util.h"
#include "stm32_cc3000.h"
#include "cc3000-host-driver/socket.h"

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

uint32_t ntp_query_time_server() {
  ntp_packet_t ntp_packet;
  uint32_t ip;
  int16_t client;
  int result;

  debug_write_line("Locating time server...");

  // Host name to IP lookup; use NTP pool (rotates through servers)
  result = gethostbyname(NTP_SERVER, strlen(NTP_SERVER), &ip);
  if (result < 0) {
    debug_write("Could not get host by name: " NTP_SERVER " (");
    debug_write_i32(result, 10);
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
  ntp_packet.transmit_timestamp.seconds = 0x35cb3dd6;
  int r = send(client, (uint8_t*) & ntp_packet, sizeof (ntp_packet), 0);
  debug_write("sent: ");
  debug_write_i32(r, 10);
  debug_write_line("bytes");

  debug_write_line("Awaiting response...");
  memset((uint8_t*) & ntp_packet, 0, sizeof (ntp_packet));

  r = recv(client, (uint8_t*) & ntp_packet, sizeof (ntp_packet), 0);
  debug_write("recv: ");
  debug_write_i32(r, 10);
  debug_write_line("bytes");

  debug_write_u8_array((uint8_t*) & ntp_packet, sizeof (ntp_packet));
  debug_write_line("");

  uint32_t time = swap_endian(ntp_packet.reference_timestamp.seconds);

  debug_write_line("OK");

  closesocket(client);

  return time - 0x83AA7E80; // the seconds from Jan 1, 1900 to Jan 1, 1970
}

