
#include <stdint.h>
#include <string.h>
#include "ntp.h"
#include "connection_info.h"
#include "debug.h"
#include "time.h"
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

void ntp_query_init(tNtpQuery* query, const char* ntpServer, uint32_t timeout) {
  query->state = NTP_QUERY_STATE_START;
  strcpy(query->ntpServer, ntpServer);
  query->timeout = timeout;
  query->socket = -1;
}

tNtpQueryResult ntp_query_loop(tNtpQuery* query) {
  int result;
  ntp_packet_t ntp_packet;

  switch (query->state) {
    case NTP_QUERY_STATE_START:
    case NTP_QUERY_STATE_GET_HOST_BY_NAME:
      query->state = NTP_QUERY_STATE_GET_HOST_BY_NAME;

      debug_write_line("Locating time server...");
      result = gethostbyname(query->ntpServer, strlen(query->ntpServer), &query->ntpServerIp);
      if (result < 0) {
        debug_write("Could not get host by name: ");
        debug_write(query->ntpServer);
        debug_write(" (");
        debug_write_i32(result, 10);
        debug_write_line(")");
        return NTP_QUERY_RESULT_ERROR_GET_HOST_BY_NAME;
      }

      query->state = NTP_QUERY_STATE_CONNECT_UDP;
      return NTP_QUERY_RESULT_PENDING;

    case NTP_QUERY_STATE_CONNECT_UDP:
      debug_write_line("Attempting connection...");

      query->socket = connect_udp(query->ntpServerIp, 123);
      if (query->socket == -1) {
        debug_write_line("Could not connect");
        return NTP_QUERY_RESULT_ERROR_CONNECT;
      }

      query->state = NTP_QUERY_STATE_REQUEST_TIME;
      debug_write_line("connected!");
      return NTP_QUERY_RESULT_PENDING;

    case NTP_QUERY_STATE_REQUEST_TIME:
      debug_write_line("Issuing request...");
      if (query->socket == -1) {
        debug_write_line("bad socket");
        return NTP_QUERY_RESULT_ERROR_BAD_SOCKET;
      }

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
      int r = send(query->socket, (uint8_t*) & ntp_packet, sizeof (ntp_packet), 0);
      if (r == -1) {
        debug_write_line("send error");
        return NTP_QUERY_RESULT_ERROR_SEND;
      }
      debug_write("sent: ");
      debug_write_i32(r, 10);
      debug_write_line("bytes");
      if (r != sizeof (ntp_packet)) {
        debug_write_line("send error");
        return NTP_QUERY_RESULT_ERROR_SEND;
      }
      query->startTime = time_ms();
      query->state = NTP_QUERY_STATE_RECV;
      return NTP_QUERY_RESULT_PENDING;

    case NTP_QUERY_STATE_RECV:
      if (query->socket == -1) {
        debug_write_line("bad socket");
        return NTP_QUERY_RESULT_ERROR_BAD_SOCKET;
      }
      
      if (available(query->socket) > 0) {
        memset((uint8_t*) & ntp_packet, 0, sizeof (ntp_packet));
        r = recv(query->socket, (uint8_t*) & ntp_packet, sizeof (ntp_packet), 0);
        if (r == -1) {
          debug_write_line("recv error");
          return NTP_QUERY_RESULT_ERROR_RECV;
        }
        debug_write("recv: ");
        debug_write_i32(r, 10);
        debug_write_line("bytes");
        if (r < sizeof (ntp_packet)) {
          debug_write_line("recv error");
          return NTP_QUERY_RESULT_ERROR_RECV;
        }
        debug_write_u8_array((uint8_t*) & ntp_packet, sizeof (ntp_packet));
        debug_write_line("");

        uint32_t time = swap_endian(ntp_packet.reference_timestamp.seconds);

        query->time = time - 0x83AA7E80; // the seconds from Jan 1, 1900 to Jan 1, 1970
        ntp_query_close(query);
        return NTP_QUERY_RESULT_OK;
      }

      if ((time_ms() - query->startTime) > query->timeout) {
        ntp_query_close(query);
        query->state = NTP_QUERY_STATE_GET_HOST_BY_NAME;
        return NTP_QUERY_RESULT_ERROR_TIMEOUT;
      }

      break;

    case NTP_QUERY_STATE_COMPLETE:
      return NTP_QUERY_RESULT_OK;

    case NTP_QUERY_STATE_CLOSED:
      return NTP_QUERY_RESULT_ERROR_CLOSED;
  }

  return NTP_QUERY_RESULT_ERROR_INVALID_STATE;
}

void ntp_query_close(tNtpQuery* query) {
  if (query->socket != -1) {
    closesocket(query->socket);
    query->socket = -1;
  }
  query->state = NTP_QUERY_STATE_CLOSED;
}
