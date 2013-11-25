#ifndef NTP_H
#define	NTP_H

#ifdef	__cplusplus
extern "C" {
#endif

  typedef enum {
    NTP_QUERY_STATE_START,
    NTP_QUERY_STATE_GET_HOST_BY_NAME,
    NTP_QUERY_STATE_CONNECT_UDP,
    NTP_QUERY_STATE_REQUEST_TIME,
    NTP_QUERY_STATE_RECV,
    NTP_QUERY_STATE_COMPLETE,
    NTP_QUERY_STATE_CLOSED
  } tNtpQueryState;

  typedef enum {
    NTP_QUERY_RESULT_OK = 0,
    NTP_QUERY_RESULT_PENDING = 1,
    NTP_QUERY_RESULT_ERROR_GET_HOST_BY_NAME = -1,
    NTP_QUERY_RESULT_ERROR_CONNECT = -2,
    NTP_QUERY_RESULT_ERROR_SEND = -3,
    NTP_QUERY_RESULT_ERROR_TIMEOUT = -4,
    NTP_QUERY_RESULT_ERROR_RECV = -5,
    NTP_QUERY_RESULT_ERROR_BAD_SOCKET = -6,
    NTP_QUERY_RESULT_ERROR_INVALID_STATE = -7,
    NTP_QUERY_RESULT_ERROR_CLOSED = -8
  } tNtpQueryResult;

  typedef struct {
    tNtpQueryState state;
    char ntpServer[100];
    uint32_t ntpServerIp;
    uint32_t timeout;
    uint32_t time;
    int16_t socket;
    uint32_t startTime;
  } tNtpQuery;

  void ntp_query_init(tNtpQuery* query, const char* ntpServer, uint32_t timeout);

  /**
   * @brief Asynchronously queries an NTP server for the time.
   * @return NTP_QUERY_RESULT_OK, ok.
   *         NTP_QUERY_RESULT_ERROR_GET_HOST_BY_NAME, gethostbyname failed.
   *         NTP_QUERY_RESULT_ERROR_CONNECT, connect_udp failed.
   *         NTP_QUERY_RESULT_ERROR_SEND, send failed.
   *         NTP_QUERY_RESULT_ERROR_TIMEOUT, timeout waiting for time.
   *         NTP_QUERY_RESULT_ERROR_RECV, recv failed.
   *         NTP_QUERY_RESULT_ERROR_BAD_SOCKET, bad socket.
   * 
   * @dot
   * digraph example {
   *   start -> gethostbyname;
   *   gethostbyname -> connect_udp;
   *   connect_udp -> request_time;
   *   request_time -> gethostbyname [ label="timeout" ];
   *   request_time -> recv;
   *   recv-> gethostbyname [ label="timeout" ]
   *   recv -> complete
   * }
   * @enddot
   */
  tNtpQueryResult ntp_query_loop(tNtpQuery* query);

  void ntp_query_close(tNtpQuery* query);

#ifdef	__cplusplus
}
#endif

#endif	/* NTP_H */

