
#include <string.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_rtc.h>
#include <stm32f10x_bkp.h>
#include <stm32f10x_pwr.h>
#include <misc.h>
#include "cc3000-host-driver/socket.h"
#include "cc3000-host-driver/netapp.h"
#include "stm32_cc3000.h"
#include "debug.h"
#include "platform_config.h"
#include "delay.h"
#include "connection_info.h"
#include "time.h"
#include "ntp.h"
#include "ring_buffer.h"
#include "util.h"

#define NTP_QUERY_FREQUENCY (10 * 60 * 1000)

#define BACKUP_CONFIG_VALUE 0xA5A5

char cc3000_device_name[] = "CC3000";

void setup();
void loop();
void assert_failed(uint8_t* file, uint32_t line);
void nvic_setup();
void rtc_setup();
void debug_write_rtc();
void debug_on_line(const char* line);

#define INPUT_BUFFER_SIZE 100
uint8_t input_buffer[INPUT_BUFFER_SIZE];
ring_buffer input_ring_buffer;

int countdown = 0;
volatile uint32_t lastNtpQuery;
volatile uint32_t lastWriteTime;
tNtpQuery ntpQuery;
uint32_t timezoneOffset = 5 * 60 * 60;

int main(void) {
  setup();
  while (1) {
    loop();
  }
  return 0;
}

void setup() {
  uint8_t cc3000MajorFirmwareVersion, cc3000MinorFirmwareVersion;

  ring_buffer_init(&input_ring_buffer, input_buffer, INPUT_BUFFER_SIZE);

  nvic_setup();
  time_setup();
  debug_setup();
  rtc_setup();

  RCC_ClearFlag(); // Clear reset flags

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

  while (!cc3000_is_connected()) {
    delay_ms(100);
  }

  cc3000_display_ipconfig();

  ntp_query_init(&ntpQuery, NTP_SERVER, 10000);
  lastNtpQuery = time_ms();
  lastWriteTime = 0;
}

void loop() {
  if ((time_ms() - lastWriteTime) > 10000) {
    debug_write_rtc();
    lastWriteTime = time_ms();
  }

  if ((time_ms() - lastNtpQuery) > NTP_QUERY_FREQUENCY) {
    debug_write_line("begin NTP query");
    ntp_query_close(&ntpQuery);
    ntp_query_init(&ntpQuery, NTP_SERVER, 10000);
    lastNtpQuery = time_ms();
  }

  if (ntp_query_loop(&ntpQuery) == NTP_QUERY_RESULT_OK) {
    ntp_query_close(&ntpQuery);

    debug_write("Current UNIX time: ");
    debug_write_u32(ntpQuery.time, 10);
    debug_write_line(" (seconds since 1/1/1970 UTC)");

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    RTC_WaitForLastTask();
    RTC_SetCounter(ntpQuery.time);
    RTC_WaitForLastTask();
  }
}

void debug_on_rx(uint8_t* data, uint16_t len) {
#define MAX_LINE_LENGTH 100
  char line[MAX_LINE_LENGTH];

  debug_write_bytes(data, len);

  ring_buffer_write(&input_ring_buffer, data, len);
  while (ring_buffer_readline(&input_ring_buffer, line, MAX_LINE_LENGTH) > 0) {
    trim_right(line);
    debug_on_line(line);
  }
}

void debug_on_line(const char* line) {
  if (!strcmp(line, "ntp")) {
    lastNtpQuery = time_ms() - NTP_QUERY_FREQUENCY - 1;
    debug_write_line("+OK");
  } else if (!strcmp(line, "time")) {
    debug_write_line("+OK");
    debug_write_rtc();
  } else {
    debug_write("-Invalid command: ");
    debug_write_line(line);
  }
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

void nvic_setup() {
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  /* Enable the RTC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void rtc_setup() {
  if (BKP_ReadBackupRegister(BKP_DR1) != BACKUP_CONFIG_VALUE) {
    // Backup data register value is not correct or not yet programmed
    // (when the first time the program is executed)

    debug_write_line("RTC not yet configured....");

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    BKP_DeInit(); // Reset Backup Domain

    RCC_LSEConfig(RCC_LSE_ON);
    debug_write_line("Waiting for LSE");
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
    debug_write_line("LSE Ready");
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForSynchro();
    RTC_WaitForLastTask();
    RTC_ITConfig(RTC_IT_SEC, ENABLE); // Enable the RTC Second Interrupt
    RTC_WaitForLastTask();

    // Set RTC pre-scaler: set RTC period to 1sec
    RTC_SetPrescaler(32767); // RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1)

    RTC_WaitForLastTask();

    debug_write_line("RTC configured.... setting initial time");

    RTC_WaitForLastTask();
    RTC_SetCounter(1385436853);
    RTC_WaitForLastTask();

    BKP_WriteBackupRegister(BKP_DR1, BACKUP_CONFIG_VALUE);
  } else {
    if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET) {
      debug_write_line("Power On Reset occurred....");
    } else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET) {
      debug_write_line("External Reset occurred....");
    }

    RTC_WaitForSynchro();
    RTC_ITConfig(RTC_IT_SEC, ENABLE);
    RTC_WaitForLastTask();
  }
}

void debug_write_rtc() {
  debug_write_line("debug_write_rtc");
  time_t rtc = (time_t) (RTC_GetCounter() - timezoneOffset);
  struct tm* t = gmtime(&rtc);

  debug_write("rtc date: ");
  debug_write_i32(t->tm_year + 1900, 10);
  debug_write("/");
  debug_write_i32(t->tm_mon + 1, 10);
  debug_write("/");
  debug_write_i32(t->tm_mday, 10);
  debug_write_line("");

  debug_write("rtc time: ");
  debug_write_i32(t->tm_hour, 10);
  debug_write(":");
  debug_write_i32(t->tm_min, 10);
  debug_write(":");
  debug_write_i32(t->tm_sec, 10);
  debug_write_line("");
}
