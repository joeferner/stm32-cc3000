
#include <string.h>

#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_misc.h>

#include "cc3000-host-driver/wlan.h"
#include "cc3000-host-driver/hci.h"
#include "cc3000-host-driver/socket.h"
#include "cc3000-host-driver/nvmem.h"

#include "stm32_cc3000.h"
#include "debug.h"
#include "delay.h"
#include "platform_config.h"

#define WLAN_CONNECT_TIMEOUT_MS 10000
#define MAX_SOCKETS             32
char cc3000_device_name[] = "CC3000";

volatile int cc3000_connected;
volatile int cc3000_dhcp;
volatile int cc3000_dhcp_configured;
volatile int cc3000_smart_config_finished;
volatile int cc3000_stop_smart_config;
volatile int cc3000_ok_to_shutdown;
volatile int cc3000_ping_report_count;
volatile int cc3000_spi_irq_enabled;
volatile int cc3000_spi_is_in_irq;
volatile netapp_pingreport_args_t cc3000_ping_report;
uint8_t cc3000_closed_sockets[MAX_SOCKETS];

void cc3000_setup_vbat_en();
void cc3000_setup_spi();
void cc3000_setup_irq();

void cc3000_async_callback(long eventType, char* data, unsigned char length);
char* cc3000_send_FW_patch(unsigned long *length);
char* cc3000_send_driver_patch(unsigned long *length);
char* cc3000_send_bootloader_patch(unsigned long *length);
long cc3000_read_wlan_irq();
void cc3000_wlan_irq_enable();
void cc3000_wlan_irq_disable();
void cc3000_write_wlan_pin(unsigned char val);
void cc3000_irq_poll();
void cc3000_spi_assert();
void cc3000_spi_deassert();

int cc3000_setup(int patchesAvailableAtHost, int useSmartConfigData) {
  cc3000_connected = 0;
  cc3000_dhcp = 0;
  cc3000_dhcp_configured = 0;
  cc3000_smart_config_finished = 0;
  cc3000_stop_smart_config = 0;
  cc3000_ok_to_shutdown = 0;
  cc3000_ping_report_count = 0;
  cc3000_spi_irq_enabled = 0;
  cc3000_spi_is_in_irq = 0;
  for (int i = 0; i < MAX_SOCKETS; i++) {
    cc3000_closed_sockets[i] = 0;
  }

  cc3000_setup_vbat_en();
  cc3000_setup_irq();
  cc3000_setup_spi();

  debug_write_line("wlan_init");
  wlan_init(
          cc3000_async_callback,
          cc3000_send_FW_patch,
          cc3000_send_driver_patch,
          cc3000_send_bootloader_patch,
          cc3000_read_wlan_irq,
          cc3000_wlan_irq_enable,
          cc3000_wlan_irq_disable,
          cc3000_write_wlan_pin);

  debug_write_line("wlan_start");
  wlan_start(patchesAvailableAtHost);

  // Check if we should erase previous stored connection details
  // (most likely written with data from the SmartConfig app)
  debug_write_line("wlan_ioctl\n\r");
  if (useSmartConfigData) {
    // Auto Connect - the C3000 device tries to connect to any AP it detects during scanning:
    // wlan_ioctl_set_connection_policy(1, 0, 0)

    // Fast Connect - the CC3000 device tries to reconnect to the last AP connected to:
    wlan_ioctl_set_connection_policy(0, 1, 0);

    // Use Profiles - the CC3000 device tries to connect to an AP from profiles:
    // wlan_ioctl_set_connection_policy(0, 0, 1);
  } else {
    // Manual connection only (no auto, profiles, etc.)
    wlan_ioctl_set_connection_policy(0, 0, 0);
    // Delete previous profiles from memory
    wlan_ioctl_del_profile(255);
  }

  if (wlan_set_event_mask(
          HCI_EVNT_WLAN_UNSOL_INIT |
          //HCI_EVNT_WLAN_ASYNC_PING_REPORT |// we want ping reports
          //HCI_EVNT_BSD_TCP_CLOSE_WAIT |
          //HCI_EVNT_WLAN_TX_COMPLETE |
          HCI_EVNT_WLAN_KEEPALIVE) != 0) {
    debug_write_line("WLAN Set Event Mask FAIL");
    return 1;
  }

  // Wait for re-connection is we're using SmartConfig data
  if (useSmartConfigData) {
    // Wait for a connection
    uint32_t timeout = 0;
    while (!cc3000_connected) {
      cc3000_irq_poll();
      if (timeout > WLAN_CONNECT_TIMEOUT_MS) {
        debug_write_line("Timed out using SmartConfig data");
      }
      return 2;
    }
    timeout += 10;
    delay_ms(10);
  }

  delay_ms(1000);
  if (cc3000_dhcp) {
    mdnsAdvertiser(1, (char *) cc3000_device_name, strlen(cc3000_device_name));
  }

  return 0;
}

void cc3000_setup_vbat_en() {
  GPIO_InitTypeDef gpioConfig;

  RCC_APB2PeriphClockCmd(CC3000_VBAT_EN_RCC, ENABLE);
  gpioConfig.GPIO_Pin = CC3000_VBAT_EN_PIN;
  gpioConfig.GPIO_Mode = GPIO_Mode_Out_PP;
  gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CC3000_VBAT_EN_PORT, &gpioConfig);

  GPIO_ResetBits(CC3000_VBAT_EN_PORT, CC3000_VBAT_EN_PIN);

  delay_ms(500);
}

void cc3000_setup_irq() {
  RCC_APB2PeriphClockCmd(CC3000_IRQ_RCC, ENABLE);

  GPIO_InitTypeDef gpioConfig;
  gpioConfig.GPIO_Pin = CC3000_IRQ_PIN;
  gpioConfig.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CC3000_IRQ_PORT, &gpioConfig);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  // Connect EXTI Line
  GPIO_EXTILineConfig(CC3000_IRQ_EXTI_PORT, CC3000_IRQ_EXTI_PIN);

  // Configure EXTI line
  EXTI_InitTypeDef extiConfig;
  extiConfig.EXTI_Line = CC3000_IRQ_EXTI_LINE;
  extiConfig.EXTI_Mode = EXTI_Mode_Interrupt;
  extiConfig.EXTI_Trigger = EXTI_Trigger_Falling;
  extiConfig.EXTI_LineCmd = ENABLE;
  EXTI_Init(&extiConfig);

  // Enable and set EXTI Interrupt to the lowest priority
  NVIC_InitTypeDef nvicConfig;
  nvicConfig.NVIC_IRQChannel = CC3000_IRQ_EXTI_CH;
  nvicConfig.NVIC_IRQChannelPreemptionPriority = 0x0F;
  nvicConfig.NVIC_IRQChannelSubPriority = 0x0F;
  nvicConfig.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicConfig);
}

void cc3000_setup_spi() {
  GPIO_InitTypeDef gpioConfig;

  RCC_APB2PeriphClockCmd(CC3000_CS_RCC, ENABLE);
  gpioConfig.GPIO_Pin = CC3000_CS_PIN;
  gpioConfig.GPIO_Mode = GPIO_Mode_Out_PP;
  gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CC3000_CS_PORT, &gpioConfig);
  cc3000_spi_deassert();

  SPI.begin();
  SPI.setDataMode(SPI_MODE1);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(g_SPIspeed);
}

int cc3000_get_firmware_version(uint8_t *major, uint8_t *minor) {
  uint8_t fwpReturn[2];

  if (nvmem_read_sp_version(fwpReturn) != 0) {
    debug_write_line("Unable to read the firmware version");
    return 1;
  }

  *major = fwpReturn[0];
  *minor = fwpReturn[1];

  return 0;
}

void cc3000_async_callback(long eventType, char* data, unsigned char length) {
  if (eventType == HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE) {
    cc3000_smart_config_finished = 1;
    cc3000_stop_smart_config = 1;
  }

  if (eventType == HCI_EVNT_WLAN_UNSOL_CONNECT) {
    cc3000_connected = 1;
  }

  if (eventType == HCI_EVNT_WLAN_UNSOL_DISCONNECT) {
    cc3000_connected = 0;
    cc3000_dhcp = 0;
    cc3000_dhcp_configured = 0;
  }

  if (eventType == HCI_EVNT_WLAN_UNSOL_DHCP) {
    cc3000_dhcp = 1;
  }

  if (eventType == HCI_EVENT_CC3000_CAN_SHUT_DOWN) {
    cc3000_ok_to_shutdown = 1;
  }

  if (eventType == HCI_EVNT_WLAN_ASYNC_PING_REPORT) {
    //PRINT_F("CC3000: Ping report\n\r");
    cc3000_ping_report_count++;
    memcpy((uint8_t*) & cc3000_ping_report, data, length);
  }

  if (eventType == HCI_EVNT_BSD_TCP_CLOSE_WAIT) {
    uint8_t socketnum;
    socketnum = data[0];
    if (socketnum < MAX_SOCKETS) {
      cc3000_closed_sockets[socketnum] = 1;
    }
  }
}

char* cc3000_send_FW_patch(unsigned long *length) {
  *length = 0;
  return NULL;
}

char* cc3000_send_driver_patch(unsigned long *length) {
  *length = 0;
  return NULL;
}

char* cc3000_send_bootloader_patch(unsigned long *length) {
  *length = 0;
  return NULL;
}

long cc3000_read_wlan_irq() {
  return (digitalRead(g_irqPin));
}

void cc3000_wlan_irq_enable() {
  debug_write_line("cc3000_wlan_irq_enable");
  // delay_ms(100);
  cc3000_spi_irq_enabled = 1;
  attachInterrupt(g_IRQnum, SPI_IRQ, FALLING);
}

void cc3000_wlan_irq_disable() {
  debug_write_line("cc3000_wlan_irq_disable");
  // delay_ms(100);
  cc3000_spi_irq_enabled = 0;
  detachInterrupt(g_IRQnum);
}

void cc3000_write_wlan_pin(unsigned char val) {
  if (val) {
    GPIO_SetBits(CC3000_VBAT_EN_PORT, CC3000_VBAT_EN_PIN);
  } else {
    GPIO_ResetBits(CC3000_VBAT_EN_PORT, CC3000_VBAT_EN_PIN);
  }
}

void cc3000_irq_poll() {
  if (digitalRead(g_irqPin) == LOW && cc3000_spi_is_in_irq == 0 && cc3000_spi_irq_enabled != 0) {
    cc3000_irq();
  }
}

void cc3000_irq() {
  zzz;
}

void cc3000_spi_assert() {
  GPIO_ResetBits(CC3000_CS_PORT, CC3000_CS_PIN);
}

void cc3000_spi_deassert() {
  GPIO_SetBits(CC3000_CS_PORT, CC3000_CS_PIN);
}
