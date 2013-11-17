
#include <string.h>

#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_misc.h>

#include "cc3000-host-driver/wlan.h"
#include "cc3000-host-driver/hci.h"
#include "cc3000-host-driver/socket.h"
#include "cc3000-host-driver/nvmem.h"
#include "cc3000-host-driver/netapp.h"

#include "stm32_cc3000.h"
#include "debug.h"
#include "delay.h"
#include "platform_config.h"

#define WLAN_CONNECT_TIMEOUT_MS 10000
#define MAX_SOCKETS             32
char cc3000_device_name[] = "CC3000";

// NOTE: Required by cc3000 Host driver
unsigned char wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];

// The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
// for the purpose of detection of the overrun. The location of the memory where the magic number
// resides shall never be written. In case it is written - the overrun occured and either recevie function
// or send function will stuck forever.
#define CC3000_BUFFER_MAGIC_NUMBER (0xDE)
#define HEADERS_SIZE_EVNT          (SPI_HEADER_SIZE + 5)
#define READ                       (3)
#define WRITE                      (1)
#define HI(value)                  (((value) & 0xFF00) >> 8)
#define LO(value)                  ((value) & 0x00FF)

typedef enum {
  SPI_STATE_POWERUP,
  SPI_STATE_INITIALIZED,
  SPI_STATE_IDLE,
  SPI_STATE_WRITE_IRQ,
  SPI_STATE_WRITE_FIRST_PORTION,
  SPI_STATE_WRITE_EOT,
  SPI_STATE_READ_IRQ,
  SPI_STATE_READ_FIRST_PORTION,
  SPI_STATE_READ_EOT
} spi_state_t;

typedef struct {
  cc3000_spi_rx_handler_t spi_rx_handler;
  unsigned short tx_packet_length;
  unsigned short rx_packet_length;
  spi_state_t state;
  unsigned char *tx_packet;
  unsigned char *rx_packet;
} spi_information_t;

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
spi_information_t cc3000_spi_info;
uint8_t cc3000_spi_buffer[CC3000_RX_BUFFER_SIZE];

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
void cc3000_spi_read_header();
void cc3000_spi_cont_read();
int cc3000_spi_read_data_cont();
int cc3000_spi_first_write(unsigned char *ucBuf, unsigned short usLength);
void cc3000_spi_write_data_sync(unsigned char *data, unsigned short size);
void cc3000_spi_read_data_sync(unsigned char *data, unsigned short size);
void cc3000_spi_trigger_rx_processing();
uint8_t cc3000_spi_transfer(uint8_t d);

int cc3000_setup(int patchesAvailableAtHost, int useSmartConfigData) {
  debug_write_line("cc3000_setup");
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
  cc3000_wlan_irq_enable();

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
  debug_write_line("wlan_ioctl");
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

  debug_write_line("END cc3000_setup");
  return 0;
}

void cc3000_setup_vbat_en() {
  debug_write_line("cc3000_setup_vbat_en");
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
  debug_write_line("cc3000_setup_irq");
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
}

void cc3000_setup_spi() {
  debug_write_line("cc3000_setup_spi");
  GPIO_InitTypeDef gpioConfig;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

  cc3000_spi_deassert(); // set pin high before initializing as output pin to not false trigger CS
  RCC_APB2PeriphClockCmd(CC3000_CS_RCC, ENABLE);
  gpioConfig.GPIO_Pin = CC3000_CS_PIN;
  gpioConfig.GPIO_Mode = GPIO_Mode_Out_PP;
  gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CC3000_CS_PORT, &gpioConfig);
  cc3000_spi_deassert();

  // Configure SPI1 pins: SCK (pin 5) and MOSI (pin 7)
  gpioConfig.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
  gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
  gpioConfig.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &gpioConfig);

  // Configure SPI1 pins: MISO (pin 6)
  gpioConfig.GPIO_Pin = GPIO_Pin_6;
  gpioConfig.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpioConfig);

  // TODO: calling this method causes a fault
  //RCC_PCLK2Config(RCC_HCLK_Div2);

  SPI_InitTypeDef spiConfig;
  SPI_StructInit(&spiConfig);
  spiConfig.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spiConfig.SPI_Mode = SPI_Mode_Master;
  spiConfig.SPI_DataSize = SPI_DataSize_8b;
  spiConfig.SPI_NSS = SPI_NSS_Soft;
  spiConfig.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // TODO: change to speed up
  spiConfig.SPI_FirstBit = SPI_FirstBit_MSB;

  // Mode 1 (CPOL = 0, CPHA = 1)
  spiConfig.SPI_CPOL = SPI_CPOL_Low;
  spiConfig.SPI_CPHA = SPI_CPHA_2Edge;

  SPI_Init(SPI1, &spiConfig);

  SPI_Cmd(SPI1, ENABLE);
}

int cc3000_get_firmware_version(uint8_t *major, uint8_t *minor) {
  uint8_t fwpReturn[2];

  debug_write_line("cc3000_get_firmware_version");
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
  return GPIO_ReadInputDataBit(CC3000_IRQ_PORT, CC3000_IRQ_PIN) == Bit_SET ? 1 : 0;
}

void cc3000_wlan_irq_enable() {
  //debug_write_line("cc3000_wlan_irq_enable");

  cc3000_spi_irq_enabled = 1;

  NVIC_InitTypeDef nvicConfig;
  nvicConfig.NVIC_IRQChannel = CC3000_IRQ_EXTI_CH;
  nvicConfig.NVIC_IRQChannelPreemptionPriority = 0x0F;
  nvicConfig.NVIC_IRQChannelSubPriority = 0x0F;
  nvicConfig.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicConfig);
}

void cc3000_wlan_irq_disable() {
  // debug_write_line("cc3000_wlan_irq_disable");

  cc3000_spi_irq_enabled = 0;

  NVIC_InitTypeDef nvicConfig;
  nvicConfig.NVIC_IRQChannel = CC3000_IRQ_EXTI_CH;
  nvicConfig.NVIC_IRQChannelPreemptionPriority = 0x0F;
  nvicConfig.NVIC_IRQChannelSubPriority = 0x0F;
  nvicConfig.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&nvicConfig);
}

void cc3000_write_wlan_pin(unsigned char val) {
  if (val) {
    GPIO_SetBits(CC3000_VBAT_EN_PORT, CC3000_VBAT_EN_PIN);
  } else {
    GPIO_ResetBits(CC3000_VBAT_EN_PORT, CC3000_VBAT_EN_PIN);
  }
}

void cc3000_irq_poll() {
  if (cc3000_read_wlan_irq() == 0 && cc3000_spi_is_in_irq == 0 && cc3000_spi_irq_enabled != 0) {
    cc3000_irq();
  }
}

void cc3000_spi_read_header() {
  cc3000_spi_read_data_sync(cc3000_spi_info.rx_packet, 10);
}

void cc3000_spi_read_data_sync(unsigned char *data, unsigned short size) {
  int i = 0;
  for (i = 0; i < size; i++) {
    data[i] = cc3000_spi_transfer(0x03);
  }

  //  debug_write("spi read: ");
  //  debug_write_u8_array(data, size);
  //  debug_write_line("");
}

void cc3000_spi_cont_read() {
  // The header was read - continue with the payload read
  if (!cc3000_spi_read_data_cont()) {
    // All the data was read - finalize handling by switching to the task and calling from task Event Handler
    cc3000_spi_trigger_rx_processing();
  }
}

void cc3000_spi_trigger_rx_processing() {
  // Trigger Rx processing
  cc3000_wlan_irq_disable();
  cc3000_spi_deassert();

  // The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
  // for the purpose of detection of the overrun. If the magic number is overriten - buffer overrun
  // occurred - and we will stuck here forever!
  if (cc3000_spi_info.rx_packet[CC3000_RX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER) {
    // You've got problems if you're here!
    debug_write_line("CC3000: ERROR - magic number missing!");
    while (1);
  }

  cc3000_spi_info.state = SPI_STATE_IDLE;
  cc3000_spi_info.spi_rx_handler(cc3000_spi_info.rx_packet + SPI_HEADER_SIZE);
}

int cc3000_spi_read_data_cont() {
  int data_to_recv;
  unsigned char *evnt_buff, type;

  // debug_write_line("cc3000_spi_read_data_cont");

  // Determine what type of packet we have
  evnt_buff = cc3000_spi_info.rx_packet;
  data_to_recv = 0;
  STREAM_TO_UINT8((uint8_t *) (evnt_buff + SPI_HEADER_SIZE), HCI_PACKET_TYPE_OFFSET, type);

  switch (type) {
    case HCI_TYPE_DATA: // 0x02
      //      debug_write_line("HCI_TYPE_DATA");

      // We need to read the rest of data..
      STREAM_TO_UINT16((char *) (evnt_buff + SPI_HEADER_SIZE), HCI_DATA_LENGTH_OFFSET, data_to_recv);
      if (!((HEADERS_SIZE_EVNT + data_to_recv) & 1)) {
        data_to_recv++;
      }

      if (data_to_recv) {
        cc3000_spi_read_data_sync(evnt_buff + 10, data_to_recv);
      }
      break;

    case HCI_TYPE_EVNT: // 0x04
      //      debug_write_line("HCI_TYPE_EVNT");

      // Calculate the rest length of the data
      STREAM_TO_UINT8((char *) (evnt_buff + SPI_HEADER_SIZE), HCI_EVENT_LENGTH_OFFSET, data_to_recv);
      data_to_recv -= 1;

      // Add padding byte if needed
      if ((HEADERS_SIZE_EVNT + data_to_recv) & 1) {
        data_to_recv++;
      }

      if (data_to_recv) {
        cc3000_spi_read_data_sync(evnt_buff + 10, data_to_recv);
      }

      cc3000_spi_info.state = SPI_STATE_READ_EOT;
      break;

    default:
      debug_write("unknown type: ");
      debug_write_u8(type, 16);
      debug_write_line("");
      break;
  }

  return 0;
}

void cc3000_spi_write_data_sync(unsigned char *data, unsigned short size) {
  uint8_t loc;

  //  debug_write("spi write: ");
  //  debug_write_u8_array(data, size);
  //  debug_write_line("");

  for (loc = 0; loc < size; loc++) {
    cc3000_spi_transfer(data[loc]);
  }
}

void cc3000_irq() {
  //  debug_write_line("cc3000_irq");
  //  debug_led_set(1);
  cc3000_spi_is_in_irq = 1;

  if (cc3000_spi_info.state == SPI_STATE_POWERUP) {
    // IRQ line was low ... perform a callback on the HCI Layer
    cc3000_spi_info.state = SPI_STATE_INITIALIZED;
  } else if (cc3000_spi_info.state == SPI_STATE_IDLE) {
    cc3000_spi_info.state = SPI_STATE_READ_IRQ;

    // IRQ line goes down - start reception
    cc3000_spi_assert();

    // Wait for TX/RX Compete which will come as DMA interrupt
    cc3000_spi_read_header();
    cc3000_spi_info.state = SPI_STATE_READ_EOT;
    cc3000_spi_cont_read();
  } else if (cc3000_spi_info.state == SPI_STATE_WRITE_IRQ) {
    cc3000_spi_write_data_sync(cc3000_spi_info.tx_packet, cc3000_spi_info.tx_packet_length);
    cc3000_spi_info.state = SPI_STATE_IDLE;

    cc3000_spi_deassert();
  }

  cc3000_spi_is_in_irq = 0;
  //  debug_led_set(0);
}

void cc3000_spi_assert() {
  GPIO_ResetBits(CC3000_CS_PORT, CC3000_CS_PIN);
}

void cc3000_spi_deassert() {
  GPIO_SetBits(CC3000_CS_PORT, CC3000_CS_PIN);
}

uint8_t cc3000_spi_transfer(uint8_t d) {
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, d);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(SPI1);
}

/**
 * NOTE: Required by cc3000 Host driver
 */
void SpiOpen(cc3000_spi_rx_handler_t rx_handler) {
  debug_write_line("SpiOpen");

  cc3000_spi_info.state = SPI_STATE_POWERUP;

  memset(cc3000_spi_buffer, 0, sizeof (cc3000_spi_buffer));
  memset(wlan_tx_buffer, 0, sizeof (wlan_tx_buffer));

  cc3000_spi_info.spi_rx_handler = rx_handler;
  cc3000_spi_info.tx_packet_length = 0;
  cc3000_spi_info.tx_packet = NULL;
  cc3000_spi_info.rx_packet = cc3000_spi_buffer;
  cc3000_spi_info.rx_packet_length = 0;

  cc3000_spi_buffer[CC3000_RX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;
  wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;

  // Enable interrupt on the GPIO pin of WLAN IRQ
  tSLInformation.WlanInterruptEnable();

  debug_write_line("END SpiOpen");
}

/**
 * NOTE: Required by cc3000 Host driver
 */
void SpiResumeSpi(void) {
  cc3000_wlan_irq_enable();
}

/**
 * NOTE: Required by cc3000 Host driver
 */
long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength) {
  unsigned char ucPad = 0;

  // Figure out the total length of the packet in order to figure out if there is padding or not
  if (!(usLength & 0x0001)) {
    ucPad++;
  }

  pUserBuffer[0] = WRITE;
  pUserBuffer[1] = HI(usLength + ucPad);
  pUserBuffer[2] = LO(usLength + ucPad);
  pUserBuffer[3] = 0;
  pUserBuffer[4] = 0;

  usLength += (SPI_HEADER_SIZE + ucPad);

  // The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
  // for the purpose of overrun detection. If the magic number is overwritten - buffer overrun
  // occurred - and we will be stuck here forever! 
  if (wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER) {
    debug_write_line("CC3000: Error - No magic number found in SpiWrite");
    while (1);
  }

  if (cc3000_spi_info.state == SPI_STATE_POWERUP) {
    while (cc3000_spi_info.state != SPI_STATE_INITIALIZED);
  }

  if (cc3000_spi_info.state == SPI_STATE_INITIALIZED) {
    // This is time for first TX/RX transactions over SPI: the IRQ is down - so need to send read buffer size command
    cc3000_spi_first_write(pUserBuffer, usLength);
  } else {
    // We need to prevent here race that can occur in case two back to back packets are sent to the
    // device, so the state will move to IDLE and once again to not IDLE due to IRQ
    tSLInformation.WlanInterruptDisable();

    while (cc3000_spi_info.state != SPI_STATE_IDLE);

    cc3000_spi_info.state = SPI_STATE_WRITE_IRQ;
    cc3000_spi_info.tx_packet = pUserBuffer;
    cc3000_spi_info.tx_packet_length = usLength;

    // Assert the CS line and wait till SSI IRQ line is active and then initialize write operation
    cc3000_spi_assert();

    // Re-enable IRQ - if it was not disabled - this is not a problem...
    tSLInformation.WlanInterruptEnable();

    // Check for a missing interrupt between the CS assertion and enabling back the interrupts
    if (tSLInformation.ReadWlanInterruptPin() == 0) {
      cc3000_spi_write_data_sync(cc3000_spi_info.tx_packet, cc3000_spi_info.tx_packet_length);

      cc3000_spi_info.state = SPI_STATE_IDLE;

      cc3000_spi_deassert();
    }
  }

  // Due to the fact that we are currently implementing a blocking situation
  // here we will wait till end of transaction */
  while (SPI_STATE_IDLE != cc3000_spi_info.state);

  return (0);
}

int cc3000_spi_first_write(unsigned char *ucBuf, unsigned short usLength) {
  // Workaround for the first transaction
  cc3000_spi_assert();

  // delay (stay low) for ~50us
  delay_ms(1); // TODO: 1ms really?

  // SPI writes first 4 bytes of data
  cc3000_spi_write_data_sync(ucBuf, 4);

  delay_ms(1);

  cc3000_spi_write_data_sync(ucBuf + 4, usLength - 4);

  /* From this point on - operate in a regular manner */
  cc3000_spi_info.state = SPI_STATE_IDLE;

  cc3000_spi_deassert();

  return (0);
}