
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_exti.h>
#include "stm32_cc3000.h"
#include "debug.h"
#include "platform_config.h"

void setup();
void loop();
void assert_failed(uint8_t* file, uint32_t line);
void display_mac_address();

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
}

void loop() {
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
