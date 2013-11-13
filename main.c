
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include "stm32_cc3000.h"
#include "debug.h"

void setup();
void loop();

int main(void) {
  setup();
  while (1) {
    loop();
  }
  return 0;
}

void setup() {
  debug_write_line("cc3000_setup");
  cc3000_setup();

  uint16_t firmwareVersion = cc3000_get_firmware_version();
  debug_write_u16(firmwareVersion, 16);
  debug_write_line("");
}

void loop() {
}

void debug_on_rx(uint8_t* data, uint16_t len) {
  
}
