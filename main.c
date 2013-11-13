
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

void setup();
void loop();
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

int main(void) {
  setup();
  while (1) {
    loop();
  }
  return 0;
}

void setup() {
  GPIO_InitTypeDef gpioConfig;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  gpioConfig.GPIO_Pin = GPIO_Pin_0;
  gpioConfig.GPIO_Mode = GPIO_Mode_Out_PP;
  gpioConfig.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpioConfig);
}

void loop() {
  GPIO_SetBits(GPIOA, GPIO_Pin_0);
  delay_ms(500);
  GPIO_ResetBits(GPIOA, GPIO_Pin_0);
  delay_ms(500);
}

void delay_ms(uint32_t ms) {
  volatile uint32_t i;
  for(i = ms; i != 0; i--) {
    delay_us(1000);
  }
}

void delay_us(uint32_t us) {
  volatile uint32_t i;
  for(i = (5 * us); i != 0; i--) {}
}
