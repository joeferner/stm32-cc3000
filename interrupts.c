
#include <stm32f10x_exti.h>
#include <stm32f10x_usart.h>
#include "time.h"
#include "stm32_cc3000.h"
#include "platform_config.h"
#include "debug.h"

void NMI_Handler(void) {
}

void HardFault_Handler(void) {
}

void MemManage_Handler(void) {
}

void BusFault_Handler(void) {
}

void UsageFault_Handler(void) {
}

void SVC_Handler(void) {
}

void DebugMon_Handler(void) {
}

void PendSV_Handler(void) {
}

void SysTick_Handler(void) {
  time_SysTick_Handler();
}

void EXTI1_IRQHandler(void) {
  if (EXTI_GetITStatus(CC3000_IRQ_EXTI_LINE) != RESET) {
    cc3000_irq();
    EXTI_ClearITPendingBit(CC3000_IRQ_EXTI_LINE);
  }
}

void USART1_IRQHandler(void) {
  if (USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET) {
    uint8_t b[1];
    b[0] = USART_ReceiveData(DEBUG_USART);
    debug_on_rx(b, 1);
  }
}