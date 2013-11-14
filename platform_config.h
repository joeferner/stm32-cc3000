
#ifndef PLATFORM_CONFIG_H
#define	PLATFORM_CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

#define DEBUG_USART            USART1
#define DEBUG_USART_BAUD       9600
#define DEBUG_USART_IRQ        USART1_IRQn
#define DEBUG_USART_RCC        RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1
#define DEBUG_USART_TX         GPIOA
#define DEBUG_USART_TX_PIN     GPIO_Pin_9
#define DEBUG_USART_RX         GPIOA
#define DEBUG_USART_RX_PIN     GPIO_Pin_10

#define CC3000_IRQ_RCC         RCC_APB2Periph_GPIOA
#define CC3000_IRQ_PORT        GPIOA
#define CC3000_IRQ_PIN         GPIO_Pin_1
#define CC3000_IRQ_EXTI_LINE   EXTI_Line1
#define CC3000_IRQ_EXTI_PORT   GPIO_PortSourceGPIOA
#define CC3000_IRQ_EXTI_PIN    GPIO_PinSource1
#define CC3000_IRQ_EXTI_CH     EXTI1_IRQn

#define CC3000_VBAT_EN_RCC     RCC_APB2Periph_GPIOA
#define CC3000_VBAT_EN_PORT    GPIOA
#define CC3000_VBAT_EN_PIN     GPIO_Pin_2

#define CC3000_CS_RCC          RCC_APB2Periph_GPIOA
#define CC3000_CS_PORT         GPIOA
#define CC3000_CS_PIN          GPIO_Pin_3

#ifdef	__cplusplus
}
#endif

#endif	/* PLATFORM_CONFIG_H */

