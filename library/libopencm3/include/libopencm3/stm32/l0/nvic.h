/* This file is part of the libopencm3 project.
 *
 * It was generated by the irq2nvic_h script.
 */

#ifndef LIBOPENCM3_STM32_L0_NVIC_H
#define LIBOPENCM3_STM32_L0_NVIC_H

#include <libopencm3/cm3/nvic.h>

/** @defgroup CM3_nvic_defines_STM32L0 User interrupts for STM32 L0 series
    @ingroup CM3_nvic_defines

    @{*/

#define NVIC_WWDG_IRQ 0
#define NVIC_PVD_IRQ 1
#define NVIC_RTC_IRQ 2
#define NVIC_FLASH_IRQ 3
#define NVIC_RCC_IRQ 4
#define NVIC_EXTI0_1_IRQ 5
#define NVIC_EXTI2_3_IRQ 6
#define NVIC_EXTI4_15_IRQ 7
#define NVIC_TSC_IRQ 8
#define NVIC_DMA1_CHANNEL1_IRQ 9
#define NVIC_DMA1_CHANNEL2_3_IRQ 10
#define NVIC_DMA1_CHANNEL4_5_IRQ 11
#define NVIC_ADC_COMP_IRQ 12
#define NVIC_LPTIM1_IRQ 13
#define NVIC_RESERVED1_IRQ 14
#define NVIC_TIM2_IRQ 15
#define NVIC_RESERVED2_IRQ 16
#define NVIC_TIM6_DAC_IRQ 17
#define NVIC_RESERVED3_IRQ 18
#define NVIC_RESERVED4_IRQ 19
#define NVIC_TIM21_IRQ 20
#define NVIC_RESERVED5_IRQ 21
#define NVIC_TIM22_IRQ 22
#define NVIC_I2C1_IRQ 23
#define NVIC_I2C2_IRQ 24
#define NVIC_SPI1_IRQ 25
#define NVIC_SPI2_IRQ 26
#define NVIC_USART1_IRQ 27
#define NVIC_USART2_IRQ 28
#define NVIC_LPUART1_IRQ 29
#define NVIC_LCD_IRQ 30
#define NVIC_USB_IRQ 31

#define NVIC_IRQ_COUNT 32

/**@}*/

/** @defgroup CM3_nvic_isrprototypes_STM32L0 User interrupt service routines (ISR) prototypes for STM32 L0 series
    @ingroup CM3_nvic_isrprototypes

    @{*/

BEGIN_DECLS

void wwdg_isr(void);
void pvd_isr(void);
void rtc_isr(void);
void flash_isr(void);
void rcc_isr(void);
void exti0_1_isr(void);
void exti2_3_isr(void);
void exti4_15_isr(void);
void tsc_isr(void);
void dma1_channel1_isr(void);
void dma1_channel2_3_isr(void);
void dma1_channel4_5_isr(void);
void adc_comp_isr(void);
void lptim1_isr(void);
void reserved1_isr(void);
void tim2_isr(void);
void reserved2_isr(void);
void tim6_dac_isr(void);
void reserved3_isr(void);
void reserved4_isr(void);
void tim21_isr(void);
void reserved5_isr(void);
void tim22_isr(void);
void i2c1_isr(void);
void i2c2_isr(void);
void spi1_isr(void);
void spi2_isr(void);
void usart1_isr(void);
void usart2_isr(void);
void lpuart1_isr(void);
void lcd_isr(void);
void usb_isr(void);

END_DECLS

/**@}*/

#endif /* LIBOPENCM3_STM32_L0_NVIC_H */
