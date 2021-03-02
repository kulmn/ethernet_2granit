/*
 * main.h
 *
 *  Created on: 21 ���. 2014 �.
 *      Author: kulish_y
 */

#ifndef MAIN_H_
#define MAIN_H_


#include <stdint.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/spi.h>




// FreeRTOS inc
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Ethernet inc
#include "enc28j60.h"
#include "lan.h"
#include "counter.h"
#include "web_if.h"



#include "delay.h"
//#include "pwm.h"
//#include "pid.h"
#include "convert_fn.h"
#include <usart_hl.h>
#include "tm_granit.h"

//#include "owi.h"
//#include "owi_stm32f0xx.h"
//#include "hd44780.h"
//#include "hd44780_stm32f030.h"

//#include "ds18b20.h"


//#define CRITICAL_SECTION_START	taskENTER_CRITICAL();
//#define CRITICAL_SECTION_END	taskEXIT_CRITICAL();



#define _PORT(Prt,Pn)         (Prt)
#define _PIN(Prt,Pn)         (Pn)

#define PORT(PP)        _PORT(PP)
#define PIN(PP)         _PIN(PP)


/**** PINs defines *******/

#define LED_CPU_LOAD		GPIOA, GPIO8
//#define LED_GREEN			GPIOA, GPIO3									// GPIO5

#define USART2_TX_PIN	GPIOA, GPIO2
#define USART2_RX_PIN	GPIOA, GPIO3


// Buttons def
#define BUTTON_DEF			GPIOB, GPIO7
//#define BUTTON_UP			GPIOC, GPIO8
//#define BUTTON_DN		GPIOC, GPIO9


#endif /* MAIN_H_ */
