#ifndef TM_GRANIT_H
#define TM_GRANIT_H

#include <string.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <bitbuf.h>



extern volatile uint16_t time_mks;

#define USE_PORT1

#define TX_TIMER						TIM17
#define TX_TIMER_RCC				RCC_TIM17
#define TX_TIMER_IRQ				NVIC_TIM17_IRQ

#define RX0_TIMER						TIM14
#define RX0_TIMER_RCC				RCC_TIM14
#define RX0_TIMER_IRQ				NVIC_TIM14_IRQ

#define SERIAL_RX0_PIN				GPIOA, GPIO4				//TIM14_CH1
#define SERIAL_TX0_PIN				GPIOA, GPIO7

#ifdef USE_PORT1
#define RX1_TIMER						TIM16
#define RX1_TIMER_RCC				RCC_TIM16
#define RX1_TIMER_IRQ				NVIC_TIM16_IRQ

#define SERIAL_RX1_PIN				GPIOA, GPIO6				//TIM16_CH1
#define SERIAL_TX1_PIN				GPIOA, GPIO5
#endif




#define SERIAL_RX_LED				GPIOA, GPIO10
#define SERIAL_TX_LED				GPIOA, GPIO9



#ifdef SERIAL_RX_LED
#define rx_led_on() 			gpio_set(SERIAL_RX_LED)
#define rx_led_off() 			gpio_clear(SERIAL_RX_LED)
#else
#define rx_led_on()
#define rx_led_off()
#endif

#ifdef SERIAL_TX_LED
#define tx_led_on() 			gpio_set(SERIAL_TX_LED)
#define tx_led_off() 			gpio_clear(SERIAL_TX_LED)
#else
#define tx_led_on()
#define tx_led_off()
#endif

#define _PORT(Prt,Pn)         (Prt)
#define _PIN(Prt,Pn)         (Pn)

#define PORT(PP)        _PORT(PP)
#define PIN(PP)         _PIN(PP)


typedef struct
{
	uint8_t 	tx_buf[32];
	uint8_t 	rx_buf[32];
	BitBuf		rx_bitbuf;
	BitBuf		tx_bitbuf;
	volatile uint8_t 		rx_packet_received;
	volatile uint8_t 	rx_bytes_read;
	volatile uint8_t 		tx_enable;
	volatile uint8_t 	tx_size;
	volatile uint8_t		rx_enabled;
	volatile uint8_t		rx_start_flag;
	volatile uint8_t		rx_end_flag;
	volatile uint8_t		rx_cur_logic_level;
}S_Port_TypeDef;


enum pkt_info_type {
	pkt_info_ts = 2,
	pkt_info_tii = 4,
	pkt_info_tit = 6,
};

enum get_info_type {
	get_info_ts = 8,
	get_info_tit = 9,
	get_info_tii = 10,
};

#define pkt_info_ts		0x02
#define pkt_info_tii		0x04
#define pkt_info_tit		0x06


void s_port_granit_init(uint16_t	speed);
void granit_getinfo_pkt(S_Port_TypeDef *s_port, const uint8_t n_kp, enum get_info_type info_tp);
void granit_kvitance_pkt(S_Port_TypeDef *s_port, uint8_t n_kp, uint8_t afb);
void granit_send_data_pkt(S_Port_TypeDef *s_port, uint8_t n_kp, uint8_t afb, enum pkt_info_type info_tp, uint8_t group, uint8_t *data);
void granit_send_pkt(S_Port_TypeDef *s_port, uint8_t	*buf, uint8_t size);
uint8_t granit_receive_pkt(S_Port_TypeDef *s_port, uint8_t *buf );

uint8_t add_bit_staff(uint8_t *data,uint8_t size);
uint16_t granit_crc16(uint8_t *data, uint8_t size);

void s_port_tx_timer_init(uint16_t tx_speed);
void s_port_rx0_timer_init(uint16_t tx_speed);
#ifdef USE_PORT1
void s_port_rx1_timer_init(uint16_t tx_speed);
#endif
void s_port_send_meandr(S_Port_TypeDef *s_port);
void s_port_send(S_Port_TypeDef *s_port, uint8_t	*buf, uint8_t size);
uint8_t s_port_read(S_Port_TypeDef *s_port, uint8_t	*buf);


#endif
