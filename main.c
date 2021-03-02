/*
 * main.c
 *
 *  Created on: 08 сент. 2014 г.
 *      Author: yura
 */

#include "main.h"

//xQueueHandle xTemperQueue;

//SemaphoreHandle_t xButUpSmphr;
//SemaphoreHandle_t xButDnSmphr;

//volatile uint8_t led_ind_buf[4],  set_temper=0;
//volatile uint16_t  max6675_temp;


/******************************************************************************/
// static uint8_t tx_buf[]= {0x7E, 0x00, 0x44, 0x00, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E};
//static uint8_t tx_buf[]= {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x00, 0x00};
// static uint8_t rx_buf[32];

// static uint8_t s_port_tx_buf[32];
// static uint8_t s_port_rx_buf[32];

 uint32_t serial_pct_rx0_cnt=0;
 uint32_t serial_pct_rx1_cnt=0;

extern  S_Port_TypeDef 		s_port0;
extern  S_Port_TypeDef 		s_port1;

 /******************************************************************************/
UBaseType_t uxHighWaterMark1;
UBaseType_t uxHighWaterMark;


#define USART2_RX_BUF_SIZE	32
#define USART2_TX_BUF_SIZE	32

//static uint8_t usart2_rx_buf[USART2_RX_BUF_SIZE];
//static uint8_t usart2_tx_buf[USART2_TX_BUF_SIZE];


//USART_HAL usart_2;

Lan_Config_TypeDef 		lan_config;
extern uint16_t				serial_speed;
extern uint16_t				granit_n_kp0;
extern uint16_t				granit_n_kp1;



void send_data_udp()
{
	eth_frame_t *frame = (void*)net_buf;
	ip_packet_t *ip = (void*)(frame->data);
	udp_packet_t *udp = (void*)(ip->data);

	uint8_t size;

	ip->to_addr = lan_config.rem_ip_addr;



//	size = s_port_read(&s_port0, udp->data);

	size = granit_receive_pkt(&s_port0, udp->data);
	if (size)
	{
		if ((udp->data[1] & 0xF0) == 0x40)
			granit_kvitance_pkt(&s_port0, granit_n_kp0, (udp->data[1] & 0x0F));

		udp->to_port = lan_config.rem_udp_port0;
		udp->from_port = lan_config.loc_udp_port0;
		udp_send(frame, size);
		serial_pct_rx0_cnt++;
	}

	size = granit_receive_pkt(&s_port1, udp->data);
	if (size)
	{
		if ((udp->data[1] & 0xF0) == 0x40)
			granit_kvitance_pkt(&s_port1, granit_n_kp1, (udp->data[1] & 0x0F));

		udp->to_port = lan_config.rem_udp_port1;
		udp->from_port = lan_config.loc_udp_port1;
		udp_send(frame, size);
		serial_pct_rx1_cnt++;
	}


}

void udp_packet(eth_frame_t *frame, uint16_t len)
{
	ip_packet_t *ip = (void*)(frame->data);
	udp_packet_t *udp = (void*)(ip->data);
	uint8_t *data = udp->data;
	uint8_t i, count;

	if (udp->to_port == lan_config.loc_udp_port0)
	{
		if (len<32)  	granit_send_pkt(&s_port0, udp->data, len);
	}

	else if (udp->to_port == lan_config.loc_udp_port1)
	{
		if (len<32)  	granit_send_pkt(&s_port1, udp->data, len);
	}

}



/******************************************************************************/
uint8_t tcp_listen(uint8_t id, eth_frame_t *frame)
{
	ip_packet_t *ip = (void*)(frame->data);
	tcp_packet_t *tcp = (void*)(ip->data);

	if( (tcp->to_port == htons(8080)) ||(tcp->to_port == htons(44444)))
	{
		return 1;
	}
	return 0;
}

/******************************************************************************/
void tcp_read(uint8_t id, eth_frame_t *frame, uint8_t re)
{
}

/******************************************************************************/
void tcp_write(uint8_t id, eth_frame_t *frame, uint16_t len)
{

//	    ip_packet_t *ip = (void*)(frame->data);
//	    tcp_packet_t *tcp = (void*)(ip->data);
//	    uint16_t len_str;

	    webif_data(id, frame, len);

	    // Отправляем данные

//	    len_str=fill_buf(tcp->data,0,HTTP_200_header);
//	    len_str=fill_buf(tcp->data,len_str,"STM32 web server");

//	    tcp_send(id, frame, len_str, TCP_OPTION_CLOSE);

}

/******************************************************************************/
void tcp_closed(uint8_t id, uint8_t reset)
{

}



/******************************************************************************/
void vGetGranit_CH0_DataTask(void *pvParameters)			// ~ 28*4 bytes
{
	TickType_t xLastWakeTime;
	portBASE_TYPE xStatus;
	const TickType_t xFrequency = 5000;		// 5 sec
	xLastWakeTime = xTaskGetTickCount();
	uint8_t cnt=0;

	for( ;; )
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		if (granit_n_kp0)
		{
			cnt++;
			s_port_send_meandr(&s_port0);
			switch (cnt)
			{
			case 5:	{ granit_getinfo_pkt(&s_port0, granit_n_kp0, get_info_ts); break;	}
			case 10:	{ granit_getinfo_pkt(&s_port0, granit_n_kp0, get_info_tit); break;	}
			case 15: { granit_getinfo_pkt(&s_port0, granit_n_kp0, get_info_tii); cnt=0; break;	}
			}
		}
	}
	vTaskDelete(NULL);
}


/******************************************************************************/
void vGetGranit_CH1_DataTask(void *pvParameters)
{
	TickType_t xLastWakeTime;
	portBASE_TYPE xStatus;
	const TickType_t xFrequency = 5000;		// 5 sec
	xLastWakeTime = xTaskGetTickCount();
	uint8_t cnt=0;

	for( ;; )
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		if (granit_n_kp1)
		{
			cnt++;
			s_port_send_meandr(&s_port1);
			switch (cnt)
			{
			case 5:	{ granit_getinfo_pkt(&s_port1, granit_n_kp1, get_info_ts); break;	}
			case 10:	{ granit_getinfo_pkt(&s_port1, granit_n_kp1, get_info_tit); break;	}
			case 15: { granit_getinfo_pkt(&s_port1, granit_n_kp1, get_info_tii); cnt=0; break;	}
			}
		}
	}
	vTaskDelete(NULL);
}


/******************************************************************************/
void vSendUdpDataTask(void *pvParameters)
{
	TickType_t xLastWakeTime;
	portBASE_TYPE xStatus;
	const TickType_t xFrequency = 1000;		// 10 Hz
	xLastWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

		send_data_udp();

	}
	vTaskDelete(NULL);
}

/******************************************************************************/
void vLanPollTask (void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 30;		// 100 Hz
	xLastWakeTime = xTaskGetTickCount();

	while(1)
    {
    	vTaskDelayUntil( &xLastWakeTime, xFrequency );

    	iwdg_reset();
    	lan_poll();


    	send_data_udp();

//    	size = s_port_read(rx_buf);

 //   	if (size)
 //		for (uint8_t i=0; i < size; i++)
// 		{
// 			usart_hl_send(&usart_2, rx_buf[i]);
// 		}

	}
    vTaskDelete(NULL);
}



static void system_clock_setup(void)
{
	// Enable external high-speed oscillator 4MHz.
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	rcc_set_sysclk_source(RCC_HSE);

//	rcc_set_sysclk_source(RCC_HSI);

	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
	rcc_set_ppre(RCC_CFGR_PPRE_NODIV);

	flash_set_ws(FLASH_ACR_LATENCY_000_024MHZ);

	// 4MHz * 4  = 16MHz
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL6);

	RCC_CFGR &= ~RCC_CFGR_PLLSRC;

	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);
	rcc_set_sysclk_source(RCC_PLL);

	rcc_apb1_frequency = 24000000;
	rcc_ahb_frequency = 24000000;
}

/******************************************************************************
void usart_setup(void)
{
	usart_2.usart=USART2;
	usart_2.baudrate=9600;
	usart_2.rx_buf_ptr=(uint8_t *)usart2_rx_buf;
	usart_2.tx_buf_ptr=(uint8_t *)usart2_tx_buf;
	usart_2.rx_buf_size=USART2_RX_BUF_SIZE;
	usart_2.tx_buf_size=USART2_TX_BUF_SIZE;

	//Setup GPIO pins for USART2 transmit.
	gpio_mode_setup(PORT(USART2_TX_PIN), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(USART2_TX_PIN));
	// Setup USART2 TX pin as alternate function.
	gpio_set_af(PORT(USART2_TX_PIN), GPIO_AF1, PIN(USART2_TX_PIN));

	// Setup GPIO pins for USART2 receive.
	gpio_mode_setup(PORT(USART2_RX_PIN), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(USART2_RX_PIN));
	// Setup USART2 RX pin as alternate function.
	gpio_set_af(PORT(USART2_RX_PIN), GPIO_AF1, PIN(USART2_RX_PIN));

	rcc_periph_clock_enable(RCC_USART2);
	uart_init(&usart_2);
	// Enable the USART2 interrupt.
	nvic_enable_irq(NVIC_USART2_IRQ);
}


/******************************************************************************/
void load_config(void)
{
	uint32_t		address;

	address = last_page;

	lan_config.ip_address = (*(volatile uint32_t*) address);
	address += sizeof(lan_config.ip_address);
	lan_config.ip_gateway = (*(volatile uint32_t*) address);
	address += sizeof(lan_config.ip_gateway);
	lan_config.ip_mask = (*(volatile uint32_t*) address);
	address += sizeof(lan_config.ip_mask);
	lan_config.rem_ip_addr = (*(volatile uint32_t*) address);
	address += sizeof(lan_config.rem_ip_addr);
	lan_config.loc_udp_port0 = (*(volatile uint16_t*) address);
	address += sizeof(lan_config.loc_udp_port0);
	lan_config.loc_udp_port1 = (*(volatile uint16_t*) address);
	address += sizeof(lan_config.loc_udp_port1);
	lan_config.rem_udp_port0 = (*(volatile uint16_t*) address);
	address += sizeof(lan_config.rem_udp_port0);
	lan_config.rem_udp_port1 = (*(volatile uint16_t*) address);
	address += sizeof(lan_config.rem_udp_port1);
	serial_speed = (*(volatile uint16_t*) address);
	address += sizeof(serial_speed);
	granit_n_kp0 = (*(volatile uint16_t*) address);
	address += sizeof(granit_n_kp0);
	granit_n_kp1 = (*(volatile uint16_t*) address);
}




/******************************************************************************/
//----------------------------
// initializations
//----------------------------
void vFreeRTOSInitAll()
{
	system_clock_setup();
	rcc_peripheral_enable_clock(&RCC_APB1ENR ,RCC_APB1ENR_PWREN);				// enable APB1 clocks

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
//	rcc_periph_clock_enable(RCC_GPIOC);
//	rcc_periph_clock_enable(RCC_GPIOD);
//	rcc_periph_clock_enable(RCC_GPIOF);

	gpio_mode_setup(PORT(LED_CPU_LOAD), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(LED_CPU_LOAD));
	gpio_mode_setup(PORT(BUTTON_DEF), GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PIN(BUTTON_DEF));

//	gpio_set(LED_CPU_LOAD);

	iwdg_set_period_ms(5000);
	iwdg_start();

	delay_init();

//	usart_setup();

	if(!gpio_get(BUTTON_DEF))
	{
		lan_config.ip_address = inet_addr(192,168,88,222);
		lan_config.ip_gateway = inet_addr(192,168,88,1);
		lan_config.ip_mask = inet_addr(255,255,255,0);
		lan_config.rem_ip_addr = inet_addr(192,168,88,2);
		lan_config.loc_udp_port0 = htons(10000);
		lan_config.loc_udp_port1 = htons(10001);
		lan_config.rem_udp_port0 = htons(10000);
		lan_config.rem_udp_port1 = htons(10001);
		serial_speed = 300;
		granit_n_kp0 = 0;
		granit_n_kp1 = 0;
	} else 	load_config();

//	serial_speed = 300;
//	lan_config.ip_address = inet_addr(192,168,88,248);
//	lan_config.ip_mask = inet_addr(255,255,255,0);

	ip_addr = lan_config.ip_address;
	ip_mask = lan_config.ip_mask;
	ip_gateway = lan_config.ip_gateway;

	lan_init();

	s_port_granit_init(serial_speed);


}



void vApplicationIdleHook( void )
{
	gpio_clear(LED_CPU_LOAD);

}

void vApplicationTickHook( void )
{
	gpio_set(LED_CPU_LOAD);
	++ms_count;
	++tick_count;

	if(ms_count == 1000)
	{
		++second_count;
		ms_count = 0;
	}
}

/******************************************************************************/
int main(void)
{
	vFreeRTOSInitAll();

	xTaskCreate(vGetGranit_CH0_DataTask,(signed char*)"", configMINIMAL_STACK_SIZE * 2,	NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(vGetGranit_CH1_DataTask,(signed char*)"", configMINIMAL_STACK_SIZE * 2,	NULL, tskIDLE_PRIORITY + 1, NULL);

	xTaskCreate(vLanPollTask,(signed char*)"", configMINIMAL_STACK_SIZE * 4,NULL, tskIDLE_PRIORITY + 2, NULL);

	vTaskStartScheduler();

	for( ;; );
}
