
#include "tm_granit.h"


S_Port_TypeDef 		s_port0;
#ifdef USE_PORT1
S_Port_TypeDef 		s_port1;
#endif


void granit_send_data_pkt(S_Port_TypeDef *s_port, uint8_t n_kp, uint8_t afb, enum pkt_info_type info_tp, uint8_t group, uint8_t *data)
{
	uint8_t tmp_buf[32], i;

	tmp_buf[0]=n_kp;
	tmp_buf[1]=(0x04<<4 | afb);
	tmp_buf[2]=(info_tp<<4 | group);

	if (info_tp==pkt_info_ts)
	{
		for ( i=0; i<8; i++) tmp_buf[i+3]=data[i];
		for ( i=0;i<8;i++) tmp_buf[i+11]=~data[i];
	} else
		for ( i=0;i<16;i++) tmp_buf[i+3]=data[i];

	tmp_buf[19]=0;  tmp_buf[20]=0;
	uint16_t crc = granit_crc16(tmp_buf,21);
	tmp_buf[19] = (uint8_t) (crc>>8);
	tmp_buf[20] = (uint8_t) crc;

   granit_send_pkt(s_port, tmp_buf, 21);
}


/******************************************************************************/
void granit_kvitance_pkt(S_Port_TypeDef *s_port, uint8_t n_kp, uint8_t afb)
{
	uint8_t buf[4];

	buf[0]=n_kp;
	buf[1]=(0x00 | afb);
	buf[2]=0;  buf[3]=0;
	uint16_t crc= granit_crc16(buf, 4);
	buf[2] = (uint8_t) (crc>>8);
	buf[3] = (uint8_t) crc;
	granit_send_pkt(s_port, buf, 4);
}


/******************************************************************************/
void granit_getinfo_pkt(S_Port_TypeDef *s_port, const uint8_t n_kp, enum get_info_type info_tp)
{
	uint8_t buf[4];

	buf[0]=n_kp;
	buf[1]=(0x01<<4 | info_tp);
	buf[2]=0;  buf[3]=0;
	uint16_t crc= granit_crc16(buf, 4);
	buf[2] = (uint8_t) (crc>>8);
	buf[3] = (uint8_t) crc;
	granit_send_pkt(s_port, buf, 4);
}


/******************************************************************************/
// data =  01111110......result data......01111110....
// return sizeof result data
uint8_t del_bit_staff(uint8_t *data, uint8_t size)
{
	uint8_t 	onecnt=0, bitcnt=0;

	for (uint8_t i=8; i < size << 3; i++)
	{
		if ( data[i >> 3] & (1<< (i & 0x07) ) )
		{
			onecnt++;
			data[bitcnt >> 3] |= (1<<(7 - (bitcnt & 0x07)) );
			if (onecnt==6)							// find end flag 0x7E
			{
				bitcnt = bitcnt - 6;
				return bitcnt >> 3;
			}
		}else
		{
			if (onecnt==5) bitcnt--;
			else data[bitcnt >> 3] &=~(1<< (7 - (bitcnt & 0x07)) );
			onecnt=0;
		}
		bitcnt++;
	}
//Error (end flag 0x7E not found)
return 0;
}


/******************************************************************************/
uint8_t add_bit_staff(uint8_t *data, uint8_t size)
{
	uint8_t 	buf[32], onecnt=0, bitcnt=0;

  buf[0]=0x7E;
  data[size]=0x7E;

  for (uint8_t i=0; i<(size+1) << 3; i++)
  {
    if ((data[i >> 3] & (1<<(7-(i & 0x07))) ) != 0 )
      {
        onecnt++;
        buf[(bitcnt >> 3)+1] |= (1<< (bitcnt & 0x07) );
        if ((onecnt==5) && (i >> 3 != size))
          {
            onecnt=0;
            bitcnt++;
            buf[(bitcnt >> 3)+1] &=~(1<< (bitcnt & 0x07) );
          }
      }else
      {
        buf[(bitcnt >> 3)+1] &=~(1<<  (bitcnt & 0x07) );
        onecnt=0;
      }
   bitcnt++;
  }

  while ((bitcnt & 0x07)!=0)
    {
      buf[(bitcnt >> 3)+1] &= ~(1<< (bitcnt & 0x07) );
      bitcnt++;
    }

  for (uint8_t i=0; i<((bitcnt >> 3)+1); i++)
    data[i]=buf[i];
  return ((bitcnt >> 3)+1);
}


/******************************************************************************/
uint16_t granit_crc16(uint8_t *data, uint8_t size)
{
	uint16_t flg,crc=0;
	uint8_t  count,c,i;

	for (i=0; i<size; i++)
    {
		c = data[i];
		for (count=0; count<8; count++)
		{
			flg=crc & 0x8000;
			crc<<=1;
			if (((c) & 0x80) !=0) crc+=1;
			if (flg!=0) crc ^=0x1021;
			c<<=1;
		}
    }
  return crc;
}


/******************************************************************************/
void s_port_init_struct(S_Port_TypeDef *s_port)
{
	s_port->tx_enable = false;
	s_port->tx_size = 0;
	s_port->rx_packet_received = false;
	s_port->rx_bytes_read = 0;
	s_port->rx_start_flag = 0;
	s_port->rx_end_flag = 0;
	s_port->rx_cur_logic_level = 0;
}


/******************************************************************************/
void s_port_granit_init(uint16_t	speed)
{
		gpio_mode_setup(PORT(SERIAL_RX_LED), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(SERIAL_RX_LED));
		gpio_mode_setup(PORT(SERIAL_TX_LED), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(SERIAL_TX_LED));

		gpio_mode_setup(PORT(SERIAL_TX0_PIN), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(SERIAL_TX0_PIN));
		gpio_set(SERIAL_TX0_PIN);
		gpio_mode_setup(PORT(SERIAL_TX1_PIN), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(SERIAL_TX1_PIN));
		gpio_set(SERIAL_TX1_PIN);

		s_port_init_struct(&s_port0);
		s_port_rx0_timer_init(speed);
		bitbufInit(&s_port0.rx_bitbuf, s_port0.rx_buf, 32);
		bitbufInit(&s_port0.tx_bitbuf, s_port0.tx_buf, 32);

#ifdef USE_PORT1
		s_port_init_struct(&s_port1);
		s_port_rx1_timer_init(speed);
		bitbufInit(&s_port1.rx_bitbuf, s_port1.rx_buf, 32);
		bitbufInit(&s_port1.tx_bitbuf, s_port1.tx_buf, 32);
#endif
	s_port_tx_timer_init(speed);

}



/******************************************************************************/
void s_port_send_meandr(S_Port_TypeDef *s_port)
{
	uint8_t buf[2];

	buf[0] = 0xA0;
	buf[1] = 0xAA;
	s_port_send(s_port, buf, 2);
}


/******************************************************************************/
void granit_send_pkt(S_Port_TypeDef *s_port, uint8_t	*buf, uint8_t size)
{
	  size=add_bit_staff(buf, size);
	  s_port_send(s_port, buf, size);
}

/******************************************************************************/
uint8_t granit_receive_pkt(S_Port_TypeDef *s_port, uint8_t *buf )
{
	uint8_t size;

	size = s_port_read(s_port, buf);
	if (size) 	size = del_bit_staff(buf, size);
	return size;
}

/******************************************************************************/
void s_port_send(S_Port_TypeDef *s_port, uint8_t	*buf, uint8_t size)
{
	while (s_port->tx_enable) {}				// wait for transmitting

	memcpy(s_port->tx_buf, buf, size);
	s_port->tx_size = size;
	bitbufFlush(&s_port->tx_bitbuf);
	s_port->tx_enable=true;
}

/******************************************************************************/
//		return packet size
uint8_t s_port_read(S_Port_TypeDef *s_port, uint8_t	*buf)
{
	uint8_t size=0;

	if (s_port->rx_packet_received)
	{
		size =  s_port->rx_bytes_read;
		memcpy(buf, s_port->rx_buf, size);
		s_port->rx_packet_received = false;
	}
	return size;
}


/******************************************************************************/
void s_port_tx_timer_init(uint16_t tx_speed)
{
		rcc_periph_clock_enable(TX_TIMER_RCC);

		timer_set_mode(TX_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
		timer_set_prescaler(TX_TIMER, rcc_apb1_frequency/(1200000)-1);				//	1.2 MHz
		timer_set_period(TX_TIMER, (1200000/tx_speed));
		timer_enable_irq(TX_TIMER, TIM_DIER_UIE );
		timer_enable_counter(TX_TIMER);
		nvic_set_priority(TX_TIMER_IRQ,64);
		nvic_enable_irq(TX_TIMER_IRQ);
}


/******************************************************************************/
void s_port_rx0_timer_init(uint16_t rx_speed)
{
	rcc_periph_clock_enable(RX0_TIMER_RCC);
	gpio_mode_setup(PORT(SERIAL_RX0_PIN), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(SERIAL_RX0_PIN));
	gpio_set_af(PORT(SERIAL_RX0_PIN), GPIO_AF4, PIN(SERIAL_RX0_PIN));

	timer_set_mode(RX0_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_period(RX0_TIMER, UINT16_MAX);
	timer_ic_set_input(RX0_TIMER, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_filter(RX0_TIMER, TIM_IC1, TIM_SMCR_ETF_OFF);
	// The CC1NP/CC1P bits select the polarity of TI1FP1 and TI2FP1 for capture operation.
	// Non-inverted/both edges: circuit is sensitive to both the rising and falling edges of TIxFP1, TIxFP1 is not inverted.
	TIM_CCER(RX0_TIMER) |= TIM_CCER_CC1P;			// CC1NP/CC1P = 11
	TIM_CCER(RX0_TIMER) |= TIM_CCER_CC1NP;

	timer_set_prescaler(RX0_TIMER, rcc_apb1_frequency/(256*rx_speed)-1);
	timer_ic_enable(RX0_TIMER, TIM_IC1); 			// Capture enabled
	timer_enable_irq(RX0_TIMER, TIM_DIER_UIE | TIM_DIER_CC1IE	);
	timer_enable_counter(RX0_TIMER);
	nvic_set_priority(RX0_TIMER_IRQ,128);
	nvic_enable_irq(RX0_TIMER_IRQ);
}

#ifdef USE_PORT1
/******************************************************************************/
void s_port_rx1_timer_init(uint16_t rx_speed)
{
	rcc_periph_clock_enable(RX1_TIMER_RCC);
	gpio_mode_setup(PORT(SERIAL_RX1_PIN), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(SERIAL_RX1_PIN));
	gpio_set_af(PORT(SERIAL_RX1_PIN), GPIO_AF5, PIN(SERIAL_RX1_PIN));

	timer_set_mode(RX1_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_period(RX1_TIMER, UINT16_MAX);
	timer_ic_set_input(RX1_TIMER, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_filter(RX1_TIMER, TIM_IC1, TIM_SMCR_ETF_OFF);
	// The CC1NP/CC1P bits select the polarity of TI1FP1 and TI2FP1 for capture operation.
	// Non-inverted/both edges: circuit is sensitive to both the rising and falling edges of TIxFP1, TIxFP1 is not inverted.
	TIM_CCER(RX1_TIMER) |= TIM_CCER_CC1P;			// CC1NP/CC1P = 11
	TIM_CCER(RX1_TIMER) |= TIM_CCER_CC1NP;

	timer_set_prescaler(RX1_TIMER, rcc_apb1_frequency/(256*rx_speed)-1);
	timer_ic_enable(RX1_TIMER, TIM_IC1); 			// Capture enabled
	timer_enable_irq(RX1_TIMER, TIM_DIER_UIE | TIM_DIER_CC1IE	);
	timer_enable_counter(RX1_TIMER);
	nvic_set_priority(RX1_TIMER_IRQ,128);
	nvic_enable_irq(RX1_TIMER_IRQ);
}
#endif


/******************************************************************************/
void s_port_rx_timer_overflow(S_Port_TypeDef *s_port)
{
	s_port->rx_cur_logic_level=0;
	s_port->rx_start_flag=0;
	s_port->rx_end_flag=0;
	bitbufFlush(&s_port->rx_bitbuf);
	if (!s_port->rx_packet_received)	s_port->rx_enabled=1;
	rx_led_off();
}


/******************************************************************************/
void s_port_rx_timer_input(S_Port_TypeDef *s_port, uint8_t 	rx_data)
{
	if (s_port->rx_enabled)
	{
		if ((rx_data == 6) && (s_port->rx_cur_logic_level==0))
		{
			rx_led_on();
			if (!s_port->rx_start_flag)
			{
				s_port->rx_start_flag=1;
				bitbufStore(&s_port->rx_bitbuf, 0);
			}else	s_port->rx_end_flag=1;
		}

		if (s_port->rx_start_flag)
		{
			if (s_port->rx_cur_logic_level) s_port->rx_cur_logic_level = 0;
				else s_port->rx_cur_logic_level = 1;

			for (uint8_t i=0; i<rx_data; i++)
			{
				if (s_port->rx_cur_logic_level)
					bitbufStore(&s_port->rx_bitbuf, 1);
				else
					bitbufStore(&s_port->rx_bitbuf, 0);
			}
		}

		if (s_port->rx_bitbuf.bytePos > (s_port->rx_bitbuf.size-1) ) s_port->rx_enabled = 0;			// Error buffer overflow

		if (s_port->rx_end_flag)
		{
			s_port->rx_bytes_read = (s_port->rx_bitbuf.bytePos)+1;
			s_port->rx_packet_received=true;
			s_port->rx_enabled = 0;
			s_port->rx_start_flag=0;
			s_port->rx_end_flag=0;
			rx_led_off();
		}
	}
}



void TIM14_IRQHandler(void)												// RX0 IRQ, change if other timer used
{
	uint16_t tmp_data;
	uint8_t 	rx_data;

	if (timer_get_flag(RX0_TIMER, TIM_SR_UIF))										// overflow
	{
		timer_clear_flag(RX0_TIMER, TIM_SR_UIF);
		s_port_rx_timer_overflow(&s_port0);
	}

	else if (timer_get_flag(RX0_TIMER, TIM_SR_CC1IF))						// Input capture
	{
		timer_set_counter(RX0_TIMER,0);
		tmp_data = TIM_CCR1(RX0_TIMER);
		rx_data = (tmp_data >> 8) + ((tmp_data & 0x0080) >> 7);

		s_port_rx_timer_input(&s_port0, rx_data);
	}
}

#ifdef USE_PORT1
void TIM16_IRQHandler(void)												// RX1 IRQ, change if other timer used
{
	uint16_t tmp_data;
	uint8_t 	rx_data;

	if (timer_get_flag(RX1_TIMER, TIM_SR_UIF))										// overflow
	{
		timer_clear_flag(RX1_TIMER, TIM_SR_UIF);
		s_port_rx_timer_overflow(&s_port1);
	}

	else if (timer_get_flag(RX1_TIMER, TIM_SR_CC1IF))						// Input capture
	{
		timer_set_counter(RX1_TIMER,0);
		tmp_data = TIM_CCR1(RX1_TIMER);
		rx_data = (tmp_data >> 8) + ((tmp_data & 0x0080) >> 7);

		s_port_rx_timer_input(&s_port1, rx_data);
	}
}
#endif


void TIM17_IRQHandler(void)									// TX IRQ, change if other timer used
{

	if (timer_get_flag(TX_TIMER, TIM_SR_UIF))
	{
		timer_clear_flag(TX_TIMER, TIM_SR_UIF);

		if (s_port0.tx_enable)				// Send tx buf	(LSB first)
		{
				tx_led_on();
				if (bitbufGet(&s_port0.tx_bitbuf) )
					gpio_clear(SERIAL_TX0_PIN);							// Out logic '1'
				else
					gpio_set(SERIAL_TX0_PIN);								// Out logic '0'
				if (s_port0.tx_bitbuf.bytePos >= s_port0.tx_size)
				{
					bitbufFlush(&s_port0.tx_bitbuf);
					s_port0.tx_enable=false;
					tx_led_off();
				}
		}else gpio_set(SERIAL_TX0_PIN);

#ifdef USE_PORT1
		if (s_port1.tx_enable)				// Send tx buf	(LSB first)
		{
			tx_led_on();
			if (bitbufGet(&s_port1.tx_bitbuf) )
				gpio_clear(SERIAL_TX1_PIN);							// Out logic '1'
			else
				gpio_set(SERIAL_TX1_PIN);								// Out logic '0'
			if (s_port1.tx_bitbuf.bytePos >= s_port1.tx_size)
			{
				bitbufFlush(&s_port1.tx_bitbuf);
				s_port1.tx_enable=false;
				tx_led_off();
			}
		}else gpio_set(SERIAL_TX1_PIN);
#endif
	}


}
