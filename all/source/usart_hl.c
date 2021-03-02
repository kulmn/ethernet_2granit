/*
 * usart_hl.c
 *
 *  Created on: 16 ����. 2016 �.
 *      Author: kulish_y
 */


#include <usart_hl.h>


// UART global variables
#ifdef USE_USART1
cBuffer *uart1_RxBuffer;				// uart1 receive buffer pointer
cBuffer *uart1_TxBuffer;				// uart1 transmit buffer pointer
#endif

#ifdef USE_USART2
cBuffer *uart2_RxBuffer;				// uart2 receive buffer pointer
cBuffer *uart2_TxBuffer;				// uart2 transmit buffer pointer
#endif

void uart_init(USART_HAL *uart)
{
	/* Setup USART2 parameters. */
	usart_set_baudrate(uart->usart, uart->baudrate);
	usart_set_databits(uart->usart, 8);
	usart_set_parity(uart->usart, USART_PARITY_NONE);
	usart_set_stopbits(uart->usart, USART_CR2_STOP_1_0BIT);
	usart_set_mode(uart->usart, USART_MODE_TX_RX);
	usart_set_flow_control(uart->usart, USART_FLOWCONTROL_NONE);

	usart_enable_rx_interrupt(uart->usart);

	// initialize the UART receive buffer
	bufferInit(&uart->rx_buf, (uint8_t *)uart->rx_buf_ptr, uart->rx_buf_size);
	// initialize the UART transmit buffer
	bufferInit(&uart->tx_buf, (uint8_t *)uart->tx_buf_ptr, uart->tx_buf_size);

	switch (uart->usart)
	{
#ifdef USE_USART1
		case USART1: uart1_TxBuffer=&uart->tx_buf; uart1_RxBuffer=&uart->rx_buf; break;
#endif

#ifdef USE_USART2
		case USART2: uart2_TxBuffer=&uart->tx_buf; uart2_RxBuffer=&uart->rx_buf; break;
#endif
	}
	/* Finally enable the USART. */
	usart_enable(uart->usart);
}

// Buffered send a byte via the uart using interrupt control.
USART_Result usart_hl_send(USART_HAL *uart, uint8_t data)
{
	if (bufferAddToEnd(&uart->tx_buf, data)==BUFFER_RESULT_OK)
	{
		usart_enable_tx_interrupt(uart->usart);	return USART_RESULT_OK;
	}else return USART_RESULT_ERROR;
}

// Buffered send a string via the uart using interrupt control.
USART_Result usart_hl_send_str(USART_HAL *uart, uint8_t* str)
{
	/* Go through entire string */
	while (*str)
	{
		if (bufferAddToEnd(&uart->tx_buf, *str++)==BUFFER_RESULT_ERROR)
			return USART_RESULT_ERROR;
	}

	usart_enable_tx_interrupt(uart->usart);
	return USART_RESULT_OK;
}


// gets a single byte from the uart receive buffer (getchar-style)
int uartGetByte(USART_HAL *uart)
{
	uint8_t c;
	if(usart_hl_get_byte(uart, &c))
		return c;
	else
		return -1;
}


// gets a byte (if available) from the uart receive buffer
USART_Result usart_hl_get_byte(USART_HAL *uart, uint8_t* rxData)
{
	// make sure we have data
	if(uart->rx_buf.datalength)
	{
		// get byte from beginning of buffer
		*rxData = bufferGetFromFront(&uart->rx_buf);
		return USART_RESULT_OK;
	}
	else
	{
		// no data
		return USART_RESULT_ERROR;
	}
}

// flush all data out of the receive buffer
void uartFlushReceiveBuffer(USART_HAL *uart)
{
	// flush all data from receive buffer
	//bufferFlush(&uartRxBuffer);
	// same effect as above
	uart->rx_buf.datalength = 0;
}


#ifdef USE_USART1
// UART1 Interrupt Handler
void USART1_IRQHandler(void)
{
	if (usart_get_flag(USART1, USART_ISR_TXE) )
	{
		if(uart1_TxBuffer->datalength)		// check if there's data left in the buffer
		{
			usart_send(USART1, bufferGetFromFront(uart1_TxBuffer));	// send byte from top of buffer
		}else	usart_disable_tx_interrupt(USART1);
	}

	if (usart_get_flag(USART1, USART_ISR_RXNE) )
	{
		bufferAddToEnd(uart1_RxBuffer, usart_recv(USART1) );
	}
}
#endif


#ifdef USE_USART2
// UART2 Interrupt Handler
void USART2_IRQHandler(void)
{
	if (usart_get_flag(USART2, USART_ISR_TXE) )
	{
		if(uart2_TxBuffer->datalength)		// check if there's data left in the buffer
		{
			usart_send(USART2, bufferGetFromFront(uart2_TxBuffer));	// send byte from top of buffer
		}else	usart_disable_tx_interrupt(USART2);
	}

	if (usart_get_flag(USART2, USART_ISR_RXNE) )
	{
		bufferAddToEnd(uart2_RxBuffer, usart_recv(USART2) );
	}
}
#endif

