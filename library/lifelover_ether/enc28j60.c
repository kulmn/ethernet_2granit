#include "enc28j60.h"

/*
 * SPI
 */

volatile uint8_t enc28j60_current_bank = 0;
volatile uint16_t enc28j60_rxrdpt = 0;

uint8_t enc28j60_rxtx(uint8_t data)
{
	spi_send8(ENC28J60_SPI,data);
//	while ((SPI_SR(SPI1) & SPI_SR_BSY));
	return (uint8_t)  spi_read8(ENC28J60_SPI);
}

#define enc28j60_rx() enc28j60_rxtx(0xff)
#define enc28j60_tx(data) enc28j60_rxtx(data)

// Generic SPI read command
uint8_t enc28j60_read_op(uint8_t cmd, uint8_t adr)
{
	uint8_t data;

	enc28j60_select();
	enc28j60_tx(cmd | (adr & ENC28J60_ADDR_MASK));
	if(adr & 0x80) // throw out dummy byte
		enc28j60_rx(); // when reading MII/MAC register
	data = enc28j60_rx();
	enc28j60_release();
	return data;
}

// Generic SPI write command
void enc28j60_write_op(uint8_t cmd, uint8_t adr, uint8_t data)
{
	enc28j60_select();
	enc28j60_tx(cmd | (adr & ENC28J60_ADDR_MASK));
	enc28j60_tx(data);

//	while ((SPI_SR(ENC28J60_SPI) & SPI_SR_BSY));
//	delay_ms(10);
	enc28j60_release();
}

// Initiate software reset
void enc28j60_soft_reset()
{
	enc28j60_select();
	enc28j60_tx(ENC28J60_SPI_SC);
	enc28j60_release();
	
	enc28j60_current_bank = 0;
	volatile uint32_t i;
	for (i=0; i<20000; i++) {

	}
delay_ms(1); // Wait until device initializes	!!!
}


/*
 * Memory access
 */

// Set register bank
void enc28j60_set_bank(uint8_t adr)
{
	uint8_t bank;

	if( (adr & ENC28J60_ADDR_MASK) < ENC28J60_COMMON_CR )
	{
		bank = (adr >> 5) & 0x03; //BSEL1|BSEL0=0x03
		if(bank != enc28j60_current_bank)
		{
			enc28j60_write_op(ENC28J60_SPI_BFC, ECON1, 0x03);
			enc28j60_write_op(ENC28J60_SPI_BFS, ECON1, bank);
			enc28j60_current_bank = bank;
		}
	}
}

// Read register
uint8_t enc28j60_rcr(uint8_t adr)
{
	enc28j60_set_bank(adr);
	return enc28j60_read_op(ENC28J60_SPI_RCR, adr);
}

// Read register pair
uint16_t enc28j60_rcr16(uint8_t adr)
{
	enc28j60_set_bank(adr);
	return enc28j60_read_op(ENC28J60_SPI_RCR, adr) |
		(enc28j60_read_op(ENC28J60_SPI_RCR, adr+1) << 8);
}

// Write register
void enc28j60_wcr(uint8_t adr, uint8_t arg)
{
	enc28j60_set_bank(adr);
	enc28j60_write_op(ENC28J60_SPI_WCR, adr, arg);
}

// Write register pair
void enc28j60_wcr16(uint8_t adr, uint16_t arg)
{
	enc28j60_set_bank(adr);
	enc28j60_write_op(ENC28J60_SPI_WCR, adr, arg);
	enc28j60_write_op(ENC28J60_SPI_WCR, adr+1, arg>>8);
}

// Clear bits in register (reg &= ~mask)
void enc28j60_bfc(uint8_t adr, uint8_t mask)
{
	enc28j60_set_bank(adr);
	enc28j60_write_op(ENC28J60_SPI_BFC, adr, mask);
}

// Set bits in register (reg |= mask)
void enc28j60_bfs(uint8_t adr, uint8_t mask)
{
	enc28j60_set_bank(adr);
	enc28j60_write_op(ENC28J60_SPI_BFS, adr, mask);
}

// Read Rx/Tx buffer (at ERDPT)
void enc28j60_read_buffer(uint8_t *buf, uint16_t len)
{
	enc28j60_select();
	enc28j60_tx(ENC28J60_SPI_RBM);
	while(len--)
		*(buf++) = enc28j60_rx();
	enc28j60_release();
}

// Write Rx/Tx buffer (at EWRPT)
void enc28j60_write_buffer(uint8_t *buf, uint16_t len)
{
	enc28j60_select();
	enc28j60_tx(ENC28J60_SPI_WBM);
	while(len--)
		enc28j60_tx(*(buf++));
	enc28j60_release();
}

// Read PHY register
uint16_t enc28j60_read_phy(uint8_t adr)
{
	enc28j60_wcr(MIREGADR, adr);
	enc28j60_bfs(MICMD, MICMD_MIIRD);
	while(enc28j60_rcr(MISTAT) & MISTAT_BUSY)
		;
	enc28j60_bfc(MICMD, MICMD_MIIRD);
	return enc28j60_rcr16(MIRD);
}

// Write PHY register
void enc28j60_write_phy(uint8_t adr, uint16_t data)
{
	enc28j60_wcr(MIREGADR, adr);
	enc28j60_wcr16(MIWR, data);
	while(enc28j60_rcr(MISTAT) & MISTAT_BUSY)
		;
}


/*
 * Init & packet Rx/Tx
 */

void enc28j60_init(uint8_t *macadr)
{
	// Initialize SPI and GPIO

	rcc_periph_clock_enable(ENC28J60_SPI_RCC);
	rcc_periph_clock_enable(ENC28J60_PORT_RCC);

	gpio_mode_setup(PORT(ENC28J60_SPI_SCK), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(ENC28J60_SPI_SCK)); // SCK
	gpio_mode_setup(PORT(ENC28J60_SPI_MISO), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(ENC28J60_SPI_MISO));	// MISO
	gpio_mode_setup(PORT(ENC28J60_SPI_MOSI), GPIO_MODE_AF, GPIO_PUPD_NONE, PIN(ENC28J60_SPI_MOSI));	// MOSI
	gpio_mode_setup(PORT(ENC28J60_SPI_CS), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(ENC28J60_SPI_CS));	// CS
	enc28j60_release();
	gpio_mode_setup(PORT(ENC28J60_RESET), GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN(ENC28J60_RESET));
	gpio_set(ENC28J60_RESET);

	spi_set_master_mode(ENC28J60_SPI);
//	spi_set_unidirectional_mode(ENC28J60_SPI);
//	spi_set_full_duplex_mode(ENC28J60_SPI);
	spi_set_data_size(ENC28J60_SPI,SPI_CR2_DS_8BIT);
	spi_fifo_reception_threshold_8bit(ENC28J60_SPI);
	spi_set_clock_polarity_0(ENC28J60_SPI);
	spi_set_clock_phase_0(ENC28J60_SPI);
	spi_enable_software_slave_management(ENC28J60_SPI);
	spi_set_nss_high(ENC28J60_SPI);
	spi_set_baudrate_prescaler(ENC28J60_SPI,SPI_CR1_BAUDRATE_FPCLK_DIV_4);
	spi_send_msb_first(ENC28J60_SPI);
	spi_enable_crc(ENC28J60_SPI);
	spi_enable(ENC28J60_SPI);


	// Reset ENC28J60
	gpio_clear(ENC28J60_RESET);
	delay_ms(50);
	gpio_set(ENC28J60_RESET);
	enc28j60_soft_reset();

	// Setup filter
	enc28j60_wcr(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN|ERXFCON_BCEN);
	enc28j60_wcr16(EPMM0, 0x303f);
	enc28j60_wcr16(EPMCS, 0xf7f9);

	// Setup Rx/Tx buffer
	enc28j60_wcr16(ERXST, ENC28J60_RXSTART);
	enc28j60_wcr16(ERXRDPT, ENC28J60_RXSTART);
	enc28j60_wcr16(ERXND, ENC28J60_RXEND);
	enc28j60_rxrdpt = ENC28J60_RXSTART;

	// Setup MAC
	enc28j60_wcr(MACON1, MACON1_TXPAUS|MACON1_RXPAUS|MACON1_MARXEN); // Enable flow control Enable MAC Rx
	enc28j60_wcr(MACON2, 0); // Clear reset
	enc28j60_wcr(MACON3, MACON3_PADCFG0| // Enable padding,
		MACON3_TXCRCEN|MACON3_FRMLNEN|MACON3_FULDPX); // Enable crc & frame len chk
	enc28j60_wcr16(MAMXFL, ENC28J60_MAXFRAME);
	enc28j60_wcr(MABBIPG, 0x15); // Set inter-frame gap
	enc28j60_wcr(MAIPGL, 0x12);
	enc28j60_wcr(MAIPGH, 0x0c);

	enc28j60_wcr(MAADR5, macadr[0]); // Set MAC address
	enc28j60_wcr(MAADR4, macadr[1]);
	enc28j60_wcr(MAADR3, macadr[2]);
	enc28j60_wcr(MAADR2, macadr[3]);
	enc28j60_wcr(MAADR1, macadr[4]);
	enc28j60_wcr(MAADR0, macadr[5]);

	// Setup PHY
	enc28j60_write_phy(PHCON1, PHCON1_PDPXMD); // Force full-duplex mode
	enc28j60_write_phy(PHCON2, PHCON2_HDLDIS); // Disable loopback
	enc28j60_write_phy(PHLCON, PHLCON_LACFG2| // Configure LED ctrl
		PHLCON_LBCFG2|PHLCON_LBCFG1|PHLCON_LBCFG0|
		PHLCON_LFRQ0|PHLCON_STRCH);

	// Enable Rx packets
	enc28j60_bfs(ECON1, ECON1_RXEN);
}




/******************************************************************************/
void enc28j60_send_packet(uint8_t *data, uint16_t len)
{
	// wait until transmission has finished; referrring to the data sheet and
        // to the errata (Errata Issue 13; Example 1) you only need to wait until either
        // TXIF or TXERIF gets set; however this leads to hangs; apparently Microchip
        // realized this and in later implementations of their tcp/ip stack they introduced
        // a counter to avoid hangs; of course they didn't update the errata sheet
        uint16_t count = 0;
        while ((enc28j60_rcr(EIR) & (EIR_TXIF | EIR_TXERIF)) == 0 && ++count < 1000U)  ;


        // latest errata sheet: DS80349C
        // always reset transmit logic (Errata Issue 12)
        // the Microchip TCP/IP stack implementation used to first check
        // whether TXERIF is set and only then reset the transmit logic
        // but this has been changed in later versions; possibly they
        // have a reason for this; they don't mention this in the errata
        // sheet
		enc28j60_write_op(ENC28J60_SPI_BFS, ECON1, ECON1_TXRST);
		enc28j60_write_op(ENC28J60_SPI_BFC, ECON1, ECON1_TXRST);
		enc28j60_write_op(ENC28J60_SPI_BFC, EIR, EIR_TXERIF|EIR_TXIF);

        // prepare new transmission
       	enc28j60_wcr16(EWRPT, ENC28J60_TXSTART);
       	enc28j60_wcr16(ETXST, ENC28J60_TXSTART);
       	enc28j60_wcr16(ETXND, ENC28J60_TXSTART + len);
       	enc28j60_write_buffer(0, 1);
       	enc28j60_write_buffer(data, len);

        // initiate transmission
       	enc28j60_bfs(ECON1, ECON1_TXRTS); // Request packet send

        /*
        if (!(enc28j60_rcr(EIR) & EIR_TXERIF) && count < 1000U) {
            // no error; start new transmission
            BREAKORCONTINUE
        }

        // cancel previous transmission if stuck
        enc28j60_write_op(ENC28J60_SPI_BFC, ECON1, ECON1_TXRTS);

        // Check whether the chip thinks that a late collision ocurred; the chip
        // may be wrong (Errata Issue 13); therefore we retry. We could check
        // LATECOL in the ESTAT register in order to find out whether the chip
        // thinks a late collision ocurred but (Errata Issue 15) tells us that
        // this is not working. Therefore we check TSV
        transmit_status_vector tsv;
        uint16_t etxnd = enc28j60_rcr(ETXND);
        enc28j60_wcr(ERDPT, etxnd+1);
        enc28j60_read_buffer((uint8_t*) &tsv, sizeof(transmit_status_vector));
        // LATECOL is bit number 29 in TSV (starting from 0)

        if (!((enc28j60_rcr(EIR) & EIR_TXERIF) && (tsv.bytes[3] & 1<<5) ) || retry > 16U) {
            // there was some error but no LATECOL so we do not repeat
            BREAKORCONTINUE
        }
*/
 }

/******************************************************************************
void enc28j60_send_packet(uint8_t *data, uint16_t len)
{

	while(enc28j60_rcr(ECON1) & ECON1_TXRTS)
	{
		// TXRTS may not clear - ENC28J60 bug. We must reset
		// transmit logic in cause of Tx error
		if(enc28j60_rcr(EIR) & EIR_TXERIF)
		{
			enc28j60_bfs(ECON1, ECON1_TXRST);
			delay_ms(5);
			enc28j60_bfc(ECON1, ECON1_TXRST);
		}
	}

	enc28j60_wcr16(EWRPT, ENC28J60_TXSTART);
	enc28j60_write_buffer((uint8_t*)"\x00", 1);
	enc28j60_write_buffer(data, len);

	enc28j60_wcr16(ETXST, ENC28J60_TXSTART);
	enc28j60_wcr16(ETXND, ENC28J60_TXSTART + len);

	enc28j60_bfs(ECON1, ECON1_TXRTS); // Request packet send
}


/******************************************************************************/
uint16_t enc28j60_recv_packet(uint8_t *buf, uint16_t buflen)
{
	uint16_t len = 0, rxlen, status, temp;

	if(enc28j60_rcr(EPKTCNT))
	{
		enc28j60_wcr16(ERDPT, enc28j60_rxrdpt);

		enc28j60_read_buffer((void*)&enc28j60_rxrdpt, sizeof(enc28j60_rxrdpt));
		enc28j60_read_buffer((void*)&rxlen, sizeof(rxlen));
		enc28j60_read_buffer((void*)&status, sizeof(status));

		if(status & 0x80) //success
		{
			len = rxlen - 4; //throw out crc
			if(len > buflen) len = buflen;
			enc28j60_read_buffer(buf, len);	
		}

		// Set Rx read pointer to next packet
		temp = (enc28j60_rxrdpt - 1) & ENC28J60_BUFEND;
		enc28j60_wcr16(ERXRDPT, temp);

		// Decrement packet counter
		enc28j60_bfs(ECON2, ECON2_PKTDEC);
	}

	return len;
}
