/*
 * Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>
 * Intel QMSI port: Copyright (C) 2016 Sergey Kiselev <skiselev@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include "nrf24l01.h"
#include "rf24_config.h"
#include "rf24.h"
#include <x86intrin.h>

/****************************************************************************
 * Enable or disable CE and CSN signals using GPIO
 * Notes:
 * - DesignWare SPI module doesn't handle slave select (CSN) correctly - 
 *   deactivates it when TX FIFO is empty, so we have to control it using GPIO
 * - CSN is active low, CE is active high
 ****************************************************************************/

#define rf24_ce_enable() qm_gpio_set_pin(QM_GPIO_0, RF24_GPIO_CE)
#define rf24_ce_disable() qm_gpio_clear_pin(QM_GPIO_0, RF24_GPIO_CE)
#define rf24_csn_enable() qm_gpio_clear_pin(QM_GPIO_0, RF24_GPIO_CSN)
#define rf24_csn_disable() qm_gpio_set_pin(QM_GPIO_0, RF24_GPIO_CSN)

/****************************************************************************
 * Get current time in milliseconds
 ****************************************************************************/
uint32_t rf24_get_msec()
{
	return _rdtsc() / (1000 * clk_sys_get_ticks_per_us());
}

/****************************************************************************
 * Send and receive a single byte using SPI bus
 * Note: This function does not control slave select signal, it needs to be
 *       activated/deactivate separately
 ****************************************************************************/
uint8_t rf24_spi_transfer(uint8_t in)
{
	qm_spi_transfer_t spi_xfer;
	qm_spi_status_t spi_status;
	uint8_t out;

	spi_xfer.tx = &in;
	spi_xfer.tx_len = 1;
	spi_xfer.rx = &out;
	spi_xfer.rx_len = 1;

	qm_spi_transfer(RF24_SPI_BUS, &spi_xfer, &spi_status);

	return out;
}

#if defined (RF24_SPI_MULTIBYTE)
/****************************************************************************
 * Send and receive multiple bytes using SPI bus in a single transaction
 ****************************************************************************/
uint8_t rf24_spi_transfernb(uint8_t *tx_buf, uint8_t *rx_buf, uint8_t len)
{
	qm_spi_transfer_t spi_xfer;
	qm_spi_status_t spi_status;

	spi_xfer.tx = tx_buf;
	spi_xfer.tx_len = len;
	spi_xfer.rx = rx_buf;
	spi_xfer.rx_len = len;
	
	rf24_csn_enable();
	qm_spi_transfer(RF24_SPI_BUS, &spi_xfer, &spi_status);
	rf24_csn_disable();

	return 0;
}

#endif

/****************************************************************************/

uint8_t rf24_read_register_buf(uint8_t reg, uint8_t* buf, uint8_t len)
{
	uint8_t status;

#if defined (RF24_SPI_MULTIBYTE)
	uint8_t * prx = spi_rxbuff;
	uint8_t * ptx = spi_txbuff;
	uint8_t size = len + 1; /* Add register value to transmit buffer */

	*ptx++ = (R_REGISTER | (REGISTER_MASK & reg));

	while (len--) {
		*ptx++ = NOP; /* Dummy operation, just for reading */
	}
	
	rf24_spi_transfernb(spi_txbuff, spi_rxbuff, size);
  
	status = *prx++; /* status is 1st byte of receive buffer */

	/* decrement before to skip status byte */
	while (--size) {
		*buf++ = *prx++;
	} 
#else
	rf24_csn_enable();
	status = rf24_spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
	while (len--) {
		*buf++ = rf24_spi_transfer(0xFF);
	}
	rf24_csn_disable();
#endif

	return status;
}

/****************************************************************************/

uint8_t rf24_read_register(uint8_t reg)
{
	uint8_t result;
  
#if defined (RF24_SPI_MULTIBYTE)
	uint8_t *prx = spi_rxbuff;
	uint8_t *ptx = spi_txbuff;	
	*ptx++ = (R_REGISTER | (REGISTER_MASK & reg));
	*ptx++ = NOP ; /* Dummy operation, just for reading */
  
	rf24_spi_transfernb(spi_txbuff, spi_rxbuff, 2);
	result = *++prx;   /* result is 2nd byte of receive buffer */
#else
	rf24_csn_enable(); 
	rf24_spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
	result = rf24_spi_transfer(0xFF);
	rf24_csn_disable();
#endif

	return result;
}

/****************************************************************************/

uint8_t rf24_write_register_buf(uint8_t reg, const uint8_t* buf, uint8_t len)
{
	uint8_t status;

#if defined (RF24_SPI_MULTIBYTE) 
	uint8_t * prx = spi_rxbuff;
	uint8_t * ptx = spi_txbuff;
	uint8_t size = len + 1; /* Add register value to transmit buffer */

	*ptx++ = (W_REGISTER | (REGISTER_MASK & reg));
	while (len--) {
		*ptx++ = *buf++;
	}
  
	rf24_spi_transfernb(spi_txbuff, spi_rxbuff, size);
	status = *prx; /* status is 1st byte of receive buffer */
#else
	rf24_csn_enable();
	status = rf24_spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
	while (len--) {
		rf24_spi_transfer(*buf++);
	}
	rf24_csn_disable();
#endif

	return status;
}

/****************************************************************************/

uint8_t rf24_write_register(uint8_t reg, uint8_t value)
{
	uint8_t status;

#if defined (RF24_SERIAL_DEBUG)
	QM_PRINTF("write_register(0x%x,0x%x)\r\n", reg, value);
#endif

#if defined (RF24_SPI_MULTIBYTE)
	uint8_t * prx = spi_rxbuff;
	uint8_t * ptx = spi_txbuff;
	*ptx++ = (W_REGISTER | (REGISTER_MASK & reg));
	*ptx = value ;	
  	
	rf24_spi_transfernb(spi_txbuff, spi_rxbuff, 2);
	status = *prx++; /* status is 1st byte of receive buffer */
#else
    rf24_csn_enable();
	status = rf24_spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
	rf24_spi_transfer(value);
	rf24_csn_disable();
#endif

	return status;
}

/****************************************************************************/

uint8_t rf24_write_payload(const void* buf, uint8_t data_len, const uint8_t write_type)
{
	uint8_t status;
	const uint8_t* current = (const uint8_t *)(buf);

	data_len = rf24_min(data_len, rf24_payload_size);
	uint8_t blank_len = rf24_dynamic_payloads_enabled ? 0 : rf24_payload_size - data_len;
  
#if defined (RF24_SERIAL_DEBUG)
	QM_PRINTF("[Writing %u bytes %u blanks]\r\n", data_len, blank_len);
#endif

#if defined (RF24_SPI_MULTIBYTE)
	uint8_t * prx = spi_rxbuff;
	uint8_t * ptx = spi_txbuff;
    uint8_t size;
	size = data_len + blank_len + 1; /* Add register value to transmit buffer */

	*ptx++ = write_type;
    while (data_len--) {
		*ptx++ = *current++;
	}
    while (blank_len--) {
		*ptx++ = 0;
	}
	
	rf24_spi_transfernb(spi_txbuff, spi_rxbuff, size);
	status = *prx; /* status is 1st byte of receive buffer */
#else
	rf24_csn_enable();	
	status = rf24_spi_transfer(write_type);
	while (data_len--) {
		rf24_spi_transfer(*current++);
	}
	while (blank_len--) {
		rf24_spi_transfer(0);
	}  
	rf24_csn_disable();
#endif

	return status;
}

/****************************************************************************/

uint8_t rf24_read_payload(void* buf, uint8_t data_len)
{
	uint8_t status;
	uint8_t* current = (uint8_t *)(buf);

	if(data_len > rf24_payload_size) data_len = rf24_payload_size;
	uint8_t blank_len = rf24_dynamic_payloads_enabled ? 0 : rf24_payload_size - data_len;

#if defined (RF24_SERIAL_DEBUG)
	QM_PRINTF("[Reading %u bytes %u blanks]\r\n", data_len, blank_len);
#endif
	
#if defined (RF24_SPI_MULTIBYTE)
	uint8_t * prx = spi_rxbuff;
	uint8_t * ptx = spi_txbuff;
    uint8_t size;
    size = data_len + blank_len + 1; /* Add register value to transmit buffer */

	*ptx++ =  R_RX_PAYLOAD;
	while(--size) {
		*ptx++ = NOP;
	}
		
	size = data_len + blank_len + 1; /* Size has been lost during while, re affect */
	
	rf24_spi_transfernb(spi_txbuff, spi_rxbuff, size);
	
	status = *prx++; /* 1st byte is status */
    
    if (data_len > 0) {
		while ( --data_len ) { /* Decrement before to skip 1st status byte */
			*current++ = *prx++;
		}
		*current = *prx;
    }
#else
	rf24_csn_enable();
	status = rf24_spi_transfer(R_RX_PAYLOAD);
	while (data_len--) {
		*current++ = rf24_spi_transfer(0xFF);
	}
	while (blank_len--) {
		rf24_spi_transfer(0xFF);
	}
	rf24_csn_disable();
#endif

	return status;
}
/****************************************************************************/

uint8_t rf24_spi_send_cmd(uint8_t cmd)
{
	uint8_t status;
	rf24_csn_enable();
	status = rf24_spi_transfer(cmd);
	rf24_csn_disable();

	return status;
}

/****************************************************************************/

uint8_t rf24_flush_rx(void)
{
	return rf24_spi_send_cmd(FLUSH_RX);
}

/****************************************************************************/

uint8_t rf24_flush_tx(void)
{
	return rf24_spi_send_cmd(FLUSH_TX);
}

/****************************************************************************/

uint8_t rf24_get_status(void)
{
	return rf24_spi_send_cmd(NOP);
}

/****************************************************************************/
#if !defined (RF24_MINIMAL)
void rf24_print_status(uint8_t status)
{
	QM_PRINTF("STATUS\t\t = 0x%x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=0x%x TX_FULL=%x\r\n",
			status,
			(status & (1 << RX_DR)) ? 1 : 0,
			(status & (1 << TX_DS)) ? 1 : 0,
			(status & (1 << MAX_RT)) ? 1 : 0,
			((status >> RX_P_NO) & 0b111),
			(status & (1 << TX_FULL)) ? 1 : 0
	);
}

/****************************************************************************/

void rf24_print_observe_tx(uint8_t value)
{
	QM_PRINTF("OBSERVE_TX=0x%x: POLS_CNT=0x%x ARC_CNT=0x%x\r\n",
			value,
			(value >> PLOS_CNT) & 0b1111,
			(value >> ARC_CNT) & 0b1111
	);
}

/****************************************************************************/

void rf24_print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
	uint8_t value;
    QM_PRINTF("%s\t =", name);

	while (qty--) {
		value = rf24_read_register(reg++);
		/* QM_PRINTF does not support padding - add zero for single digit numbers */
		if (value < 0x10) {
			QM_PRINTF(" 0x0%x", value);
		} else {
			QM_PRINTF(" 0x%x", value);
		}
	}
	QM_PRINTF("\r\n");
}

/****************************************************************************/

void rf24_print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
    QM_PRINTF("%s\t =",name);
	while (qty--) {
		uint8_t buffer[rf24_addr_width];
		rf24_read_register_buf(reg++, buffer, sizeof(buffer));

		QM_PRINTF(" 0x");
		uint8_t* bufptr = buffer + sizeof buffer;
		while(--bufptr >= buffer) {
			/* QM_PRINTF does not support padding - add zero for single digit numbers */
			if (*bufptr < 0x10) {
				QM_PRINTF("0");
			}
			QM_PRINTF("%x", *bufptr);
		}
  }

  QM_PRINTF("\r\n");
}
#endif

/****************************************************************************/

void rf24_set_channel(uint8_t channel)
{
	const uint8_t max_channel = 125;
	rf24_write_register(RF_CH, rf24_min(channel, max_channel));
}

uint8_t rf24_get_channel()
{
	return rf24_read_register(RF_CH);
}
/****************************************************************************/

void rf24_set_payload_size(uint8_t size)
{
	rf24_payload_size = rf24_min(size, 32);
}

/****************************************************************************/

uint8_t rf24_get_payload_size(void)
{
	return rf24_payload_size;
}

/****************************************************************************/

#if !defined (RF24_MINIMAL)

static const char * const rf24_datarate_e_str_p[] = {
	"1MBPS", "2MBPS", "250KBPS"
};
static const char * const rf24_model_e_str_p[] = {
	"nRF24L01", "nRF24L01+"
};
static const char * const rf24_crclength_e_str_p[] = {
	"Disabled", "8 bits", "16 bits"
};

static const char * const rf24_pa_dbm_e_str_p[] = {
	"PA_MIN", "PA_LOW", "PA_HIGH", "PA_MAX"
};

void rf24_print_details(void)
{
	rf24_print_status(rf24_get_status());

	rf24_print_address_register("RX_ADDR_P0-1", RX_ADDR_P0, 2);
	rf24_print_byte_register("RX_ADDR_P2-5", RX_ADDR_P2, 4);
	rf24_print_address_register("TX_ADDR\t", TX_ADDR, 1);

	rf24_print_byte_register("RX_PW_P0-6", RX_PW_P0, 6);
	rf24_print_byte_register("EN_AA\t", EN_AA, 1);
	rf24_print_byte_register("EN_RXADDR", EN_RXADDR, 1);
	rf24_print_byte_register("RF_CH\t", RF_CH, 1);
	rf24_print_byte_register("RF_SETUP", RF_SETUP, 1);
	rf24_print_byte_register("CONFIG\t", NRF_CONFIG, 1);
	rf24_print_byte_register("DYNPD/FEATURE", DYNPD, 2);

	QM_PRINTF("Data Rate\t = %s\r\n", rf24_datarate_e_str_p[rf24_get_data_rate()]);
	QM_PRINTF("Model\t\t = %s\r\n", rf24_model_e_str_p[rf24_is_plus()]);
	QM_PRINTF("CRC Length\t = %s\r\n", rf24_crclength_e_str_p[rf24_get_crc_length()]);
	QM_PRINTF("PA Power\t = %s\r\n", rf24_pa_dbm_e_str_p[rf24_get_pa_level()]);
}

#endif
/****************************************************************************/

bool rf24_init(void)
{
	uint8_t setup = 0;

	/* Initialize SPI/GPIOs on Intel Quark MCU */
	qm_spi_config_t spi_cfg;
  	qm_gpio_port_config_t gpio_cfg;

  	/* Initialize GPIO pins */
	gpio_cfg.direction = BIT(RF24_GPIO_CE);
	gpio_cfg.direction |= BIT(RF24_GPIO_CSN);
	
	gpio_cfg.int_en = 0x0;
	gpio_cfg.int_type = 0x0;
	gpio_cfg.int_polarity = 0x0;
	gpio_cfg.int_debounce = 0x0;
	gpio_cfg.int_bothedge = 0x0;
	gpio_cfg.callback = NULL;
	
	qm_gpio_set_config(QM_GPIO_0, &gpio_cfg);

	/* Use GPIO for slave select due to bug in DesignWare SPI */
	/* qm_pmux_select(QM_PIN_ID_0, QM_PMUX_FN_2);*/  /* SS0 */
	qm_pmux_select(QM_PIN_ID_16, QM_PMUX_FN_2); /* SCK */
	qm_pmux_select(QM_PIN_ID_17, QM_PMUX_FN_2); /* TXD */
	qm_pmux_select(QM_PIN_ID_18, QM_PMUX_FN_2); /* RXD */
	qm_pmux_input_en(QM_PIN_ID_18, true);       /* RXD input */
	
	/*  Set SPI configuration */
	spi_cfg.frame_size = QM_SPI_FRAME_SIZE_8_BIT;
	spi_cfg.transfer_mode = QM_SPI_TMOD_TX_RX;
	spi_cfg.bus_mode = QM_SPI_BMODE_0;
	spi_cfg.clk_divider = RF24_SPI_CLOCK_DIV;
	/*clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_SPI_M0_REGISTER);*/
	qm_spi_set_config(RF24_SPI_BUS, &spi_cfg);
	qm_spi_slave_select(RF24_SPI_BUS, RF24_SPI_SS);

	/* set defaults */
	rf24_payload_size = 32;
	rf24_dynamic_payloads_enabled = false;
	rf24_addr_width = 5;
	pipe0_reading_address[0] = 0;
	
	/* disable CE and CSN */
	
    rf24_ce_disable();
  	rf24_csn_disable();

	/*
	 * Must allow the radio time to settle else configuration bits will not necessarily stick.
	 * This is actually only required following power up but some settling time also appears to
	 * be required after resets too. For full coverage, we'll always assume the worst.
	 * Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
	 * Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
	 * WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
	 */
	clk_sys_udelay(5000);

	/* Reset NRF_CONFIG and enable 16-bit CRC. */
	rf24_write_register(NRF_CONFIG, 0b00001100);

	/*
	 * Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
	 * WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
	 * sizes must never be used. See documentation for a more complete explanation.
	 */
	rf24_set_retries(5, 15);

	/* Reset value is MAX */
	/* rf24_set_pa_level(RF24_PA_MAX); */

	/* check for connected module and if this is a p nRF24l01 variant */
	if (rf24_set_data_rate(RF24_250KBPS)) {
		rf24_plus = true;
	}
	setup = rf24_read_register(RF_SETUP);
  
	/* Set the data rate to the slowest (and most reliable) speed
	 * supported by all hardware.
	 */
	rf24_set_data_rate(RF24_1MBPS);

	/* Disable dynamic payloads, to match rf24_dynamic_payloads_enabled setting - Reset value is 0 */
	rf24_toggle_features();
	rf24_write_register(FEATURE, 0);
	rf24_write_register(DYNPD, 0);

	/*
	 * Reset current status
	 * Notice reset and flush is the last thing we do
	 */
	rf24_write_register(NRF_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

	/*
	 * Set up default configuration.  Callers can always change it later.
	 * This channel should be universally safe and not bleed over into adjacent
	 * spectrum.
	 */
	rf24_set_channel(76);

	/* Flush buffers */
	rf24_flush_rx();
	rf24_flush_tx();

	rf24_power_up(); /* Power up by default when rf24_init() is called */

	/* Enable PTX, do not write CE high so radio will remain in standby I mode
	 * (130us max to transition to RX or TX instead of 1500us from powerUp)
	 * PTX should use only 22uA of power
	 */
	rf24_write_register(NRF_CONFIG, rf24_read_register(NRF_CONFIG) & ~(1 << PRIM_RX));

	/* if setup is 0 or 0xFF then there was no response from module */
	/* FIXME: why not to check this in the beginning, and exit immediately?! */
	return !(setup != 0 && setup != 0xFF);
}

/****************************************************************************/

void rf24_start_listening(void)
{
	rf24_power_up();
	rf24_write_register(NRF_CONFIG, rf24_read_register(NRF_CONFIG) | (1 << PRIM_RX));
	rf24_write_register(NRF_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
	rf24_ce_enable();
	/* Restore the pipe0 address, if exists */
	if (pipe0_reading_address[0] > 0) {
		rf24_write_register_buf(RX_ADDR_P0, pipe0_reading_address, rf24_addr_width);
	} else {
		rf24_close_reading_pipe(0);
	}

	/* flush buffers */
	if (rf24_read_register(FEATURE) & (1 << EN_ACK_PAY)) {
		rf24_flush_tx();
	}
}

/****************************************************************************/
static const uint8_t child_pipe_enable[] =
	{ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5};

void rf24_stop_listening(void)
{  
	rf24_ce_disable();

	clk_sys_udelay(rf24_tx_rx_delay);
  
	if (rf24_read_register(FEATURE) & (1 << EN_ACK_PAY)) {
		clk_sys_udelay(rf24_tx_rx_delay); /* 200 */
		rf24_flush_tx();
	}

	rf24_write_register(NRF_CONFIG, rf24_read_register(NRF_CONFIG) & ~(1 << PRIM_RX));
	rf24_write_register(EN_RXADDR, rf24_read_register(EN_RXADDR) | (1 << child_pipe_enable[0])); /* Enable RX on pipe0 */
}

/****************************************************************************/

void rf24_power_down(void)
{
	rf24_ce_disable(); /* Guarantee CE is low on rf24_power_down */
	rf24_write_register(NRF_CONFIG, rf24_read_register(NRF_CONFIG) & ~(1 << PWR_UP));
}

/****************************************************************************/

/* Power up now. Radio will not power down unless instructed by MCU for config changes etc. */
void rf24_power_up(void)
{
	uint8_t cfg = rf24_read_register(NRF_CONFIG);

	/* if not powered up then power up and wait for the radio to initialize */
	if (!(cfg & (1 << PWR_UP))) {
		rf24_write_register(NRF_CONFIG, cfg | (1 << PWR_UP));

		/*
		 * For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
		 * There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
		 * the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
		 */
		clk_sys_udelay(5000);
	}
}

/******************************************************************/
#if defined (RF24_FAILURE_HANDLING)
void rf24_err_notify()
{
#if defined (RF24_SERIAL_DEBUG)
	QM_PRINTF("RF24 HARDWARE FAIL: Radio not responding, verify pin connections, wiring, etc.\r\n");
#endif
	rf24_failure_detected = 1;
}
#endif
/******************************************************************/

/* Similar to the previous write, clears the interrupt flags */
bool rf24_write_multicast(const void* buf, uint8_t len, const bool multicast)
{
	/* Start Writing */
	rf24_start_fast_write(buf, len, multicast, 1);

	/* Wait until complete or failed */
#if defined (RF24_FAILURE_HANDLING)
	uint32_t timer = rf24_get_msec();
#endif 
	
	while (!(rf24_get_status() & ((1 << TX_DS) | (1 << MAX_RT)))) { 
#if defined (RF24_FAILURE_HANDLING)
		if (rf24_get_msec() - timer > 95) {
			rf24_err_notify();
			return 0;		
		}
#endif
	}
    
	rf24_ce_disable();

	uint8_t status = rf24_write_register(NRF_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

	/* Max retries exceeded */
	if (status & (1 << MAX_RT)) {
		rf24_flush_tx(); /* Only going to be 1 packet int the FIFO at a time using this method, so just flush */
		return 0;
	}
	//TX OK 1 or 0
	return 1;
}

bool rf24_write(const void* buf, uint8_t len)
{
	return rf24_write_multicast(buf, len, 0);
}
/****************************************************************************/

/* For general use, the interrupt flags are not important to clear */
bool rf24_write_blocking(const void* buf, uint8_t len, uint32_t timeout)
{
	/*
	 * Block until the FIFO is NOT full.
	 * Keep track of the MAX retries and set auto-retry if seeing failures
	 * This way the FIFO will fill up and allow blocking until packets go through
	 * The radio will auto-clear everything in the FIFO as long as CE remains high
	 */
	uint32_t timer = rf24_get_msec();				/* Get the time that the payload transmission started */

	while(rf24_get_status() & (1 << TX_FULL)) {		/* Blocking only if FIFO is full. This will loop and block until TX is successful or timeout */
		if(rf24_get_status() & (1 << MAX_RT)) {		/* If MAX Retries have been reached */
			rf24_reuse_tx();						/* Set re-transmit and clear the MAX_RT interrupt flag */
			if(rf24_get_msec() - timer > timeout) {
				return 0;							/* If this payload has exceeded the user-defined timeout, exit and return 0 */
			}
		}
#if defined (RF24_FAILURE_HANDLING)
		if(rf24_get_msec() - timer > (timeout + 95)) {			
			rf24_err_notify();
			return 0;			
		}
#endif
  	}

  	//Start Writing
	rf24_start_fast_write(buf, len, 0, 1);			/* Write the payload if a buffer is clear */

	return 1;										/* Return 1 to indicate successful transmission */
}

/****************************************************************************/

void rf24_reuse_tx()
{
	rf24_write_register(NRF_STATUS, 1 << MAX_RT);	/* Clear max retry flag */
	rf24_spi_send_cmd(REUSE_TX_PL);
	rf24_ce_disable();									/* Re-Transfer packet */
	rf24_ce_enable();
}

/****************************************************************************/

bool rf24_write_fast_multicast(const void* buf, uint8_t len, const bool multicast)
{
	/*
	 * Block until the FIFO is NOT full.
	 * Keep track of the MAX retries and set auto-retry if seeing failures
	 * Return 0 so the user can control the retrys and set a timer or failure counter if required
	 * The radio will auto-clear everything in the FIFO as long as CE remains high
     */
#if defined (RF24_FAILURE_HANDLING)
	uint32_t timer = rf24_get_msec();
#endif
	
	while((rf24_get_status() & (1 << TX_FULL))) {	/* Blocking only if FIFO is full. This will loop and block until TX is successful or fail */

		if(rf24_get_status() & (1 << MAX_RT)) {
			rf24_write_register(NRF_STATUS, 1 << MAX_RT);	/* Clear max retry flag */
			return 0;										/* Return 0. The previous payload has been retransmitted */
															/* From the user perspective, if you get a 0, just keep trying to send the same payload */
		}
#if defined (RF24_FAILURE_HANDLING)
		if(rf24_get_msec() - timer > 95 ){			
			rf24_err_notify();
			return 0;							
		}
#endif
  	}
	/* Start Writing */
	rf24_start_fast_write(buf, len, multicast, 1);

	return 1;
}

bool rf24_write_fast(const void* buf, uint8_t len)
{
	return rf24_write_fast_multicast(buf, len, 0);
}

/****************************************************************************/

/*
 * Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
 * In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
 * Otherwise we enter Standby-II mode, which is still faster than standby mode
 * Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data
 */
void rf24_start_fast_write(const void* buf, uint8_t len, const bool multicast, bool start_tx)
{
	rf24_write_payload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
	if(start_tx){
		rf24_ce_enable();
	}
}

/****************************************************************************/

/*
 * Added the original rf24_start_write back in so users can still use interrupts, ack payloads, etc
 * Allows the library to pass all tests
 */
void rf24_start_write(const void* buf, uint8_t len, const bool multicast)
{
	/* Send the payload */

	rf24_write_payload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD) ;
	rf24_ce_enable();
	clk_sys_udelay(10); /* FIXME: is it needed? Better be safe than sorry?! */
	rf24_ce_disable();
}

/****************************************************************************/

bool rf24_rx_fifo_full()
{
	return rf24_read_register(FIFO_STATUS) & (1 << RX_FULL);
}
/****************************************************************************/

bool rf24_tx_standby()
{
#if defined (RF24_FAILURE_HANDLING)
	uint32_t timeout = rf24_get_msec();
#endif
	while(!(rf24_read_register(FIFO_STATUS) & (1 << TX_EMPTY))) {
		if (rf24_get_status() & (1 << MAX_RT)){
			rf24_write_register(NRF_STATUS, 1 << MAX_RT);
			rf24_ce_disable();
			rf24_flush_tx();    /* Non blocking, flush the data */
			return 0;
		}
#if defined (RF24_FAILURE_HANDLING)
		if(rf24_get_msec() - timeout > 95){
			rf24_err_notify();
			return 0;	
		}
#endif
	}

	rf24_ce_disable();			   /* Set STANDBY-I mode */
	return 1;
}

/****************************************************************************/

bool rf24_tx_standby_timeout(uint32_t timeout, bool start_tx)
{
    if(start_tx){
	  rf24_stop_listening();
	  rf24_ce_enable();
	}
	uint32_t start = rf24_get_msec();

	while (!(rf24_read_register(FIFO_STATUS) & (1 << TX_EMPTY))) {
		if (rf24_get_status() & (1 << MAX_RT)) {
			rf24_write_register(NRF_STATUS, 1 << MAX_RT);
				rf24_ce_disable();								/* Set re-transmit */
				rf24_ce_enable();
				if (rf24_get_msec() - start >= timeout) {
					rf24_ce_disable();
					rf24_flush_tx();
					return 0;
				}
		}
#if defined (RF24_FAILURE_HANDLING)
		if (rf24_get_msec() - start > (timeout+95)) {
			rf24_err_notify();
			return 0;	
		}
#endif
	}

	rf24_ce_disable();				   /* Set STANDBY-I mode */
	return 1;
}

/****************************************************************************/

void rf24_mask_irq(bool tx, bool fail, bool rx){

	uint8_t config = rf24_read_register(NRF_CONFIG);
	/* clear the interrupt flags */
	config &= ~(1 << MASK_MAX_RT | 1 << MASK_TX_DS | 1 << MASK_RX_DR);
	/* set the specified interrupt flags */
	config |= fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR;
	rf24_write_register(NRF_CONFIG, config);
}

/****************************************************************************/

uint8_t rf24_get_dynamic_payload_size(void)
{
	uint8_t result = 0;

#if defined (RF24_SPI_MULTIBYTE)  
	spi_txbuff[0] = R_RX_PL_WID;
	spi_rxbuff[1] = 0xFF;

	rf24_spi_transfernb(spi_txbuff, spi_rxbuff, 2);
	result = spi_rxbuff[1];  
#else
	rf24_csn_enable();
	rf24_spi_transfer(R_RX_PL_WID);
	result = rf24_spi_transfer(0xFF);
	rf24_csn_disable();
#endif

	if (result > 32) {
		rf24_flush_rx();
		clk_sys_udelay(2000);
		return 0;
	}
	return result;
}

/****************************************************************************/

bool rf24_available(void)
{
	return rf24_available_pipe(NULL);
}

/****************************************************************************/

bool rf24_available_pipe(uint8_t* pipe_num)
{
	if (!(rf24_read_register(FIFO_STATUS) & (1 << RX_EMPTY))) {

		/* If the caller wants the pipe number, include that */
		if (pipe_num) {
			uint8_t status = rf24_get_status();
			*pipe_num = (status >> RX_P_NO) & 0b111;
		}
		return 1;
	}

	return 0;
}

/****************************************************************************/

void rf24_read(void* buf, uint8_t len)
{
	/* Fetch the payload */
	rf24_read_payload(buf, len);

	/* Clear the two possible interrupt flags with one command */
	rf24_write_register(NRF_STATUS, (1 << RX_DR) | (1 << MAX_RT) | (1 << TX_DS));
}

/****************************************************************************/

void rf24_what_happened(bool *tx_ok, bool *tx_fail, bool *rx_ready)
{
	/* Read the status & reset the status in one easy call
	 * Or is that such a good idea?
	 */
	uint8_t status = rf24_write_register(NRF_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

	/* Report to the user what happened */
	*tx_ok = status & (1 << TX_DS);
	*tx_fail = status & (1 << MAX_RT);
	*rx_ready = status & (1 << RX_DR);
}

/****************************************************************************/

void rf24_open_writing_pipe_uint64(uint64_t value)
{
	rf24_write_register_buf(RX_ADDR_P0, (uint8_t*)(&value), rf24_addr_width);
	rf24_write_register_buf(TX_ADDR, (uint8_t*)(&value), rf24_addr_width);
	rf24_write_register(RX_PW_P0, rf24_payload_size);
}

/****************************************************************************/
void rf24_open_writing_pipe(const uint8_t *address)
{
	rf24_write_register_buf(RX_ADDR_P0, address, rf24_addr_width);
	rf24_write_register_buf(TX_ADDR, address, rf24_addr_width);
	rf24_write_register(RX_PW_P0, rf24_payload_size);
}

/****************************************************************************/
static const uint8_t child_pipe[] = {
	RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] = {
	RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};

void rf24_open_reading_pipe_uint64(uint8_t child, uint64_t address)
{
	/* If this is pipe 0, cache the address.  This is needed because
	 * rf24_open_writing_pipe() will overwrite the pipe 0 address, so
	 * rf24_start_listening() will have to restore it.
	 */
	if (child == 0) {
		memcpy(pipe0_reading_address, &address, rf24_addr_width);
	}

	if (child <= 6) {
		/* For pipes 2-5, only write the LSB */
		if (child < 2) {
			rf24_write_register_buf(child_pipe[child], (const uint8_t*)(&address), rf24_addr_width);
		} else {
			rf24_write_register_buf(child_pipe[child], (const uint8_t*)(&address), 1);
		}

		rf24_write_register(child_payload_size[child], rf24_payload_size);

		/* Note it would be more efficient to set all of the bits for all open
		 * pipes at once.  However, I thought it would make the calling code
		 * more simple to do it this way.
		 */
		rf24_write_register(EN_RXADDR, rf24_read_register(EN_RXADDR) | (1 << child_pipe_enable[child]));
	}
}

/****************************************************************************/
void rf24_set_address_width(uint8_t a_width) {
	if(a_width -= 2){
		rf24_write_register(SETUP_AW, a_width % 4);
		rf24_addr_width = (a_width % 4) + 2;
	}
}

/****************************************************************************/

void rf24_open_reading_pipe(uint8_t child, const uint8_t *address)
{
	/* If this is pipe 0, cache the address.  This is needed because
	 * rf24_open_writing_pipe() will overwrite the pipe 0 address, so
	 * rf24_start_listening() will have to restore it.
	 */
	if (child == 0) {
		memcpy(pipe0_reading_address, address, rf24_addr_width);
	}
	if (child <= 6) {
		/* For pipes 2-5, only write the LSB */
		if (child < 2){
			rf24_write_register_buf(child_pipe[child], address, rf24_addr_width);
		} else {
			rf24_write_register_buf(child_pipe[child], address, 1);
		}
		rf24_write_register(child_payload_size[child], rf24_payload_size);

		/* Note it would be more efficient to set all of the bits for all open
		 * pipes at once.  However, I thought it would make the calling code
		 * more simple to do it this way.
		 */
		rf24_write_register(EN_RXADDR, rf24_read_register(EN_RXADDR) | (1 << child_pipe_enable[child]));
	}
}

/****************************************************************************/

void rf24_close_reading_pipe(uint8_t pipe)
{
	rf24_write_register(EN_RXADDR, rf24_read_register(EN_RXADDR) & ~(1 << child_pipe_enable[pipe]));
}

/****************************************************************************/

void rf24_toggle_features(void)
{
    rf24_csn_enable();
	rf24_spi_transfer(ACTIVATE);
    rf24_spi_transfer(0x73);
	rf24_csn_disable();
}

/****************************************************************************/

void rf24_enable_dynamic_payloads(void)
{
	/* Enable dynamic payload throughout the system */
	/* toggle_features(); */
	rf24_write_register(FEATURE, rf24_read_register(FEATURE) | (1 << EN_DPL));

#if defined (RF24_SERIAL_DEBUG)
	QM_PRINTF("FEATURE=0x%x\r\n", rf24_read_register(FEATURE));
#endif

	/* Enable dynamic payload on all pipes
	 *
	 * Not sure the use case of only having dynamic payload on certain
	 * pipes, so the library does not support it.
	 */
	rf24_write_register(DYNPD, rf24_read_register(DYNPD) | (1 << DPL_P5) | (1 << DPL_P4) | (1 << DPL_P3) | (1 << DPL_P2) | (1 << DPL_P1) | (1 << DPL_P0));

	rf24_dynamic_payloads_enabled = true;
}

/****************************************************************************/

void rf24_enable_ack_payload(void)
{
	/* enable ack payload and dynamic payload features */
	/* toggle_features(); */
	rf24_write_register(FEATURE, rf24_read_register(FEATURE) | (1 << EN_ACK_PAY) | (1 << EN_DPL));

#if defined (RF24_SERIAL_DEBUG)
	QM_PRINTF("FEATURE=%x\r\n", rf24_read_register(FEATURE));
#endif

	/* Enable dynamic payload on pipes 0 & 1 */
	rf24_write_register(DYNPD, rf24_read_register(DYNPD) | (1 << DPL_P1) | (1 << DPL_P0));
	rf24_dynamic_payloads_enabled = true;
}

/****************************************************************************/

void rf24_enable_dynamic_ack(void)
{
	/* enable dynamic ack features */
	/* toggle_features(); */
	rf24_write_register(FEATURE, rf24_read_register(FEATURE) | (1 << EN_DYN_ACK));

#if defined (RF24_SERIAL_DEBUG)
	QM_PRINTF("FEATURE=%x\r\n", rf24_read_register(FEATURE));
#endif
}

/****************************************************************************/

void rf24_write_ack_payload(uint8_t pipe, const void* buf, uint8_t len)
{
	const uint8_t* current = (const uint8_t *)(buf);

	uint8_t data_len = rf24_min(len, 32);

	rf24_csn_enable();
#if defined (RF24_SPI_MULTIBYTE)
	uint8_t * ptx = spi_txbuff;
	uint8_t size = data_len + 1 ; // Add register value to transmit buffer
	*ptx++ =  W_ACK_PAYLOAD | ( pipe & 0b111 );
	while (data_len--) {
		*ptx++ =  *current++;
	}
	
	rf24_spi_transfernb(spi_txbuff, spi_rxbuff, size);
#else
	rf24_spi_transfer(W_ACK_PAYLOAD | ( pipe & 0b111 ) );

	while ( data_len-- ) {
		rf24_spi_transfer(*current++);
	}
#endif
	rf24_csn_disable();
}

/****************************************************************************/

bool rf24_is_ack_payload_available(void)
{
	return !(rf24_read_register(FIFO_STATUS) & (1 << RX_EMPTY));
}

/****************************************************************************/

bool rf24_is_plus(void)
{
	return rf24_plus;
}

/****************************************************************************/

void rf24_set_auto_ack_all(bool enable)
{
	if (enable) {
		rf24_write_register(EN_AA, 0b111111);
	} else {
		rf24_write_register(EN_AA, 0);
	}
}

/****************************************************************************/

void rf24_set_auto_ack(uint8_t pipe, bool enable)
{
	if (pipe <= 6) {
		uint8_t en_aa = rf24_read_register(EN_AA) ;
		if (enable) {
			en_aa |= 1 << pipe;
		} else {
			en_aa &= ~(1 << pipe);
		}
		rf24_write_register(EN_AA, en_aa) ;
	}
}

/****************************************************************************/

bool rf24_test_carrier(void)
{
	return rf24_read_register(CD) & 0x01;
}

/****************************************************************************/

bool rf24_test_rpd(void)
{
	return rf24_read_register(RPD) & 0x01;
}

/****************************************************************************/

void rf24_set_pa_level(uint8_t level)
{
	uint8_t setup = rf24_read_register(RF_SETUP) & 0b11111000;

	if(level > 3) {						/* If invalid level, go to max PA */
		level = (RF24_PA_MAX << 1) + 1;	/* +1 to support the SI24R1 chip extra bit */
	} else {
		level = (level << 1) + 1;		/* Else set level as requested */
	}

	rf24_write_register(RF_SETUP, setup |= level);	/* Write it to the chip */
}

/****************************************************************************/

uint8_t rf24_get_pa_level(void)
{
	return (rf24_read_register(RF_SETUP) & ((1 << RF_PWR_LOW) | (11 << RF_PWR_HIGH))) >> 1;
}

/****************************************************************************/

bool rf24_set_data_rate(rf24_datarate_e speed)
{
	bool result = false;
	uint8_t setup = rf24_read_register(RF_SETUP) ;

	/* HIGH and LOW '00' is 1Mbs - our default */
	setup &= ~((1 << RF_DR_LOW) | (1 << RF_DR_HIGH)) ;
  
	rf24_tx_rx_delay = 250; /* FIXME: 85 for 16 MHz Arduino */
	if (speed == RF24_250KBPS) {
		/* Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0 */
		/* Making it '10'. */
		setup |= 1 << RF_DR_LOW;
		rf24_tx_rx_delay = 450; /* FIXME: 155 for 16 MHz Arduino */
	} else {
		/* Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1 */
		/* Making it '01' */
		if (speed == RF24_2MBPS) {
			setup |= 1 << RF_DR_HIGH;
			rf24_tx_rx_delay = 190; /* FIXME: 65 for 16 MHz Arduino */
		}
	}
	rf24_write_register(RF_SETUP, setup);

	/* Verify our result */
	if (rf24_read_register(RF_SETUP) == setup) {
		result = true;
	}
	return result;
}

/****************************************************************************/

rf24_datarate_e rf24_get_data_rate( void )
{
	rf24_datarate_e result ;
	uint8_t dr = rf24_read_register(RF_SETUP) & ((1 << RF_DR_LOW) | (1 << RF_DR_HIGH));

	/* Order matters in our case below */
	if (dr == 1 << RF_DR_LOW) {
		/* '10' = 250KBPS */
		result = RF24_250KBPS ;
	} else if (dr == 1 << RF_DR_HIGH) {
		/* '01' = 2MBPS */
		result = RF24_2MBPS ;
	} else {
		/* '00' = 1MBPS */
		result = RF24_1MBPS ;
	}
	return result ;
}

/****************************************************************************/

void rf24_set_crc_length(rf24_crclength_e length)
{
	uint8_t config = rf24_read_register(NRF_CONFIG) & ~((1 << CRCO) | (1 << EN_CRC));

	if (length == RF24_CRC_DISABLED) {
		/* Do nothing, we turned it off above. */
	} else if (length == RF24_CRC_8) {
		config |= 1 << EN_CRC;
	} else {
		config |= (1 << EN_CRC) | (1 << CRCO);
	}
	rf24_write_register(NRF_CONFIG, config) ;
}

/****************************************************************************/

rf24_crclength_e rf24_get_crc_length(void)
{
	rf24_crclength_e result = RF24_CRC_DISABLED;
  
	uint8_t config = rf24_read_register(NRF_CONFIG) & (1 << CRCO | 1 << EN_CRC);
	uint8_t AA = rf24_read_register(EN_AA);
  
	if (config & (1 << EN_CRC) || AA) {
		if (config & (1 << CRCO)) {
			result = RF24_CRC_16;
		} else {
			result = RF24_CRC_8;
		}
	}

	return result;
}

/****************************************************************************/

void rf24_disable_crc( void )
{
	uint8_t disable = rf24_read_register(NRF_CONFIG) & ~(1 << EN_CRC) ;
	rf24_write_register(NRF_CONFIG, disable) ;
}

/****************************************************************************/
void rf24_set_retries(uint8_t delay, uint8_t count)
{
	rf24_write_register(SETUP_RETR, (delay & 0x0F) << ARD | (count & 0x0F) << ARC);
}
