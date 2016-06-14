/*
 * Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>
 * Intel QMSI port: Copyright (C) 2016 Sergey Kiselev <skiselev@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */
 
#ifndef __RF24_CONFIG_H__
#define __RF24_CONFIG_H__

#include <qm_common.h>
#include <qm_gpio.h>
#include <qm_spi.h>
#include <qm_pinmux.h>
#include <clk.h>
#include <string.h>

/* Intel Quark D2000 / QMSI specific defines */
#define RF24_SPI_BUS	QM_SPI_MST_0
#define RF24_SPI_SS		QM_SPI_SS_0
#define RF24_SPI_CLOCK_DIV (32)	/* 1 MHz using 32 MHz system clock */

/*
 * CE	- F9 / Arduino header D8
 * CSN	- F0 / Arduino header D10
 * MOSI - Arduino header D11
 * MISO - Arduino header D12
 * SCK	- Arduino header D13
 */
#define RF24_GPIO_CSN	0	/* Arduino header 10 */
#define RF24_GPIO_CE	9	/* Arduino header 8 */

/* RF24 library configuration */  
#define RF24_FAILURE_HANDLING
/* #define RF24_SERIAL_DEBUG */
#define RF24_MINIMAL
/* #define RF24_SPI_MULTIBYTE */
  
#define rf24_min(a,b) (a<b?a:b)

#endif // __RF24_CONFIG_H__

