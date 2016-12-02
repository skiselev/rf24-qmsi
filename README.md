# rf24-qmsi - A port of RF24 library for Intel QMSI

## Overview

This driver provides support for Nordic* nRF24L01 and compatible radios with Intel QMSI. The API is similar to the original
Arduino library, although it had been converted to C-style (from C++/Arduino style).

## Usage Instructions

### Software Dependencies

Intel(r) System Studio for Microcontrollers needs to be installed and configured on the host system. Refer to Intel System Studio 2016 for Microcontrollers User and Reference Guide for more information.

### Integrating rf24-qmsi to an Intel(r) System Studio for Microcontrollers Project

* Download the rf24-qmsi code from the GitHib as a ZIP file and unpack it to a directory on the host system. Alternatively it is possible to clone the Git repository.
* Create a new Intel(r) System Studio for Microcontrollers project, using QMSI 1.1 as the project type.
* Copy the content of the board/ directory from the rf24-qmsi to the project's bsp/board directory.
* Copy the Makefile from one of the two examples, and the main.c to the project.
* Modify, compile and flash the project to the board.

## Driver configuration

Several driver configuration parameters are set in the board/drivers/rf24/rf24_config.h file.

## Hardware Setup

### Components
* Intel(r) Quark(tm) Microcontroller D2000 Development Platform
* nRF24L01, nRF24L01+, or compatible breakout board.
* FTDI 3.3V USB to Serial cable (for debugging)
* Some jumper wires:
  * seven male/female type for connecting the nRF24L01 breakout to the D2000 board
  * three male/male type for connecting the FTDI cable to the D2000 board

### Wiring
* Connect the nRF24L01 breakout board to the D2000 board as follows:
  * D2000 board GND pin to nRF24L01 breakout board GND pin
  * D2000 board 3.3V pin to nRF24L01 breakout board VCC pin
  * D2000 board pin 8 to nRF24L01 breakout board CE pin
  * D2000 board pin 10/SS0 to nRF24L01 breakout board CSN pin
  * D2000 board pin 11/MOSI to nRF24L01 breakout board MOSI pin
  * D2000 board pin 12/MISO to nRF24L01 breakout board MISO pin
  * D2000 board pin 13/SCK to nRF24L01 breakout board SCK pin
* (Optionally) connect FTDI cable to pins 0, 1, and GND as follows:
  * D2000 board pin 0/RX to FTDI cable pin 4
  * D2000 board pin 1/TX to FTDI cable pin 5
  * D2000 board GND pin to FTDI cable pin 1

![](https://github.com/skiselev/rf24-qmsi/raw/master/quark_d2000_nrf24l01.jpg)
