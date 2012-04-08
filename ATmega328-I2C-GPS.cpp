/*! \file ATmega168-I2C-GPS.cpp \brief Provides a bridge between serial GPS and I2C */
//*****************************************************************************
//  File Name   :   ' ATmega168-I2C-GPS.cpp'
//  Title       :   Provides a bridge between serial GPS and I2C
//  Author      :   Alan Duncan - Copyright (c) 2012
//  Created     :   2012-04-05 12-27-07
//  Revised     :
//  Version     :   0.7
//  Target MCU  :   ATmega 168/328
//
/// \par    Overview
///     This is a bridge between a serial GPS and I2C.  In this implementation the MCU acts as an 
///     I2C slave, providing data on request from the host MCU.  Meanwhile, it collects incoming 
///     NMEA data from the GPS and parses it into discrete 'registers' which the host MCU can
///     access on demand.
/// \par    Notes
///     This software has not been tested at all.  It is intended to provide a minimum quantity of
///     data from an eTrex Legend GPS.  Ultimately, it is destined for a high-altitude balloon 
///     project where it will provide backup low-altitude location information if the high-altitude
///     GPS fails.
///
///
//*****************************************************************************

#ifndef F_CPU
#define F_CPU 14745600UL
#endif

/*	ETREX LEGEND GPS COMMUNICATES AT 4800 BAUD	*/
#define BAUD 4800

#include "global.h"
#include "TWI_slave.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <string.h>
#include <compat/twi.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include "gps.h"

#define I2C_SLAVE_ADDRESS   	0xA0	//	we will listen on this address
#define I2C_DEBUG_CONFIRM_BYTE	0xF0	//	this byte is returned when debug mode is changed
#define I2C_ERROR				0xF2	//	code return when error encountered

/*  OPCODES FOR OUR I2C INTERFACE */

#define VEL_KTS     0x20    //  velocity in knots
#define LAT			0x40	//	return 4 bytes representing the latitude
#define LON			0x41	//	return 4 bytes representing the longitude
#define FIX_TIME	0x50	//	return the time of the most recent fix
#define DEBUG_ON	0x60	//	turn on debugging mode
#define DEBUG_OFF	0x61	//	turn off debugging mode

struct settings_record_t {
	uint8_t debug_mode;
	uint8_t pwr_on_dx_count;
	uint8_t error_dx_count;
}

/*
	See: https://brezn.muc.ccc.de/svn/moodlamp-rf/trunk/bussniffer/settings.c
*/

/*	GLOBAL VARS	*/
struct settings_record_t global_settings_record EEMEM = {1,5,3};
struct settings_record_t global_settings;
GPS gps;


#define IS_DEBUGGING global_settings_record.debug_mode == 1

/*	FUNCTION PROTOTYPES */
void settings_read(void);
void settings_write(void);
void opcode process(unsigned char opcode );
void serial_init();
unsigned char serial_read(void);

int main(void)
{
	settings_read();
	if( IS_DEBUGGING ) {
		DDRD |= (1<<PD2);
	
		blink(global_settings.pwr_on_blink_count);
		_delay_ms(500);
	}
	
	serial_init();
	
    TWI_slaveAddress = I2C_SLAVE_ADDRESS;
	
	// Initialise TWI module for slave operation. Include address and/or enable General Call.
	TWI_Slave_Initialise( (unsigned char)((TWI_slaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT) )); 
	
	sei();
	
	TWI_Start_Transceiver( ); 
	
    while(1) {
		if( !TWI_Transceiver_Busy() ) {
			if( TWI_statusReg.RxDataInBuf ) {
				TWI_Get_Data_From_Transceiver(outbuffer, 1);  
			}
			process_opcode(outbuffer[0]);
		}
  	} 
}

void opcode process(unsigned char opcode ) {
	bool error = false;
	switch( opcode ) {
		case  LAT:
			outbuffer[0] = gps.latitude.degrees;
			outbuffer[1] = gps.latitude.minutes;
			outbuffer[2] = gps.latitude.seconds;
			outbuffer[3] = gps.latitude.direction;
			TWI_Start_Transceiver_With_Data(outbuffer, 4); 
			break;
		case LON:
			outbuffer[0] = gps.longitude.degrees;
			outbuffer[1] = gps.longitude.minutes;
			outbuffer[2] = gps.longitude.seconds;
			outbuffer[3] = gps.longitude.direction;
			TWI_Start_Transceiver_With_Data(outbuffer, 4); 
			break;
		case FIX_TIME:
			outbuffer[0] = gps.time.hour;
			outbuffer[1] = gps.time.minute;
			outbuffer[2] = gps.time.second;
			TWI_Start_Transceiver_With_Data(outbuffer, 3); 
		case VELOCITY:
			outbuffer[0] = gps.velocity;
			TWI_Start_Transceiver_With_Data(outbuffer, 1); 
			break;
		case DEBUG_ON:
			outbuffer[0] = I2C_DEBUG_CONFIRM_BYTE;
			TWI_Start_Transceiver_With_Data(outbuffer, 1);
			break;
		case DEBUG_OFF:
			outbuffer[0] = I2C_DEBUG_CONFIRM_BYTE;
			TWI_Start_Transceiver_With_Data(outbuffer, 1);
			break;
		default: 
			error = true;
			outbuffer[0] = I2C_EEROR;
			TWI_Start_Transceiver_With_Data(outbuffer, 1); 
			break;
	}	/* opcode switch */
	if( IS_DEBUGGING && error)
		blink(global_settings.error_dx_count);
}	/*	processOpcode()	*/

void settings_read(void) {
	eeprom_read_block(&global_settings, &global_settings_record, sizeof(global_settings));
}	/*	settings_read	*/

void settings_write(void) {
	eeprom_write_block(&global_settings, &global_settings_record, sizeof(global_settings));
}	/*	settings_write	*/

void serial_init()
{
	/* Set the baud rate */
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	/* set the framing to 8N1 */
	UCSR0C = (3 << UCSZ00);
	/* Engage! */
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	return;
}

unsigned char serial_read(void)
{
	while( !(UCSR0A & (1 << RXC0)) )
		;
	return UDR0;
}