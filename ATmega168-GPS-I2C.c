/*! \file atmega168-GPS-I2C \brief Provides a bridge between serial GPS and I2C */
//*****************************************************************************
//  File Name   :   'ATMega168-GPS-I2C.c'
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
///     The device I2C address is 0xA0 and the host MCU can read one of several registers at a time:
///            LAT_DEG     0x01    //  latitude degrees
///            LAT_MIN     0x02    Latitude minutes
///            LAT_SEC     0x03    Latitude seconds
///            LAT_NS      0x04    north/south
///            LON_DEG     0x11    Longitude degrees
///            LON_MIN     0x12    Longitude minutes
///            LON_SEC     0x13    Longitude seconds
///            LON_EW      0x14    east/west
///            VEL_KTS     0x20    velocity in knots
///            TIME_HR     0x30    hour
///            TIME_MIN    0x31    minute
///            TIME_SEC    0x32    second
///
///
//*****************************************************************************


#ifndef F_CPU
#define F_CPU 14745600L
#endif

#import "global.h"
#import <avr/io.h>
#import <avr/interrupt.h>
#import <string.h>
#import <compat/twi.h>
#import <util/delay.h>

enum {
    kSlaveActionRead,
    kSlaveActionWrite
};
typedef unsigned char SlaveAction;

/*  OPCODES FOR OUR I2C INTERFACE */

#define LAT_DEG     0x01    //  latitude degrees
#define LAT_MIN     0x02    //  latitude minutes
#define LAT_SEC     0x03    //  latitude seconds
#define LAT_NS      0x04    //  north/south
#define LON_DEG     0x11    //  longitude degrees
#define LON_MIN     0x12    //  longitude minutes
#define LON_SEC     0x13    //  longitude seconds
#define LON_EW      0x14    //  east/west
#define VEL_KTS     0x20    //  velocity in knots
#define TIME_HR     0x30    //  hour
#define TIME_MIN    0x31    //  minute
#define TIME_SEC    0x32    //  second

#define RMC_RMC_START       0x01    //  GPRMC
#define RMC_FIX_TIME        0x01    //  123519 12:35:19 UTC
#define RMC_VALID_INDEX     0x02    //  A or V
#define RMC_LAT_INDEX       0x03    //
#define RMC_LAT_DIR_INDEX   0x04
#define RMC_LON_INDEX       0x05
#define RMC_LON_DIR_INDEX   0x06
#define RMC_VEL_KTS_INDEX   0x07
#define RMC_TRK_ANGEL_INDEX 0x08
#define RMC_DATE_INDEX      0x09    //  230394 March 23, 1994
#define RMC_MAG_VAR_INDEX   0x10    //  003.1, 3.1 degrees
#define RMC_MVAR_DIR_INDEX  0x11    //  W/E
#define RMC_CHECKSUM_INDEX  0x12    //  *6A, always begins with *

uint8_t lat[4];
uint8_t lon[4];
uint8_t time[3];
uint velocity;
uint8_t valid_data;

char serial_buffer[300];
uint8_t buffer_index;

#define DEGREE_INDEX 0
#define MINUTE_INDEX 1
#define SECONDS_INDEX 2
#define DIR_INDEX 3

#define TIME_HR_INDEX   0
#define TIME_MIN_INDEX  1
#define TIME_SEC_INDEX  2

#define LAT_NORTH 0
#define LAT_SOUTH 1
#define LON_EAST 0
#define LON_WEST 1

#define I2C_SLAVE_ADDRESS   0xA0

unsigned char regaddr;      //  the requested register address
unsigned char regdata;      //  the requested register data

int main(void) {
    DDRD |= (1<<PD2);
    
    //  set I2C slave address, ignore general adr bit
    TWAR = 0xA0 & 0xFE;        
    
    // Start Slave Listening: 
    //      Clear TWINT Flag, 
    //      Enable ACK, 
    //      Enable TWI, 
    //      TWI Interrupt Enable
    TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
    
    // Enable Global Interrupt
    sei();
    
    //  
    serial_init();
    buffer_index = 0;
    valid_data = 0;
    
    while(1) {
        unsigned char serial_data = serial_read();
        if( serial_data ) {
            process_serial(serial_data);
        }   /*  valid serial data on port */
    } /* main loop */
    
}   /*  main */

void process_serial(unsigned char data ) {
    PORTD |= (1<<PD2);
    serial_buffer[buffer_index] = data;
    buffer_index++;
    if( data == 0x0D )
    {   
        PORTD &= ~(1<<PD2);
        if( strcmp(serial_buffer,"$GPR") )
        {
            char parts[15][20];
            char *p_start, *p_end;
            char i;
            
            while(1) {
                p_end = strchr(p_start, ',' );
                if( p_end ) {
                    strncpy(parts[i], p_start, p_end-p_start);
                    parts[i][[_end-p_start] = 0;
                    i++;
                    p_start = p_end + 1;
                }
                else {
                    strncpy(parts[i], p_start, 20);
                    break;
                }
            }   /*  parsing $GPR string */
            if( parts[RMC_VALID_INDEX] == "V" ) {
                valid_data = 0;
                return;
            }   /*  check for valid data */
            
            //  obtain the time in UTC
            char *hr = (char *)malloc(2);
            char *min = (char *)malloc(2);
            char *sec = (char *)malloc(2);
            strncpy(hr, parts[RMC_FIX_TIME]+0,2);
            strncpy(min, parts[RMC_FIX_TIME]+2,2);
            strncpy(sec, parts[RMC_FIX_TIME]+4,2);
            //  store time in our registers
            time[TIME_HR_INDEX] = atoi(hr);
            time[TIME_MIN_INDEX] = atoi(min);
            time[TIME_SEC_INDEX] = atoi(sec);
            
            //  free substring memory
            free(hr);
            free(min);
            free(sec);
            
            //  obtain the latitude
            char *min_dec = (char *)malloc(2);
            char *min_int = (char *)malloc(2);
            char *deg = (char *)malloc(3);
            
            uint8_t len = strlen(parts[RMC_LAT_INDEX]);
            strncpy(min_dec,parts[RMC_LAT_INDEX] + len-2,2);
            strncpy(min_int,parts[RMC_LAT_INDEX] + len-5,2);
            if( len == 8 )
                strncpy(deg,parts[RMC_LAT_INDEX],3);
            else
                strncpy(deg,parts[RMC_LAT_INDEX],2);
            
            //  store the latitude
            lat[DEGREE_INDEX] = atoi(deg);
            lat[MINUTE_INDEX] = atoi(min_int);
            lat[SECONDS_INDEX] = atoi(min_dec) * 6/10;
            
            //  obtain the latitude direction
            if( parts[RMC_LAT_DIR_INDEX] == "N" )
                lat[DIR_INDEX] = LAT_NORTH;
            else
                lat[DIR_INDEX] = LAT_SOUTH;
            
            len = strlen(parts[RMC_LON_INDEX]);
            strncpy(min_dec,parts[RMC_LON_INDEX] + len-2,2);
            strncpy(min_int,parts[RMC_LON_INDEX] + len-5,2);
            strncpy(deg,parts[RMC_LAT_INDEX],(len==8)?3:2);
            
            //  store the longitdue
            lon[DEGREE_INDEX] = atoi(deg);
            lon[MINUTE_INDEX] = atoi(min_int);
            lon[SECONDS_INDEX] = atoi(min_dec) * 6/10;
            
            free(min_dec);
            free(min_int);
            free(deg);
            
            //  obtain the longitude direction
            if( parts[RMC_LON_DIR_INDEX] = "E" )
                lon[DIR_INDEX] = LON_EAST;
            else
                lon[DIR_INDEX] = LON_WEST;
                
            //  parse the velocity
            //  assumes velocity is xxx.x or xx.x
            char *velocity_str = (char *)malloc(3);
            len = strlen(parts[RMC_VEL_KTS_INDEX]);
            strncpy(velocity_str,parts[RMC_VEL_KTS_INDEX],len-2);
            velocity = atoi(velocity_str);
            
            free(velocity_str);
            
        }   /*  $GPR line */
    }   /*  EOL */
    buffer_index = 0;
    serial_buffer[0] = '\0';
}   /*  process serial */

unsigned char serial_read(void)
{
    while( !(UCSR0A & (1 << RXC0)) )
		;
	return UDR0;
}   /*  serial_read */

void serial_init();
{
    /*  UBRR should be 191 for a clock speed 0f 14.7456 */
    UBRR0 = 0xBF;
    /* set the framing to 8N1 */
	UCSR0C = (3 << UCSZ00);
	/* Enable */
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
}

void i2c_slave_action(SlaveAction rw_status)
{
    if( rw_status == kSlaveActionWrite )
        return;
    switch( regaddr )
    {
        case LAT_DEG:
            regdata = lat[DEGREE_INDEX];
            break;
        case LAT_MIN:
            regdata = lat[MINUTE_INDEX];
            break;
        case LAT_SEC:
            regdata = lat[SECONDS_INDEX];
            break;
        case LAT_NS:
            regdata = lat[DIR_INDEX];
            break;
        case LON_DEG;
            regdata = lon[DEGREE_INDEX];
            break;
        case LON_MIN:
            regdata = lon[MINUTE_INDEX];
            break;
        case LON_SEC:
            regdata = lon[SECONDS_INDEX];
            break;
        case LON_EW:
            regdata = lon[DIR_INDEX];
            break;
        case TIME_HR:
            regdata = time[TIME_HR_INDEX];
            break;
        case TIME_MIN:
            regdata = time[TIME_MIN_INDEX];
            break;
        case TIME_SEC:
            regdata = time[TIME_SEC_INDEX];
            break;
        case VEL_KTS:
            regdata = velocity
            break;
    }
}

ISR(TWI_vect)
{
    static unsigned char i2c_state;
    unsigned char twi_status;
    // Disable Global Interrupt
    cli();
    // Get TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
    twi_status=TWSR & 0xF8;     
    
    switch(twi_status) {
        case TW_SR_SLA_ACK:      
            // 0x60: SLA+W received, ACK returned
            i2c_state = 0;           // Start I2C State for Register Address required	 
    
            TWCR |= (1<<TWINT);    // Clear TWINT Flag
            break;
     case TW_SR_DATA_ACK:     
        // 0x80: data received, ACK returned
        if( i2c_state == 0 ) {
            regaddr = TWDR;      // Save data to the register address
            i2c_state = 1;
        } 
        else {
            regdata = TWDR;      // Save to the register data
            i2c_state = 2;
        }

        TWCR |= (1<<TWINT);    // Clear TWINT Flag
        break;
     case TW_SR_STOP:
        // 0xA0: stop or repeated start condition received while selected
        if( i2c_state == 2 ) {
            i2c_slave_action(1); // Call Write I2C Action (rw_status = 1)
            i2c_state = 0;	      // Reset I2C State
       }	   
       TWCR |= (1<<TWINT);    // Clear TWINT Flag
       break;
    
     case TW_ST_SLA_ACK:      // 0xA8: SLA+R received, ACK returned
     case TW_ST_DATA_ACK:     // 0xB8: data transmitted, ACK received
        if( i2c_state == 1 ) {
            i2c_slave_action(0); // Call Read I2C Action (rw_status = 0)
            TWDR = regdata;      // Store data in TWDR register
            i2c_state = 0;	      // Reset I2C State
        }	   	  
    
       TWCR |= (1<<TWINT);    // Clear TWINT Flag
       break;
     case TW_ST_DATA_NACK:    // 0xC0: data transmitted, NACK received
     case TW_ST_LAST_DATA:    // 0xC8: last data byte transmitted, ACK received
     case TW_BUS_ERROR:       // 0x00: illegal start or stop condition
     default:
       TWCR |= (1<<TWINT);    // Clear TWINT Flag
       i2c_state = 0;         // Back to the Begining State
    }
    // Enable Global Interrupt
    sei();
}