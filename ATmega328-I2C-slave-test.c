
#ifndef F_CPU
#define F_CPU 14745600L
#endif

//#include "global.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>
#include <util/delay.h>
#include "TWI_Slave.h"

// Sample TWI transmission commands
#define TWI_CMD_MASTER_WRITE 0x10
#define TWI_CMD_MASTER_READ  0x20

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
uint8_t velocity;
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

void blink(uint8_t count) {
	for(uint8_t i = 0; i < count; i++) {
		PORTD |= (1<<PD2);
		_delay_ms(50);
		PORTD &= ~(1<<PD2);
		_delay_ms(50);
	}
}
unsigned char messageBuf[TWI_BUFFER_SIZE];
unsigned char TWI_slaveAddress;
unsigned char temp;
unsigned char outbuffer[2];

int main(void) {
	
	
	TWI_slaveAddress = I2C_SLAVE_ADDRESS;
	
	// Initialise TWI module for slave operation. Include address and/or enable General Call.
	TWI_Slave_Initialise( (unsigned char)((TWI_slaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT) )); 
	
    DDRD |= (1<<PD2);
	
	blink(10);
	_delay_ms(1000);
	
	sei();
	
	TWI_Start_Transceiver( ); 
	
    while(1) {
		if( !TWI_Transceiver_Busy() )                              
		{
			if( TWI_statusReg.RxDataInBuf )
			{
				TWI_Get_Data_From_Transceiver(outbuffer, 2);  
			}
			if( outbuffer[0] == 0x02 ) {
				outbuffer[1] = 0xAA;
			}
			else 
				outbuffer[1] = 0xBB;
			
			TWI_Start_Transceiver_With_Data(outbuffer, 2); 
		}
  } 
}   /*  main */