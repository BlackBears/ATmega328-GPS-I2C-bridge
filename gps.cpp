/*! \file gps.cpp \brief Parses incoming NMEA data from eTrex Legend */
//*****************************************************************************
//  File Name   :   'gps.cpp'
//  Title       :   Parses incoming NMEA data from eTrex Legend
//  Author      :   Alan Duncan - Copyright (c) 2012
//  Created     :   7 April 2012
//  Revised     :
//  Version     :   0.7
//  Target MCU  :   ATmega 168/328
//
/// \par    Overview
///     This is part of a bridge between eTrex Legend and high altitude balloon flight
///		computer.  This component parses NMEA data from the GPS.
/// \par    Notes
///     This software has not been tested at all.  It is intended to provide a minimum quantity of
///     data from an eTrex Legend GPS.  Ultimately, it is destined for a high-altitude balloon 
///     project where it will provide backup low-altitude location information if the high-altitude
///     GPS fails.
///
///
//*****************************************************************************

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

#define GPS_DATA_INVALID	0xFE

bool GPS::appendCharacter(unsigned char c) {
	bool gpr_found = false;
	buffer[buffer_index] = c;
    buffer_index++;
    if( c == 0x0D )
    {   
        if( strcmp(buffer,"$GPR") )
        {
        	gpr_found = true;
        	
        	//	temporarily mark as complete.  If there are empty params in parsing,
        	//	then later mark as incomplete.
        	isComplete = true;
        	
            char parts[15][20];
            char *p_start, *p_end;
            char i;
            
            p_start = buffer;
            while(1) {
                p_end = strchr(p_start, ',' );
                if( p_end ) {
                    strncpy(parts[i], p_start, p_end-p_start);
                    parts[i][p_end-p_start] = 0;
                    i++;
                    p_start = p_end + 1;
                }
                else {
                    strncpy(parts[i], p_start, 20);
                    break;
                }
            }   /*  parsing $GPR string */
            if( parts[RMC_VALID_INDEX] == "V" ) {
                isValid = false;
                return;
            }   /*  check for valid data */
            isValid = true;
            
            //  obtain the time in UTC
           
            if( strlen(parts[RMC_FIX_TIME]) == 0 ) {
            	time.hour = GPS_DATA_INVALID;
            	time.minute = GPS_DATA_INVALID;
            	time.second = GPS_DATA_INVALID;
            	
            	isComplete = false;
            }	/* no time is available */	
            else {
             	char *hr = (char *)malloc(2);
            	char *min = (char *)malloc(2);
            	char *sec = (char *)malloc(2);
            	
            	strncpy(hr, parts[RMC_FIX_TIME]+0,2);
				strncpy(min, parts[RMC_FIX_TIME]+2,2);
				strncpy(sec, parts[RMC_FIX_TIME]+4,2);
				//  store time in our registers
				time.hour = atoi(hr);
				time.minute = atoi(min);
				time.second = atoi(sec);
				
				//  free substring memory
            	free(hr);
            	free(min);
            	free(sec);
            }	/* valid time is available */
            

            if( strlen(parts[RMC_LAT_INDEX]) == 0 ) {
            	latitude.degrees = GPS_DATA_INVALID;
            	latitude.minutes = GPS_DATA_INVALID;
            	latitude.seconds = GPS_DATA_INVALID;
            	
            	isComplete = false;
            }	/*	empty latitude */
            else {
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
				latitude.degrees = atoi(deg);
				latitude.minutes = atoi(min_int);
				latitude.seconds = atoi(min_dec) * 6/10;
            }	/* valid latitude */
            
            if( strlen(parts[RMC_LAT_DIR_INDEX] == 0 ) {
				latitude.direction = GPS_DATA_INVALID;
				
				isComplete = false;
			}	/*	empty latitude direction */
			else
			{
				//  obtain the latitude direction
				if( parts[RMC_LAT_DIR_INDEX] == "N" )
					latitude.direction = DIR_NORTH;
				else
					latitude.direction = DIR_SOUTH;
			}	/* valid latitude direction */
            
            len = strlen(parts[RMC_LON_INDEX]);
            if( len == 0 ) {
            	longitude.degrees = GPS_DATA_INVALID;
            	longitude.minutes = GPS_DATA_INVALID;
            	longitude.seconds = GPS_DATA_INVALID;
            	
            	isComplete = false;
            }	/* empty longitude */
            else {
            	strncpy(min_dec,parts[RMC_LON_INDEX] + len-2,2);
				strncpy(min_int,parts[RMC_LON_INDEX] + len-5,2);
				strncpy(deg,parts[RMC_LAT_INDEX],(len==8)?3:2);
				
				//  store the longitdue
				longitude.degrees = atoi(deg);
				longitude.minutes = atoi(min_int);
				longitude.seconds = atoi(min_dec) * 6/10;
				
				free(min_dec);
				free(min_int);
				free(deg);
            }	/* valid longitude */
           
            
            //  obtain the longitude direction
            if( strlen(parts[RMC_LON_DIR_INDEX]) == 0 ) {
            	longitude.direction = GPS_DATA_INVALID;
            	isComplete = false;
            }	/* longitude direction is empty */
            else {
            	if( parts[RMC_LON_DIR_INDEX] = "E" )
                	longitude.direction = DIR_EAST;
           	 	else
                	longitude.direction = DIR_WEST;
            }	/* longitude direction is non-empty */
            
                
            //  parse the velocity
            //  assumes velocity is xxx.x or xx.x
            len = strlen(parts[RMC_VEL_KTS_INDEX]);
            if( len == 0 ) {
            	velocity = GPS_DATA_INVALID;
            	isComplete = false;
            }
            else {
            	char *velocity_str = (char *)malloc(3);
            
            	strncpy(velocity_str,parts[RMC_VEL_KTS_INDEX],len-2);
            	velocity = atoi(velocity_str);
            
            	free(velocity_str);
            }            
        }   /*  $GPR line */
        
        //	at the end of line, we can reset our buffer
        buffer_index = 0;
    	buffer[0] = '\0';
    }   /*  EOL */
    return gpr_found;
}	/* appendCharacter */

bool GPS::isValid() {
	return isValid;
}

bool GPS::isComplete() {
	return isComplete;
}

CoordinateComponent GPS::getLatitude() {
	return latitude;
}

CoordinateComponent GPS::getLogitude() {
	return longitude;
}

FixTime GPS::getTime() {
	return time;
}

uint8_t GPS::getVelocity() {
	return velocity;
}