/*
 * IncFile1.h
 *
 * Created: 4/7/2012 7:39:09 AM
 *  Author: Owner
 */ 


#ifndef INCFILE1_H_
#define INCFILE1_H_

#include <inttypes.h>

enum {
	DIR_NORTH,
	DIR_SOUTH,
	DIR_EAST = 0,
	DIR_WEST
};
typedef uint8_t CoordinateDirection;

struct FixTime {
	uint8_t hour;
	uint8_t	minute;
	uint8_t second;
};

struct CoordinateComponent {
	uint8_t degrees;
	uint8_t minutes;
	uint8_t seconds;
	CoordinateDirection direction;
}

class GPS
{
	private:
		CoordinateComponent latitude;
		CoordinateComponent longitude;
		FixTime time;
		uint8_t velocity;
		char buffer[300];
		uint16_t buffer_index;
		bool isValid;
		bool isComplete;
	public:
		CoordinateComponent getLatitude();
		CoordinateComponent getLogitude();
		FixTime getTime();
		uint8_t getVelocity();
		bool appendCharacter(unsigned char c);
		bool isValid();
		bool isComplete();
	protected:
	
};




#endif /* INCFILE1_H_ */