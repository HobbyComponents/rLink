/* FILE:    rLink.h
   DATE:    24/12/25
   VERSION: 1.0.0
   AUTHOR:  Andrew Davies

24/12/25 version 1.0.0: Original version


This library adds hardware support for the Hobby Components rLink range of 
RS485 based modules to the Arduino IDE. 
Current supported boards:

rLink 2 channel relay (SKU: HCMODU0271)

See https://hobbycomponents.com/rlink for more information

See LICENCE in the library folder for terms of use.
*/




#ifndef RLINK_h
#define RLINK_h


#include "Arduino.h"

#include <Stream.h>



/**********************************************************/
// 						Constants
/**********************************************************/
#define RLINK_MAX_DATA   2		// Maximum payload size per frame - currently 2ch relay supported only needs 2 bytes
#define RLINK_FRAME_OVERHEAD 8 	// Fixed frame overhead
#define RLINK_MAX_FRAME (RLINK_MAX_DATA + RLINK_FRAME_OVERHEAD) // Max total frame length

#define RLINK_ON		1
#define RLINK_OFF		0

// Command codes
#define CMD_WRITE       0x01	// Write command
#define CMD_READ        0x02	// Read command
/**********************************************************/



/**********************************************************/
// 				  Common to all modules
/**********************************************************/
// Generic module registers
enum RLINK_GEN_REGISTERS
{
    RLINK_REG_STATUS =     		0,  	// Module status register
    RLINK_REG_ADD =        		1,  	// Module address register
    RLINK_REG_MOD_TYPE =   		2,  	// Module type
    RLINK_REG_MOD_SUBTYPE = 	3,  	// Module subtype
    RLINK_REG_SW_VER =     		4,  	// Firmware version
    RLINK_REG_COM_LED =    		5,  	// COM LED control
    RLINK_REG_BAUD_RATE =  		6,   	// Baud rate register
	RLINK_REG_REPLY_DELAY = 	7		// Reply delay
};


// COM LED modes
enum RLINK_COMLED_MODES
{
	RLINK_COMLED_BLINK_OFF =	0,
	RLINK_COMLED_BLINK_ON =		1,
	RLINK_COMLED_ALWAYS_ON =	2,
	RLINK_COMLED_ALWAYS_OFF =	3
};


// Supported baud rates
enum RLINK_BAUD_RATES
{
	RLINK_BAUD_1200 =			0,
	RLINK_BAUD_2400 =			1,
	RLINK_BAUD_4800 =			2,
	RLINK_BAUD_9600 =			3,
	RLINK_BAUD_19200 =			4,
	RLINK_BAUD_38400 =			5,
	RLINK_BAUD_57600 =			6,
	RLINK_BAUD_115200 =			7
};



// --------------------------------------------------------------------
// rLink core bus class
//
// Handles RS485 framing, CRC, half-duplex control, sending and receiving.
//
// Public Methods:
//   rLink(Stream& serial, uint8_t dirPin) - constructor
//   void init() - initializes bus and direction pin
//   void send(...) - sends write frame
//   void read(...) - sends read request
//   uint8_t readReg(...) - reads data from a register
//   void request(...) - sends a read request, doesn't wait for reply
//   uint8_t read(...) - sends a read request and waits for the response from the device
//   uint8_t receive(...) - waits for a response from the device after sending a request()
//
// Private Methods:
//   _waitBusQuiet() - waits for bus idle
//   _crc8() - calculates CRC
// --------------------------------------------------------------------
class rLink 
{
	public:
		rLink(Stream& serial, uint8_t dirPin);
		void init(void);
		void send(uint8_t add, uint8_t type, uint8_t subType, uint8_t reg, uint8_t data);
		void send(uint8_t add, uint8_t type, uint8_t subType, uint8_t reg, uint8_t* data, uint8_t dataLen);
		void request(uint8_t add, uint8_t type, uint8_t subType, uint8_t reg);
		uint8_t read(uint8_t add, uint8_t type, uint8_t subType, uint8_t reg, uint8_t* data, uint8_t maxLen, unsigned long timeoutMs = 100);
		uint8_t receive(uint8_t add, uint8_t type, uint8_t subType, uint8_t reg, uint8_t* data, uint8_t maxLen, unsigned long timeoutMs);

	private:
		bool _waitBusQuiet(unsigned long timeoutUs);
		uint8_t _crc8(uint8_t* data, uint8_t len);
		void _baud(unsigned long baud);
		int _available();
		int _read();
		size_t _write(uint8_t b);
		void _flush();

		Stream* _serial;
		uint8_t _dirPin;
		unsigned long _baudRate, _lastRxMicros = 0;
};




// --------------------------------------------------------------------
// Base class for rLink modules
//
// Stores module address, type, and provides register read/write.
//
// Public Methods:
//   rLinkModule(rLink& bus, uint8_t addr, uint8_t type, uint8_t subType) - constructor
//   uint8_t readReg(...) - read register data
//   void writeReg(...) - write register data
//   uint8_t getStatus() - get module status
//   uint8_t getVersion() - get firmware version
//   void setCOMLED(uint8_t mode) - set communication LED
//   void setAddress(uint8_t add) - change module address
//   void useAddress(uint8_t add) - temporarily override address
//   void setBAUDRate(uint8_t baud) - change baud rate
// --------------------------------------------------------------------
class rLinkModule 
{
	protected:
		rLink* bus;
		uint8_t addr;
		uint8_t type;
		uint8_t subType;
		

	public:
		rLinkModule(rLink& bus, uint8_t addr, uint8_t type, uint8_t subType);
		
		uint8_t readReg(uint8_t reg, uint8_t* data, uint8_t maxLen, unsigned long timeoutMs = 100) ;
		void writeReg(uint8_t reg, uint8_t* data, uint8_t len);
		uint8_t getStatus();
		uint8_t getVersion();
		void setCOMLED(uint8_t mode);
		void setAddress(uint8_t add);
		void useAddress(uint8_t add);
		void setBAUDRate(uint8_t baud);
		void setReplyDelay(uint8_t delayMs);
};






/***********************************************************
			RLINK 2CH RELAY (HCMODU0271)
***********************************************************/

#define RLY_TYPE				2
#define RLY_2CH_SUBTYPE			1

// Module specific registers
enum RLINK_RELAY_REGISTERS
{
	RLINK_RLY_REG_R0 = 			10,
	RLINK_RLY_REG_R1 = 			11,
	RLINK_RLY_REG_R2 = 			12,
	RLINK_RLY_REG_R3 = 			13,
	RLINK_RLY_REG_IN_STATE 	= 	14,
	RLINK_RLY_REG_IN_ENABLE = 	15,
	RLINK_RLY_REG_R0_ONTIME = 	16,
	RLINK_RLY_REG_R1_ONTIME = 	17,
	RLINK_RLY_REG_R2_ONTIME = 	18,
	RLINK_RLY_REG_R3_ONTIME	= 	19,
};


// 2-channel relay module class
class rLinkRelay2CH : public rLinkModule 
{
	public:
		rLinkRelay2CH(rLink& bus, uint8_t addr);

		void setRelay(uint8_t channel, bool on);
		bool readRelay(uint8_t channel, bool& state, unsigned long timeoutMs = 100);
		bool readInputs(uint8_t& state, unsigned long timeoutMs = 100);
		void setInputs(uint8_t state);
		void setOnTime(uint8_t channel, uint16_t time100Ms);
		bool readOnTime(uint8_t channel, uint16_t& time100Ms, unsigned long timeoutMs = 100);
};


#endif