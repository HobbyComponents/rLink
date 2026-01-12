/* FILE:    rLink.cpp
   DATE:    24/12/25
   VERSION: 1.0.0
   AUTHOR:  Andrew Davies

24/12/25 version 1.0.0: Original version


This library adds hardware support for the Hobby Components rLink range of 
RS485 based modules to the Arduino IDE. 
Current supported boards:

rLink 2 channel relay (SKU: HCMODU0271)

See https://hobbycomponents.com/rlink for more information

See LICENCE file in the library folder for terms of use.
*/

#include "rLink.h"



/**********************************************************************
 **********************************************************************
 * rLinkModule
 * --------------------------------------------------------------------
 * Base class for all rLink devices.
 * Holds common addressing information and provides generic
 * register-level read/write helpers that all modules share.
 **********************************************************************
 **********************************************************************/

// Constructor
// Stores a pointer to the shared rLink bus and the module identity
rLinkModule::rLinkModule(rLink& b, uint8_t a, uint8_t t, uint8_t s) : bus(&b), addr(a), type(t), subType(s)
{
}



// --------------------------------------------------------------------
// Common register access helpers
// --------------------------------------------------------------------

// --------------------------------------------------------------------
// Reads a register from this module instance
//
// Parameters:
//   reg        - Register address to read from
//   data       - Pointer to a buffer where the received data will be stored
//   maxLen     - Maximum number of bytes to read
//   timeoutMs  - Timeout in milliseconds while waiting for a response
//                (default = 100 ms)
//
// Returns:
//   uint8_t    - Number of bytes successfully read
//                0  = No data received
//
// Notes:
//   This function sends a read request to the module and blocks until
//   a response is received or the timeout expires.
// --------------------------------------------------------------------
uint8_t rLinkModule::readReg(uint8_t reg, uint8_t* data, uint8_t maxLen, unsigned long timeoutMs = 100) 
{
	return bus->read(addr, type, subType, reg, data, maxLen, timeoutMs);
}


// --------------------------------------------------------------------
// Writes raw data to a register on this module instance
//
// Parameters:
//   reg   - Register address to write to
//   data  - Pointer to a buffer containing the data to write
//   len   - Number of bytes to write
//
// Returns:
//   None
//
// Notes:
//   This function only transmits the write command and does not wait
//   for a response from the module.
// --------------------------------------------------------------------
void rLinkModule::writeReg(uint8_t reg, uint8_t* data, uint8_t len) 
{
	bus->send(addr, type, subType, reg, data, len);
}


// --------------------------------------------------------------------
// Generic module register functions (present on all rLink devices)
// --------------------------------------------------------------------

// --------------------------------------------------------------------
// Reads the module status register from the device
// No parameters are required
//
// Returns:
//   uint8_t status value read from the module
//   (bit meanings are module/firmware specific)
// --------------------------------------------------------------------	
uint8_t rLinkModule::getStatus() 
{
	uint8_t s;
	readReg(RLINK_REG_STATUS, &s, 1);
	return s;
}



// --------------------------------------------------------------------
// Get firmware version of the module
//
// Sends:
//   - READ command to the module
//   - Register: RLINK_REG_SW_VER
//
// Parameters:
//   - None
//
// Returns:
//   - uint8_t : Firmware version reported by the module
// --------------------------------------------------------------------
uint8_t rLinkModule::getVersion() 
{
	uint8_t s;
	readReg(RLINK_REG_SW_VER, &s, 1);
	return s;
}


// --------------------------------------------------------------------
// Sets the communication LED mode on the module.
//
// Parameters:
//   uint8_t mode : one of RLINK_COMLED_BLINK_OFF, RLINK_COMLED_BLINK_ON,
//                  RLINK_COMLED_ALWAYS_ON, RLINK_COMLED_ALWAYS_OFF.
//
// Returns: nothing
// --------------------------------------------------------------------
void rLinkModule::setCOMLED(uint8_t mode)
{
	if(mode <= RLINK_COMLED_ALWAYS_OFF)
		writeReg(RLINK_REG_COM_LED, &mode, 1);
}


// --------------------------------------------------------------------
// Writes a new device address to the module.
// The address change is protected by an unlock sequence to prevent
// accidental modification.
//
// Parameters:
//   add  - New module address (valid range: 0–127)
//
// Behaviour:
//   - Sends the unlock sequence followed by the new address
//   - Updates the local cached address
//
// Returns:
//   Nothing
// --------------------------------------------------------------------
void rLinkModule::setAddress(uint8_t add)
{
	if(add < 128)
	{
		uint8_t data[] = {0xAA, 0x55, 0xAA, add};

		writeReg(RLINK_REG_ADD, data, 4);
		
		addr = add;
	}
}


// --------------------------------------------------------------------
// Overrides the module address used for all subsequent
// commands issued by this module instance.
//
// Parameters:
//   add - The rLink address to use for future operations (0–127).
//
// Returns:
//   None.
//
// Notes:
//   This does NOT change the address stored in the physical module.
//   It only affects which address this software instance targets.
// --------------------------------------------------------------------
void rLinkModule::useAddress(uint8_t add) 
{ 
	addr = add; 
}


// --------------------------------------------------------------------
// setBAUDRate(uint8_t baud)
// ----------------------
// Changes the module baud rate (requires unlock sequence).
//
// Parameters:
//   uint8_t baud : one of RLINK_BAUD_1200..RLINK_BAUD_115200
//
// Returns: nothing
// --------------------------------------------------------------------
void rLinkModule::setBAUDRate(uint8_t baud)
{
	if(baud <= RLINK_BAUD_115200)
	{
		uint8_t data[] = {0xAA, 0x55, 0xAA, baud};

		writeReg(RLINK_REG_BAUD_RATE, data, 4);
	}
}




// --------------------------------------------------------------------
// Set the reply delay time for the module.
//
// Parameters:
//   uint8_t delay : reply delay value (firmware-defined units)
//
// Returns: nothing
// --------------------------------------------------------------------
void rLinkModule::setReplyDelay(uint8_t delay)
{
	writeReg(RLINK_REG_REPLY_DELAY, &delay, 1);
}




/**********************************************************************
 **********************************************************************
 * rLink (core bus implementation)
 * --------------------------------------------------------------------
 * Handles RS485 framing, timing, CRC, and half-duplex direction control.
 **********************************************************************
 **********************************************************************/

// --------------------------------------------------------------------
// Constructor for the rLink bus object.
//
// Parameters:
//   Stream& serial : reference to the serial port (HardwareSerial, SoftwareSerial, or any Stream).
//   uint8_t dirPin : pin used to control RS485 direction (HIGH = transmit, LOW = receive).
//
// Returns: nothing (constructor)
// --------------------------------------------------------------------
rLink::rLink(Stream& serial, uint8_t dirPin)
{
	_serial = &serial;
	_dirPin = dirPin;
}



// --------------------------------------------------------------------
// Initialise the rLink bus object.
//
// Parameters: none
//
// Returns: nothing
// --------------------------------------------------------------------
void rLink::init(void)
{
	_baudRate = 9600;
    _lastRxMicros = 0;

    pinMode(_dirPin, OUTPUT);
    digitalWrite(_dirPin, LOW);
}




// --------------------------------------------------------------------
// Frame transmission
// --------------------------------------------------------------------

// --------------------------------------------------------------------
// Convenience function to send a single-byte WRITE command to a module.
//
// Parameters:
//   uint8_t add      : address of the target module
//   uint8_t type     : module type
//   uint8_t subType  : module subtype
//   uint8_t reg      : register to write to
//   uint8_t data     : single byte of data to write
//
// Returns: nothing
// --------------------------------------------------------------------
void rLink::send(uint8_t add, uint8_t type, uint8_t subType, uint8_t reg, uint8_t data)
{
	send(add, type, subType, reg, &data, 1);
}


// --------------------------------------------------------------------
// Sends a WRITE command frame to a module with multiple bytes of data.
//
// Parameters:
//   uint8_t add       : address of the target module
//   uint8_t type      : module type
//   uint8_t subType   : module subtype
//   uint8_t reg       : register to write to
//   uint8_t* data     : pointer to the data bytes to send
//   uint8_t dataLen   : number of bytes to send (must not exceed RLINK_MAX_DATA)
//
// Returns: nothing
// --------------------------------------------------------------------
void rLink::send(uint8_t add, uint8_t type, uint8_t subType, uint8_t reg, uint8_t* data, uint8_t dataLen)
{
	// Check data doesnt exceed maximum playload size
    if (dataLen > RLINK_MAX_DATA)
        return;

	// Create a buffer to holed the complete frame
    uint8_t frame[RLINK_MAX_FRAME];

    frame[0] = 0x55;
    frame[1] = add;
    frame[2] = type;
    frame[3] = subType;
    frame[4] = CMD_WRITE;
    frame[5] = reg;
    frame[6] = dataLen;

	// Copy the data intot he frame
    for (uint8_t i = 0; i < dataLen; i++)
        frame[7 + i] = data[i];

	// Calculate the CRC
    frame[7 + dataLen] = _crc8(&frame[1], 6 + dataLen);

    uint8_t totalLen = 8 + dataLen;

    // Wait until the bus has been quiet (max 5ms)
    if (!_waitBusQuiet(5000))
        return;

    // Enable transmitter
    digitalWrite(_dirPin, HIGH);

    // Send frame
    _serial->write(frame, totalLen);
    _serial->flush();

    // Return to receive mode
    digitalWrite(_dirPin, LOW);
}



// --------------------------------------------------------------------
// Send a READ request to a device without waiting for a reply.
//
// Parameters:
//   uint8_t add      : device address
//   uint8_t type     : module type
//   uint8_t subType  : module subtype
//   uint8_t reg      : register to read
//
// Returns: nothing (sends request only)
// --------------------------------------------------------------------
void rLink::request(uint8_t add, uint8_t type, uint8_t subType, uint8_t reg)
{
    uint8_t frame[RLINK_FRAME_OVERHEAD];

	// Create a read request frame
    frame[0] = 0x55;
    frame[1] = add;
    frame[2] = type;
    frame[3] = subType;
    frame[4] = CMD_READ;
    frame[5] = reg;
    frame[6] = 0x00; // LEN = 0
    frame[7] = _crc8(&frame[1], 6);

    // Wait until the bus has been quiet
    if (!_waitBusQuiet(5000))  // wait up to 5ms for bus idle
        return;

    // Enable transmitter
    digitalWrite(_dirPin, HIGH);

    // Send frame
    _serial->write(frame, RLINK_FRAME_OVERHEAD);
    _serial->flush();

    // Return to receive mode
    digitalWrite(_dirPin, LOW);

    // After this, the slave will start sending a reply
}



// --------------------------------------------------------------------
// Receive a response from the device after a request has been sent.
//
// Parameters:
//   uint8_t add      : device address
//   uint8_t type     : module type
//   uint8_t subType  : module subtype
//   uint8_t reg      : register expected in the response
//   uint8_t* data    : pointer to buffer to store received data
//   uint8_t maxLen   : maximum number of bytes to copy into buffer
//   unsigned long timeoutMs : timeout in milliseconds to wait for response
//
// Returns: number of bytes copied into the buffer, or 0 if timeout/invalid
// --------------------------------------------------------------------
uint8_t rLink::receive(uint8_t add, uint8_t type, uint8_t subType, uint8_t reg,
                       uint8_t* data, uint8_t maxLen, unsigned long timeoutMs)
{
    uint8_t rxBuf[RLINK_MAX_FRAME];
    uint8_t idx = 0;
    uint8_t expectedLen = 0;
    unsigned long startMs = millis();

	// Wait until timout exceeded
    while ((millis() - startMs) < timeoutMs)
    {
		// Check for new data
        while (_serial->available())
        {
			// Get the next byte
            uint8_t b = _serial->read();
            _lastRxMicros = micros();

			// If data is start byte skip the next bit
            if (idx == 0 && b != 0x55) continue;

			// Store the data in the frame buffer
            rxBuf[idx++] = b;
            if (idx >= RLINK_MAX_FRAME) return 0;

			// If we have recevied the data length byte then calulate the expected frame length
			// and quit if too large for buffer
            if (idx == 7)
            {
                expectedLen = RLINK_FRAME_OVERHEAD + rxBuf[6];
                if (expectedLen > RLINK_MAX_FRAME) return 0;
            }

			
            if (expectedLen && idx >= expectedLen)
            {
                uint8_t payloadLen = rxBuf[6];

				// Check address, type & subtype match
                if (rxBuf[1] != add || (rxBuf[2] != type && type != 0xFF) || (rxBuf[3] != subType && subType != 0xFF)) return 0;
                // Check that it is a reply
				if (rxBuf[4] != 0x03) return 0;
				// Check that reply is for the correct register
                if (rxBuf[5] != reg) return 0;
                // Check CRC
				if (_crc8(&rxBuf[1], 6 + payloadLen) != rxBuf[7 + payloadLen]) return 0;

				// Copy the data into the data array
                uint8_t copyLen = (payloadLen > maxLen) ? maxLen : payloadLen;
                for (uint8_t i = 0; i < copyLen; i++) data[i] = rxBuf[7 + i];
                return copyLen;
            }
        }
    }

    return 0;
}



// --------------------------------------------------------------------
// Send a READ request and wait for a response.
//
// Parameters:
//   uint8_t add      : device address
//   uint8_t type     : module type
//   uint8_t subType  : module subtype
//   uint8_t reg      : register to read
//   uint8_t* data    : pointer to buffer to store received data
//   uint8_t maxLen   : maximum number of bytes to copy into buffer
//   unsigned long timeoutMs : timeout in milliseconds to wait for response
//
// Returns: number of bytes copied into the buffer, or 0 if timeout/invalid
// --------------------------------------------------------------------
uint8_t rLink::read(uint8_t add, uint8_t type, uint8_t subType, uint8_t reg,
                    uint8_t* data, uint8_t maxLen, unsigned long timeoutMs)
{
    request(add, type, subType, reg);   // send the request
	
    return receive(add, type, subType, reg, data, maxLen, timeoutMs); // wait for reply
}



/**********************************************************************
 * Internal helpers
 **********************************************************************/

// --------------------------------------------------------------------
// Waits until the RS485 bus has been idle for at least 2ms or until
// the specified timeout is reached.
//
// Parameters:
//   unsigned long timeoutUs : maximum time to wait for bus idle (microseconds)
//
// Returns:
//   bool : true if bus was quiet for 2ms, false if timeout occurred
// --------------------------------------------------------------------
bool rLink::_waitBusQuiet(unsigned long timeoutUs)
{
    unsigned long start = micros();
    while ((micros() - _lastRxMicros) < 2000) // 2ms quiet
    {
        if ((micros() - start) > timeoutUs)
            return false;
    }
    return true;
}



// --------------------------------------------------------------------
// Calculates an 8-bit CRC (Cyclic Redundancy Check) using polynomial 0x07.
//
// Parameters:
//   uint8_t* data : pointer to the data array to compute CRC over
//   uint8_t len   : number of bytes in the data array
//
// Returns:
//   uint8_t : computed CRC-8 value
// --------------------------------------------------------------------
uint8_t rLink::_crc8(uint8_t *data, uint8_t len) 
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else crc <<= 1;
    }
  }
  return crc;
}




/**********************************************************************
 **********************************************************************
 * 					RLINK 2CH RELAY (HCMODU0271)
 **********************************************************************
 **********************************************************************/

// --------------------------------------------------------------------
// Constructor for the 2-channel rLink relay module (HCMODU0271).
//
// Parameters:
//   rLink& b  : reference to the shared rLink bus object
//   uint8_t a : module address on the rLink bus
//
// Returns: nothing (constructor)
// --------------------------------------------------------------------
rLinkRelay2CH::rLinkRelay2CH(rLink& b, uint8_t a) : rLinkModule(b, a, RLY_TYPE, RLY_2CH_SUBTYPE)
{
}



// --------------------------------------------------------------------
// Set the state of a relay channel on the 2-channel rLink relay module.
//
// Parameters:
//   uint8_t channel : relay channel number (0 or 1)
//   bool on         : true to turn ON, false to turn OFF
//
// Returns: nothing
// --------------------------------------------------------------------
void rLinkRelay2CH::setRelay(uint8_t channel, bool on)
{
	uint8_t reg = RLINK_RLY_REG_R0 + channel;
	uint8_t data = on ? 1 : 0;
	
    writeReg(reg, &data, 1);
}



// --------------------------------------------------------------------
// Read the current state of a relay channel on the 2-channel rLink relay module.
//
// Parameters:
//   uint8_t channel        : relay channel number (0 or 1)
//   bool& state            : reference variable where the relay state will be stored
//                            (true = ON, false = OFF)
//   unsigned long timeoutMs: maximum time in milliseconds to wait for a response
//
// Returns:
//   true  : if the state was successfully read
//   false : if no valid response was received within the timeout
// --------------------------------------------------------------------
bool rLinkRelay2CH::readRelay(uint8_t channel, bool& state, unsigned long timeoutMs)
{
	uint8_t reg = RLINK_RLY_REG_R0 + channel;
    uint8_t data;
    if (readReg(reg, &data, 1, timeoutMs) == 1)
    {
        state = data & 0x01;
        return true;
    }
    return false;
}



// --------------------------------------------------------------------
// Read the current digital input states of the 2-channel rLink relay module.
//
// Parameters:
//   uint8_t& state         : reference variable where input states will be stored.
//                            Each bit represents one input (bit 0 = IN0, bit 1 = IN1).
//   unsigned long timeoutMs: maximum time in milliseconds to wait for a response.
//
// Returns:
//   true  : if the input states were successfully read
//   false : if no valid response was received within the timeout
// --------------------------------------------------------------------
bool rLinkRelay2CH::readInputs(uint8_t& state, unsigned long timeoutMs)
{

	if (readReg(RLINK_RLY_REG_IN_STATE, &state, 1, timeoutMs) == 1) 
        return true;

    return false;
}



// --------------------------------------------------------------------
// Enable or disable the digital inputs on the 2-channel rLink relay module.
//
// Parameters:
//   uint8_t state : each bit enables/disables an input (bit 0 = IN0, bit 1 = IN1).
//                   Only the lowest 2 bits are used; higher bits are ignored.
//
// Returns: nothing
// --------------------------------------------------------------------
void rLinkRelay2CH::setInputs(uint8_t state)
{
	state &= 0b11;
    writeReg(RLINK_RLY_REG_IN_ENABLE, state, 1);
}



// --------------------------------------------------------------------
// Set the ON-time for a relay channel on the 2-channel rLink relay module.
//
// Parameters:
//   uint8_t channel    : relay channel to configure (0 or 1).
//   uint16_t time100Ms : ON-time duration in units of 100 milliseconds.
//
// Returns: nothing
// --------------------------------------------------------------------
void rLinkRelay2CH::setOnTime(uint8_t channel, uint16_t time100Ms)
{
	if(channel < 2)
	{
		uint8_t data[2]; 
		data[0] = time100Ms;
		data[1] = time100Ms >> 8;
		uint8_t reg = RLINK_RLY_REG_R0_ONTIME + channel;
		writeReg(reg, data, 2);
	}
}



// --------------------------------------------------------------------
// Read the ON-time for a relay channel on the 2-channel rLink relay module.
//
// Parameters:
//   uint8_t channel        : relay channel to read (0 or 1).
//   uint16_t& time100Ms    : reference to variable where ON-time (in 100 ms units) will be stored.
//   unsigned long timeoutMs: maximum time to wait for a response from the module in milliseconds.
//
// Returns:
//   true  : if the ON-time was successfully read.
//   false : if the read operation timed out or failed.
// --------------------------------------------------------------------
bool rLinkRelay2CH::readOnTime(uint8_t channel, uint16_t& time100Ms, unsigned long timeoutMs)
{
    uint8_t reg = RLINK_RLY_REG_R0_ONTIME + channel;
    uint8_t data;

    if (readReg(reg, (uint8_t *) &time100Ms, 2, timeoutMs) == 2) 
        return true;
 
    return false;
}


