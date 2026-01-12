/* FILE:    rLink_Bus_Scanner.ino
   DATE:    29/12/25
   VERSION: 1.0.0
   AUTHOR:  Andrew Davies

This sketch demonstrates how to scan the RS485 bus for rLink
modules using a SoftwareSerial interface. It iterates through
all possible addresses (0–127) and reports modules that respond.
*/

#include <SoftwareSerial.h>
#include <rLink.h>

// SoftwareSerial pins
#define RX_PIN   2
#define TX_PIN   3
#define DIR_PIN  4

// Create SoftwareSerial instance
SoftwareSerial softSerial(RX_PIN, TX_PIN);

// Create rLink bus using SoftwareSerial
rLink rlink(softSerial, DIR_PIN);

void setup() 
{
  Serial.begin(9600);
  while (!Serial);

  Serial.println(F("rLink RS485 bus scanner starting..."));

  // Start the software serial interface
  softSerial.begin(9600);
  rlink.init();

  Serial.println(F("Scanning addresses 0-127..."));
}

void loop() 
{
  // Step through each address
  for (uint8_t addr = 0; addr < 128; addr++) 
  {
    uint8_t moduleType, moduleSubType, fwVersion;
        
    // Read module type, subtype, and firmware version
    if (rlink.read(addr, 0xFF, 0xFF, RLINK_REG_MOD_TYPE, &moduleType, 1, 100) == 1 &&
        rlink.read(addr, 0xFF, 0xFF, RLINK_REG_MOD_SUBTYPE, &moduleSubType, 1, 100) == 1 &&
        rlink.read(addr, 0xFF, 0xFF, RLINK_REG_SW_VER, &fwVersion, 1, 100) == 1) 
    {
        // Device found; print details
        Serial.println("----------------------------------------");
        Serial.print(F("Found device at address: 0x"));
        Serial.println(addr, HEX);

        Serial.print(F("Type:\t\t"));
        printType(moduleType);
        Serial.print(F(" (0x")); 
        Serial.print(moduleType, HEX);
        Serial.println(F(")"));

        Serial.print(F("SubType:\t"));
        printSubType(moduleType, moduleSubType);
        Serial.print(F(" (0x"));
        Serial.print(moduleSubType, HEX);
        Serial.println(F(")"));

        uint8_t fwMajor = (fwVersion >> 4) & 0x0F;
        uint8_t fwMinor = fwVersion & 0x0F;

        Serial.print(F("Firmware:\tV"));
        Serial.print(fwMajor);
        Serial.print(F("."));
        Serial.println(fwMinor);
      }

      // Small delay to allow bus to settle
      delay(10);
  }

  Serial.println("----------------------------------------");
  Serial.println("Scan complete.");
  while (1);
}

// Print out the device type
void printType(uint8_t type)
{
  switch (type)
  {
    case 0x02:
      Serial.print(F("Relay"));
      break;

      default:
        Serial.print(F("Unknown"));
        break;
  }
}

// Print out the device subtype (depends on type)
void printSubType(uint8_t type, uint8_t sub)
{
  switch (type)
  {
    // Type = Relay
    case 0x02:
      switch (sub)
      {
        // Subtype = 2-channel relay
        case 0x01:
          Serial.print(F("2 Channel"));
          break;

        default:
          Serial.print(F("Unknown"));
          break;
      }
      break;

    default:
      Serial.print(F("Unknown"));
      break;
  }
}