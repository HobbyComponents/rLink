/* FILE:    rLink_Change_Address_Example.ino
   DATE:    29/12/25
   VERSION: 1.0.0
   AUTHOR:  Andrew Davies

This sketch demonstrates how to change the address of an rLink
module using a SoftwareSerial interface. 

It assumes the module is currently at a known address and that
you wish to assign it a new address. The change will be stored
in the module's non-volatile memory and will survive power cycles.

REMEMBER to set the current address, new address, type and subtype
below before running !

Please see the LICENSE file in the library folder for terms of
use.
*/

#include <SoftwareSerial.h>
#include <rLink.h>

// SoftwareSerial pins
#define RX_PIN   2
#define TX_PIN   3
#define DIR_PIN  4

// Current address of the module you want to change
#define CURRENT_ADDR    0x02
// New address to assign to the module (0–127)
#define NEW_ADDR        0x03
// This must match your module type
#define MODULE_TYPE     0x02
// This must match your module subType
#define MODULE_SUBTYPE  0x01


// Create SoftwareSerial instance
SoftwareSerial softSerial(RX_PIN, TX_PIN);

// Create rLink bus object
rLink rlink(softSerial, DIR_PIN);

void setup() 
{
    Serial.begin(9600);
    while (!Serial);

    Serial.println("rLink Address Change Example Starting...");

    // Start software serial
    softSerial.begin(9600);

    // Initialize rLink bus
    rlink.init();

    Serial.print("Changing module at address 0x");
    Serial.print(CURRENT_ADDR, HEX);
    Serial.print(" to new address 0x");
    Serial.println(NEW_ADDR, HEX);

    // Create a module instance at the current address
    rLinkModule module(rlink, CURRENT_ADDR, MODULE_TYPE, MODULE_SUBTYPE);

    // Change the module address
    module.setAddress(NEW_ADDR);

    Serial.println("Address change command sent.");

    // Confirm by reading the module firmware version from new address
    uint8_t fwVersion;
    if (rlink.read(NEW_ADDR, MODULE_TYPE, MODULE_SUBTYPE, RLINK_REG_MOD_TYPE, &fwVersion, 1, 100) == 1)
    {
        Serial.print("Module is now responding at new address 0x");
        Serial.println(NEW_ADDR, HEX);
    }
    else
    {
        Serial.println("No response from module at new address!");
    }
}

void loop() 
{
    // Nothing required here
}