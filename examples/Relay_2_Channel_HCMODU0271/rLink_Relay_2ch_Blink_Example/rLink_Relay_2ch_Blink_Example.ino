/* FILE:    rLink_Relay_2ch_Blink_Example.ino
   DATE:    29/12/25
   VERSION: 1.0.0
   AUTHOR:  Andrew Davies

This sketch demonstrates how to control a 2-channel rLink relay
module using the Arduino SoftwareSerial library.

It is useful for Arduino boards that only have a single, fixed
hardware UART, such as the Uno R3, Nano, Pro Micro, and similar
boards.

If you wish to use your Arduino's hardware serial interface
instead, please see the rLink library forum thread for examples:

https://forum.hobbycomponents.com/viewtopic.php?f=139&t=3184

Please see LICENSE file in the library folder for terms of use.
*/

#include <SoftwareSerial.h>
#include "rLink.h"

#define RX_PIN   2
#define TX_PIN   3
#define DIR_PIN  4
#define RLY_ADD  0x02

// Create SoftwareSerial instance
SoftwareSerial softserial(RX_PIN, TX_PIN); // RX, TX

// Create rLink bus using SoftwareSerial
rLink rlink(softserial, DIR_PIN);

// Create relay module at address 2 and attach to the rLink bus
rLinkRelay2CH relay(rlink, RLY_ADD);


void setup()
{
  // Start the software serial interface
  softserial.begin(9600);

  // Initialise rLink library.
  rlink.init();
}


void loop()
{
  // Turn the relay on
  relay.setRelay(0, true);
  delay(5000);

  // Turn the relay off
  relay.setRelay(0, false);
  delay(5000);
}