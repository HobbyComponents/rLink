/* FILE:    rLink_Relay_2ch_OnTime_Example.ino
   DATE:    29/12/25
   VERSION: 1.0.0
   AUTHOR:  Andrew Davies

This sketch demonstrates how to set the relay on-time feature
for a 2-channel rLink relay module. Setting the on-time for a
specific relay will cause that relay to automatically turn off
after being triggered for a set duration.

The on-time value is specified in units of 100 ms. For example,
a value of 50 corresponds to a 5-second on-time.

To disable the on-time for a relay, set the on-time to 0.

Note: The on-time is stored in the module's non-volatile memory
and will survive power cycles, so you only need to set it once!

Please see the LICENSE file in the library folder for terms of
use.
*/

#include <SoftwareSerial.h>
#include <rLink.h>

#define RX_PIN   2
#define TX_PIN   3
#define DIR_PIN  4
#define RLY_ADD  0x02

// Create SoftwareSerial instance
SoftwareSerial softserial(RX_PIN, TX_PIN);

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

  // Set relay 0 on-time to 5 seconds (50 x 100 ms)
  relay.setOnTime(0, 50);

  // Turn relay 0 ON
  relay.setRelay(0, true);
}


void loop()
{
    // Nothing required here.
    // Relay 0 will automatically turn OFF after the on-time expires.
}