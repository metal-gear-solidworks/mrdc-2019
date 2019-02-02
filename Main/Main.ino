#include "lib/xbee-arduino/XBee.h"

// pin constants


// other constants
#define BAUDRATE = 9600 // serial baud rate
#define TIMEOUT = 50 // millisecond timeout for wireless comm

// XBee setup
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();

void setup() {
  Serial.begin(BAUDRATE);
  xbee.setSerial(Serial);
}

void loop() {
  
}
