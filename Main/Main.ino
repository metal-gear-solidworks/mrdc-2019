#include "XBee.h"

// pin constants


// other constants
#define BAUDRATE 9600 // serial baud rate
#define TIMEOUT 50 // millisecond timeout for wireless comm

// XBee setup
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();

void failsafe() {
  // TODO: finish failsafe code
}

void setup() {
  Serial.begin(BAUDRATE);
  xbee.setSerial(Serial);
}

void loop() {
  xbee.readPacket(TIMEOUT);

  if (xbee.getResponse().isAvailable()) {
    // packet received
    
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      // received 16 bit addressed rx packet

      xbee.getResponse().getRx16Response(rx16);
      // data of bit 'i' can be accessed with xbee.getData(i)
      // TODO: process data
    }
  }

  else if (xbee.getResponse().isError()) {
    return; // if the packet has an error, try again
  }

  else {
    failsafe(); // if the comm times out, activate the failsafe
  }
}
