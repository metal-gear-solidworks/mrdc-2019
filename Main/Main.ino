#include "XBee.h"

// pin constants


// other constants
#define BAUDRATE 9600 // serial baud rate
#define TIMEOUT 50 // millisecond timeout for wireless comm

// XBee setup
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();

// controller variables
bool a = false;
bool b = false;
bool x = false;
bool y = false;
bool dup = false;
bool ddown = false;
bool dleft = false;
bool dright = false;
int leftY = 0;
int rightY = 0;
int ltrigger = 0;
int rtrigger = 0;


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

      // set controller variables according to the packet
      // data of bit 'i' can be accessed with rx16.getData(i)
      a = (rx16.getData(0) & 1 != 0) ? true: false; // & 00000001
      b = (rx16.getData(0) & 2 != 0) ? true: false; // & 00000010
      x = (rx16.getData(0) & 4 != 0) ? true: false; // & 00000100
      y = (rx16.getData(0) & 8 != 0) ? true: false; // & 00001000
      dup = (rx16.getData(1) & 1 != 0) ? true: false;
      ddown = (rx16.getData(1) & 2 != 0) ? true: false;
      dleft = (rx16.getData(1) & 4 != 0) ? true: false;
      dright = (rx16.getData(1) & 8 != 0) ? true: false;
      leftY = rx16.getData(3);
      rightY = rx16.getData(5);
      ltrigger = rx16.getData(6);
      rtrigger = rx16.getData(7);
    }
  }

  else if (xbee.getResponse().isError()) {
    return; // if the packet has an error, try again
  }

  else {
    failsafe(); // if the comm times out, activate the failsafe
  }
}
