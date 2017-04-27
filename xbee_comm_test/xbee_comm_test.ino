// We'll use SoftwareSerial to communicate with the XBee
// Input data via the serial monitor and it will send the data you inputted out to the other xbees
// Data sent from other xbees will be received and printed out to the serial monitor
#include <SoftwareSerial.h>
// XBee's DOUT (TX, pin 2 of xbee) is connected to pin 11 of Arduino board (Arduino's Software RX)
// XBee's DIN (RX, pin 3 of xbee) is connected to pin 12 of Arduino board (Arduino's Software TX)
SoftwareSerial XBee(11, 12); // RX, TX

void setup()
{
  // set up baud rates for XBee and Serial monitor, they can be different as long as all Xbees are configured to same baud rate
  XBee.begin(57600); // we found this to be the max baud rate of the Xbees, set all Xbee modules to this baud rate
  Serial.begin(115200); // set the Serial monitor to this buad rate
}

void loop()
{
  if (Serial.available())
  { // If data comes in from serial monitor, send it out to XBee
    XBee.write(Serial.read()); // send data to other xbee
    Serial.write("sent"); // print to serial monitor to confirm data was sent
  }
  if (XBee.available())
  { // If data comes in from XBee, write that data out to serial monitor
    Serial.write(XBee.read());
  }
}
