// We'll use SoftwareSerial to communicate with the XBee
// Input data via the serial monitor and it will send the data you inputted out to the other xbees
// Data sent from other xbees will be received and printed out to the serial monitor
#include <SoftwareSerial.h>
// XBee's DOUT (TX, pin 2 of xbee) is connected to pin 2 of Arduino board (Arduino's Software RX)
// XBee's DIN (RX, pin 3 of xbee) is connected to pin 12 of Arduino board (Arduino's Software TX)
SoftwareSerial XBee(2, 12); // RX, TX

void setup()
{
  // set up baud rates for XBee and Serial monitor
  XBee.begin(57600);
  Serial.begin(115200);
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
