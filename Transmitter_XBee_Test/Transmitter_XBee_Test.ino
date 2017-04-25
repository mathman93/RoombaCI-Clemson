/* Roomba XBee Transmitter Controller
 *  To be used with Receiver_XBee_Test
 *  Last Updated: 4/25/17
 */

#include <SoftwareSerial.h>
const int yellowPin = 6;      // Indicate Setup of Transmitter  
const int redPin = 7;         // Indicate sending spin character
const int greenPin = 8;       // Indicate sending forward character
const int transmit_pin = 11;  // Xbee Transmitter pin
const int receive_pin = 12;   // Xbee Receiver pin

SoftwareSerial XBee(receive_pin, transmit_pin); // Set up communication with Xbee

void setup() {
  Serial.begin(115200);        // Begin communication with the Serial Monitor
  display_Running_Sketch();    // Show sketch information in the serial monitor at startup
  
  pinMode(yellowPin, OUTPUT);  // LED lights
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);

  XBee.begin(57600); // Declare XBee communication baud rate
  
  //Receiver Setup
  delay(500);
  Serial.println("Transmitter Setup complete");
  digitalWrite(yellowPin, HIGH);
  delay(500);
  digitalWrite(yellowPin, LOW);

  delay(500); // Give it a second...
}

void loop() {
  sendPulse();
  delay(100);
  sendPulse();
  delay(50);
  sendPalse();
  delay(100);
  sendPalse();
  delay(50);
}

/* SUBROUTINES */
/* Sends out pulse */
void sendPulse() {
  
  digitalWrite(greenPin, HIGH); // Tell me that I'm sending a pulse
  XBee.write("a"); // Pulse
  digitalWrite(greenPin, LOW);  // Tell me that I'm done sending a pulse
}

/* Sends out palse */
void sendPalse() {
  //Serial.println("Reset Pulse Sent.");    // Include for debugging 
  digitalWrite(redPin, HIGH);// Tell me that I'm sending a palse
  XBee.write("b"); // Palse
  digitalWrite(redPin, LOW);  // Tell me that I'm done sending a palse
}

/* Displays the Sketch running on the Arduino. Use at startup on all code. */
void display_Running_Sketch(void) {
  /* Find the necessary informaiton */
  String the_path = __FILE__;
  int slash_loc = the_path.lastIndexOf('\\');
  String the_cpp_name = the_path.substring(slash_loc + 1);
  /* Display to the Serial Monitor */
  Serial.print("\nSketch Name: ");
  Serial.println(the_cpp_name);
  Serial.print("Compiled on: ");
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.print(__TIME__);
  Serial.println("\n");
}
