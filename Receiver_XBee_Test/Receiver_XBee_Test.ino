/* Roomba XBee Receiver Controller
 *  Listens to values coming in, and displays them to the serial monitor
 *  Last Updated: 4/25/17
 */
 
#include <SoftwareSerial.h>
const int yellowPin = 6;      // Indicate Setup of Transmitter  
const int redPin = 7;         // Indicate sending spin character
const int greenPin = 8;       // Indicate sending forward character
const int transmit_pin = 11;  // Xbee Transmitter pin
const int receive_pin = 12;   // Xbee Receiver pin

SoftwareSerial XBee(receive_pin, transmit_pin); // Set up communication with Xbee

unsigned char message = 0;

/* Variables for Light Blink */
unsigned long LightBlink = 0;
boolean gled = LOW;
int count = 0;

void setup() {
  Serial.begin(115200);         // Begin communication with the Serial Monitor
  display_Running_Sketch();    // Show sketch information in the serial monitor at startup
  
  pinMode(yellowPin, OUTPUT);  // LED lights
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);
 
  XBee.begin(57600); // Declare XBee communication baud rate
  
  //Receiver Setup
  delay(500);
  Serial.println("Receiver Setup complete");
  digitalWrite(yellowPin, HIGH);
  delay(500);
  digitalWrite(yellowPin, LOW);

  delay(500); // Give it a second...
  while (XBee.available()) {
    message = XBee.read(); // Clear out XBee buffer
  }
}

void loop() {
  /* Receive a pulse signal */
  recievePulse();
  if (message == 'b') {
    count++;
    Serial.print(count);
    Serial.print(": ");
    Serial.print((char)message);
    Serial.println(" Palse");
    message = 0;
  } else if (message == 'a') {
    count++;
    Serial.print(count);
    Serial.print(": ");
    Serial.print((char)message);
    Serial.println(" Pulse");
    message = 0;
  }

  recievePulse();

  if (millis() - LightBlink >= 500) {
    LightBlink = millis();
    gled = !gled;
    digitalWrite(greenPin, gled);
  }

  recievePulse();
}

/* SUBROUTINES */
void recievePulse() {
  if (XBee.available()) {  // If I receive a pulse ...
    message = XBee.read(); // Return pulse character
  }
}

/* Displays the Sketch running on the Arduino. Use at startup on all code. */
void display_Running_Sketch (void) {
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
