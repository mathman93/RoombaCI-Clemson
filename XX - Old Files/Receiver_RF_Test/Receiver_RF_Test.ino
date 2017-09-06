/* Roomba Receiver Controller
 *  Listens to values coming in, and displays them to the serial monitor
 *  Last Updated: 6/7/16
 */
#include "VirtualWire.h"   
const int greenPin = 7;       // Indicate sending forward character
const int redPin = 8;         // Indicate sending spin character
const int yellowPin = 11;     // Indicate Setup of Transmitter
const int transmitPin = 12;   // RF Transmitter pin
const int receivePin = 2;     // RF Receiver pin

/* Variables for transmitting */
unsigned char buf[VW_MAX_MESSAGE_LEN];
uint8_t buflen = VW_MAX_MESSAGE_LEN;
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
 
  // Initialise the IO and ISR
  vw_set_tx_pin(transmitPin);
  vw_set_rx_pin(receivePin);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);       // Bits per sec
  
  //Receiver Setup
  delay(1000);
  Serial.print("Receiver setup ");
  
  vw_rx_start();       // Start the receiver PLL running 
  
  digitalWrite(yellowPin, HIGH);
  Serial.println("... complete");
  delay(500);
  digitalWrite(yellowPin, LOW);

  delay(1000); // Give it a second...

  LightBlink = millis();
}

void loop() {
  /* Receive a pulse signal */
  recievePulse();
  if (message == 'b') {
    Serial.print((char)message);
    Serial.print(" Reset Palse. ");
    count++;
    Serial.println(count);
    message = 0;
  } else if (message == 'a') {
    Serial.print((char)message);
    Serial.print(" Sync Pulse. ");
    count++;
    Serial.println(count);
    message = 0;
  }

  recievePulse();
  /*
  if (vw_get_message(buf, &buflen)) {  // If I receive a message
    Serial.print((char)buf[0]);
    if (buf[0] == 'b') {          // Charater of palse signal
      Serial.print(" Reset Palse. ");
    } else if (buf[0] == 'a') {
      Serial.print(" Sync Pulse. ");
    }
    count++;
    Serial.println(count);
  }
  */
  if (millis() - LightBlink >= 500) {
    LightBlink = millis();
    gled = !gled;
    digitalWrite(greenPin, gled);
  }

  recievePulse();
}

/* SUBROUTINES */
void recievePulse() {
  if (vw_get_message(buf, &buflen)) {  // If I receive a pulse ...
    message = buf[0]; // Return pulse character
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
