/* Roomba Transmitter Controller
 *  To be used with Receiver_RF_Test
 *  Last Updated: 9/30/16
 */
#include "VirtualWire.h"   
const int greenPin = 7;       // Indicate sending forward character
const int redPin = 8;         // Indicate sending spin character
const int yellowPin = 11;     // Indicate Setup of Transmitter
const int transmitPin = 12;   // RF Transmitter pin
const int receivePin = 2;     // RF Receiver pin

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
}

void loop() {
  sendFWD();
  delay(1000);
  sendFWD();
  delay(500);
  sendSPN();
  delay(1000);
  sendSPN();
  delay(500);
}

/* SUBROUTINES */
/* Sends out the forward character over the transmitter */
void sendFWD(void) {
  char fwd[2] = {'b'};          // Forward Pulse
  digitalWrite(greenPin, HIGH); // Tell me that I'm sending a forward character
  vw_send((uint8_t *)fwd, strlen(fwd));
  vw_wait_tx();                 // Wait until the whole message is gone
  digitalWrite(greenPin, LOW);  // Tell me that I'm done sending
}

/* Sends out the spin character over the transmitter */
void sendSPN(void) {
  char spn[2] = {'a'};          // Spin Pulse
  digitalWrite(redPin, HIGH);   // Tell me that I'm sending a spin character
  vw_send((uint8_t *)spn, strlen(spn));
  vw_wait_tx();                 // Wait until the whole message is gone
  digitalWrite(redPin, LOW);    // Tell me that I'm done sending
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
