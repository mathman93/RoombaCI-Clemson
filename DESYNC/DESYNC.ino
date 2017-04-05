#include <SoftwareSerial.h>
#include <SPI.h>
#include "VirtualWire.h"
#include "Wire.h"
#include "compass.h"

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

const int rxPin = 3;            // Communication links to Roomba
const int txPin = 4;
const int ddPin = 5;
const int greenPin = 7;         // On when the Roomba sends a pulse. (very fast, may not see)
const int redPin = 8;           // On when the Roomba is told to turn 0 mm/s.
const int yellowPin = 11;       // On when the Roomba is turning.
const int transmit_pin = 12;    // Communication links to RF transmitter/receiver
const int receive_pin = 2;

/* Global variables for transmission and receiving */
uint8_t buf[VW_MAX_MESSAGE_LEN];
uint8_t buflen = VW_MAX_MESSAGE_LEN;

/* Global variables needed to implement turn functions */
float angle;                      // Heading of Roomba (found from digital compass)
float counter;                    // Global counter for Roomba (works with angle to compute "phase")
float d_angle;                    // Change in angle that Roomba will turn (updates each cycle)
float DesiredHeading;             // Heading set point of Roomba;
int forward = 0;                  // Speed in mm/s that Roomba wheels turn to move forward - must be in range [-400,400]
const float pi = 3.1415926;       // PI to 7 decimal places

/* Adjustable Synchronization Parameters */
const float RATIO = 0.7;         // Ratio for amount to turn - must be in range (0 1]
const float EPSILON = 1.0;        // (ideally) Smallest resolution of digital compass (used in PRC function) - 5.0

/* Roomba parameters */
const int WHEELDIAMETER = 72;     // 72 millimeter wheel diameter
const int WHEELSEPARATION = 235;  // 235 millimeters between center of main wheels
const float ENCODER = 508.8;      // 508.8 Counts per wheel revolution

/* Sync Counter Setup */
unsigned long millisCounter;                // Base time for Counter difference calculation
/* Adjust this value to vary the speed/frequency of the counter */
const float counterspeed = 30;                  // Number of times per second that the counter increments - range (0,1000]
const float millisRatio = counterspeed / 1000;  // Counter increments per millisecond
const float millisAdjust = 360000 / counterspeed; // Amount of counter adjustment per firing
const unsigned long counterAdjust = (unsigned long) millisAdjust; // Truncate to unsigned long   

/* Roomba Serial Setup */
SoftwareSerial Roomba(rxPin, txPin); // Set up communnication with Roomba

/* Data Point Collector Setup */
unsigned long deltime;
unsigned long resettime;
long sno = 0;

/* Global Varibales needed to run DESYNC */
const int N = 3;  // Number of Roombas being used
const float ratio1 = 0.5;
const float ratio2 = 0.5;

void setup() {
  
  Serial.begin(57600);
  display_Running_Sketch();     // Show sketch information in the serial monitor at startup
  Serial.println("Loading...");
  pinMode(ddPin,  OUTPUT);      // sets the pins as output
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  Roomba.begin(115200);         // Declare Roomba communication baud rate.
  digitalWrite(greenPin, HIGH); // say we're alive

  // set up ROI to receive commands
  Roomba.write(byte(7));  // RESTART
  delay(10000);
  Serial.print("STARTING ROOMBA.");
  Roomba.write(byte(128));  // START
  delay(50);
  Roomba.write(byte(131));  // CONTROL
  //131 - Safe Mode
  //132 - Full mode (Be ready to catch it!)

  //Light up the blue dirt detect light (Needs better comments!)
  Roomba.write(byte(139)); //Begin (power?) LED control
  Roomba.write(byte(25)); //LED Bits
  Roomba.write(byte(0)); //LED color, 0 should be green
  Roomba.write(byte(128)); //Start command, not sure why you have to send this again...
  delay(500);
  Roomba.write(byte(139)); //Same thing as above, but flash red.
  Roomba.write(byte(25));
  Roomba.write(byte(255));
  Roomba.write(byte(128));
  delay(500);
  Roomba.write(byte(139));
  Roomba.write(byte(25));
  Roomba.write(byte(255));
  Roomba.write(byte(0));
  delay(50);

  //Transmitter Setup
  //Initialize Serial and I2C communications
  Wire.begin();

  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  // Initialise the IO and ISR
  vw_set_tx_pin(transmit_pin);
  vw_set_rx_pin(receive_pin);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);       // Bits per sec

  //Receiver Setup
  delay(1000);
  Serial.print(" Setup");

  vw_rx_start();       // Start the receiver PLL running

  digitalWrite(yellowPin, HIGH);
  Serial.println("...complete");
  delay(500);
  digitalWrite(yellowPin, LOW);
  
  /* Compass Calibration: */
  //Keep spinning for calibration
  compass_init(1); // Set Compass Gain
  Move(0, -75);    // Set roomba spinning to calibrate the compass
                   // Spins ~4 rotations CCW.
  compass_debug = 1; // Show Debug Code in Serial Monitor (Set to 0 to hide Debug Code)
  compass_offset_calibration(2); // Find compass axis offsets
  Move(0, 0); // Stop spinning after completing calibration
  /* Wait for command to initialize synchronization */
  digitalWrite(greenPin, LOW);  // say we've finished setup
  delay(1000);
  angle = Calculate_Heading();  // Throw away first calculation
  delay(200);
  /* Initialize synchronization */
  angle = Calculate_Heading();  // Determine initial heading information
  sendPalse();                  // Send reset Palse
  resetCounters();              // Reset counter values
}

void loop() {
  /* Send a pulse signal */
  if (angle + counter >= 360) { // If my "phase" reaches 360 degrees...
    sendPulse();                                    // Fire pulse
    millisCounter = millisCounter + counterAdjust;  // Adjust base counter.
  } // Ignore if the angle and counter are less than 360 degrees.
}
  /* Receive a pulse signal */
  if (vw_get_message(buf, &buflen)) {  // If I receive a pulse signal (a high bit)... (perhaps something stored in a buffer?)
    //Serial.print((char)buf[0]);    // Include for debugging
    if (buf[0] == 'b') {          // Charater of palse signal
      //Serial.println(" Reset Palse.");    // Include for debugging
      digitalWrite(redPin, HIGH);   // Notify that we received palse
      digitalWrite(greenPin, HIGH);
      resetCounters();
      digitalWrite(redPin, LOW);    // End Notify that we received palse
      digitalWrite(greenPin, LOW);
    } // End If statement
    else if (buf[0] == 'a') {      // Charater of pulse signal
      //Serial.println(" Sync Pulse.");     // Include for debugging
      digitalWrite(yellowPin, HIGH);  // Tell me that the robot is turning
      PRC_DESync(angle + counter); // Find desired amount of turn based on PRC for synchronization
      /* Turn by d_angle */
      DesiredHeading = angle + d_angle;        // Set new desired heading set point
      if (DesiredHeading < 0) {         // Normalize the heading value to between 0 and 360
        DesiredHeading += 360;
      } else if (DesiredHeading >= 360) {
        DesiredHeading -= 360;
      }
    } // End elseif statement
  } // Ignore if no pulse has been received
}

/* SUBROUTINES */
/* Sends out a pulse when phase equals 360 degrees */
void sendPulse() {
  char pulse[2] = {'a'};
  //Serial.println("Sync Pulse Sent.");     // Include for debugging
  digitalWrite(greenPin, HIGH); // Tell me that I'm sending a pulse
  vw_send((uint8_t *)pulse, strlen(pulse));
  vw_wait_tx();                 // Wait until the whole message is gone
  digitalWrite(greenPin, LOW);  // Tell me that I'm done sending a pulse
}

/* Sends out a palse when new Roomba finishes setup */
void sendPalse() {
  char palse[2] = {'b'};
  //Serial.println("Reset Pulse Sent.");    // Include for debugging
  digitalWrite(greenPin, HIGH); // Tell me that I'm sending a palse
  digitalWrite(redPin, HIGH);
  vw_send((uint8_t *)palse, strlen(palse));
  vw_wait_tx();                 // Wait until the whole message is gone
  digitalWrite(greenPin, LOW);  // Tell me that I'm done sending a palse
  digitalWrite(redPin, LOW);
}

void PRC_DESync(float phase) {
  /* Set d_angle, the amount to turn */
  /* Red LED turns on if d_angle is set to 0 */
  if ((phase) < (360/N + EPSILON)) {          // If the phase is less than 360/N...
    d_angle = ratio1*360/N - phase; //gt angle to be 360/N
    digitalWrite(redPin, HIGH); // Indicates received pulse, but no turning.
  } else if ((phase) >= 360-360/N+EPSILON) {     //If phase > 360-360/N degrees...
    /* Increase Heading */
    d_angle = (360 - 360/N - EPSILON - phase) * ratio2;  // Get angle to be 360-360/N
    digitalWrite(redPin, LOW); // Indicates the last pulse received caused a turn
  }// else if ((phase) > EPSILON) {           
    /* Decrease Heading */
//    d_angle = -1 * (phase) * RATIO; // Amount I want to change the heading
//    digitalWrite(redPin, LOW); // Indicates the last pulse received caused a turn
 /* }*/ else {                                  // If the phase is between 360/N & 360-360/N
    d_angle = 0;  // Don't turn at all.
    digitalWrite(redPin, HIGH); // Indicates received pulse, but no turning.
  }
}

/* Reset ALL the counters */
void resetCounters() {
  millisCounter = millis();   // Reset base counter (on all robots)
  deltime = millis();         // Reset base value for data output
  resettime = millis();       // Reset base value for reset timer
  sno = 0;                    // Reset data point counter
  DesiredHeading = angle;     // Reset heading set point
  Print_Heading_Data();       // Display initial heading information

  float Calculate_Heading(void) {
  float t;
  compass_scalled_reading(); // Get Raw data from the compass
  /* Heading Calculation (y-axis front) */
  t = (atan2(compass_y_scalled,compass_x_scalled) * (180/pi) - 90);
  if (t < 0) {
    t = t + 360;
  }

  return t;
}

/* Display Heading Information to the Serial Monitor */
void Print_Heading_Data(void) {
  Serial.print(sno);      // Data point number
  Serial.print(", ");
  Serial.print(angle);    // Robot Heading
  Serial.print(", ");
  Serial.print(counter);  // Counter value
  Serial.print(", ");
  Serial.print(compass_x_scalled);  // Raw X magnetometer value
  Serial.print(", ");
  Serial.print(compass_y_scalled);  // Raw Y magnetometer value
  Serial.print(", ");
  Serial.print(compass_z_scalled);  // Raw Z magnetometer value
  Serial.print(", ");
  Serial.print(DesiredHeading);     // Desired Set point (angle should follow this)
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
