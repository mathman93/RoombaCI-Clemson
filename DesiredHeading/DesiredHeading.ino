/*  Acheived Desired Heading through direct command

    Last Updated: 6/13/2016
*/

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

/* Global variables needed to implement turn functions */
float angle;                      // Heading of Roomba (found from digital compass)
float d_angle;                    // change in angle that Roomba will turn (updates each cycle)
int forward = 0;                  // Speed in mm/s that Roomba wheels turn to move forward - must be in range [-128,128]
int turn;                         // Speed in mm/s that Roomba wheels turn to rotate a given d_angle - must be in range [-128,128]
int WheelDir = 1;                 // Determines direction of rotation for FindTurnTime() function (1 = CCW; -1 = CW)
const float pi = 3.1415926;       // pi to 7 decimal places

/* Adjustable Synchronization Parameters */
const float RATIO = 0.35;         // Ratio for amount to turn - must be in range (0 1]
int TIMER = 1025;                 // Amount of time in milliseconds that the Roomba spends turning (can be adjusted)
// TIMER = 2.0507 seconds results in approximately 1 degree of spin per 1 mm/s turn speed.
const float EPSILON = 1.0;        // (ideally) Smallest resolution of digital compass (used in PRC function) - 5.0
const int SPEED = 40;             // Amount of speed in mm/s that the wheels will spin when the Roomba turns (can be adjusted)

/* Desired Heading variables */
float DesiredHeading;             // Desired Roomba heading in degrees

/* Roomba parameters */
const int WHEELDIAMETER = 72;     // 72 millimeter wheel diameter
const int WHEELSEPARATION = 235;  // 235 millimeters between center of main wheels
const float ENCODER = 508.8;      // 508.8 Counts per wheel revolution

/* Copied from original 'heading_leader' file; */
SoftwareSerial Roomba(rxPin, txPin); // Set up communnication with Roomba

/* Data Point Collector Setup */
unsigned long deltime;
unsigned long turntime;
long sno = 0;

/* Start up Roomba */
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
  Serial.print("STARTING ROOMBA");
  Roomba.write(byte(128));  // START
  delay(50);
  Roomba.write(byte(131));  // CONTROL
  //131 - Safe Mode
  //132 - Full mode (Be ready to catch it!)

  //Light up the blue dirt detect light (Needs better comments!)
  Roomba.write(byte(139));
  Roomba.write(byte(25));
  Roomba.write(byte(0));
  Roomba.write(byte(128));
  delay(500);
  Roomba.write(byte(139));
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
  Serial.print("Receiver setup");

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
  /* Initialize Roomba */
  angle = Calculate_Heading();  // Determine initial heading information
  deltime = millis();           // Set base value for data output.
  turntime = millis();          // Set base value for turning
  sno = 0;                      // Reset data point counter
  Print_Heading_Data();         // Display initial heading information
  DesiredHeading = 0;           // Start heading to zero degrees
}

void loop() { // Swarm "Heading Synchronizaiton" Code
  /* Read angle from compass */
  angle = Calculate_Heading();        // Set angle from the compass reading

  //if (millis() - turntime >= 10000) {
  //  DesiredHeading += 90;
  //  if (DesiredHeading >= 360) {
  //    DesiredHeading = 0;
  //  }
  //  turntime += 10000;
  //}

  DH_Turn();
 
  /* Send to Serial monitor a data point */
  if (millis() - deltime >= 50) { // If 1 second = 1000 milliseconds have passed...
    deltime += 50;     // Reset base value for data points
    sno++; // Increment the data point number
    Serial.println(";");
    Print_Heading_Data();
  }

}/* Go back and check everything again. Should be fast */

/* SUBROUTINES */
/* Set Roomba spin to acheive the value of DesiredHeading */
void DH_Turn(void) {
  // DesiredHeading is the set point for the heading
  // EPSILON is desirably small (probably in range [0.5, 1])
  // Need the amount of spin to be less than (2 * # of cycles through main loop in 1 second)
     // or may need to grade the amount of spin (proportional to amount of heading change)
  int spinValue; // Speed of wheels in mm/s to spin toward DesiredHeading
  float holder;
  float thresh1 = 25; // First threshold value
  float thresh2 = 5;  // Second threshold value
  holder = angle - DesiredHeading;
  holder = abs(holder);   // absolute difference of where I am (angle) and where I want to be (DesiredHeading)
  // Determine spin speed based on Thresholds
  if (holder > thresh1 && holder < (360 - thresh1)) {
    spinValue = 100;   // Move faster
  } else if (holder > thresh2 && holder < (360 - thresh2)) { // and <= thresh1
    spinValue = 50;   // Move fast
  } else { // if (holder <= thresh2)
    spinValue = 15;   // Move slow (keeps down oscillations due to main loop execution rate)
  }
  // Determine direction of spin
  if(DesiredHeading < EPSILON) {
    if(angle > (DesiredHeading + EPSILON) && angle < (DesiredHeading + 180) ) {
      // Spin Left (CCW)
      Move(forward, -spinValue);
    } else if ((angle < (360 + DesiredHeading - EPSILON)) && (angle >= (DesiredHeading + 180)) ) {
      // Spin Right (CW)
      Move(forward, spinValue);
    } else { // if ((360 + DesiredHeading - EPSILON) < angle < (DesiredHeading + EPSILON))
      // Stop Spinning
      Move(forward, 0);
    }
  } else if(DesiredHeading < 180) { // and DesiredHeading >= EPSILON...
    if(angle > (DesiredHeading + EPSILON) && angle < (DesiredHeading + 180) ) {
      // Spin Left (CCW)
      Move(forward, -spinValue);
    } else if ((angle < (DesiredHeading - EPSILON)) || (angle >= (DesiredHeading + 180)) ) {
      // Spin Right (CW)
      Move(forward, spinValue);
    } else { // if ((DesiredHeading - EPSILON) < angle < (DesiredHeading + EPSILON))
      // Stop Spinning
      Move(forward, 0);
    }
  } else if(DesiredHeading < (360 - EPSILON)) { // and DesiredHeading >= 180)...
    if ((angle < (DesiredHeading - EPSILON)) && (angle > (DesiredHeading - 180)) ) {
      // Spin Right (CW)
      Move(forward, spinValue);
    } else if ((angle > (DesiredHeading + EPSILON)) || (angle <= (DesiredHeading - 180)) ) {
      // Spin Left (CCW)
      Move(forward, -spinValue);
    } else {  // if ((DesiredHeading - EPSILON) < angle < (DesiredHeading + EPSILON))
      // Stop Spinning
      Move(forward, 0);
    }
  } else { // if DesiredHeading >= (360 - EPSILON)...
    if ((angle < (DesiredHeading - EPSILON)) && (angle > (DesiredHeading - 180)) ) {
      // Spin Right (CW)
      Move(forward, spinValue);
    } else if ((angle > (DesiredHeading + EPSILON - 360)) && (angle <= (DesiredHeading - 180)) ) {
      // Spin Left (CCW)
      Move(forward, -spinValue);
    } else {  // if ((DesiredHeading - EPSILON) < angle < (DesiredHeading + EPSILON))
      // Stop Spinning
      Move(forward, 0);
    } // End "else angle"
  } // End "else DesiredHeading"
} // End Function

/* General Wheel Motor command function.
    X = common wheel speed (mm/s); Y = differential wheel speed;
    X > 0 -> forward motion; Y > 0 -> CW motion
    This function allows for both turning and forward motion.
    Updated to use bitshift logic.
    Error may result if |X|+|Y| > 500 (Max value is 500)
    */
void Move(int X, int Y) {
 unsigned int RW = (X - Y);
 unsigned int LW = (X + Y);
 Roomba.write(byte(145));
 Roomba.write(byte((RW & 0xff00) >> 8));
 Roomba.write(byte(RW & 0xff));
 Roomba.write(byte((LW & 0xff00) >> 8));
 Roomba.write(byte(LW & 0xff));
}

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
  //Serial.print(", ");
  //Serial.print(0); // Counter value (Not used)
  Serial.print(", ");
  Serial.print(compass_x_scalled);        // Raw X magnetometer value
  Serial.print(", ");
  Serial.print(compass_y_scalled);        // Raw Y magnetometer value
  Serial.print(", ");
  Serial.print(compass_z_scalled);        // Raw Z magnetometer value
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
