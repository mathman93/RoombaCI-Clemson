/* Compass Measurement Testing
 * 
 * 
 * Last Updated: 5/23/2016
 */
 
#include <SoftwareSerial.h>
#include <SPI.h>
#include "VirtualWire.h"
#include <Wire.h>
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
float x, y, z; //triple axis data

/* Global variables needed to implement turn functions */
float angle;                     // Heading of Roomba (found from digital compass)
float angle2;                    // Correct Heading calculation :)
int forward = 0;                 // Speed in mm/s that Roomba wheels turn to move forward - must be in range [0,128]
float pi = 3.1415926;            // pi to 7 decimal places

unsigned long TIMER;                       // Amount of time in milliseconds that the Roomba spends turning (can be adjusted)
//TIMER = 2.0507 seconds results in approximately 1 degree of spin per 1 mm/s turn speed.

/* Sync Counter Setup */
unsigned long millisCounter;                // Base time for Counter difference calculation

/* Turn Counter Setup */
unsigned long turnCounter;   // Base time for Turn Timer difference calculation

/* Copied from original 'heading_leader' file; */
SoftwareSerial Roomba(rxPin, txPin); // Set up communnication with Roomba

unsigned long deltime = 0;
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

  // wake up the robot (Is this needed, since we are reseting the roomba 9 lines later? See page 7 of Roomba manual.)
  digitalWrite(ddPin, HIGH);
  delay(100);
  digitalWrite(ddPin, LOW);
  delay(500);
  digitalWrite(ddPin, HIGH);
  delay(2000);
  
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
  // May not need, since this is part of the "compass_reading()" function in "compass.cpp"
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
  Serial.print("setup");

  vw_rx_start();       // Start the receiver PLL running

  digitalWrite(yellowPin, HIGH);
  Serial.println("...complete");
  delay(500);
  digitalWrite(yellowPin, LOW);

  delay(1000);
/* Compass Calibration: */
  //Keep spinning for calibration
  compass_init(1); // Set Compass Gain
  Move(0, -100);    // Set roomba spinning to calibrate the compass
                   // Spins 5 + 1/4 rotations CCW.
  compass_debug = 1; // Show Debug Code in Serial Monitor (Set to 0 to hide Debug Code)
  compass_offset_calibration(2); // Find compass axis offsets
  Move(0, 0); // Stop spinning after completing calibration
  /* Wait for command to initialize synchronization */
  
  digitalWrite(greenPin, LOW);  // say we've finished setup
  delay(1000);

  Serial.print("["); // For MATLAB matrix form
  calculate_heading();  // Find Initial heading Information
  Print_Heading_Data(); // Display Initial heading Information
  TIMER = 60000; // Milliseconds

  Move(0,30); // millimeters per second
  turnCounter = millis();  
  deltime = millis();           // Set base value for data output.
  millisCounter = millis();     // Set base value for counter.
  
}

void loop() { // Swarm "Heading Synchronizaiton" Code
 
  calculate_heading(); // Update heading information
  
  /* Stop turning if TIMER has passed */
  if ((millis() - turnCounter) >= TIMER && (millis() - turnCounter) < (TIMER + 1000)) { // If I've been turning long enough
    Move(0, 0);               // Stop turning
    Serial.println("]");    // For MATLAB matrix form
    digitalWrite(yellowPin, LOW);   // Tell me that the robot is done turning
    delay(2000);
    Serial.print("[");
  }

  /* Send to Serial monitor a data point */
  if (millis() - deltime >= 100) { // If 1 second = 1000 milliseconds have passed...
    deltime = millis();     // Reset base value for data points
    sno++;
    Serial.println(";");    // For MATLAB matrix form
    Print_Heading_Data();
  }

}// Go back and check everything again. 

/* SUBROUTINES */
/* General Wheel Motor command function.
    X = common wheel speed (mm/s); Y = differential wheel speed;
    X > 0 -> forward motion; Y > 0 -> CW motion
    This function is in essence a combination of Spin and Straight,
    allowing for both turning and forward motion */
void Move(int X, int Y) {
  /* Local Variables needed for function */
  int RWHigh, LWHigh;
  /* Determine what the high 8-bits should be for each wheel*/
  /* Right Wheel High byte */
  if (X - Y >= 0) { // If the desired right wheel speed is a positive number
    RWHigh = 0;    // Positive filler for a 2's complement number
  }  else  {       // If the desired right wheel speed is a negative number
    RWHigh = 255;  // Negative filler for a 2's complement number
  }
  /* Left Wheel High byte */
  if (X + Y >= 0) { // If the desired left wheel speed is a positive number
    LWHigh = 0;    // Positive filler for a 2's complement number
  }  else  {       // If the desired left wheel speed is a negative number
    LWHigh = 255;  // Negative filler for a 2's complement number
  }

  Roomba.write(byte(145));  // Syntax: [145] [RW High 8-bit] [RW Low 8-bit] [LW High 8-bit] [LW Low 8-bit]
  Roomba.write(byte(RWHigh));
  Roomba.write(byte(X - Y)); // Combine common and differential speeds for right wheel
  Roomba.write(byte(LWHigh));
  Roomba.write(byte(X + Y)); // Combine common and differential speeds for left wheel
}

/* Perform Heading Calculations */
void calculate_heading (void) {
  compass_heading(); // Get Bearing information from compass
  // Read angle from compass 
  angle = bearing;   // Set the value of our angle
  x = compass_x_scalled;
  y = compass_y_scalled;
  z = compass_z_scalled;
  // Calculated desired angle from raw data
  angle2 = (atan2(y,x) * (180/pi) - 90);
  if (angle2 < 0){
  angle2 = angle2 + 360;
  }
}

/* Display Important Heading Data to Serial Monitor */
void Print_Heading_Data (void) {
  Serial.print(sno);       // Data point number
  Serial.print(", ");    // For MATLAB matrix form
  Serial.print(angle);     // Robot Heading (from compass.cpp)
  Serial.print(", ");    // For MATLAB matrix form
  Serial.print(angle2);    // Robot Heading (desired heading)
  Serial.print(", ");    // For MATLAB matrix form
  Serial.print(x);         // Raw X value
  Serial.print(", ");    // For MATLAB matrix form
  Serial.print(y);         // Raw Y value
  Serial.print(", ");    // For MATLAB matrix form
  Serial.print(z);         // Raw Z value
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
