/*  Acheived Desired Heading through direct command

    Last Updated: 10/14/2016
*/

#include <SoftwareSerial.h>
#include <SPI.h>
#include "Wire.h"
#include "compassCUCI.h"

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

const int rxPin = 3;            // Communication links to Roomba
const int txPin = 4;
const int ddPin = 5;
const int yellowPin = 6;        // On when the Roomba is turning.
const int redPin = 7;           // On when the Roomba is told to turn 0 mm/s.
const int greenPin = 8;         // On when the Roomba sends a pulse. (very fast, may not see)

// Communication links to XBee module
const int transmit_pin = 11;    // to pin 3 of XBee (TX)
const int receive_pin = 12;     // to pin 2 of Xbee (RX)

/* Global variables for transmission and receiving */
unsigned char message = 0;

/* Global variables needed to implement turn functions */
float angle;                      // Heading of Roomba (found from digital compass)
float d_angle;                    // change in angle that Roomba will turn (updates each cycle)
int forward = 0;                  // Speed in mm/s that Roomba wheels turn to move forward - must be in range [-128,128]
int turn;                         // Speed in mm/s that Roomba wheels turn to rotate a given d_angle - must be in range [-128,128]
int WheelDir = 1;                 // Determines direction of rotation for FindTurnTime() function (1 = CCW; -1 = CW)
const float pi = 3.1415926;       // pi to 7 decimal places

/* Adjustable Synchronization Parameters */
const float RATIO = 0.35;         // Ratio for amount to turn - must be in range (0 1]
const float EPSILON = 1.0;        // (ideally) Smallest resolution of digital compass (used in PRC function) < 5.0
boolean DHFlag = false;           // Desired Heading function indicator (was the last command not to turn?)

/* Desired Heading variables */
float DesiredHeading;             // Desired Roomba heading in degrees

/* Roomba parameters */
const int WHEELDIAMETER = 72;     // 72 millimeter wheel diameter
const int WHEELSEPARATION = 235;  // 235 millimeters between center of main wheels
const float ENCODER = 508.8;      // 508.8 Counts per wheel revolution

/* Copied from original 'heading_leader' file; */
SoftwareSerial Roomba(rxPin, txPin); // Set up communnication with Roomba
SoftwareSerial XBee(receive_pin, transmit_pin); // Set up communication with Xbee

/* Data Point Collector Setup */
unsigned long deltime;    // Base time for data to the Serial monitor
unsigned long turntime;   // Base time for changing Roomba heading
unsigned long TIMER;      // Base time for blinking green LED
unsigned long datatime;   // Base time for Roomba data query
long sno = 0;

/* Roomba Data collection Setup */
byte BumperByte;
byte waste;
int rWheel;
int lWheel;
boolean gled = LOW;

/* Start up Roomba */
void setup() {
  Serial.begin(115200);
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

  //Light up the Blue Dirt detect light and flash Clean button
  //Syntax: [139] [LED Code] [LED Color] [LED Intensity]
  Roomba.write(byte(139));  // Turn on Dirt Detect light and Green Clean button
  Roomba.write(byte(25));
  Roomba.write(byte(0));
  Roomba.write(byte(128));
  delay(500);
  Roomba.write(byte(139));  // Change Green to Red
  Roomba.write(byte(25));
  Roomba.write(byte(255));
  Roomba.write(byte(128));
  delay(500);
  Roomba.write(byte(139));  // Turn off Clean Button
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

  /* Compass Calibration: */
  //Keep spinning for calibration
  compass_init(1); // Set Compass Gain
  Move(0, -75);    // Set roomba spinning to calibrate the compass
                   // Spins ~2 rotations CCW.
  compass_debug = 1; // Show Debug Code in Serial Monitor (Set to 0 to hide Debug Code)
  compass_offset_calibration(2); // Find compass axis offsets
  Move(0, 0); // Stop spinning after completing calibration
  
      //Receiver Setup
  delay(500); 
  Serial.println(" Setup complete");
  digitalWrite(yellowPin, HIGH);
  delay(500);
  digitalWrite(yellowPin, LOW);
  digitalWrite(greenPin, LOW);  // say we've finished setup
  while(Roomba.available() > 0) {
    Serial.print((char)Roomba.read()); // Clear out Roomba buffer to Serial monitor
  }
  while(XBee.available()){ // Clear out Xbee buffer
    message = XBee.read();
  }
  message = 0; // Clear message variable
  
  /* Wait for command to initialize synchronization */
  delay(1000);
  /* Initialize Roomba */
  turntime = millis();          // Set base value for turning
  
  DesiredHeading = 0;           // Start heading to zero degrees

  /* Get Roomba to face Desired Heading before continuing. */
  while(millis() - turntime <= 4000) {
    angle = Calculate_Heading();  // Determine initial heading information
    DH_Turn();
  }
  // Ask for initial Roomba data
  Data_Query();
  delay(100);
  Read_Data();
  sno = 0;                      // Reset data point counter
  Serial.println();
  Print_Heading_Data();         // Display initial heading information
  
  deltime = millis();           // Set base value for data output.
  datatime = millis();          // Set base value for Roomba query
  TIMER = millis();             // Set base value for green LED blink
  forward = 75; // Change forward speed of Roomba
  Move(forward, 0);
}

void loop() { // Swarm "Heading Synchronizaiton" Code
  /* Read angle from compass */
  angle = Calculate_Heading();        // Set angle from the compass reading

  /* Ask Roomba for data packets */
  if (millis() - datatime >= 100) {
    datatime += 100;  // Reset base value for data query
    Data_Query(); // Ask for data packets from Roomba
  }
  
  // Read Data from Roomba, if any is available
  Read_Data();

  DH_Turn();
 
  /* Send to Serial monitor a data point */
  if (millis() - deltime >= 500) { // If 1 second = 1000 milliseconds have passed...
    deltime += 500;     // Reset base value for data points
    sno++; // Increment the data point number
    Serial.println(";");
    Print_Heading_Data();
  }
  
  // Blinks the green light to show that it is going through the loop
  if (millis() - TIMER > 600) {
    TIMER = millis();
    gled = !gled; // switch state of the green LED
    digitalWrite(greenPin, gled);
  }

}/* Go back and check everything again. Should be fast */

/* SUBROUTINES */
void Data_Query(void) {
  Roomba.write(byte(149));    // Ask for a query
  Roomba.write(byte(3));      // Ask for # of packets
  Roomba.write(byte(7));      // Ask for Bumper Byte (1 byte)
  Roomba.write(byte(43));     // Ask for Requested Right velocity (2 bytes)
  Roomba.write(byte(44));     // Ask for Requested Left velocity (2 bytes)
  //Roomba.write(byte(22));     // Ask for voltage packet
}

void Read_Data(void) {
  if (Roomba.available() > 4) {
    BumperByte = Roomba.read();   // First byte (reads Bumper data)
    
    rWheel = ((Roomba.read()<<8) | Roomba.read()); // Reads two bytes about rWheel
    lWheel = ((Roomba.read()<<8) | Roomba.read()); // Reads two bytes about lWheel
    
    // erase buffer
    while(Roomba.available() > 0) {
      waste = Roomba.read();
    }
  }
}

/* Set Roomba spin to acheive the value of DesiredHeading */
void DH_Turn(void) {
  // DesiredHeading is the set point for the heading
  // EPSILON is desirably small (probably in range [0.5, 1])
  // Need the amount of spin to be less than (2 * # of cycles through main loop in 1 second)
     // or may need to grade the amount of spin (proportional to amount of heading change)
  if (angle < (DesiredHeading + EPSILON) && angle > (DesiredHeading - EPSILON) && DHFlag == false) {
    // If it's not moving, and I'm close enough...
    digitalWrite(yellowPin, LOW); // Say we have stopped turning.
    return; // Leave function
  }
  /*
   * DHFlag = false if last command was to not spin
   * DHFlag = true if last command was to spin
   * if (angle - DesiredHeading < EPSILON && DHFlag == false)
   *    then return from function (return to loop)
   */
  
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
      DHFlag = true;
    } else if ((angle < (360 + DesiredHeading - EPSILON)) && (angle >= (DesiredHeading + 180)) ) {
      // Spin Right (CW)
      Move(forward, spinValue);
      DHFlag = true;
    } else { // if ((360 + DesiredHeading - EPSILON) < angle < (DesiredHeading + EPSILON))
      // Stop Spinning
      Move(forward, 0);
      DHFlag = false;
      digitalWrite(yellowPin, LOW); // Say we have stopped turning.
    }
  } else if(DesiredHeading < 180) { // and DesiredHeading >= EPSILON...
    if(angle > (DesiredHeading + EPSILON) && angle < (DesiredHeading + 180) ) {
      // Spin Left (CCW)
      Move(forward, -spinValue);
      DHFlag = true;
    } else if ((angle < (DesiredHeading - EPSILON)) || (angle >= (DesiredHeading + 180)) ) {
      // Spin Right (CW)
      Move(forward, spinValue);
      DHFlag = true;
    } else { // if ((DesiredHeading - EPSILON) < angle < (DesiredHeading + EPSILON))
      // Stop Spinning
      Move(forward, 0);
      DHFlag = false;
      digitalWrite(yellowPin, LOW); // Say we have stopped turning.
    }
  } else if(DesiredHeading < (360 - EPSILON)) { // and DesiredHeading >= 180)...
    if ((angle < (DesiredHeading - EPSILON)) && (angle > (DesiredHeading - 180)) ) {
      // Spin Right (CW)
      Move(forward, spinValue);
      DHFlag = true;
    } else if ((angle > (DesiredHeading + EPSILON)) || (angle <= (DesiredHeading - 180)) ) {
      // Spin Left (CCW)
      Move(forward, -spinValue);
      DHFlag = true;
    } else {  // if ((DesiredHeading - EPSILON) < angle < (DesiredHeading + EPSILON))
      // Stop Spinning
      Move(forward, 0);
      DHFlag = false;
      digitalWrite(yellowPin, LOW); // Say we have stopped turning.
    }
  } else { // if DesiredHeading >= (360 - EPSILON)...
    if ((angle < (DesiredHeading - EPSILON)) && (angle > (DesiredHeading - 180)) ) {
      // Spin Right (CW)
      Move(forward, spinValue);
      DHFlag = true;
    } else if ((angle > (DesiredHeading + EPSILON - 360)) && (angle <= (DesiredHeading - 180)) ) {
      // Spin Left (CCW)
      Move(forward, -spinValue);
      DHFlag = true;
    } else {  // if ((DesiredHeading - EPSILON) < angle < (DesiredHeading + EPSILON))
      // Stop Spinning
      Move(forward, 0);
      DHFlag = false;
      digitalWrite(yellowPin, LOW); // Say we have stopped turning.
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
  Serial.print(", ");
  Serial.print(compass_x_scalled);        // Raw X magnetometer value
  Serial.print(", ");
  Serial.print(compass_y_scalled);        // Raw Y magnetometer value
  Serial.print(", ");
  Serial.print(compass_z_scalled);        // Raw Z magnetometer value
  Serial.print(", ");
  Serial.print(BumperByte);    // Roomba Bumber value
  Serial.print(", ");
  Serial.print(rWheel);        // Right Wheel Encoder value
  Serial.print(", ");
  Serial.print(lWheel);        // Left Wheel Encoder value
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
