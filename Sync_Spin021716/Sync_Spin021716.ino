/*  Heading Synchronization Code
    Bases the global counter on the clock speed of the Arduino Uno (millis() function).
    Eliminates use of the delay() function to improve synchronization between robots.
    Updated getHeading() subroutine to give correct heading direction.

    Last Updated: 6/1/2016
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

/* Global variables for transmission and receiving */
char pulse[2] = {'a'};
char palse[2] = {'b'};
int x, y, z, i; //triple axis data
//float t;
uint8_t buf[VW_MAX_MESSAGE_LEN];
uint8_t buflen = VW_MAX_MESSAGE_LEN;

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
/* TIMER = 2.0507 seconds results in approximately 1 degree of spin per 1 mm/s turn speed.*/
const float EPSILON = 4.0;        // (ideally) Smallest resolution of digital compass (used in PRC function) - 5.0
const int SPEED = 40;             // Amount of speed in mm/s that the wheels will spin when the Roomba turns (can be adjusted)

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

/* Turn Counter Setup */
unsigned long turnCounter = 0;   // Base time for Turn Timer difference calculation

/* Copied from original 'heading_leader' file; */
SoftwareSerial Roomba(rxPin, txPin); // Set up communnication with Roomba

/* Data Point Collector Setup */
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
  /* Initialize synchronization */
  angle = getHeading();         // Determine initial heading information
  sendPalse();                  // Reset counters for all online robots.
  millisCounter = millis();     // Set base value for counter.
  deltime = millis();           // Set base value for data output.
  sno = 0;                      // Reset data point counter
  Serial.print("[");    // For MATLAB matrix form
  Print_Heading_Data();         // Display initial heading information 


/*Added 3/24/2016
 * This part makes the Roomba turn to heading 200. It is used to see how a Roomba's magnetometer reading
 * differs from other Roomba's.
 * NOTE: Occasionally the Roomba may freak out and spin really fast. 
 */
 /*
digitalWrite(redPin, HIGH);
digitalWrite(greenPin, HIGH);
digitalWrite(yellowPin, LOW);

  int initAngle, angleDiff, currHeading;
  initAngle = 200;
  Serial.println("Format: [current heading]-->[target heading]. [diff between both]");
  while (angleDiff > 5) {
    currHeading = getHeading();
    angleDiff = abs(currHeading - initAngle);
    Serial.print("[setup] ");
    Serial.print(currHeading);
    Serial.print("-->");
    Serial.print(initAngle);
    Serial.print(". headng diff = ");
    Serial.println(angleDiff);
    Move(0, 20);
    delay(100);
  }
  Move(0, 0);
digitalWrite(redPin, LOW);
digitalWrite(greenPin, LOW);
//end 3/24/2016 addition
*/
  
}

void loop() { // Swarm "Heading Synchronizaiton" Code
  /* Read angle from compass */
  angle = getHeading();        // Set angle to the compass reading

  /* Send a pulse signal */
  if (angle + millisRatio * (long)(millis() - millisCounter) >= 360) { // If my "phase" reaches 360 degrees...
    sendPulse();                                    // Fire pulse
    millisCounter = millisCounter + counterAdjust;  // Adjust base counter.
  } // Ignore if the angle and counter are less than 360 degrees.

  /* Receive a pulse signal */
  if (vw_get_message(buf, &buflen)) {  // If I receive a pulse signal (a high bit)... (perhaps something stored in a buffer?)
    if (buf[i] == 'b') {          // Charater of palse signal
      millisCounter = millis();   // Reset base counter (on all robots)
      deltime = millis();         // Reset base value for data output
      sno = 0;                    // Reset data point counter
      Print_Heading_Data();       // Display initial heading information 
    }

    else if (buf[i] == 'a') {      // Charater of pulse signal
      digitalWrite(yellowPin, HIGH);  // Tell me that the robot is turning
      PRC_Sync(angle + millisRatio * (long)(millis() - millisCounter)); // Find desired amount of turn based on PRC for synchronization
      /* Turn by d_angle */           // Now that I have the angle that I want to change, spin by that amount
      // turn = FindTurnSpeed(d_angle, TIMER);     // Calculate the turn speed for that angle and amount of time
      // We will want to implement code that moves at a constant speed and varies the time to turn
      TIMER = FindTurnTime(d_angle, SPEED);     // Calculate the turn time for that angle at given constant speed
      // Move(forward, turn);            // Turn Roomba by d_angle
      Move(forward, (WheelDir * SPEED));    // Turn Roomba by d_angle
      turnCounter = millis();         // Set Turn counter base
    }
  } // Ignore if no pulse has been received

  /* Send to Serial monitor a data point */
  if (millis() - deltime >= 1000) { // If 1 second = 1000 milliseconds have passed...
    deltime = millis();     // Reset base value for data points
    sno++; // Increment the data point number
    Serial.println(";");
    Print_Heading_Data();
  }

  /* Stop turning if TIMER has passed */
  if ((millis() - turnCounter >= TIMER)) { // If I've been turning long enough
    Move(forward, 0);               // Stop turning
    digitalWrite(yellowPin, LOW);   // Tell me that the robot is done turning
  }

}/* Go back and check everything again. Should be fast */

/* SUBROUTINES */
/* Sends out a pulse when phase equals 360 degrees */
void sendPulse() {
  //Serial.print("\n Sending Pulse \n");
  digitalWrite(greenPin, HIGH); // Tell me that I'm sending a pulse
  vw_send((uint8_t *)pulse, strlen(pulse));
  vw_wait_tx();                 // Wait until the whole message is gone
  digitalWrite(greenPin, LOW);  // Tell me that I'm done sending a pulse
}

/* Sends out a palse when new Roomba finishes setup */
void sendPalse() {
  digitalWrite(greenPin, HIGH); // Tell me that I'm sending a palse
  digitalWrite(redPin, HIGH);
  vw_send((uint8_t *)palse, strlen(palse));
  vw_wait_tx();                 // Wait until the whole message is gone
  digitalWrite(greenPin, LOW);  // Tell me that I'm done sending a palse
  digitalWrite(redPin, LOW);
}

/* Determines the necessary change in heading of the robot according to the pulse received */
void PRC_Sync(float phase) {
  /* Set d_angle, the amount to turn */
  /* Red LED turns on if d_angle is set to 0 */
  if ((phase) > (360 - EPSILON)) {          // If the phase is close to where I want...
    d_angle = 0;  // Don't turn at all.
    digitalWrite(redPin, HIGH); // Indicates received pulse, but no turning.
  } else if ((phase) >= 180) {              // If phase is greater than 180 degrees...
    /* Increase Heading */
    d_angle = (360 - (phase)) * RATIO;  // Amount I want to change the heading
    digitalWrite(redPin, LOW); // Indicates the last pulse received caused a turn
  } else if ((phase) > EPSILON) {           // If phase is less than 180, but not close...
    /* Decrease Heading */
    d_angle = -1 * (phase) * RATIO; // Amount I want to change the heading
    digitalWrite(redPin, LOW); // Indicates the last pulse received caused a turn
  } else {                                  // If the phase is close to where I want.
    d_angle = 0;  // Don't turn at all.
    digitalWrite(redPin, HIGH); // Indicates received pulse, but no turning.
  }
}

/* Finds necessary speed of wheels to turn at given angle (angledegrees) in given time (TIMER) */
int FindTurnSpeed(float angledegrees, const int TIMER) {
  /* Local variables for computation */
  int turnspeed;
  float angleradians, holder, timeseconds;
  /* Computation */
  angleradians = (angledegrees * pi) / 180;   // Find angle in radians
  timeseconds = TIMER / 1000;                 // Find time in seconds
  holder = (angleradians * WHEELSEPARATION) / (2 * timeseconds); // Calculate the turning speed (mm/s)
  holder = holder + 0.5;                      // Needed for rounding turnspeed value on next line
  turnspeed = (int) holder;                   // Truncate value of buffer to an integer
  /* Limit the value, so the Roomba doesn't turn too aggresively */
  if (turnspeed > 128) {
    turnspeed = 128;  // Cap value off at 128 mm/s (slightly arbitrary)
  }
  else if (turnspeed < -128) {
    turnspeed = -128; // Cap value off at -128 mm/s (slightly arbitrary)
  }

  return turnspeed;   // Send back the speed at which to turn.
}

/* Finds necessary time for wheels to turn a given angle (angledegrees) at given speed (SPEED) */
int FindTurnTime(float angledegrees, const int SPEED) {
  /* Local variables for computation */
  int turntime;
  float holder;
  /* Computation */
  holder = (1000 * angledegrees * pi * WHEELSEPARATION) / (360 * SPEED); // Calculate the turning time (ms)
  holder = holder + 0.5;                   // Needed for rounding turntime value on next line
  turntime = (int) holder;                 // Truncate value of holder to an integer
  /* Determine if wheel direction should be CW or CCW */
  if (turntime < 0) {
    WheelDir = -1;      // Set wheels to turn CW
    turntime = turntime * -1; // Negate time value
  } else {
    WheelDir = 1;       // Set wheels to turn CCW
  }

  return turntime;      // Send back the speed at which to turn.
}

/* General Wheel Motor command function.
    X = common wheel speed (mm/s); Y = differential wheel speed;
    X > 0 -> forward motion; Y > 0 -> CW motion
    This function allows for both turning and forward motion.
    Error may result if |X|+|Y| > 255 */
void Move(int X, int Y) {
  /* Local Variables needed for function */
  int RWHigh, LWHigh;
  /* Determine what the high 8-bits should be for each wheel*/
  /* Right Wheel High byte */
  if (X - Y >= 0) {   // If the desired right wheel speed is a positive number
    RWHigh = 0;       // Positive filler for a 2's complement number
  }  else  {          // If the desired right wheel speed is a negative number
    RWHigh = 255;     // Negative filler for a 2's complement number
  }
  /* Left Wheel High byte */
  if (X + Y >= 0) {   // If the desired left wheel speed is a positive number
    LWHigh = 0;       // Positive filler for a 2's complement number
  }  else  {          // If the desired left wheel speed is a negative number
    LWHigh = 255;     // Negative filler for a 2's complement number
  }
  /* Roomba Wheel Command */
  Roomba.write(byte(145));  // Syntax: [145] [RW High 8-bit] [RW Low 8-bit] [LW High 8-bit] [LW Low 8-bit]
  Roomba.write(byte(RWHigh));
  Roomba.write(byte(X - Y)); // Combine common and differential speeds for right wheel
  Roomba.write(byte(LWHigh));
  Roomba.write(byte(X + Y)); // Combine common and differential speeds for left wheel
}

/* This function gets the data from the magnetometer and returns the heading in degrees */
int getHeading() {
  /* Local variables need for function (already declared as global) */
  //int x, y, z;
  float t;
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();

  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if (6 <= Wire.available()) {
    x = Wire.read() << 8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read() << 8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read() << 8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  
  /* Convert coordinates to an angle for heading */
  /* Old Calculation (assumes x is forward) */
  //t = 180 + (atan2(-y, x) * 180 / pi);
  //if (t >= 360) t = t - 360;
  /* New Calculation (assumes y is forward) */
  t = (atan2(y,x) * (180/pi) - 90);
  if (t < 0){
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
  Serial.print(millisRatio * (long)(millis() - millisCounter)); // Counter value
  Serial.print(", ");
  Serial.print(x);        // Raw X magnetometer value
  Serial.print(", ");
  Serial.print(y);        // Raw Y magnetometer value
  Serial.print(", ");
  Serial.print(z);        // Raw Z magnetometer value
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
