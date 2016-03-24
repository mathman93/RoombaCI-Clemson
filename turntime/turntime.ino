/* Compass Calibration Code
 * Taken from Vijay over Christmas. Runs compass calibration code.
 * We need to figure out what this does, and how to modify the existing code to be compatible with SyncSpin.ino
 * 
 * Last Updated: 3/24/2016
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
char pulse[2] = {'a'};
char palse[2] = {'b'};
int x, y, z, t, i; //triple axis data
uint8_t buf[VW_MAX_MESSAGE_LEN];
uint8_t buflen = VW_MAX_MESSAGE_LEN;

/* global variables needed to implement turn functions */
float angle;                     // Heading of Roomba (found from digital compass)
float counter = 0;                 // Angle counter to implement swarm behavior
float d_angle;                   // change in angle that Roomba will turn (updates each cycle)
int forward = 0;                 // Speed in mm/s that Roomba wheels turn to move forward - must be in range [0,128]
unsigned long turn_time;                   // Time in ms that Roomba wheels turn to rotate a given d_angle
const float RATIO = 1.0;         // Ratio for amount to turn - must be in range (0 1] - 0.85
const int DELAY = 3;            // Counter delay - must be in range of (0 1000]
const int TIMER = 5000;          // Amount of time in milliseconds that the Roomba spends turning (can be adjusted)
const float EPSILON = 5.0;       // Smallest resolution of digital compass (used in PRC function) - 5.0
const float pi = 3.1415926;      // pi to 7 decimal places

/* Roomba parameters */
const int WHEELDIAMETER = 72;    // 72 millimeter wheel diameter
const int WHEELSEPARATION = 235; // 235 millimeters between center of main wheels
const float ENCODER = 508.8;     // 508.8 Counts per wheel revolution

const int TSPEED = 100;           //Speed of the turns

/* Copied from original 'heading_leader' file; */
SoftwareSerial Roomba(rxPin, txPin);
unsigned long deltime = 0;
unsigned long delaytime = 0;
bool delayflag;
bool negative = false; //check if angle is +ve or -ve
long sno = 0;
unsigned long LookUp[7][2]; //Initialize the lookup table


/* Copied from original 'heading_leader' file; Start up Roomba */
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

  digitalWrite(greenPin, LOW);  // say we've finished setup
  /* I removed the "happiness indicator". You're welcome. */

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
  Serial.print("setup");

  // Initialise the IO and ISR
  vw_set_tx_pin(transmit_pin);
  vw_set_rx_pin(receive_pin);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);   // Bits per sec

  vw_rx_start();       // Start the receiver PLL running

  digitalWrite(yellowPin, HIGH);
  Serial.println("...complete");
  delay(500);
  digitalWrite(yellowPin, LOW);

  delay(1000);
/* Compass Calibration: I'm not sure what this code does. Need to look up documentation */
  //Keep spinning for calibration
  compass_init(1);
  Move(0, 100);
  compass_debug = 1;
  compass_offset_calibration(2);
  Move(forward, 0);
  /* Wait for command to initialize synchronization */

  sendPalse();                  // Reset counters for all online robots.

  deltime = millis();           // Set base value for data output.
  delayflag = true;
  
}

void loop() { // Swarm "Set Orientation" Code
 
  compass_scalled_reading();
  compass_heading();
  

  // Read angle from compass 
  angle = bearing;

  //Lookup Table tries
/*

  Move(forward, 100); //176 , 88, 11           // Stop Roomba from turning
  delay(1650); //235
  Move(forward, 0); 

  Serial.print("\nCompass Heading angle = ");
  Serial.print(bearing);
  Serial.println(" Degree\n");

  delay(5000); 

*/
  

  // Send a pulse signal 
  digitalWrite(greenPin, HIGH);
  if (angle + (int)counter >= 360 ) { // If my angle and counter reach 360 degrees...
    sendPulse();
  } // Ignore if the angle and counter are less than 360 degrees.

  // Receive a pulse signal 
  if (vw_get_message(buf, &buflen)) {  // If I receive a pulse signal (a high bit)... (perhaps something stored in a buffer?)


    //Ahhhhhhhhhhhhhhhhhhhhhhhhhhh FLASH EVERYTHING if there is a b
    if (buf[i] == 'b') {           // charater of pulse signal
      Serial.print("\nReceived Broadcast\n");
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, HIGH);
      digitalWrite(yellowPin, HIGH);
      counter = 0;
      delay(500);                 //Not happy about this delay - remove later
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(yellowPin, LOW);     // Tell me that the robot is done
    }

    if (buf[i] == 'a') {           // charater of pulse signal
      Serial.print("\nReceived Pulse\n");
      changeAngle();                //Adjust the angle of the robot

      if(d_angle<0)
      {
        negative = true;
        d_angle*=-1;
      
      }
      
      if(d_angle > 1) //Roomba is not accurate enough to turn by 1 degree (actually 3 degrees)
      {
        turn_time = LookupTurnTime(d_angle);
      }
      else
        turn_time = 0;
        
      Serial.print("Lookup Value");
      Serial.println(turn_time);

      if(turn_time)
      {
        digitalWrite(yellowPin, HIGH);   // Tell me that the robot is turning
        if(!negative)
          Move(forward, TSPEED);         // Turn Roomba by d_angle
        else
          Move(forward, -1*TSPEED);
        
        if(delayflag)
        {
          delaytime = millis();
          delayflag = false;
        }
      }
    }

  } // Ignore if no pulse has been received

  if(!delayflag && (millis()-delaytime >= turn_time))
  {
    Move(forward, 0);            // Stop Roomba from turning
    delayflag = true;
    Serial.print("\nTurn completed\n");
    digitalWrite(yellowPin, LOW);     // Tell me that the robot is done
  }

  //counter++; // Increment the counter variable
  counter = counter + 0.1;

//  if(millis() - deltime >= 1000)  //--Toggle to enable per second reading
//  {
  Serial.print(sno);
  Serial.print(". ");
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print("    Counter: ");
  Serial.println(counter);
  deltime = millis();
  sno++;
  //if(sno % 6000 == 0) sendPalse(); //ensure synchronization by resetting counter - modify this to (if there is a mismatch in the counter
//  }

  
  delay(DELAY); // Include small delay for the counter variable


}// Go back and check everything again. 


/* SUBROUTINES */
/*Sends out a pulse when the counter and angle equal 360 degrees*/
void sendPulse() {
  Serial.print("\nSending Pulse \n");
  digitalWrite(greenPin, HIGH);
  vw_send((uint8_t *)pulse, strlen(pulse));
  vw_wait_tx();             // Wait until the whole message is gone
  digitalWrite(redPin, HIGH);
 // delay(TIMER);             // Wait for the other Roombas to finish turning
  counter = (int)counter - 360;              // Reset counter back so that the sum of counter and angle is less than 360 degrees
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
}

void sendPalse() {
  Serial.print("\nBcast \n");
  digitalWrite(greenPin, HIGH);
  vw_send((uint8_t *)palse, strlen(palse));
  vw_wait_tx();             // Wait until the whole message is gone
  digitalWrite(redPin, HIGH);
  delay(500);             // Wait to see the lights in the receivers
  counter = 0;              // Reset counter back so that the sum of counter and angle is less than 360 degrees
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
}


/*Changes the angle of the robot according to the pulse received*/
void changeAngle() {
  /* Set d_angle, the amount to turn */
 
  if ((angle + (int)counter) > (360 - EPSILON)) { // If the angle is close to where I want...
    d_angle = 0;                    // Don't turn at all.
    Serial.print("\nNo Turn\n");
    return;
  } else if ((angle + (int)counter) >= 180) {      // If angle + counter is greater than 180 degrees...
    // increase angle 
    d_angle = (360 - (angle + (int)counter)) * RATIO; // amount I want to change angle
    Serial.print("\nClockwise\n");
    return;
  } else if ((angle + (int)counter) > EPSILON) {   // If angle + counter is less than 180, but not close...
    // decrease angle 
    d_angle = -1 * (angle + (int)counter) * RATIO;    // amount I want to change angle
    Serial.print("\nCounterclockwise\n");
    return;
  } else {                                    // If the angle is close to where I want.
    d_angle = 0;                    // Don't turn at all.
    Serial.print("\nNo Turn\n");
  }
}


unsigned long LookupTurnTime(int angle)
{
  int delta = 0, i, index;
  int minval=700; //greater than 180 would do
  unsigned long LookUp[7][2] = {{180,3700},{90,1800},{40,900},{20,400},{10,235},{5,80},{3,50}}; //Roomba 1 
 // unsigned long LookUp[7][2] = {{180,3400},{90,1750},{40,900},{20,400},{10,235},{5,80},{3,50}}; //Roomba 2
 // unsigned long LookUp[7][2] = {{180,3300},{90,1650},{40,800},{20,400},{10,200},{5,80},{3,50}}; //Roomba 3

  
  Serial.print("\nThe fn lookup value");
  
  if(angle < 0)   //This is an unnecessary check - scrap it
  {
    angle*=-1;
  }
  
  for(i=0;i<7;i++)
  {
    delta = angle - (int)LookUp[i][0];
    Serial.print(angle);
    Serial.print(" - ");
    Serial.println(LookUp[i][0]);
    if(delta<0) delta *= -1;
    if(minval>delta)
    { 
      minval = delta;
      index = i;
    }
  }
  Serial.print("\nThe final lookup value returned");
  Serial.println(LookUp[index][1]);
  return LookUp[index][1];

/*  //Uncomment for future Roombas -- replacement for lookup table
  unsigned long turntime;
  float angleradians, holder, timeseconds;
  // Computation 
  angleradians = (angle * pi) / 180; // Find angle in radians
  holder = (angleradians * WHEELSEPARATION * 1000) / (2 * TSPEED); // Calculate the turning time (ms for speed of TSPEED)
  holder = holder + 0.5;                   // Needed for rounding turnspeed value on next line
  turntime = (int) holder;                // Truncate value of buffer to an integer
  // Limit the value, so the Roomba doesn't turn too aggresively 

  return turntime;     // Send back the speed at which to turn.
*/
}

/* General Wheel Motor command function.
    X = common wheel speed (mm/s); Y = differential wheel speed;
    X > 0 -> forward motion; Y > 0 -> CCW motion
    This function is in essence a combination of Spin and Straight,
    allowing for both turning and forward motion */
    
void Move(int X, int Y) {
  /* Local Variables needed for function */
  int RWHigh, LWHigh;
  /* Determine what the high 8-bits should be for each wheel*/
  /* Right Wheel High byte */
  if (X + Y >= 0) { // If the desired right wheel speed is a positive number
    RWHigh = 0;    // Positive filler for a 2's complement number
  }  else  {       // If the desired right wheel speed is a negative number
    RWHigh = 255;  // Negative filler for a 2's complement number
  }
  /* Left Wheel High byte */
  if (X - Y >= 0) { // If the desired left wheel speed is a positive number
    LWHigh = 0;    // Positive filler for a 2's complement number
  }  else  {       // If the desired left wheel speed is a negative number
    LWHigh = 255;  // Negative filler for a 2's complement number
  }

  Roomba.write(byte(145));  // Syntax: [145] [RW High 8-bit] [RW Low 8-bit] [LW High 8-bit] [LW Low 8-bit]
  Roomba.write(byte(RWHigh));
  Roomba.write(byte(X + Y)); // Combine common and differential speeds for right wheel
  Roomba.write(byte(LWHigh));
  Roomba.write(byte(X - Y)); // Combine common and differential speeds for left wheel
}


// displays at startup the Sketch running on the Arduino
void display_Running_Sketch (void) {
  String the_path = __FILE__;
  int slash_loc = the_path.lastIndexOf('\\');
  String the_cpp_name = the_path.substring(slash_loc + 1);

  Serial.print("\nSketch Name: ");
  Serial.println(the_cpp_name);
  Serial.print("Compiled on: ");
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.print(__TIME__);
  Serial.println("\n");
}
