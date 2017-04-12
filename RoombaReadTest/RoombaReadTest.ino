// TESTING
/*  Gather Roomba Information and Display to Serial Monitor

    Last Updated: 3/29/2017

    HAS TO BE MANUALLY SET TO 19200 BAUD RATE

    Currently works at 19200 Baud rate BUT NOT 115200

    When reading data at 115200 the data we get makes no sense (possible bit shift)

    Currently trying to set the Roomba's baud rate lower DOES NOT WORK ANY OTHER WAY BAUD RATE IS THE PROBLEM
    Failed Solutions: 
        setting a function to correct data (works for bumper but nothing else)
        Setting a bitmask to make sure we are getting the right data (does not work with signed ints)
*/
// Testing
#include <SoftwareSerial.h>
#include <SPI.h>
#include "VirtualWire.h"
#include "Wire.h"

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
int forward = 40;                  // Speed in mm/s that Roomba wheels turn to move forward - must be in range [-128,128]
const float pi = 3.1415926;       // pi to 7 decimal places

/* Adjustable Synchronization Parameters */
const float RATIO = 0.35;         // Ratio for amount to turn - must be in range (0 1]
const float EPSILON = 1.0;        // (ideally) Smallest resolution of digital compass (used in PRC function) - 5.0

/* Roomba parameters */
const int WHEELDIAMETER = 72;     // 72 millimeter wheel diameter
const int WHEELSEPARATION = 235;  // 235 millimeters between center of main wheels
const float ENCODER = 508.8;      // 508.8 Counts per wheel revolution

/* Copied from original 'heading_leader' file; */
SoftwareSerial Roomba(rxPin, txPin); // Set up communnication with Roomba

/* Data Point Collector Setup */
unsigned long deltime;
unsigned long TIMER;
long sno = 0;
byte header;
byte number;
byte ID;
byte BumperByte;
byte waste;
int rWheel;
int lWheel;
int RoombaVoltage;
int RoombaCharge;
byte RoombaDud;
boolean gled = LOW;


/* Start up Roomba */
void setup() {
  
  //void display_Running_Sketch();
  
  Serial.begin(19200);
  display_Running_Sketch();     // Show sketch information in the serial monitor at startup
  Serial.println("Loading...");
  pinMode(ddPin, OUTPUT);       // sets the pins as output
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  Roomba.begin(115200);         // Declare Roomba communication baud rate.
  digitalWrite(greenPin, HIGH); // say we're alive

  // set up ROI to receive commands
  Roomba.write(byte(7));  // RESTART
  delay(1000);
  
/* EXPERIMENTING BAUD RATES
  // Set Baud rate to 19200
  Roomba.write(byte(129));  // Explicitly set Baud rate
  Roomba.write(byte(7));    // 10 => 57600 Baud; 7 -> 19200
  delay(1000);
  Roomba.flush();
  delay(2);
  Roomba.end();           // ends the serial thingyeeee
  Roomba.begin(19200);   // Re-declare Roomba communication baud rate.
  delay(100);   // Wait before sending more commands
  // end of changing baud rate.
*/

  
  Serial.print("STARTING ROOMBA\n");
  Roomba.write(byte(128));  // START
  delay(50);
  Roomba.write(byte(131));  // CONTROL
  //131 - Safe Mode
  //132 - Full mode (Be ready to catch it!)
  delay(100);
  

  
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
  
  digitalWrite(greenPin, LOW);  // say we've finished setup
  delay(1000);
  /* Clear Roomba Serial stream */
  while(Roomba.available() > 0) {
    Serial.print((char)Roomba.read());
  }

    /* Initialize Roomba */
  deltime = millis();           // Set base value for data output.
  TIMER = millis();
  sno = 0;                      // Reset data point counter
}

void loop() { // Read data and send to Serial monitor
  
  // -------------------------------3-29-2017----------------------------------
  // EVERYTHING WORKS FOR 19200 BAUD RATE
  // NEXT WEEK: - learn how to change the BAUD RATE
  // -------------------------------------------------------------------------
  
  // Send to Serial monitor a data point 
  if (millis() - deltime >= 500) { // If 1 second = 1000 milliseconds have passed...
    deltime += 500;     // Reset base value for data points

    Move(100,0);
    
    Roomba.write(byte(149));    // Ask for a query
    Roomba.write(byte(3));      // Ask for # of packets
    Roomba.write(byte(7));      // Ask for Bumper Byte (1 byte)
    Roomba.write(byte(41));   // Ask for Requested Right velocity (2 bytes)
    Roomba.write(byte(42));   // Ask for Requested Left velocity (2 bytes)
    //Roomba.write(byte(22));     // Ask for voltage packet
  }
  if (Roomba.available() >= 5) {
    /* EXPERIMENTAL
    
    */
    BumperByte = Roomba.read();   // First byte (reads Bumper data)
    
    rWheel = ((Roomba.read()<<8) | Roomba.read()); // Reads two bytes about rWheel
    lWheel = ((Roomba.read()<<8) | Roomba.read()); // Reads two bytes about lWheel
    
    // erase buffer
    while(Roomba.available() > 0) {
      waste = Roomba.read();
    }
    
    /*  Sometimes will give us weird value but most of the time it is accurate
     * found while testing left wheel right wheel and the bumper TESTED ON ROOMBA 2 + 3
     */
    // Print rWheel data
    Serial.print("rWheel = ");
    Serial.print(rWheel);
    Serial.print("\n"); 
    // Print lWheel data
    Serial.print("lWheel = ");
    Serial.print(lWheel);
    Serial.print("\n");
    // Print Bumper data (Also shows what bumpers are pressed)  
    Serial.print("Bumpers: ");
    Serial.print(BumperByte);
    Serial.print(" ");
    switch(BumperByte){
      case 0:
        Serial.println("No Bumpers Pressed");
        break;
      case 1:
        Serial.println("Right Bumper Pressed");
        break;
      case 2:
        Serial.println("Left Bumper Pressed");
        break;
      case 3:
        Serial.println("Both Bumpers Pressed");
        break;
      default:
        Serial.println("ERROR!");
        break;
    }
  }
  // Blinks the green light to show that it is going through the loop
  if (millis() - TIMER > 600) {
    TIMER = millis();
    gled = !gled;
    digitalWrite(greenPin, gled);
  }
}/* Go back and check everything again. Should be fast */

/* SUBROUTINES */
/* General Wheel Motor command function.
    X = common wheel speed (mm/s); Y = differential wheel speed (mm/s);
    X > 0 -> forward motion; Y > 0 -> CW motion
    This function allows for both turning and forward motion.
    Error may result if |X|+|Y| > 500 (Max value is 500)
    */

void Move(unsigned int X, unsigned int Y) {
// Example Code
 unsigned int RW = (X - Y);
 unsigned int LW = (X + Y);
 Roomba.write(byte(145));
 Roomba.write(byte((RW & 0xff00) >> 8));
 Roomba.write(byte(RW & 0xff));
 Roomba.write(byte((LW & 0xff00) >> 8));
 Roomba.write(byte(LW & 0xff));
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

/* Checks the byte that is recieved by the roomba*/
byte byteCheck(byte byteC, int Opcode){   // Add a third variable for byte size
    while(byteC>15){
        Roomba.write(byte(142));  // Ask for a Query from the Roomba
        //Roomba.write(byte(1));    // Ask for one byte
        Roomba.write(byte(Opcode)); //Ask for specific data
        while(Roomba.available() == 0){
        }
        byteC = 0;
        byteC = Roomba.read();
    }
  return byteC;
}
