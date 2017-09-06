#include <SPI.h>
#include <SoftwareSerial.h>
#include <VirtualWire.h>
#include <Wire.h> //I2C Arduino Library
#include <math.h>

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

//START SETTINGS
//Adjust these accordingly
int rxPin = 3;
int txPin = 4;
int ddPin = 5;
const int yellow_pin = 11;
const int green_pin = 7;
const int red_pin = 8;
const int transmit_pin = 12; //Pin of the transmitter (Tx) chip
const int receive_pin = 2; //Pin of the reciever (Rx) chip
const int transmit_en_pin = 3; //This probably isn't used
long searchTime = 30000; //The time in milliseconds the robot should spend trying to determine it's name before it has an existential crisis.
int headingTolerance = 5; //The tolerence (in degrees) that the heading can be to whatever heading it is told to go to
//
//END SETTINGS

SoftwareSerial Roomba(rxPin, txPin);
char currName[2];
const float pi = 3.1415926; //In case the arduino gets hungry
unsigned long previousTime;
char data[6];
String strSend, strRec;
char names[10] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J'};

void setup()
{ //Transmitter Setup
  //Initialize Serial and I2C communications
  Serial.begin(57600);
  pinMode(green_pin, OUTPUT);
  pinMode(yellow_pin, OUTPUT);
  pinMode(red_pin, OUTPUT);


  Serial.print("Loading..");
  pinMode(ddPin,  OUTPUT);
  Roomba.begin(115200);

  digitalWrite(yellow_pin, HIGH);
  digitalWrite(green_pin, HIGH);
  digitalWrite(red_pin, HIGH);
  delay(1000);
  digitalWrite(green_pin, LOW);
  digitalWrite(yellow_pin, LOW);
  digitalWrite(red_pin, LOW);
  Serial.println("....");

  digitalWrite(green_pin, HIGH); //Indicate setup has begun

  //Wake up the robot
  digitalWrite(ddPin, HIGH);
  delay(100);
  digitalWrite(ddPin, LOW);
  delay(500);
  digitalWrite(ddPin, HIGH);
  delay(2000);

  // set up ROI to receive commands
  Roomba.write(byte(7));  // RESTART
  delay(10000);
  Roomba.write(byte(128));  // START
  delay(50);
  Roomba.write(byte(131));  // CONTROL
  //131 - Safe Mode
  //132 - Full mode (be ready to catch it!)
  delay(50);
  digitalWrite(green_pin, LOW);

  Roomba.begin(115200);
  display_Running_Sketch();
  Wire.begin();


  //Wake up the robot
  digitalWrite(ddPin, HIGH);
  delay(100);
  digitalWrite(ddPin, LOW);
  delay(500);
  digitalWrite(ddPin, HIGH);
  delay(2000);

  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  // Initialise the IO and ISR
  vw_set_tx_pin(transmit_pin);
  vw_set_rx_pin(receive_pin);
  vw_set_ptt_pin(transmit_en_pin);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);       // Bits per sec
  pinMode(yellow_pin, OUTPUT);

  //Receiver Setup
  delay(1000);
  Serial.begin(57600);	// Debugging only
  Serial.print("setup");

  // Initialise the IO and ISR
  vw_set_tx_pin(transmit_pin);
  vw_set_rx_pin(receive_pin);
  vw_set_ptt_pin(transmit_en_pin);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);	 // Bits per sec

  vw_rx_start();       // Start the receiver PLL running

  digitalWrite(yellow_pin, HIGH);
  digitalWrite(green_pin, HIGH);
  Serial.println("...complete");
  delay(500);
  digitalWrite(green_pin, LOW);
  digitalWrite(yellow_pin, LOW);





  /*
    This set of code determines the robots name.
    It transmits it's own name (starting with A) and checks to see if
    any of the other robots are sending that name. If another robot
    is sending that name, it will go onto the next one.
  */
  digitalWrite(green_pin, HIGH);
  int nameAmount = sizeof(names);
  currName[0] = { names[0] };
  String recName;
  boolean NoName = true;
  int x = 0;
  while (NoName || !passiveDelay(searchTime)) {
    digitalWrite(yellow_pin, HIGH);
    recName = "";
    if (recName == "") {
      rfSend(currName);
      recName = rfRecieve();
      recName.trim();

      /*
         FOR CODE BELOW:-
         When other robots are running and have already determined their name
         they should normally be transmitting their heading and maybe some other data, along with their name
         at the end of the transmission. For example, if robot C is already online and at heading 243, it would be printing
         out something like this:     DATA.234C
         The purpose of this code below is to strip all that data and just get the name that is printed at the end
         of each transmission. If all the robots are in the setup sequence at the same time, than this code doesn't
         really do anything because all the transmissions will just be a robots name, which is doesn't meet the condition
         for the if statement at the beggining.
      */

      //If the recieved name is not just a name, but data from an already-online robot, it will be bigger than 1 character
      if (recName.length() > 1) {
        for (int z = 0; z < sizeof(names); z++) { //<-- Loop through all the possible names that a robot can have
          //Get the position of a the current name spicified by the for loop in the recieved string
          //This will just be -1 if the character isn't there.
          int namePos = recName.indexOf(names[z]);
          //If the name found in the recieved data is equal to the robots current name
          if (recName.charAt(namePos) == currName[0]) {
            /*Serial.print("(");
              Serial.print(recName);
              Serial.print(",");*/
            //Make the name that was found in the received string equal to the recieved string,
            //essentially stripping all the data that was in the string to the found name.
            //This is necessaru for the code below to work.
            recName = recName.charAt(namePos);
            /* Serial.print(recName);
              Serial.print(")");*/
            namePos = sizeof(names); //Stop the loop from running if a name was found
          }
        }
      }

      Serial.print(" R.Name: ");
      Serial.println(recName);
      delay(random(10, 100)); //wait a random time
    }



    //If recieved name and the robot's current name aren't the same, don't increment in the name list.
    if ((recName != currName)) {
      NoName = false;
    } else { //else...The names are same

      //Loop back around if the end of the name list has been reached, otherwise increment.
      if (x == nameAmount) {
        x = 0;
      } else {
        x++;
      }

      currName[0] = names[x]; //Change the robots name

    }
    digitalWrite(yellow_pin, LOW);
  }
  Serial.print("I am robot ");
  Serial.println(currName);
  digitalWrite(yellow_pin, LOW);
  digitalWrite(green_pin, LOW);
  Serial.println(' ');

  spinLeft();
  delay(750);
  spinRight();
  delay(750);
  Stop();

  if (currName[0] == 'A') {
    digitalWrite(yellow_pin, HIGH);
    digitalWrite(green_pin, HIGH);
    digitalWrite(red_pin, HIGH);
    delay(3000);
    digitalWrite(green_pin, LOW);
    digitalWrite(yellow_pin, LOW);
    digitalWrite(red_pin, LOW);
  }

  delay(3000);
}

byte count = 1;

void loop()
{
  int x, y, z, t, namePos, targetHeading; //triple axis data
  String strTargetHeading;

  if (currName[0] == 'A') { //If it is robot A
    //For reading the sent axis data on the receiving computer
    //char msgt[5] = {currName[0]};

    t = getHeading();

    //Convert magnetometer integers to char arrays so the rfSend subroutine can transmit it
    strSend = 'H';
    strSend += String(t);
    strSend += currName[0]; //append the robots name to the data
    strSend.toCharArray(data, 6);

    rfSend(data); //Send data. Note: The rfSend function only likes it if you use a char array.
    count = count + 1;
    Serial.println("");
    delay(500);

  } else {
    strRec = rfRecieve();
    digitalWrite(green_pin, HIGH);
    if (strRec.length() > 1) {
      int headingSymbolIndex = strRec.indexOf('H');

      //Check for name collision(s).
      for (int z = 0; z < sizeof(names); z++) {
        namePos = strRec.indexOf(names[z]);
        if (strRec.charAt(namePos) == currName[0]) {
          Serial.print(" ERROR: Name collision");
          ERRORLOOP();
          namePos = sizeof(names);
        }
      }
      //Extract the heading from the string received from robot A
      strTargetHeading = strRec.substring(headingSymbolIndex + 1, namePos - 1);
      targetHeading = strTargetHeading.toInt(); //convert the extracted heading to an integer

      //send own name every 5 seconds
      if ((millis() % 5000) == 0) {
        rfSend(currName);
      }

      goToHeading(targetHeading);
      //Serial.print("Robot A Heading =  ");
      //Serial.println(targetHeading);
    }
    digitalWrite(green_pin, LOW);

  }
}




void rfSend(char MSG[]) {
  Serial.print(" Sending: ");
  //Serial.print(msgA);
  Serial.print(MSG);
  delay(10);

  // Transmit data to other robots twice
  digitalWrite(yellow_pin, HIGH); // Flash a light to show transmitting
  vw_send((uint8_t *)MSG, strlen(MSG)); //Adjust this length to send all of the message
  vw_wait_tx(); // Wait until the whole message is gone

  vw_send((uint8_t *)MSG, strlen(MSG)); //Adjust this length to send all of the message
  vw_wait_tx(); // Wait until the whole message is gone

  digitalWrite(yellow_pin, LOW);
  delay(500);
}

int getHeading() {
  //This function gets the data from the magnetometer and returns the heading in degrees

  int x, y, z, t;

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

  //Convert coordinates to an angle for heading
  if (y > 0) {
    t = 90 - atan2(y, x) * (180 / pi);
  }
  else if (y < 0) {
    t = 270 - atan2(y, x) * (180 / pi);
  }
  else if (y = 0 && x < 0) {
    t = 180;
  }
  else {
    t = 0;
  }

  //Scale the angle to 0-360 degrees
  t = map(t, -90, 450, 0, 360);

  return t;
}

String rfRecieve() {
  //This function returns the string of the recieved char array
  char CHAR;
  int strlength = 0;
  String strRecieved = "";
  digitalWrite(yellow_pin, HIGH); // Flash a light to show received good message

  //Receive heading from other robots
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;


  /*If there is a message to be received, put it into a string.
    NOTE: Putting the FOR loop outside of this if statement
    can cause some odd data to be returned because there is nothing
    legible to read from the buffer.
  */
  if (vw_get_message(buf, &buflen)) // Non-blocking
  {
    //build up string from recieved array
    // Serial.print("RECIEVED: ");
    for (int k = 0; k < buflen; k++) {
      CHAR = buf[k];
      strRecieved += CHAR ;
      //Serial.write(buf[k]);
    }
  }

  digitalWrite(yellow_pin, LOW);
  return strRecieved; //Return the fully-acquired buffer
}

//This funciton should return true when the specified amount of time has passed
//ONLY CALL THIS FUNCTION ONCE FROM A BLOCK OF CODE.
boolean passiveDelay(int userDelay) {
  unsigned long currentTime = millis();
  //Serial.println((currentTime - previousTime) / 60);
  if (currentTime - previousTime > userDelay) {
    Serial.println("Times up!"); //For debugging
    previousTime = currentTime;
    return true;
  } else {
    return false;
  }

}

// displays at startup the Sketch running in the Arduino
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


void goToHeading(int targetHeading) {
  int currentHeading = getHeading();
  int headingDifferences = currentHeading - targetHeading;

  Serial.print("Current Heading ");
  Serial.println(currentHeading);
  //Serial.println("[Target Heading], [Current Heading], [Heading Differences], [Command Issued]");

  while (abs(headingDifferences) > headingTolerance) {
    currentHeading = getHeading();
    headingDifferences = currentHeading - targetHeading;

    //For debugging:
    Serial.print(targetHeading);
    Serial.print(", ");
    Serial.print(currentHeading);
    Serial.print(", ");
    Serial.print(headingDifferences);
    Serial.print(", ");

    if (headingDifferences > headingTolerance) {
      Serial.println("RIGHT");
      spinRight(); //Roomba turn right command
    } else if (headingDifferences < headingTolerance) {
      Serial.println("LEFT");
      spinLeft(); //Roomba turn left command
    } else {
      Serial.println("STOP");
      Stop(); //Stop the Roomba from moving
    }

    delay(500);
  }

  //Serial.println("goToHeading: DONE");
  Stop();
}

void spinLeft() {
  Roomba.write(byte(145));   // DRIVE
  Roomba.write(byte(0x00));   // 12mm/s
  Roomba.write(byte(0x0b));
  Roomba.write(byte(0xFF));
  Roomba.write(byte(0xF5));
}
void spinRight() {
  Roomba.write(byte(145));   // DRIVE
  Roomba.write(byte(0xFF));   // 12mm/s
  Roomba.write(byte(0xF5));
  Roomba.write(byte(0x00));
  Roomba.write(byte(0x0b));
}
void Stop() {
  Roomba.write(byte(145));   // DRIVE
  Roomba.write(byte(0x00));   // 0mm/s
  Roomba.write(byte(0x00));
  Roomba.write(byte(0x00));
  Roomba.write(byte(0x00));
}

void ERRORLOOP() {
  Serial.println("ERROR");
  //this function is designed to prevent any more code from executing when it is called
  while (1) {
    digitalWrite(yellow_pin, HIGH);
    digitalWrite(green_pin, HIGH);
    delay(2000);
    digitalWrite(yellow_pin, LOW);
    digitalWrite(green_pin, LOW);
    delay(1000);
  }
}

