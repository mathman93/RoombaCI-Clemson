#define ASDF 512
#define aasdf 1000000000
int mic1 = A0;
int mic2 = A1;
int mic3 = A2;
int led1 = 7;
int led2 = 6;
int led3 = 5;
int data1, data2, data3, loops, lg, sm, md, data1o, data2o, data3o;
//int high = 800;
int avg[3];
int sum[3];
float limits[3];
int valueHistory[6][3];
boolean triggered = false;
boolean change1 = true, change2 = true, change3 = true;
boolean change1o = false, change1oo = false, change2o = false, change2oo = false, change3o = false, change3oo = false, change1r = false, change2r = false, change3r = false;
boolean change1rr = false, change2rr = false, change3rr = false;
long t1 = 0, t2 = 0, t3 = 0;
long d12 = 0, d13 = 0, d23 = 0;

const int NFA = 6; //"Number for Average", the amount of data points to collect before getting the average
const bool AUTOSTOP = true; //Stop for a certain period when a sound is picked up
const int RECORDTIME = 60; //time in seconds, default 30 seconds
const float TOLERANCE = 0.6; //Percent difference as a decimal that a value needs to be to be considered abnormal
const int TIMEOUT = 5; //Time to wait in seconds before resuming scan after detecting a beep, ignored if AUTOSTOP is false
const bool DEBUG = false; //Show verbose information about collected data
const bool DEBUG_ALLDATA = false; //Show the readings from the microphones at all times

void setup() {
  Serial.begin(57600);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);
  digitalWrite(led3, HIGH);
  delay(500);
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
  delay(500);

  data1 = 0;
  data2 = 0;
  data3 = 0;
  
  Serial.println(ASDF);
  Serial.println("setup not");
}

void loop() {
  // read the value from the sensor:



  
  change1oo = change1o;
  change1o = change1;
  data1o = data1;
  data1 = abs(analogRead(mic1));
  if ((abs(data1 - data1o) > ASDF) && ~change1oo && ~change1o) {
    t1 = micros();
    change1 = true;
  }

  change2oo = change2o;
  change2o = change2;
  data2o = data2;
  data2 = abs(analogRead(mic2));
  if ((abs(data2 - data2o) > ASDF) && ~change2oo && ~change2o) {
    t2 = micros();
    change2 = true;
  }

  change3oo = change3o;
  change3o = change3;
  data3o = data3;
  data3 = abs(analogRead(mic3));
  if ((abs(data3 - data3o) > ASDF) && ~change3oo && ~change3o) {
    t3 = micros();
    change3 = true;
  }



  if(change1 && change2 && change3/* || (((t1 - micros()) > aasdf) && (change2 || change3)) || (((t2 - micros()) > aasdf) && (change1 || change3)) || (((t3 - micros()) > aasdf) && (change1 || change2))*/){
   d23 = t2 - t3;
   d12 = t1 - t2;
   d13 = t1 - t3;
   Serial.println("");
   Serial.println("");
   Serial.println("");
   Serial.println(t1);
   Serial.println(t2);
   Serial.println(t3);
   Serial.println("");
   Serial.println("");
   Serial.println("");
   change1 = false;
   change2 = false;
   change3 = false;
   if (t1 < t2 && t1 < t3) {
    digitalWrite(led1,HIGH);
    delay(500);
    digitalWrite(led1,LOW);
   }
   if (t2 < t1 && t2 < t3) {
    digitalWrite(led2,HIGH);
    delay(500);
    digitalWrite(led2,LOW);
   }
   if (t3 < t1 && t3 < t2) {
    digitalWrite(led3,HIGH);
    delay(500);
    digitalWrite(led3,LOW);
   }
   t1 = 0;
   t2 = 0;
   t3 = 0;
  }
}

