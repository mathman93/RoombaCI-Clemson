// Uses the math found in Localization->Matrix - Method Matrix_Method.ppt


#include <math.h>

// Pin Assignments
const int mic1 = 2;
const int mic2 = 9;
const int mic3 = 10;
const int led1 = 13;

// Timing Variables
unsigned long th1, th2, th3; // thx is the time that the microphone "hears" the sound where x is the mic number [us]
bool hit1 = false, hit2 = false, hit3 = false; // hitx is whether the microphone has heard anything where x is the mic number

// Calculation Values
float a1 = 3.535533905932738, a2 = -3.535533905932738, a3 = -1.464466094067262, a4 = -1.464466094067262; // [a1, a2; a3, a4] is the first matrix in the formula [m]
const double c = 340.27; // speed of sound [m/s]

// Calculation Variables
double cdt12, cdt13; // cdt12 is the difference in time bw mics 1 and 2 multiplied by the speed of sound
double u, v; // result matrix [u; v]
double angle[5]; // angle that is the result of calculations
double pi = 3.14159265359;


int t12, t13;
double x, y;

long int number = 0;
 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(led1, OUTPUT);
  pinMode(mic1, INPUT);
  pinMode(mic2, INPUT);
  pinMode(mic3, INPUT);

  digitalWrite(led1,1);
  delay(1000);
  digitalWrite(led1,0);

  Serial.println("setup done.\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  int i = 0;
  bool j = true;

  while (j) {

number++;
    if (!hit1) {
      hit1 = digitalRead(mic1);
      if (hit1) {
        th1 = micros();
        number = 0;
      }
    }
    if (!hit2){ 
      hit2 = digitalRead(mic2);
      if (hit2) {
        th2 = micros();
      }
    }
    if (!hit3) {
      hit3 = digitalRead(mic3);
      if (hit3) {
        th3 = micros();
      }
    }

  // perform calculations after all microphones have heard a sound
  if (hit1 && hit2 && hit3) {

    t12 = th1 - th2;
    t13 = th1 - th3;

    x = (double) t12;
    y = (double) t13;

    cdt12 = c * x / 1000000;
    cdt13 = c * y / 1000000;

    Serial.println(cdt12);
    Serial.println(number);

    u = a1 * cdt12 + a2 * cdt13;
    v = a3 * cdt12 + a4 * cdt13;

    angle[i++] = 360*atan2(u,v)/(2*pi);

    Serial.print(th1);
    Serial.print("-");
    Serial.print(th2);
    Serial.print("-");
    Serial.println(th3);
    
    if (i == 5) {
      // print angle results to serial monitor after five "hits"
      Serial.println(angle[0]);
      Serial.println(angle[1]);
      Serial.println(angle[2]);
      Serial.println(angle[3]);
      Serial.println(angle[4]);
      while(1){
        digitalWrite(led1,1);
        delay(1000);
        digitalWrite(led1,0);
        delay(1000);
      }
    }
    delay(500);
    Serial.println("done calc");
    hit1 = false;
    hit2 = false;
    hit3 = false;
  }
  }
  
}
