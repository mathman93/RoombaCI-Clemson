int mic1 = A0;
int mic2 = A1;
int mic3 = A2;
int led1 = 7;
int led2 = 6;
int led3 = 5;
int data1, data2, data3, loops, lg, sm, md;
//int high = 800;
int avg[3];
int sum[3];
float limits[3];
int valueHistory[6][3];
boolean triggered = false;

unsigned long timeStart, timeStop, t1, t2, t3, diff;

const int NFA = 6; //"Number for Average", the amount of data points to collect before getting the average
const bool AUTOSTOP = true; //Stop for a certain period when a sound is picked up
const int RECORDTIME = 60; //time in seconds, default 30 seconds
const float TOLERANCE = 0.5; //Percent difference as a decimal that a value needs to be to be considered abnormal
const int TIMEOUT = 5; //Time to wait in seconds before resuming scan after detecting a beep, ignored if AUTOSTOP is false
const bool DEBUG = false; //Show verbose information about collected data

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

  Serial.println("setup complete");
}

void loop() {
  // read the value from the sensor:
  timeStart = millis();
  data1 = abs(analogRead(mic1) - 200);
  if(data1 > limits[0]) t1 = millis();
  data2 = abs(analogRead(mic2) - 200);
  if(data2 > limits[1]) t2 = millis();
  data3 = abs(analogRead(mic3) - 200);
  if(data3 > limits[2]) t3 = millis();
  timeStop = millis();

  Serial.print("RT_");
  Serial.print(timeStop - timeStart);
  Serial.print("_t1:");
  Serial.print(t1);
  Serial.print("_t2:");
  Serial.print(t2);  
  Serial.print("_t3:");
  Serial.println(t3);
  
  //reset the loop and sum variables
  if (loops >= NFA) {
    loops = 0;
    sum[0] = 0;
    sum[1] = 0;
    sum[2] = 0;
  }

  sum[0] += data1;
  sum[1] += data2;
  sum[2] += data3;
  loops++;

  //Determine the runnning average
  avg[0] = sum[0] / loops;
  avg[1] = sum[1] / loops;
  avg[2] = sum[2] / loops;

  //Calculate the  percent difference a noise needs to be in order to trigger
  for (int MIC = 0; MIC < 3; MIC++) {
    limits[MIC] = avg[MIC] * (TOLERANCE + 1);
  }

  if (DEBUG) Serial.print("CURRENT VALUES: ");
  Serial.print(data1);
  Serial.print(", ");
  Serial.print(data2);
  Serial.print(", ");
  Serial.print(data3);

  if (DEBUG) {
    Serial.println("");
    Serial.print("SUMS: ");
    Serial.print(sum[0]);
    Serial.print(", ");
    Serial.print(sum[1]);
    Serial.print(", ");
    Serial.println(sum[2]);

    Serial.print("AVERAGES: ");
    Serial.print(avg[0]);
    Serial.print(", ");
    Serial.print(avg[1]);
    Serial.print(", ");
    Serial.println(avg[2]);

    Serial.print("TOLERANCE: ");
    Serial.print(limits[0]);
    Serial.print(", ");
    Serial.print(limits[1]);
    Serial.print(", ");
    Serial.println(limits[2]);
  }
  Serial.println("");

  //This if-tree will temporarily stop the reporting of output if the output exceeds the threshold
  //specfied by the user
  if (AUTOSTOP) {
    if ((data1 > limits[0]) || (data2 > limits[1]) || (data3 > limits[2])) {
      Serial.println("");
      triggered = false;

      //Added FEB 24

      lg = compare(data1, data2, data3, false);
      sm = compare(data1, data2, data3, true);
      //This pile of ifs is used to find the middle of the three values
      if (data1 == lg) {
        if (sm == data2) {
          md = data3;
        } else if (sm == data3) {
          md = data2;
        }
      }
      if (data2 == lg) {
        if (sm == data1) {
          md = data3;
        } else if (sm == data3) {
          md = data1;
        }
      }
      if (data3 == lg) {
        if (sm == data1) {
          md = data2;
        } else if (sm == data2) {
          md = data1;
        }
      }

      Serial.print("SORTED VALUES: ");
      Serial.print(lg);
      Serial.print(", ");
      Serial.print(md);
      Serial.print(", ");
      Serial.println(sm);
      Serial.print("DIFFERENCES: ");
      Serial.print(difference(lg, md, sm, true));
      Serial.print(", ");
      Serial.println(difference(lg, md, sm, false));
      //end addition of FEB 24


      if ((data1 > limits[0]) && !triggered) {
        digitalWrite(led1, HIGH);
        Serial.println("Microphone on A0");
        delay(TIMEOUT * 1000);
        digitalWrite(led1, LOW);
        triggered = true;
      }
      if ((data2 > limits[1]) && !triggered) {
        digitalWrite(led2, HIGH);
        Serial.println("Microphone on A1");
        delay(TIMEOUT * 1000);
        digitalWrite(led2, LOW);
        triggered = true;
      }
      if ((data3 > limits[2]) && !triggered) {
        digitalWrite(led3, HIGH);
        Serial.println("Microphone on A2");
        delay(TIMEOUT * 1000);
        digitalWrite(led3, LOW);
        triggered = true;
      }
    }
  }

  //This will stop the program from running after a certain amount of time
  //This makes it easier to take the output and put in something like Excel
  while (millis() >= (RECORDTIME * 1000)) {
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
    delay(1000);
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    delay(1000);
  }
}

int compare(int value1, int value2, int value3, boolean SMALLEST) {
  //This function will return that largest number from 3
  //given numbers.
  //The SMALLEST (true) boolean value is used to switch from finding the largest
  //to finding the smallest of the given values
  if (!SMALLEST) {
    if (value1 > value2) {
      if (value1 > value3) {
        if (DEBUG) Serial.println("Value 1 largest");
        return value1;
      }
    } else if (value2 > value1) {
      if (value2 > value3) {
        if (DEBUG) Serial.println("Value 2 largest");
        return value2;
      }
    } else if (value3 > value2) {
      if (value3 > value1) {
        if (DEBUG) Serial.println("Value 3 largest");
        return value3;
      }
    } else {
      Serial.println("ERROR: Unable to make comparison");
      return -1;
    }
  } else {
    //Find the smallest instead
    if (value1 < value2) {
      if (value1 < value3) {
        if (DEBUG) Serial.println("Value 1 smallest");
        return value1;
      }
    } else if (value2 < value1) {
      if (value2 < value3) {
        if (DEBUG) Serial.println("Value 2 smallest");
        return value2;
      }
    } else if (value3 < value2) {
      if (value3 < value1) {
        if (DEBUG) Serial.println("Value 3 smallest");
        return value3;
      }
    } else {
      Serial.println("ERROR: Unable to make comparison");
      return -1;
    }
  }
}

int difference(int largest, int smallest, int middle, boolean LGGAP) {
  //This function will return the gap between the largest and the middle values
  //of 3 passed values. Alternativley it will return the gap between the largest
  //and smallest values if the LGGAP value is set to FALSE.
  int result;
  if (LGGAP) {
    result = largest - middle;
  } else {
    result = largest - smallest;
  }
  return result;
}

