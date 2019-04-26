#define M1 8
#define M2 9
#define M3 10

#define HIT 1

#define SIZE 500

bool m1[SIZE+1], m2[SIZE+1], m3[SIZE+1];

void setup() {
  // put your setup code here, to run once:
  pinMode(M1,INPUT);
  pinMode(M2,INPUT);
  pinMode(M3,INPUT);
  Serial.begin(9600);
  Serial.println("BEGIN");
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long i = 0;

digitalRead(M1);
Serial.println("1");

digitalRead(M2);
Serial.println("2");

digitalRead(M3);
Serial.println("3");
delay(500);
  
  while((digitalRead(M1) != HIT) && (digitalRead(M2) != HIT) && (digitalRead(M3) != HIT));
  m1[0] = digitalRead(M1);
  m2[0] = digitalRead(M2);
  m3[0] = digitalRead(M3);
  
  if (m1[0] == HIT)
  {
    for (i = 0; i < SIZE; i++)
    {
      m2[i] = digitalRead(M2);
      m3[i] = digitalRead(M3);
      //Serial.println(i);
      //delay(100);
    }
    for (i = 0; i < SIZE; i++)
    {
      m1[i] = 1;
    }
    Serial.println("1 hit first.");
  }
  else if (m2[0] == HIT)
  {
    for (i = 0; i < SIZE; i++)
    {
      m1[i] = digitalRead(M1);
      m3[i] = digitalRead(M3);
      //Serial.println(i);
      //delay(100);
    }
    for (i = 0; i < SIZE; i++)
    {
      m2[i] = 1;
    }
    Serial.println("2 hit first.");
  }
  else
  {
    for (i = 0; i < SIZE; i++)
    {
      m1[i] = digitalRead(M1);
      m2[i] = digitalRead(M2);
      //Serial.println(i);
      //delay(100);
    }
    Serial.println("3 hit first.");
    for (i = 0; i < SIZE; i++)
    {
      m3[i] = 1;
    }
  }

  //Serial.print("START TIME: ");
  //Serial.println(start);
  for (i = 0; i < SIZE; i++)
  {
    Serial.print(m1[i]);
    Serial.print(" ");
    Serial.print(m2[i]);
    Serial.print(" ");
    Serial.println(m3[i]);
  }
  
  while(1);
}
