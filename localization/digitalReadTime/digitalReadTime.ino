#define M1 8
#define M2 9
#define M3 10

#define HIT 1

#define SIZE 50000

void setup() {
  Serial.begin(9600);
  Serial.println("START");
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long int i,j;
  unsigned long start1, start2, start3;
  unsigned long t1, t2, t3;

  j = 0;
  start1 = micros();
  for (i= 0; i < SIZE; i++)
  {
    digitalRead(M1);
  }
  t1 = micros()-start1;

  start2 = micros();
  for (i= 0; i < SIZE; i++)
  {
    digitalRead(M1);
    digitalRead(M2);
  }
  t2 = micros()-start2;

  start3 = micros();
  for (i= 0; i < SIZE; i++)
  {
    digitalRead(M1);
    digitalRead(M2);
    digitalRead(M3);
  }
  t3 = micros() - start3;

  Serial.print("t1 - ");
  Serial.println(t1);
  Serial.print("Time Per - ");
  Serial.println(((double)t1)/SIZE);
  Serial.print("Start - ");
  Serial.println(start1);
  Serial.println();

  Serial.print("t2 - ");
  Serial.println(t2);
  Serial.print("Time Per - ");
  Serial.println(((double)t2)/SIZE);
  Serial.print("Start - ");
  Serial.println(start2);
  Serial.println();

  Serial.print("t3 - ");
  Serial.println(t3);
  Serial.print("Time Per - ");
  Serial.println(((double)t3)/SIZE);
  Serial.print("Start - ");
  Serial.println(start3);
  Serial.println();

  while(1);
}
