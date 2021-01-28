#define M1 8
#define M2 9
#define M3 10

#define HIT 1

#define SIZE 1000

void setup() {
  // put your setup code here, to run once:
  pinMode(M1,INPUT);
  pinMode(M2,INPUT);
  pinMode(M3,INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  bool m1, m2, m3;

  m1 = digitalRead(M1);
  m2 = digitalRead(M2);
  m3 = digitalRead(M3);

  Serial.print(m1);
  Serial.print(" ");
  Serial.print(m2);
  Serial.print(" ");
  Serial.println(m3);  
}
