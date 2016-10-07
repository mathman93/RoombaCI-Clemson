#include <VirtualWire.h>

/*
 * October 7, 2017
 * Eddie Bear
 * This code will get readings from the magnetometer and send them over RF to a base receiver.
 * If the magnetometer could not be found or started correctly, the yellow and green lights will flash continuously.
*/
#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;

float heading, headingDegrees;
int counter = 0;
String message = String("Robot_3 heading is ");
String data;
char buf[50];
void setup()
{
  // Initialize the IO and ISR
  pinMode(8, OUTPUT);
  vw_setup(2000); // Bits per sec
  Serial.begin(57600);

  pinMode(11, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);

  //Test LEDs
  digitalWrite(11, HIGH);
  delay(500);
  digitalWrite(7, HIGH);
  delay(500);
  digitalWrite(8, HIGH);
  delay(1000);
  digitalWrite(11, LOW);
  delay(500);
  digitalWrite(7, LOW);
  delay(500);
  digitalWrite(8, LOW);

  Serial.begin(57600);

  // Initialize Initialize HMC5883L
  Serial.println("Initializing HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    digitalWrite(11, HIGH);
    delay(500);
    digitalWrite(7, HIGH);
    delay(500);
    digitalWrite(11, LOW);
    delay(500);
    digitalWrite(7, LOW);
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);

  Serial.println("[Counter], [Heading], [Heading Degrees]");
}
void loop()
{
  getHeading();
  String data2Send = String(message + headingDegrees);
  data2Send.toCharArray(buf, 50);

  send(buf);
  delay(1000);
}
void send (char *message)
{
  digitalWrite(8, HIGH);
  Serial.print("Sending data: ");
  Serial.println(message);
  vw_send((uint8_t *)message, strlen(message));
  vw_wait_tx(); // Wait until the whole message is gone
  delay(1000);
  digitalWrite(8, LOW);
}

void getHeading() {
  Vector norm = compass.readNormalize();

  // Calculate heading
  heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  headingDegrees = heading * 180 / M_PI;
}

