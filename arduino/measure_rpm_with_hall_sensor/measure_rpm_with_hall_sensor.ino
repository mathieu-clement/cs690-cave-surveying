#include <Servo.h>

const int hallSensorPin = 8;
const int rotationServoPin = 9;

Servo rotationServo;

int oldVal = 99;
unsigned long rotationTimeMs = 0;
unsigned long oldTime = 0;
int count = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(hallSensorPin, INPUT);
  rotationServo.attach(rotationServoPin);
  rotationServo.write(180); // 0=full speed in reverse, 90=stop

  Serial.println("Finding magnet for the first time...");
  while (digitalRead(hallSensorPin) == 1) {
    delay(20);
  }
  Serial.println("Found!");
  oldTime = millis();
}

void loop()
{
  delay(10);
  int currentVal = digitalRead(hallSensorPin);
  if (oldVal == 0 && currentVal == 1) {
    Serial.print(count);
    Serial.print(" magnet detected --> ");
    count++;
    unsigned long timeNow = millis();
    unsigned long rotationTimeMs = timeNow - oldTime;
    oldTime = timeNow;
    Serial.print(rotationTimeMs);
    Serial.print(" ms per rotation <=> ");
    Serial.print(60000.0/rotationTimeMs);
    Serial.println(" RPM");
  }
  oldVal = currentVal;
}
