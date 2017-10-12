#include <Servo.h>

const int hallSensorPin = 8;
const int rotationServoPin = 9;

Servo rotationServo;

int oldVal = 99;
int count = 0;

void setup() {
  Serial.begin(9600);
  pinMode(hallSensorPin, INPUT);
  rotationServo.attach(rotationServoPin);
  rotationServo.write(180); // 0=full speed in reverse, 90=stop
}

void loop() {
  delay(30);
  int currentVal = digitalRead(hallSensorPin);
  if (oldVal == 0 && currentVal == 1) {
    Serial.print(count);
    Serial.println(" detect");
    count++;
  }
  oldVal = currentVal;
}
