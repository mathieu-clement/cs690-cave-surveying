#include <Servo.h>

Servo hor_servo;  // Horizontal axis
Servo vert_servo; // Vertical axis

int HORIZONTAL_AXIS_PIN = 9;
int VERTICAL_AXIS_PIN = 10;

int degrees;

void setup() {
  Serial.begin(9600);
  hor_servo.attach(HORIZONTAL_AXIS_PIN); 
  vert_servo.attach(VERTICAL_AXIS_PIN);

  hor_servo.write(90);
  delay(500);
  vert_servo.write(130);
  delay(500);
}

void loop() {
    Serial.println("Horizontal axis (degrees)");
    while (Serial.available() == 0);
    degrees = Serial.parseInt();
    hor_servo.write(degrees);
    delay(100);

    Serial.println("Vertical axis (degrees)");
    while (Serial.available() == 0);
    degrees = Serial.parseInt();
    vert_servo.write(degrees);
    delay(100);
} // loop
