#include <Servo.h> 

Servo myservo;

int speed;

void setup() 
{ 
  Serial.begin(9600);
  myservo.attach(9);
  myservo.write(0);  // set servo to mid-point
} 

void loop() {
  Serial.println("Speed:");
    while (Serial.available() == 0);
    speed = Serial.parseInt();
    myservo.write(speed);
    delay(100);
    Serial.print("Speed is now ");
    Serial.println(speed);
} 
