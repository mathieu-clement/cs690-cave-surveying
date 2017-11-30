#include <LIDARLite.h>
#include <Servo.h>
#include <Wire.h>

// ROTARY ENCODER
// Brown Vcc
// Blue GND
// Black 2
#define rotaryA 2
// White 3
#define rotaryB 3

// LIDAR
// Red Vcc
// Black GND
// Blue (SDA) A4
// Green (SCL) to A5
LIDARLite lidar; //LIDAR-Lite
Servo servo; //Continuous rotation servo
Servo hor_servo;  // Horizontal axis servo
Servo vert_servo; //Vertical axis servo

// HORIZONTAL AND VERTICAL SERVOS
// Red Vcc
// Brown GND 
// Orange 9 
int HORIZONTAL_AXIS_PIN = 9;
// Orange 10
int VERTICAL_AXIS_PIN = 10;

// CONTINUOUS ROTATION SERVO
// Red Vcc
// Black GND
// White 5
int CONTINUOUS_SERVO_PIN = 5;

volatile int stateA = LOW;
volatile int stateB = LOW;
volatile int counter = 0;
volatile int lastCounter = 0;

void setup() {
  Serial.begin(9600);

  lidar.begin(0, true);
  lidar.configure(0);

  hor_servo.attach(HORIZONTAL_AXIS_PIN); 
  vert_servo.attach(VERTICAL_AXIS_PIN);

  hor_servo.write(90);
  delay(500);
  vert_servo.write(120);
  delay(500);

  pinMode (rotaryA, INPUT_PULLUP);
  pinMode (rotaryB, INPUT_PULLUP);

  servo.attach(CONTINUOUS_SERVO_PIN);
  //servo.write(100); // max speed. 90 to stop, 0 for max speed in reverse

  attachInterrupt(digitalPinToInterrupt(rotaryA), changeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryB), changeB, CHANGE);

  stateA = digitalRead(rotaryA);
  stateB = digitalRead(rotaryB);
}

void loop() {
    // Let client know we are waiting for a command
    Serial.println("Ready");
  
    // Wait until "sweep" is sent over the serial line
    while (Serial.available() == 0);

    servo.write(100);

    String inputStr = Serial.readString();
    if (inputStr == "sweep") {
      sweep();
     }
}

void sweep() {
    int i, j;

    for(i=140; i>=30; i--) {   
      for(j=0; j<1000; j++) {
        vert_servo.write(i);
        if (counter != lastCounter) {
          lastCounter = counter;
          }
        printval(i, counter);
        }
       }
}

void printval(int i, int j) {
      Serial.print("V");
      Serial.print(i);
      Serial.print(" H");
      Serial.print(j);
      Serial.print(" = ");
      Serial.println(lidar.distance());
}

void changeA() {
  stateA = !stateA;

  if (stateA == HIGH) {
    if (stateB == LOW ) {
      counter++;
      if (counter == 1000) {
        counter = 0;
      }
    } else {
        counter--;
        if (counter == -1) {
          counter = 999;
          }
        }
    }
}

void changeB() {
  stateB = !stateB;
}
