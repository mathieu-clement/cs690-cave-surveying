#include <LIDARLite.h>
#include <Servo.h>
#include <Wire.h>


class Bitset {
    int bins[125]; // ceil(1000/8) = 125

  public:
    Bitset() {
      for (int i = 0 ; i < 125; i++) {
        bins[i] = 0;
      }
    }

    void set(int i) {
      int binNo = i / 8;
      int indexInBin = i - binNo * 8;
      int bin = bins[binNo];
      int mask = 1 << indexInBin;
      bin |= mask;
      bins[binNo] = bin;
    }

    bool isSet(int i) {
      int binNo = i / 8;
      int indexInBin = i - binNo * 8;
      int bin = bins[binNo];
      int mask = 1 << indexInBin;
      int result = bin & mask;
      return result != 0;
    }

    bool all() {
      for (int i = 0 ; i < 125; ++i) {
        if (bins[i] != 0xff) {
          return false;
        }
      }

      return true;
    }
};



// ---------------
// !!! WARNING !!!
// ---------------
// ALL COLORS REFER TO THE COLOR OF THE WIRES
// COMING OUT OF COMPONENTS/
// They might be connected through other color wires
// through the slip ring.

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
Servo cont_servo; //Continuous rotation servo
Servo hor_servo;  // Horizontal axis servo
Servo vert_servo; //Vertical axis servo

// HORIZONTAL AND VERTICAL SERVOS
// Red Vcc
// Brown GND
// Orange 9
int HORIZONTAL_AXIS_PIN = 9;
// Orange 10
int VERTICAL_AXIS_PIN = 10;
#define LOW_VERT_ANGLE 30
#define HIGH_VERT_ANGLE 113

// CONTINUOUS ROTATION SERVO
// Red Vcc
// Black GND
// White 5
int CONTINUOUS_SERVO_PIN = 5;
// try small decrements/increments
#define CONT_SPEED 100
#define MAX_TURNS 4

volatile int stateA = LOW;
volatile int stateB = LOW;
volatile int counter = 0;
volatile int lastCounter = 0;
volatile int turns = 0;

void setup() {
  Serial.begin(115200);
  delay(50);

  lidar.begin(0, true); // min 22 ms before taking measurements
  lidar.configure(0);

  hor_servo.attach(HORIZONTAL_AXIS_PIN);
  vert_servo.attach(VERTICAL_AXIS_PIN);

  hor_servo.write(90);
  delay(500);
  vert_servo.write(HIGH_VERT_ANGLE);
  delay(500);

  pinMode (rotaryA, INPUT_PULLUP);
  pinMode (rotaryB, INPUT_PULLUP);

  cont_servo.attach(CONTINUOUS_SERVO_PIN);
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

  String inputStr = Serial.readString();
  if (inputStr == "sweep") {
    sweep();
  }
}

void sweep() {
  cont_servo.write(CONT_SPEED);

  for (int i = HIGH_VERT_ANGLE; i >= LOW_VERT_ANGLE; i--) {
    vert_servo.write(i);
    delay(50);

    Bitset bitset;

    turns = 0;
    int cntBit = 0;
    while (cntBit !=  1000 && turns < MAX_TURNS) {
      if (counter != lastCounter) {
        lastCounter = counter;
        if (!bitset.isSet(counter)) {
          bitset.set(counter);
          cntBit++;
        }
      }
      printval(i, 1000-counter);
    }
  } // for i

  vert_servo.write(HIGH_VERT_ANGLE);
  cont_servo.write(90);
} // sweep()


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
        turns++;
      }
    } else {
      counter--;
      if (counter == -1) {
        counter = 999;
        turns++;
      }
    }
  }
}

void changeB() {
  stateB = !stateB;
}
