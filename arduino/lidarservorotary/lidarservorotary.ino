#include <LIDARLite.h>
#include <Servo.h>
#include <Wire.h>

// This bitset acts the same as an array of 1000 booleans
// We use it to keep track of which point has been seen
// for a particular longitude. There are 1000 data points
// per revolution.
class Bitset {
    int bins[125]; // ceil(1000/8) = 125

  public:
    Bitset() {
      // Init everything to false
      for (int i = 0 ; i < 125; i++) {
        bins[i] = 0;
      }
    }

    // Set index i to true
    void set(int i) {
      int binNo = i / 8;
      int indexInBin = i - binNo * 8;
      int bin = bins[binNo];
      int mask = 1 << indexInBin;
      bin |= mask;
      bins[binNo] = bin;
    }

    // Returns true if index i was set to true
    bool isSet(int i) {
      int binNo = i / 8;
      int indexInBin = i - binNo * 8;
      int bin = bins[binNo];
      int mask = 1 << indexInBin;
      int result = bin & mask;
      return result != 0;
    }

    // Returns true if all indices are set to true
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
int HORIZONTAL_AXIS_PIN = 9; // optional
// Orange 10
int VERTICAL_AXIS_PIN = 10;
#define LOW_VERT_ANGLE 30
#define HIGH_VERT_ANGLE 140

// CONTINUOUS ROTATION SERVO
// Red Vcc
// Black GND
// White 5
int CONTINUOUS_SERVO_PIN = 5;
// try small decrements/increments
#define CONT_SPEED 100
#define MAX_TURNS 4

volatile int stateA = LOW; // rotary encoder data line A
volatile int stateB = LOW; // rotary encoder data line B
volatile int counter = 0;  // rotary encoder position
volatile int lastCounter = 0; // last rotary encoder position printed
volatile int turns = 0; // counts how many turns on the same longitude

void setup() {
  // Prepare serial input / output
  Serial.begin(115200);
  delay(50);

  // Prepare LIDAR
  lidar.begin(0, true); // min 22 ms before taking measurements
  lidar.configure(0);

  // Configure servos
  hor_servo.attach(HORIZONTAL_AXIS_PIN); // optional
  vert_servo.attach(VERTICAL_AXIS_PIN);
  hor_servo.write(90);
  delay(500);

  // Move the vertical servo to its lowest position
  vert_servo.write(HIGH_VERT_ANGLE);
  delay(500);

  // Prepare rotary encoder input lines
  // We are using Arduino's internal pull up resistors
  // These must also be present in a permanent installation,
  // in which case the pinMode can be set to INPUT. 
  pinMode (rotaryA, INPUT_PULLUP);
  pinMode (rotaryB, INPUT_PULLUP);

  // Configure the continuous servomotor
  cont_servo.attach(CONTINUOUS_SERVO_PIN);
  //servo.write(100); // max speed. 90 to stop, 0 for max speed in reverse

  // React to events from rotary encoder using interrupts
  attachInterrupt(digitalPinToInterrupt(rotaryA), changeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryB), changeB, CHANGE);

  // Initial state for interrupt code
  stateA = digitalRead(rotaryA);
  stateB = digitalRead(rotaryB);
}

void loop() {
  // Let client know we are waiting for a command
  Serial.println("Ready");

  // Wait until "sweep" is sent over the serial line
  while (Serial.available() == 0);

  String inputStr = Serial.readString();
  if (inputStr == "sweep") { // Yes, this is correct in C++ / Arduino
    sweep();
  }
}

void sweep() {
  // Get the continuous servo up to speed
  cont_servo.write(CONT_SPEED);

  for (int i = HIGH_VERT_ANGLE; i >= LOW_VERT_ANGLE; i--) {
    // Move the vertical servo up
    vert_servo.write(i);
    delay(50);

    Bitset bitset; // see documentation in class definition

    turns = 0; // reset turn counter for each longitude
    int cntBit = 0; // counts how many bits are set
                    // faster than checking bitset.all()

    // Turn until all points have been seen for the current longitude
    // but stop if we already turned MAX_TURNS times.
    while (cntBit !=  1000 && turns < MAX_TURNS) {
      if (counter != lastCounter) {
        lastCounter = counter;
        if (!bitset.isSet(counter)) {
          bitset.set(counter);
          cntBit++;
        }
      }
      printval(i, 1000 - counter); // print coordinates and distance
                                   // 1000 - counter mirrors image
    }
  } // for i

  // Reset vertical servo at the end of the measurement
  vert_servo.write(HIGH_VERT_ANGLE);
  
  // Stop the continuous rotation servo
  cont_servo.write(90);
} // sweep()


// Prints for example "V120 H140 = 23"
// if we measured 23 cm for a longitude of 120 and a latitude of 140
void printval(int i, int j) {
  Serial.print("V");
  Serial.print(i);
  Serial.print(" H");
  Serial.print(j);
  Serial.print(" = ");
  Serial.println(lidar.distance());
}

// ISR called when rotary encoder data line A changes
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
      /*
      counter--;
      if (counter == -1) {
        counter = 999;
        turns++;
      }
      */
      // Workaround: Normally we would use the code above,
      // but in this application direction should be always the same.
      // As we have experienced random direction changes, 
      // maxing sure the direction stays the same avoids problems.
      counter++;
      if (counter == 1000) {
        counter = 0;
        turns++;
      }
    }
  }
}

// ISR called when rotary encoder data line B changes
void changeB() {
  stateB = !stateB;
}
