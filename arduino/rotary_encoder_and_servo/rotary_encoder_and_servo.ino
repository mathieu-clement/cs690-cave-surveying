#define rotaryA 2
 #define rotaryB 3
 #define LED 13

 volatile int stateA = LOW;
 volatile int stateB = LOW;
 volatile int counter = 0;
 volatile int lastCounter = 0;

 void setup() { 
   pinMode (rotaryA, INPUT_PULLUP);
   pinMode (rotaryB, INPUT_PULLUP);
   pinMode (LED, OUTPUT);

   Serial.begin (9600);

   attachInterrupt(digitalPinToInterrupt(rotaryA), changeA, CHANGE);
   attachInterrupt(digitalPinToInterrupt(rotaryB), changeB, CHANGE);

   stateA = digitalRead(rotaryA);
   stateB = digitalRead(rotaryB);
 } 
 void loop() {
   digitalWrite(LED, stateA);

   if (counter != lastCounter) {
    Serial.println(counter);
    lastCounter = counter;
   }
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

 void changeB() {
   stateB = !stateB;
 }
