 #define outputA 2
 #define outputB 3
 #define LED 13
 
 volatile int stateA = LOW;
 volatile int stateB = LOW;
 volatile int counter = 0;
 volatile int lastCounter = 0;
 
 void setup() { 
   pinMode (outputA, INPUT_PULLUP);
   pinMode (outputB, INPUT_PULLUP);
   // pinMode (outputB, INPUT);
   pinMode (LED, OUTPUT);
   
   Serial.begin (9600);

   attachInterrupt(0 /* pin 2 */, changeA, CHANGE);
   attachInterrupt(1 /* pin 3 */, changeB, CHANGE);

   stateA = digitalRead(outputA);
   stateB = digitalRead(outputB);
 } 
 void loop() {
   //state = digitalRead(outputA);
   digitalWrite(LED, stateA);

   if (counter != lastCounter) {
    Serial.println(counter);
    lastCounter = counter;
   }
   //Serial.println(state);
 }

 void changeA() {
   stateA = !stateA;

  if (stateA == HIGH && stateB == HIGH) {
      counter++;
      if (counter == 1000) {
        counter = 0;
      }
      
   }   
   if (stateA == HIGH && stateB == LOW) {
      counter--;
      if (counter == -1) {
        counter = 999;
      }
   }
 }

 void changeB() {
   stateB = !stateB;
 }

