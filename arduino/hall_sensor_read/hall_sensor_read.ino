
volatile long count = 0;

 void setup()
 {
   Serial.begin(9600);
   attachInterrupt(digitalPinToInterrupt(2), magnet_detect, FALLING);//Initialize the intterrupt pin (Arduino digital pin 2)
   
 }
 void loop()
 {
   delay(5000); 
 }
 void magnet_detect()//This function is called whenever a magnet/interrupt is detected by the arduino
 {
   Serial.print(count);
   Serial.println(" detect");
   count++;
 }
