#include <SoftwareSerial.h>

SoftwareSerial apc220Serial(10, 11);

void setup()
{
  Serial.begin(9600); // Start serial communication at 9600 baud
  while (!Serial){}
  apc220Serial.begin(9600);
  while (!apc220Serial){}

}

int i = 0;
void loop() {
  
  //if (Serial.available() > 0){
    delay(2000);
    //char received = Serial.read(); // Read the incoming byte
    Serial.println(i);
    apc220Serial.println(i);
    i = (i + 1) % 9;
    
  //}
}
