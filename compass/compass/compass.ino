#include "Wire.h"
#include "I2Cdev.h"
#include "HMC5883L.h"
HMC5883L compass;
int16_t mx, my, mz;

// Can be deleted if you don't want to check activity
#define LED_PIN 13
bool blinkState = false;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize compass
  Serial.println("Initializing I2C devices...");
  compass.initialize();

  // verify connection to compass
  Serial.println("Testing device connections...");
  Serial.println(compass.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

  // configure Arduino LED pin for output (it can be deleted if you don't want to check activity)
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // read raw heading measurements from device
  compass.getHeading(&mx, &my, &mz);

  // To calculate heading in degrees. 0 degree indicates North
    float heading = atan2(my, mx);
    if(heading < 0)
      heading += 2 * M_PI;
    Serial.print("heading:\t");
    Serial.println(heading * 180/M_PI);

    // blink LED to indicate activity (it can be deleted if you don't want to check activity)
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

}
