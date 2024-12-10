/*///////////////////////////////////////////////////////////////////////////////////////////////
This is one half of a script for the 2024 7th semester project "Adaptive and Self-tuning Control
Strategies for Dynamic Positioning of Marine Crafts". This script is for the Arduino Mega at the
base station on solid ground. It records 100, 100 second windows, creating a roughly 3 hour averaged
GPS coordinate with minimal noise interference. It then calculates the error for each new GPS 
coordinate. The error indicates disturbances in the GPS signal, which is relatively similar enough
for the two locations, that it can be used for noise compensation. The error is sent over radio.

Input: GPS over 100s, New GPS at 1Hz   Output: GPS error measurements at 1Hz over 344mHz radio

*////////////////////////////////////////////////////////////////////////////////////////////////

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define RADIO_RX 8
#define RADIO_TX 9
#define GPS_RX 10
#define GPS_TX 11

#define SHORT_WINDOW 100 //Sizes of the averages
#define LONG_WINDOW 120

// LED pins for analog debugging
const int LED_RED = 2; // Red LED indicates the GPS is receiving periodic input, if its blinking periodically, signal is being lost periodically
const int LED_GREEN = 3; // Each time the short-window is done measuring, the green LED blinks
bool radio_ok = 0; // Commands green LED
int gps_stall = 0; // Commands red LED. If we lose communication this starts counting, resets on restored conmmunication
int gps_stall_threshold = 520; // The program loops ~450-490 times between recieved signals. If a signal is missed, this jumps to ~1000.
                               // Resulting in gps_stall becoming negative, and red LED going dark.

// Serial connections
TinyGPSPlus gps;
SoftwareSerial radioSerial(RADIO_RX, RADIO_TX);
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// The arrays for the long and the short window
double shortLat[SHORT_WINDOW];
double shortLng[SHORT_WINDOW];
double longLat[LONG_WINDOW];
double longLng[LONG_WINDOW];


// Averaging and computation
double avgLat = 0, avgLng = 0;
double trueLat = 0; // These store the long window average, our best position estimate
double trueLng = 0;
double cptDiffLat = 0; // Computed Difference Lattitude
double cptDiffLng = 0; // & Longditude

// Counter for short window filling and long window ring buffer
int shortCount = 0;
int longCount = 0;
int count = 0; // This is internal to the rolling average


void setup() {
  Serial.begin(9600);  // For Debugging
  radioSerial.begin(9600); 
  gpsSerial.begin(9600);  

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

/* This memory allocation was devised by Asbjørn, I (Hector) couldnt get it to work when I made major changes to the program.
  shortLat = (double *)malloc(sizeof (double)*SHORT_WINDOW);
  shortLng = (double *)malloc(sizeof (double)*SHORT_WINDOW);
  longLat = (double *)malloc(sizeof (double)*LONG_WINDOW);
  longLng = (double *)malloc(sizeof (double)*LONG_WINDOW);
*/
}

void loop() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
    
    if (gps.location.isUpdated()) {  
      double lat = gps.location.lat();
      double lng = gps.location.lng();

      shortLat[shortCount] = lat;
      shortLng[shortCount] = lng;
    
      if (shortCount >= SHORT_WINDOW -1) {
        double latAverage = 0, lngAverage = 0;
        double latSum = 0, lngSum = 0;
        for (int i = 0; i < SHORT_WINDOW; i++) {
           latSum += shortLat[i];
           lngSum += shortLng[i];
          }
        latAverage = latSum / SHORT_WINDOW;
        lngAverage = lngSum / SHORT_WINDOW;

        if(1){
        ////////// Rolling Average For The Long Window ////////////////////////
        double latSum = 0, lngSum = 0;    
        double newValue = 0;
        int storedValues = 0;

          longLat[longCount % LONG_WINDOW] = latAverage;
          longLng[longCount % LONG_WINDOW] = lngAverage;
          longCount++;

            for (int i = 0; i < LONG_WINDOW; i++) {
              if (longLat[i] > 1) {
                latSum += longLat[i];
                storedValues++;
                //Serial.print(longLat[i],6);
                //Serial.print(", ");
              }
            }
          trueLat = latSum / storedValues;
          longLat[longCount % LONG_WINDOW] = trueLat;
          //Serial.println(trueLat,10);

          storedValues = 0;
            for (int i = 0; i < LONG_WINDOW; i++) {
              if (longLng[i] > 1) {
                lngSum += longLng[i];
                storedValues++;
                //Serial.print(longLng[i],6);
                //Serial.print(", ");
              }
            }
          trueLng = lngSum / storedValues;
          longLng[longCount % LONG_WINDOW] = trueLng;
          //Serial.println(trueLng,10);

           digitalWrite(LED_GREEN, HIGH);
        ///////////////////////////////////////////////////////////////////////
        }
        shortCount = 0;
      } else {
        shortCount++;
      }

      // Computing the difference, then sending it over radio.
      cptDiffLat = lat - trueLat;
      cptDiffLng = lng - trueLng;
      radioSerial.print(cptDiffLat, 10);
      radioSerial.print(",");
      radioSerial.println(cptDiffLng, 10);

      // Graphics For Testing And Report
      /*
      Serial.print("True_Lat:");
      Serial.print(trueLat, 8);
      Serial.print(",");
      Serial.print("Raw_Lat:");
      Serial.println(lat, 8);
      */
      /*
      Serial.print("True_Lng:");
      Serial.print(trueLng, 8);
      Serial.print(",");
      Serial.print("Computed_Lng:");
      Serial.print((lng - cptDiffLng), 8);
      Serial.print(",");
      Serial.print("Raw_Lng:");
      Serial.println(lng, 8);
      */

      gps_stall = gps_stall_threshold; // If gps signal is lost, the red LED turns off. When this runs, the stall counter is always reset
    } 
  }
if (gps_stall > 0){
      digitalWrite(LED_RED, HIGH);
    } else {
      digitalWrite(LED_RED, LOW);
    }
  gps_stall--;
}