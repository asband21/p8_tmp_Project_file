#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define WINDOW_SIZE 5
#define NUM_PARAMS 3 // Latitude, Longitude, Altitude

TinyGPSPlus gps;
SoftwareSerial ss(10, 11); // RX, TX pins

int rolling_index = 0;
double sums[NUM_PARAMS] = {0};                 // Sums for rolling averages
double readings[NUM_PARAMS][WINDOW_SIZE] = {0}; // Rolling window for each parameter

void setup() {
  Serial.begin(9600);
  ss.begin(9600);
  Serial.println("GPS Module with Rolling Average");
  Serial.println("Raw Latitude,Raw Longitude,Raw Altitude,Avg Latitude,Avg Longitude,Avg Altitude"); // CSV Header
}

void loop() {
  while (ss.available() > 0) {
    char c = ss.read();
    gps.encode(c);

    if (gps.location.isUpdated()) { // Process new GPS data
      double rawValues[NUM_PARAMS] = {
          gps.location.lat(),
          gps.location.lng(),
          gps.altitude.meters()};

      double avgValues[NUM_PARAMS];
      updateRollingAverage(rawValues, avgValues);

      sendToSerial(rawValues, avgValues);
    }
  }
}

/**
 * Updates the rolling averages and calculates the average for each parameter.
 */
void updateRollingAverage(double rawValues[], double avgValues[]) {
  for (int i = 0; i < NUM_PARAMS; i++) {
      // Debug: Print current rolling_index and sums
    //Serial.print("Index: "); Serial.print(rolling_index);
    //Serial.print(", Old Value: "); Serial.print(readings[i][rolling_index]);
    //Serial.print(", New Value: "); Serial.println(rawValues[i]);
    sums[i] -= readings[i][rolling_index];   // Remove the oldest value
    readings[i][rolling_index] = rawValues[i]; // Store the new value
    sums[i] += readings[i][rolling_index];  // Add the new value
    avgValues[i] = sums[i] / min(rolling_index + 1, WINDOW_SIZE); // Calculate average
  }

  rolling_index = (rolling_index + 1) % WINDOW_SIZE; // Update the rolling index
}

/**
 * Sends the raw and averaged GPS values over Serial in CSV format.
 */
void sendToSerial(double rawValues[], double avgValues[]) {
  Serial.print(rawValues[0], 2); Serial.print(",");
  Serial.print(rawValues[1], 2); Serial.print(",");
  Serial.print(rawValues[2]); Serial.print(",");
  Serial.print(avgValues[0], 2); Serial.print(",");
  Serial.print(avgValues[1], 2); Serial.print(",");
  Serial.println(avgValues[2]);
}