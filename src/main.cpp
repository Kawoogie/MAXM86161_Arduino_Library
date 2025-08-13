#include <Arduino.h>
#include "maxm86161.h"

#include <Wire.h>


#define i2c_address 92


  
MAXM86161 sensor;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize the I2C connection
  Wire.begin();

  // Start Serial Communication
  Serial.begin(115200);


  while (!Serial);             // Leonardo: wait for serial monitor
}

// the loop function runs over and over again forever
void loop() {
  byte error;
  // Define the MAXM86161 device
  error = sensor.begin(2, 3);
  
  Serial.print("Initialization Variable: ");
  Serial.println(error);

  if (error){
    Serial.println("MAXM86161 initialized!");
  }

  else {
    Serial.println("Problem initializing device.");
  }

  Serial.println("Scanning...");

  Wire.beginTransmission(i2c_address);
  error = Wire.endTransmission();

  if (error){
    Serial.println("MAXM86161 Detected!");
  }

  else {
    Serial.println("Not Detected :(");
  }

  uint8_t id[8];
  id[0] = 0x00;
  error = sensor.data_from_reg(0xFF, *id);

  if (!error){
    Serial.println("Read Error!");
    Serial.print("ID: ");
    Serial.println(id[0]);
  }

  else {
    Serial.println("Read Successful");
    Serial.print("ID: ");
    Serial.println(id[0]);
  }
  
  Serial.println();

  delay(3000);           // wait 3 seconds for next scan

  // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  // Serial.println("LED On");
  // delay(500);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  // Serial.println("LED Off");
  // delay(500);                      // wait for a second
}